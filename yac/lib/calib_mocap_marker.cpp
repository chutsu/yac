#include "calib_mocap_marker.hpp"

namespace yac {

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             camera_params_t &cam,
                             calib_pose_t *T_MC,
                             calib_pose_t *T_WM,
                             calib_pose_t *T_WF,
                             ceres::Problem *problem) {
  const std::string proj_model = cam.proj_model;
  const std::string dist_model = cam.dist_model;
  double *intrinsics = cam.proj_params().data();
  double *distortion = cam.dist_params().data();

  for (const auto &tag_id : aprilgrid.ids) {
    // Get keypoints
    vec2s_t keypoints;
    if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp = keypoints[i];
      const auto obj_pt = object_points[i];
      const auto residual = new mocap_marker_residual_t{proj_model, dist_model,
                                                        kp, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<mocap_marker_residual_t,
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_MC
                                          3, // Size of: t_MC
                                          4, // Size of: q_WM
                                          3, // Size of: t_WM
                                          4, // Size of: q_WF
                                          3  // Size of: t_WF
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                intrinsics,
                                distortion,
                                T_MC->q,
                                T_MC->r,
                                T_WM->q,
                                T_WM->r,
                                T_WF->q,
                                T_WF->r);
    }
  }

  return 0;
}

int calib_mocap_marker_solve(const aprilgrids_t &aprilgrids,
                             camera_params_t &cam,
                             mat4s_t &T_WM,
                             mat4_t &T_MC,
                             mat4_t &T_WF) {
  assert(aprilgrids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == aprilgrids.size());

  // Optimization variables
  calib_pose_t T_MC_param{T_MC};
  calib_pose_t T_WF_param{T_WF};
  std::vector<calib_pose_t> T_WM_params;
  for (size_t i = 0; i < T_WM.size(); i++) {
    T_WM_params.push_back(T_WM[i]);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_opts;
  problem_opts.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_opts));
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   cam,
                                   &T_MC_param,
                                   &T_WM_params[i],
                                   &T_WF_param,
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    // Set quaternion parameterization for T_WM
    problem->SetParameterization(T_WM_params[i].q,
                                 &quaternion_parameterization);

    // Fixing the marker pose - assume mocap is calibrated and accurate
    problem->SetParameterBlockConstant(T_WM_params[i].q);
    problem->SetParameterBlockConstant(T_WM_params[i].r);
  }

  // // Fix camera parameters
  // problem->SetParameterBlockConstant(cam.proj_params.data());
  // problem->SetParameterBlockConstant(cam.dist_params.data());

  // Fix T_WF parameters
  // problem->SetParameterBlockConstant(T_WF_param.q);
  // problem->SetParameterBlockConstant(T_WF_param.r);

  // Set quaternion parameterization for T_MC
  problem->SetParameterization(T_MC_param.q,
                               &quaternion_parameterization);
  problem->SetParameterization(T_WF_param.q,
                               &quaternion_parameterization);

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  options.num_threads = 4;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Estimate covariance matrix
  std::vector<std::pair<const double*, const double*>> covar_blocks;
  covar_blocks.push_back({T_MC_param.q, T_MC_param.q});
  covar_blocks.push_back({T_MC_param.q, T_MC_param.r});
  covar_blocks.push_back({T_MC_param.r, T_WF_param.r});

  ceres::Covariance::Options covar_options;
  covar_options.algorithm_type = ceres::SPARSE_QR;
  covar_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  ceres::Covariance covar(covar_options);
  const auto retval = covar.Compute(covar_blocks, problem.get());
  if (retval == false) {
    printf("Estimate covariance failed!\n");
  }

  double q_MC_q_MC_covar[3 * 3] = {0};
  double q_MC_r_MC_covar[3 * 3] = {0};
  double r_MC_r_MC_covar[3 * 3] = {0};

  covar.GetCovarianceBlockInTangentSpace(T_MC_param.q, T_MC_param.q, q_MC_q_MC_covar);
  covar.GetCovarianceBlockInTangentSpace(T_MC_param.q, T_MC_param.r, q_MC_r_MC_covar);
  covar.GetCovarianceBlockInTangentSpace(T_MC_param.r, T_MC_param.r, r_MC_r_MC_covar);

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covar_rot(q_MC_q_MC_covar);
  auto rotx_std = (covar_rot(0, 0) < 1e-8) ? 0 : std::sqrt(covar_rot(0, 0));
  auto roty_std = (covar_rot(1, 1) < 1e-8) ? 0 : std::sqrt(covar_rot(1, 1));
  auto rotz_std = (covar_rot(2, 2) < 1e-8) ? 0 : std::sqrt(covar_rot(2, 2));
  print_matrix("covar_rot", covar_rot);
  printf("rotx_std: %f [deg]\n", rad2deg(rotx_std));
  printf("roty_std: %f [deg]\n", rad2deg(roty_std));
  printf("rotz_std: %f [deg]\n", rad2deg(rotz_std));

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covar_trans(r_MC_r_MC_covar);
  auto rx_std = std::sqrt(covar_trans(0, 0));
  auto ry_std = std::sqrt(covar_trans(1, 1));
  auto rz_std = std::sqrt(covar_trans(2, 2));
  printf("rx_std: %e\n", rx_std);
  printf("ry_std: %e\n", ry_std);
  printf("rz_std: %e\n", rz_std);

  // Finish up
  // -- Marker pose
  T_WM.clear();
  for (auto T_WM_param : T_WM_params) {
    T_WM.push_back(T_WM_param.T());
  }
  // -- Marker to camera extrinsics
  T_MC = T_MC_param.T();
  // -- Fiducial pose
  T_WF = T_WF_param.T();

  return 0;
}

} //  namespace yac
