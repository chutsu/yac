#include "calib_vicon_marker.hpp"

namespace yac {

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             calib_params_t &cam,
                             calib_pose_t *T_MC,
                             calib_pose_t *T_WM,
                             calib_pose_t *T_WF,
                             ceres::Problem *problem) {
  const std::string proj_model = cam.proj_model;
  const std::string dist_model = cam.dist_model;
  double *intrinsics = cam.proj_params.data();
  double *distortion = cam.dist_params.data();

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
      const auto residual = new vicon_marker_residual_t{proj_model, dist_model,
                                                        kp, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<vicon_marker_residual_t,
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

int calib_vicon_marker_solve(const aprilgrids_t &aprilgrids,
                             calib_params_t &cam,
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

    // Fixing the marker pose - assume vicon is calibrated and accurate
    problem->SetParameterBlockConstant(T_WM_params[i].q);
    problem->SetParameterBlockConstant(T_WM_params[i].r);
  }

  // Fix camera parameters
  problem->SetParameterBlockConstant(cam.proj_params.data());
  problem->SetParameterBlockConstant(cam.dist_params.data());

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

double evaluate_vicon_marker_cost(const aprilgrids_t &aprilgrids,
                                  calib_params_t &cam,
                                  mat4s_t &T_WM,
                                  mat4_t &T_MC) {
  assert(aprilgrids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == aprilgrids.size());

  // Optimization variables
  calib_pose_t T_MC_param{T_MC};
  calib_pose_t T_WF_param{T_WM[0] * T_MC * aprilgrids[0].T_CF};
  std::vector<calib_pose_t> T_WM_params;
  for (size_t i = 0; i < T_WM.size(); i++) {
    T_WM_params.push_back(T_WM[i]);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));

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
  }

  double cost;
  problem->Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
  return cost;
}

} //  namespace yac
