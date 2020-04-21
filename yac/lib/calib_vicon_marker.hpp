#ifndef YAC_CALIB_VICON_MARKER_HPP
#define YAC_CALIB_VICON_MARKER_HPP

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "calib_camera.hpp"

namespace yac {

/**
 * Stereo camera calibration residual
 */
struct vicon_marker_residual_t {
  double z_[2] = {0.0, 0.0};        ///< Measurement from cam0
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  vicon_marker_residual_t(const vec2_t &z, const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~vicon_marker_residual_t() {}

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
                  const T *const distortion_,
                  const T *const q_MC_,
                  const T *const r_MC_,
                  const T *const q_WM_,
                  const T *const r_WM_,
                  const T *const q_WF_,
                  const T *const r_WF_,
                  T *residual) const {
    // Map optimization variables to Eigen
    // -- Camera intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam_params;
    cam_params << intrinsics_[0], intrinsics_[1], intrinsics_[2], intrinsics_[3],
                  distortion_[0], distortion_[1], distortion_[2], distortion_[3];
    // -- Marker to camera extrinsics pose
    const Eigen::Quaternion<T> q_MC(q_MC_[3], q_MC_[0], q_MC_[1], q_MC_[2]);
    const Eigen::Matrix<T, 3, 3> C_MC = q_MC.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_MC{r_MC_[0], r_MC_[1], r_MC_[2]};
    const Eigen::Matrix<T, 4, 4> T_MC = tf(C_MC, r_MC);
    // -- Marker pose
    const Eigen::Quaternion<T> q_WM(q_WM_[3], q_WM_[0], q_WM_[1], q_WM_[2]);
    const Eigen::Matrix<T, 3, 3> C_WM = q_WM.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_WM{r_WM_[0], r_WM_[1], r_WM_[2]};
    const Eigen::Matrix<T, 4, 4> T_WM = tf(C_WM, r_WM);
    // -- Fiducial pose
    const Eigen::Quaternion<T> q_WF(q_WF_[3], q_WF_[0], q_WF_[1], q_WF_[2]);
    const Eigen::Matrix<T, 3, 3> C_WF = q_WF.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_WF{r_WF_[0], r_WF_[1], r_WF_[2]};
    const Eigen::Matrix<T, 4, 4> T_WF = tf(C_WF, r_WF);

    // Project fiducial object point to camera image plane
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
    const Eigen::Matrix<T, 4, 4> T_CM = T_MC.inverse();
    const Eigen::Matrix<T, 4, 4> T_MW = T_WM.inverse();
    const Eigen::Matrix<T, 4, 1> hp_C = T_CM * T_MW * T_WF * p_F.homogeneous();
    const Eigen::Matrix<T, 3, 1> p_C = hp_C.head(3);
    Eigen::Matrix<T, 2, 1> z_hat;
    if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
      return false;
    }

    // Residual
    residual[0] = T(z_[0]) - z_hat(0);
    residual[1] = T(z_[1]) - z_hat(1);

    return true;
  }
};

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_t *T_MC,
                             calib_pose_t *T_WM,
                             calib_pose_t *T_WF,
                             ceres::Problem *problem) {
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
      const auto residual = new vicon_marker_residual_t{kp, obj_pt};

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

template <typename CM>
double evaluate_vicon_marker_cost(const std::vector<aprilgrid_t> &aprilgrids,
                                  mat4s_t &T_WM,
                                  CM &cm,
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
                                   cm.params.data(),
                                   cm.distortion.data(),
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

template <typename CM>
int calib_vicon_marker_solve(const aprilgrids_t &aprilgrids,
                             CM &cm,
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
                                   cm.params.data(),
                                   cm.distortion.params.data(),
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
  problem->SetParameterBlockConstant(cm.params.data());
  problem->SetParameterBlockConstant(cm.distortion.params.data());

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

} //  namespace yac
#endif // YAC_CALIB_VICON_MARKER_HPP
