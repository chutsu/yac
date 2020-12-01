#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "calib_mono.hpp"

namespace yac {

/* Stereo Camera Calibration Data */
struct calib_stereo_data_t {
  mat2_t covar;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  camera_params_t cam0;
  camera_params_t cam1;
  mat4_t T_C1C0;
  mat4s_t T_C0F;
  mat4s_t T_C1F;
};

/** Stereo camera calibration residual */
template <typename CAMERA_TYPE>
struct calib_stereo_residual_t
    : public ceres::SizedCostFunction<4, 7, 7, 8, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_C0_{0.0, 0.0};
  vec2_t z_C1_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_stereo_residual_t(const int cam_res[2],
                          const vec3_t &r_FFi,
                          const vec2_t &z_C0,
                          const vec2_t &z_C1,
                          const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        r_FFi_{r_FFi}, z_C0_{z_C0}, z_C1_{z_C1},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(4, 6));  // T_CF
    J_min.push_back(zeros(4, 6));  // T_C0C1
    J_min.push_back(zeros(4, 8));  // cam0 params
    J_min.push_back(zeros(4, 8));  // cam1 params
  }

  ~calib_stereo_residual_t() {}

  /**
   * Calculate residual
   */
  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    // -- Map camera-fiducial pose
    const mat4_t T_C0F = tf(params[0]);
    const mat3_t C_C0F = tf_rot(T_C0F);
    const quat_t q_C0F{C_C0F};
    // -- Map camera-camera pose
    const mat4_t T_C1C0 = tf(params[1]);
    const mat4_t T_C1F = T_C1C0 * T_C0F;
    const mat3_t C_C1C0 = tf_rot(T_C1C0);
    const mat3_t C_C1F = tf_rot(T_C1F);
    const quat_t q_C1F{C_C1F};
    const quat_t q_C1C0{C_C1C0};
    // -- Map camera params
    Eigen::Map<const vecx_t> cam0_params(params[2], 8);
    Eigen::Map<const vecx_t> cam1_params(params[3], 8);

    // Transform and project point to image plane
    mat_t<2, 3> cam0_Jh, cam1_Jh;
    matx_t cam0_J_params, cam1_J_params;
    vec2_t z_C0_hat, z_C1_hat;
    bool valid = true;
    // -- Transform point from fiducial frame to cam0 and cam1
    const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi_);
    const vec3_t r_C1Fi = tf_point(T_C1F, r_FFi_);
    // -- Project point from cam0 frame to cam0 image plane
    const CAMERA_TYPE cam0{cam_res_, cam0_params};
    if (cam0.project(r_C0Fi, z_C0_hat, cam0_Jh) != 0) {
      valid = false;
    }
    const vec2_t p_C0{r_C0Fi(0) / r_C0Fi(2), r_C0Fi(1) / r_C0Fi(2)};
    cam0_J_params = cam0.J_params(p_C0);
    // -- Project point from cam1 frame to cam1 image plane
    const CAMERA_TYPE cam1{cam_res_, cam1_params};
    if (cam1.project(r_C1Fi, z_C1_hat, cam1_Jh) != 0) {
      valid = false;
    }
    const vec2_t p_C1{r_C1Fi(0) / r_C1Fi(2), r_C1Fi(1) / r_C1Fi(2)};
    cam1_J_params = cam1.J_params(p_C1);

    // Residual
    Eigen::Map<vec4_t> r(residuals);
    r.segment<2>(0) = sqrt_info_ * (z_C0_ - z_C0_hat);
    r.segment<2>(2) = sqrt_info_ * (z_C1_ - z_C1_hat);

    // Jacobians
    matx_t cam0_weighted_Jh = -1 * sqrt_info_ * cam0_Jh;
    matx_t cam1_weighted_Jh = -1 * sqrt_info_ * cam1_Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        J_min[0].block(0, 0, 2, 3) = cam0_weighted_Jh * I(3);
        J_min[0].block(0, 3, 2, 3) = cam0_weighted_Jh * -skew(C_C0F * r_FFi_);
        J_min[0].block(2, 0, 2, 3) = cam1_weighted_Jh * I(3);
        J_min[0].block(2, 3, 2, 3) = cam1_weighted_Jh * -skew(C_C1F * r_FFi_);
        if (valid == false) {
          J_min[0].setZero();
        }

        // Convert from minimal 2x6 jacobian to full 2x7 jacobian
        mat_t<6, 7, row_major_t> J_lift_cam0;
        mat_t<6, 7, row_major_t> J_lift_cam1;

        lift_pose_jacobian(q_C0F, J_lift_cam0);
        lift_pose_jacobian(q_C1F, J_lift_cam1);

        Eigen::Map<mat_t<4, 7, row_major_t>> J0(jacobians[0]);
        // J0.block(0, 0, 2, 7) = J_min[0].block(0, 0, 2, 6) * J_lift_cam0;
        // J0.block(2, 0, 2, 7) = J_min[0].block(2, 0, 2, 6) * J_lift_cam1;
        J0.setZero();
        J0.block(0, 0, 2, 6) = J_min[0].block(0, 0, 2, 6);
        J0.block(2, 0, 2, 6) = J_min[0].block(2, 0, 2, 6);
      }

      // Jacobians w.r.t T_C1C0
      if (jacobians[1]) {
        J_min[1].block(0, 0, 2, 6).setZero();
        J_min[1].block(2, 0, 2, 3) = cam1_weighted_Jh * I(3);
        J_min[1].block(2, 3, 2, 3) = cam1_weighted_Jh * -skew(C_C1C0 * r_C0Fi);
        if (valid == false) {
          J_min[1].setZero();
        }

        // Convert from minimal 2x6 jacobian to full 2x7 jacobian
        mat_t<6, 7, row_major_t> J_lift_cam1;
        // lift_pose_jacobian(q_C1F, J_lift_cam1);
        lift_pose_jacobian(q_C1C0, J_lift_cam1);

        Eigen::Map<mat_t<4, 7, row_major_t>> J1(jacobians[1]);
        J1.setZero();
        J1.block(0, 0, 2, 7).setZero();
        // J1.block(2, 0, 2, 7) = J_min[1].block(2, 0, 2, 6) * J_lift_cam1;
        J1.block(2, 0, 2, 6) = J_min[1].block(2, 0, 2, 6);
      }

      // Jacobians w.r.t cam0 params
      if (jacobians[2]) {
        J_min[2].block(0, 0, 2, 8) = -1 * sqrt_info_ * cam0_J_params;
        J_min[2].block(2, 0, 2, 8) = zeros(2, 8);
        if (valid == false) {
          J_min[2].setZero();
        }

        Eigen::Map<mat_t<4, 8, row_major_t>> J2(jacobians[2]);
        J2 = J_min[2];
      }

      // Jacobians w.r.t cam1 params
      if (jacobians[3]) {
        J_min[3].block(0, 0, 2, 8) = zeros(2, 8);
        J_min[3].block(2, 0, 2, 8) = -1 * sqrt_info_ * cam1_J_params;
        if (valid == false) {
          J_min[3].setZero();
        }

        Eigen::Map<mat_t<4, 8, row_major_t>> J3(jacobians[3]);
        J3 = J_min[3];
      }
    }

    return true;
  }
};

// /** Stereo camera calibration residual */
// template <typename CAMERA_TYPE>
// struct calib_stereo_residual2_t
//     : public ceres::SizedCostFunction<4, 7, 7, 7, 8, 8> {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//   int cam_res_[2] = {0, 0};
//   vec3_t r_FFi_{0.0, 0.0, 0.0};
//   vec2_t z_C0_{0.0, 0.0};
//   vec2_t z_C1_{0.0, 0.0};
//
//   const mat2_t covar_;
//   const mat2_t info_;
//   const mat2_t sqrt_info_;
//   mutable matxs_t J_min;
//
//   calib_stereo_residual2_t(const int cam_res[2],
//                           const vec3_t &r_FFi,
//                           const vec2_t &z_C0,
//                           const vec2_t &z_C1,
//                           const mat2_t &covar)
//       : cam_res_{cam_res[0], cam_res[1]},
//         r_FFi_{r_FFi}, z_C0_{z_C0}, z_C1_{z_C1},
//         covar_{covar},
//         info_{covar.inverse()},
//         sqrt_info_{info_.llt().matrixL().transpose()} {
//     J_min.push_back(zeros(4, 6));  // T_CF
//     J_min.push_back(zeros(4, 6));  // T_C0C1
//     J_min.push_back(zeros(4, 8));  // cam0 params
//     J_min.push_back(zeros(4, 8));  // cam1 params
//   }
//
//   ~calib_stereo_residual2_t() {}
//
//   /**
//    * Calculate residual
//    */
//   bool Evaluate(double const * const *params,
//                 double *residuals,
//                 double **jacobians) const {
//     // Map parameters out
//     // -- Map camera0-fiducial pose
//     const mat4_t T_C0F = tf(params[0]);
//     const mat3_t C_C0F = tf_rot(T_C0F);
//     const quat_t q_C0F{C_C0F};
//     // -- Map camera1-fiducial pose
//     const mat4_t T_C1F = tf(params[1]);
//     const mat3_t C_C1F = tf_rot(T_C1F);
//     const quat_t q_C1F{C_C0F};
//     // -- Map camera0-camera1 pose
//     const mat4_t T_C1C0 = tf(params[2]);
//     const mat4_t T_C0C1 = T_C1C0.inverse();
//     const mat3_t C_C1C0 = tf_rot(T_C1C0);
//     // -- Map camera params
//     Eigen::Map<const vecx_t> cam0_params(params[3], 8);
//     Eigen::Map<const vecx_t> cam1_params(params[4], 8);
//
//     // Transform and project point to image plane
//     mat_t<2, 3> cam0_Jh, cam1_Jh;
//     matx_t cam0_J_params, cam1_J_params;
//     vec2_t z_C0_hat, z_C1_hat;
//     bool valid = true;
//     // -- Transform point from fiducial frame to cam0 and cam1
//     const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi_);
//     const vec3_t r_C1Fi = tf_point(T_C1F, r_FFi_);
//     // -- Project point observed from cam1 frame to cam0 image plane
//     const CAMERA_TYPE cam0{cam_res_, cam0_params};
//     if (cam0.project(tf_point(T_C0C1, r_C1Fi), z_C0_hat, cam0_Jh) != 0) {
//       valid = false;
//     }
//     const vec2_t p_C0{r_C0Fi(0) / r_C0Fi(2), r_C0Fi(1) / r_C0Fi(2)};
//     cam0_J_params = cam0.J_params(p_C0);
//     // -- Project point from cam1 frame to cam1 image plane
//     const CAMERA_TYPE cam1{cam_res_, cam1_params};
//     if (cam1.project(tf_point(T_C1C0, r_C0Fi), z_C1_hat, cam1_Jh) != 0) {
//       valid = false;
//     }
//     const vec2_t p_C1{r_C1Fi(0) / r_C1Fi(2), r_C1Fi(1) / r_C1Fi(2)};
//     cam1_J_params = cam1.J_params(p_C1);
//
//     // Residual
//     Eigen::Map<vec4_t> r(residuals);
//     r.segment<2>(0) = sqrt_info_ * (z_C0_ - z_C0_hat);
//     r.segment<2>(2) = sqrt_info_ * (z_C1_ - z_C1_hat);
//
//     // Jacobians
//     matx_t cam0_weighted_Jh = -1 * sqrt_info_ * cam0_Jh;
//     matx_t cam1_weighted_Jh = -1 * sqrt_info_ * cam1_Jh;
//
//     if (jacobians) {
//       // Jacobians w.r.t T_C0F
//       if (jacobians[0]) {
//         J_min[0].block(0, 0, 2, 3) = cam0_weighted_Jh * I(3);
//         J_min[0].block(0, 3, 2, 3) = cam0_weighted_Jh * -skew(C_C0F * r_FFi_);
//         J_min[0].block(2, 0, 2, 3) = cam1_weighted_Jh * I(3);
//         J_min[0].block(2, 3, 2, 3) = cam1_weighted_Jh * -skew(C_C1F * r_FFi_);
//         if (valid == false) {
//           J_min[0].setZero();
//         }
//
//         // Convert from minimal 2x6 jacobian to full 2x7 jacobian
//         mat_t<6, 7, row_major_t> J_lift_cam0;
//         mat_t<6, 7, row_major_t> J_lift_cam1;
//
//         lift_pose_jacobian(q_C0F, J_lift_cam0);
//         lift_pose_jacobian(q_C1F, J_lift_cam1);
//
//         Eigen::Map<mat_t<4, 7, row_major_t>> J0(jacobians[0]);
//         J0.block(0, 0, 2, 7) = J_min[0].block(0, 0, 2, 6) * J_lift_cam0;
//         J0.block(2, 0, 2, 7) = J_min[0].block(2, 0, 2, 6) * J_lift_cam1;
//       }
//
//       // Jacobians w.r.t T_C1F
//       if (jacobians[0]) {
//         J_min[1].block(0, 0, 2, 3) = cam0_weighted_Jh * I(3);
//         J_min[1].block(0, 3, 2, 3) = cam0_weighted_Jh * -skew(C_C0F * r_FFi_);
//         J_min[1].block(2, 0, 2, 3) = cam1_weighted_Jh * I(3);
//         J_min[1].block(2, 3, 2, 3) = cam1_weighted_Jh * -skew(C_C1F * r_FFi_);
//         if (valid == false) {
//           J_min[0].setZero();
//         }
//
//         // Convert from minimal 2x6 jacobian to full 2x7 jacobian
//         mat_t<6, 7, row_major_t> J_lift_cam0;
//         mat_t<6, 7, row_major_t> J_lift_cam1;
//
//         lift_pose_jacobian(q_C0F, J_lift_cam0);
//         lift_pose_jacobian(q_C1F, J_lift_cam1);
//
//         Eigen::Map<mat_t<4, 7, row_major_t>> J1(jacobians[1]);
//         J1.block(0, 0, 2, 7) = J_min[0].block(0, 0, 2, 6) * J_lift_cam0;
//         J1.block(2, 0, 2, 7) = J_min[0].block(2, 0, 2, 6) * J_lift_cam1;
//       }
//
//       // Jacobians w.r.t T_C1C0
//       if (jacobians[2]) {
//         J_min[2].block(0, 0, 2, 6).setZero();
//         J_min[2].block(2, 0, 2, 3) = cam1_weighted_Jh * I(3);
//         J_min[2].block(2, 3, 2, 3) = cam1_weighted_Jh * -skew(C_C1C0 * r_C0Fi);
//         if (valid == false) {
//           J_min[2].setZero();
//         }
//
//         // Convert from minimal 2x6 jacobian to full 2x7 jacobian
//         mat_t<6, 7, row_major_t> J_lift_cam1;
//         lift_pose_jacobian(q_C1F, J_lift_cam1);
//
//         Eigen::Map<mat_t<4, 7, row_major_t>> J2(jacobians[2]);
//         J2.block(0, 0, 2, 7).setZero();
//         J2.block(2, 0, 2, 7) = J_min[2].block(2, 0, 2, 6) * J_lift_cam1;
//       }
//
//       // Jacobians w.r.t cam0 params
//       if (jacobians[3]) {
//         J_min[3].block(0, 0, 2, 8) = -1 * sqrt_info_ * cam0_J_params;
//         J_min[3].block(2, 0, 2, 8) = zeros(2, 8);
//         if (valid == false) {
//           J_min[3].setZero();
//         }
//
//         Eigen::Map<mat_t<4, 8, row_major_t>> J3(jacobians[3]);
//         J3 = J_min[3];
//       }
//
//       // Jacobians w.r.t cam1 params
//       if (jacobians[4]) {
//         J_min[4].block(0, 0, 2, 8) = zeros(2, 8);
//         J_min[4].block(2, 0, 2, 8) = -1 * sqrt_info_ * cam1_J_params;
//         if (valid == false) {
//           J_min[4].setZero();
//         }
//
//         Eigen::Map<mat_t<4, 8, row_major_t>> J4(jacobians[4]);
//         J4 = J_min[4];
//       }
//     }
//
//     return true;
//   }
// };

/* Process AprilGrid - Adding it to the calibration problem */
template <typename CAMERA_TYPE>
static int process_grid(const aprilgrid_t &cam0_aprilgrid,
                        const aprilgrid_t &cam1_aprilgrid,
                        const mat2_t &covar,
                        pose_t &T_C0F,
                        pose_t &T_C1C0,
                        camera_params_t &cam0_params,
                        camera_params_t &cam1_params,
                        ceres::Problem &problem,
                        ceres::LossFunction *loss=nullptr) {
  int *cam_res = cam0_params.resolution;

  for (const auto &tag_id : cam0_aprilgrid.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_keypoints(cam0_aprilgrid, tag_id, cam0_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_keypoints(cam1_aprilgrid, tag_id, cam1_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(cam0_aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto z_C0 = cam0_keypoints[i];
      const auto z_C1 = cam1_keypoints[i];
      const auto r_FFi = object_points[i];
      const auto cost_fn = new calib_stereo_residual_t<CAMERA_TYPE>{
        cam_res, r_FFi, z_C0, z_C1, covar};
        problem.AddResidualBlock(cost_fn, // Cost function
                                 loss,    // Loss function
                                 T_C0F.param.data(),
                                 T_C1C0.param.data(),
                                 cam0_params.param.data(),
                                 cam1_params.param.data());
    }
  }

  return 0;
}

template <typename CAMERA_TYPE>
static void show_results(const aprilgrids_t &cam0_grids,
                         const aprilgrids_t &cam1_grids,
                         const camera_params_t &cam0,
                         const camera_params_t &cam1,
                         const mat4s_t &T_C0F,
                         const mat4s_t &T_C1F,
                         const mat4_t &T_C1C0) {
  // Show results
  std::vector<double> cam0_errs, cam1_errs;
  reproj_errors<CAMERA_TYPE>(cam0_grids, cam0, T_C0F, cam0_errs);
  reproj_errors<CAMERA_TYPE>(cam1_grids, cam1, T_C1F, cam1_errs);

  printf("Optimization results:\n");
  printf("---------------------\n");
  printf("cam0 reproj_error [px] [rmse: %f, ", rmse(cam0_errs));
  printf("mean: %f, ", mean(cam0_errs));
  printf("median: %f]\n", median(cam0_errs));
  printf("cam1 reproj_error [px] [rmse: %f, ", rmse(cam1_errs));
  printf("mean: %f, ", mean(cam1_errs));
  printf("median: %f]\n", median(cam1_errs));
  printf("\n");
  print_vector("cam0.proj_params", cam0.proj_params());
  print_vector("cam0.dist_params", cam0.dist_params());
  print_vector("cam1.proj_params", cam1.proj_params());
  print_vector("cam1.dist_params", cam1.dist_params());
  printf("\n");
  print_matrix("T_C1C0", T_C1C0);
  printf("\n");
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
template <typename CAMERA_TYPE>
int calib_stereo_solve(const aprilgrids_t &cam0_grids,
                       const aprilgrids_t &cam1_grids,
                       const mat2_t &covar,
                       camera_params_t &cam0,
                       camera_params_t &cam1,
                       mat4_t &T_C1C0,
                       mat4s_t &T_C0F,
                       mat4s_t &T_C1F) {
  // Pre-check
  if (cam0_grids.size() != cam1_grids.size()) {
    LOG_ERROR("cam0_grids.size() != cam1_grids.size()!");
    return -1;
  }
  if (cam0_grids.size() != T_C0F.size()) {
    LOG_ERROR("cam0_grids.size() != T_C0F.size()!");
    return -1;
  }
  if (cam1_grids.size() != T_C1F.size()) {
    LOG_ERROR("cam1_grids.size() != T_C1F.size()!");
    return -1;
  }

  // -- Stereo camera extrinsics
  T_C1C0 = I(4);
  pose_t extrinsic_param{0, 0, T_C1C0};
  // -- Relative pose between cam0 and fiducial target
  std::vector<pose_t> rel_poses;
  for (size_t i = 0; i < cam0_grids.size(); i++) {
    rel_poses.emplace_back(i, i, T_C0F[i]);
  }

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  // ceres::LossFunction *loss = new ceres::CauchyLoss(1);
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_grids.size(); i++) {
    int retval = process_grid<CAMERA_TYPE>(cam0_grids[i],
                                           cam1_grids[i],
                                           covar,
                                           rel_poses[i],
                                           extrinsic_param,
                                           cam0,
                                           cam1,
                                           problem,
                                           loss);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem.SetParameterization(rel_poses[i].param.data(),
                                &pose_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  // std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Finish up
  T_C1C0 = extrinsic_param.tf();
  T_C0F.clear();
  T_C1F.clear();
  for (auto pose : rel_poses) {
    T_C0F.emplace_back(pose.tf());
    T_C1F.emplace_back(T_C1C0 * pose.tf());
  }
  if (loss) {
    delete loss;
  }

  // Show results
  show_results<CAMERA_TYPE>(cam0_grids, cam1_grids,
                            cam0, cam1,
                            T_C0F, T_C1F,
                            T_C1C0);

  return 0;
}

/* Initialize Stereo-Camera Intrinsics */
template <typename CAMERA_TYPE>
static void init_stereo_intrinsics(const aprilgrids_t &grids0,
                                   const aprilgrids_t &grids1,
                                   const mat2_t &covar,
                                   camera_params_t &cam0,
                                   camera_params_t &cam1,
                                   mat4s_t &T_C0F,
                                   mat4s_t &T_C1F) {
  // -- cam0
  LOG_INFO("Calibrating cam0 intrinsics ...");
  if (calib_mono_solve<CAMERA_TYPE>(grids0, covar, cam0, T_C0F) != 0) {
    FATAL("Failed to calibrate cam0 intrinsics!");
  }
  // -- cam1
  LOG_INFO("Calibrating cam1 intrinsics ...");
  if (calib_mono_solve<CAMERA_TYPE>(grids1, covar, cam1, T_C1F) != 0) {
    FATAL("Failed to calibrate cam1 intrinsics!");
  }
}

/* Initialize Stereo-Camera Extrinsics */
static void init_stereo_extrinsics(const mat4s_t &T_C0F,
                                   const mat4s_t &T_C1F,
                                   mat4_t &T_C1C0) {
  std::vector<double> rx, ry, rz;
  std::vector<double> roll, pitch, yaw;

  for (size_t k = 0; k < T_C0F.size(); k++) {
    const mat4_t T_C1C0_k = T_C1F[k] * T_C0F[k].inverse();
    const vec3_t r_C1C0_k = tf_trans(T_C1C0_k);
    const vec3_t rpy_C1C0_k = quat2euler(tf_quat(T_C1C0_k));

    rx.push_back(r_C1C0_k(0));
    ry.push_back(r_C1C0_k(1));
    rz.push_back(r_C1C0_k(2));

    roll.push_back(rpy_C1C0_k(0));
    pitch.push_back(rpy_C1C0_k(1));
    yaw.push_back(rpy_C1C0_k(2));
  }

  const vec3_t r_C1C0{median(rx), median(ry), median(rz)};
  const vec3_t rpy_C1C0{median(roll), median(pitch), median(yaw)};
  const mat3_t C_C1C0 = euler321(rpy_C1C0);
  T_C1C0 = tf(C_C1C0, r_C1C0);
}

/* Estimate relative poses */
static void estimate_relative_poses(const aprilgrid_t &grids0,
                                    const aprilgrid_t &grids1,
                                    camera_params_t *cam0,
                                    camera_params_t *cam1,
                                    id_t *param_counter,
                                    pose_t *T_C0F,
                                    pose_t *T_C1F) {
  if (grids0.timestamp != grids1.timestamp) {
    LOG_ERROR("grids0.timestamp != grids1.timestamp");
    FATAL("Bad data! Stopping stereo-calibration!");
  }
  const auto ts = grids0.timestamp;

  // Estimate T_CF at ts
  const vecx_t cam0_proj = cam0->proj_params();
  const vecx_t cam0_dist = cam0->dist_params();
  const vecx_t cam1_proj = cam1->proj_params();
  const vecx_t cam1_dist = cam1->dist_params();
  mat4_t T_C0F_;
  mat4_t T_C1F_;
  aprilgrid_calc_relative_pose(grids0, cam0_proj, cam0_dist, T_C0F_);
  aprilgrid_calc_relative_pose(grids1, cam1_proj, cam1_dist, T_C1F_);

  // Create new relative pose T_CF at ts
  T_C0F = new pose_t{(*param_counter)++, ts, T_C0F_};
  T_C1F = new pose_t{(*param_counter)++, ts, T_C1F_};
}

// /* Estimate Stereo Camera Covariance */
// int calib_stereo_covar(const camera_params_t &cam0,
//                        const camera_params_t &cam1,
//                        ceres::Problem &problem,
//                        matx_t &covar);

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
template <typename CAMERA_TYPE>
int calib_stereo_inc_solve(calib_stereo_data_t &data) {
  // Map out the calibration data
  mat2_t &covar = data.covar;
  aprilgrids_t &grids0 = data.cam0_grids;
  aprilgrids_t &grids1 = data.cam1_grids;
  camera_params_t &cam0 = data.cam0;
  camera_params_t &cam1 = data.cam1;
  mat4_t &T_C1C0 = data.T_C1C0;
  mat4s_t &T_C0F = data.T_C0F;
  mat4s_t &T_C1F = data.T_C1F;

  // Pre-check
  if (grids0.size() != grids1.size()) {
    LOG_ERROR("grids0.size() != grids1.size()!");
    return -1;
  }
  if (grids0.size() != T_C0F.size()) {
    LOG_ERROR("grids0.size() != T_C0F.size()!");
    return -1;
  }
  if (grids1.size() != T_C1F.size()) {
    LOG_ERROR("grids1.size() != T_C1F.size()!");
    return -1;
  }

  // Initialize stereo intrinsics and extrinsics
  init_stereo_intrinsics<CAMERA_TYPE>(grids0, grids1, covar, cam0, cam1, T_C0F, T_C1F);
  init_stereo_extrinsics(T_C0F, T_C1F, T_C1C0);

  // Create random index vector (same length as grids)
  // -- Create indicies vector
  std::vector<int> indicies;
  for (size_t i = 0; i < grids0.size(); i++) {
    indicies.push_back(i);
  }
  // -- Randomize the indicies vector
  std::random_shuffle(std::begin(indicies), std::end(indicies));

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  id_t param_counter = 0;
  std::deque<pose_t *> cam0_rel_poses;
  std::deque<pose_t *> cam1_rel_poses;
  pose_t *extrinsic = new pose_t{param_counter++, 0, T_C1C0};
  calib_mono_views_t<CAMERA_TYPE> cam0_views;
  calib_mono_views_t<CAMERA_TYPE> cam1_views;
  double info = -1;
  size_t counter = 0;
  size_t nb_outliers = 0;

  for (const auto &i : indicies) {
    const auto grid0 = grids0[i];
    const auto grid1 = grids1[i];
    const auto ts = grids0[i].timestamp;
    if (grid0.timestamp != grid1.timestamp) {
      LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
      LOG_ERROR("Bad data! Stopping stereo-calibration!");
      return -1;
    }

    // Estimate T_C0F and T_C1F at ts
    // -- T_C0F
    mat4_t T_C0F_;
    const vecx_t cam0_proj = cam0.proj_params();
    const vecx_t cam0_dist = cam0.dist_params();
    aprilgrid_calc_relative_pose(grid0, cam0_proj, cam0_dist, T_C0F_);
    // -- T_C1F
    mat4_t T_C1F_;
    const vecx_t cam1_proj = cam1.proj_params();
    const vecx_t cam1_dist = cam1.dist_params();
    aprilgrid_calc_relative_pose(grid1, cam1_proj, cam1_dist, T_C1F_);
    // -- Keep track of relative poses
    pose_t *cam0_rel_pose = new pose_t{param_counter++, ts, T_C0F_};
    pose_t *cam1_rel_pose = new pose_t{param_counter++, ts, T_C1F_};

    // Create view
    // -- cam0 view
    calib_mono_view_t<CAMERA_TYPE> cam0_view;
    cam0_view.grid = grid0;
    cam0_view.cam_params = &cam0;
    cam0_view.T_CF = cam0_rel_pose;
    // -- cam1 view
    calib_mono_view_t<CAMERA_TYPE> cam1_view;
    cam1_view.grid = grid1;
    cam1_view.cam_params = &cam1;
    cam1_view.T_CF = cam1_rel_pose;

    // Add AprilGrid to problem
    process_grid<CAMERA_TYPE>(grids0[i],
                              grids1[i],
                              covar,
                              *cam0_rel_pose,
                              *extrinsic,
                              cam0,
                              cam1,
                              problem,
                              loss);
    // cam0_views.push_back(cam0_view);
    // cam1_views.push_back(cam1_view);
    problem.SetParameterization(cam0_rel_pose->param.data(), &pose_parameterization);
    problem.SetParameterization(extrinsic->param.data(), &pose_parameterization);
    // // problem.SetParameterization(cam1_rel_pose->param.data(), &pose_parameterization);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 3;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;

    // Update cam1 relative pose
    cam1_rel_pose->set_tf(extrinsic->tf() * cam0_rel_pose->tf());
    cam0_rel_poses.emplace_back(cam0_rel_pose);
    cam1_rel_poses.emplace_back(cam1_rel_pose);
    cam0_views.push_back(cam0_view);
    cam1_views.push_back(cam1_view);

    // Filter views
    nb_outliers += filter_views<CAMERA_TYPE>(problem, cam0_views);
    nb_outliers += filter_views<CAMERA_TYPE>(problem, cam1_views);

    // Show progress
    std::vector<double> cam0_errs;
    std::vector<double> cam1_errs;
    // reproj_errors<CAMERA_TYPE>(cam0_views, cam0, cam0_rel_poses, cam0_errs);
    // reproj_errors<CAMERA_TYPE>(cam1_views, cam1, cam1_rel_poses, cam1_errs);
    reproj_errors<CAMERA_TYPE>(cam0_views, cam0, cam0_errs);
    reproj_errors<CAMERA_TYPE>(cam1_views, cam1, cam1_errs);
    printf("Processed [%ld / %ld]\n", counter++, indicies.size());
    printf("Reprojection Error: ");
    printf("\n");
    printf("  cam0: [mean: %.2f,", mean(cam0_errs));
    printf(" rmse: %.2f]", rmse(cam0_errs));
    printf("\n");
    printf("  cam1: [mean: %.2f,", mean(cam1_errs));
    printf(" rmse: %.2f]", rmse(cam1_errs));
    printf("\n");
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // // Finish up
  // T_C1C0 = extrinsic_param.tf();
  // T_C0F.clear();
  // T_C1F.clear();
  // for (auto pose : rel_poses) {
  //   T_C0F.emplace_back(pose.tf());
  //   T_C1F.emplace_back(T_C1C0 * pose.tf());
  // }
  // if (loss) {
  //   delete loss;
  // }

  return 0;
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       sigma_vision: 0.5
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25[-]
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 *     cam1:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 */
int calib_stereo_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_STEREO_HPP
