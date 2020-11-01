#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "calib_mono.hpp"

namespace yac {

/**
 * Stereo camera calibration residual
 */
template <typename CAMERA_TYPE>
struct calib_stereo_residual_t
    : public ceres::SizedCostFunction<4, 7, 7, 8, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  vec2_t z_C0_{0.0, 0.0};
  vec2_t z_C1_{0.0, 0.0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};

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
        J0.block(0, 0, 2, 7) = J_min[0].block(0, 0, 2, 6) * J_lift_cam0;
        J0.block(2, 0, 2, 7) = J_min[0].block(2, 0, 2, 6) * J_lift_cam1;
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
        lift_pose_jacobian(q_C1F, J_lift_cam1);

        Eigen::Map<mat_t<4, 7, row_major_t>> J1(jacobians[1]);
        J1.block(0, 0, 2, 7).setZero();
        J1.block(2, 0, 2, 7) = J_min[1].block(2, 0, 2, 6) * J_lift_cam1;
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

template <typename CAMERA_TYPE>
static int process_aprilgrid(const aprilgrid_t &cam0_aprilgrid,
                             const aprilgrid_t &cam1_aprilgrid,
                             const mat2_t &covar,
                             pose_t *T_C0F,
                             pose_t *T_C1C0,
                             camera_params_t *cam0_params,
                             camera_params_t *cam1_params,
                             ceres::Problem *problem) {
  int *cam_res = cam0_params->resolution;

  for (const auto &tag_id : cam0_aprilgrid.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_get(cam0_aprilgrid, tag_id, cam0_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_get(cam1_aprilgrid, tag_id, cam1_keypoints) != 0) {
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
        problem->AddResidualBlock(cost_fn, // Cost function
                                  NULL,    // Loss function
                                  T_C0F->param.data(),
                                  T_C1C0->param.data(),
                                  cam0_params->param.data(),
                                  cam1_params->param.data());
    }
  }

  return 0;
}

/** Error Metrics **/
struct calib_stereo_results_t {
  std::vector<double> cam0_errors;
  std::vector<double> cam1_errors;

  double cam0_rmse_error = 0;
  double cam0_mean_error = 0;

  double cam1_rmse_error = 0;
  double cam1_mean_error = 0;
};

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
template <typename CAMERA_TYPE>
int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_grids,
                       const std::vector<aprilgrid_t> &cam1_grids,
                       const mat2_t &covar,
                       camera_params_t *cam0,
                       camera_params_t *cam1,
                       mat4_t *T_C1C0,
                       mat4s_t *T_C0F,
                       mat4s_t *T_C1F) {
  assert(cam0_grids.size() == cam1_grids.size());

  // Setup optimization variables
  // -- Stereo camera extrinsics
  pose_t extrinsic_param{0, 0, *T_C1C0};
  // -- Relative pose between cam0 and fiducial target
  std::vector<pose_t> rel_poses;
  for (size_t i = 0; i < cam0_grids.size(); i++) {
    rel_poses.emplace_back(i, i, cam0_grids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(prob_options));
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_grids.size(); i++) {
    int retval = process_aprilgrid<CAMERA_TYPE>(cam0_grids[i],
                                                cam1_grids[i],
                                                covar,
                                                &rel_poses[i],
                                                &extrinsic_param,
                                                cam0,
                                                cam1,
                                                problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(rel_poses[i].param.data(),
                                 &pose_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Finish up
  *T_C1C0 = extrinsic_param.tf();
  for (auto pose : rel_poses) {
    (*T_C0F).emplace_back(pose.tf());
    (*T_C1F).emplace_back(*T_C1C0 * pose.tf());
  }

  // Show results
  std::vector<double> cam0_errs, cam1_errs;
  double cam0_rmse, cam1_rmse = 0.0;
  double cam0_mean, cam1_mean = 0.0;
  calib_mono_stats<CAMERA_TYPE>(cam0_grids, *cam0, *T_C0F, &cam0_errs, &cam0_rmse, &cam0_mean);
  calib_mono_stats<CAMERA_TYPE>(cam1_grids, *cam1, *T_C1F, &cam1_errs, &cam1_rmse, &cam1_mean);

  printf("Optimization results:\n");
  printf("---------------------\n");
  print_vector("cam0.proj_params", (*cam0).proj_params());
  print_vector("cam0.dist_params", (*cam0).dist_params());
  print_vector("cam1.proj_params", (*cam1).proj_params());
  print_vector("cam1.dist_params", (*cam1).dist_params());
  printf("\n");
  print_matrix("T_C1C0", *T_C1C0);
  printf("cam0 rms reproj error [px]: %f\n", cam0_rmse);
  printf("cam1 rms reproj error [px]: %f\n", cam1_rmse);
  printf("cam0 mean reproj error [px]: %f\n", cam0_mean);
  printf("cam1 mean reproj error [px]: %f\n", cam1_mean);
  printf("\n");

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
