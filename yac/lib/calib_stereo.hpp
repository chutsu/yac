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

  std::vector<double> cam0_errs;
  std::vector<double> cam1_errs;
};

/** Stereo camera calibration residual */
template <typename CAMERA_TYPE>
struct calib_stereo_residual_t
    : public ceres::SizedCostFunction<4, 7, 7, 8, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  int tag_id_ = -1;
  int corner_id = -1;
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_C0_{0.0, 0.0};
  vec2_t z_C1_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_stereo_residual_t(const int cam_res[2],
                          const int tag_id,
                          const int corner_id,
                          const vec3_t &r_FFi,
                          const vec2_t &z_C0,
                          const vec2_t &z_C1,
                          const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_id{corner_id},
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
        // mat_t<6, 7, row_major_t> J_lift_cam0;
        // mat_t<6, 7, row_major_t> J_lift_cam1;
        //
        // lift_pose_jacobian(q_C0F, J_lift_cam0);
        // lift_pose_jacobian(q_C1F, J_lift_cam1);
        //
        // Eigen::Map<mat_t<4, 7, row_major_t>> J0(jacobians[0]);
        // J0.block(0, 0, 2, 7) = J_min[0].block(0, 0, 2, 6) * J_lift_cam0;
        // J0.block(2, 0, 2, 7) = J_min[0].block(2, 0, 2, 6) * J_lift_cam1;

        Eigen::Map<mat_t<4, 7, row_major_t>> J0(jacobians[0]);
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
        // mat_t<6, 7, row_major_t> J_lift_cam1;
        // lift_pose_jacobian(q_C1F, J_lift_cam1);
        // lift_pose_jacobian(q_C1C0, J_lift_cam1);
        // Eigen::Map<mat_t<4, 7, row_major_t>> J1(jacobians[1]);
        // J1.setZero();
        // J1.block(0, 0, 2, 7).setZero();
        // J1.block(2, 0, 2, 7) = J_min[1].block(2, 0, 2, 6) * J_lift_cam1;

        Eigen::Map<mat_t<4, 7, row_major_t>> J1(jacobians[1]);
        J1.setZero();
        J1.block(0, 0, 2, 7).setZero();
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

/* Process AprilGrid - Adding it to the calibration problem */
template <typename CAMERA_TYPE>
static int process_grid(const aprilgrid_t &cam0_aprilgrid,
                        const aprilgrid_t &cam1_aprilgrid,
                        const mat2_t &covar,
                        pose_t &T_C0F,
                        pose_t &T_C1C0,
                        camera_params_t &cam0_params,
                        camera_params_t &cam1_params,
                        std::vector<ceres::ResidualBlockId> &res_ids,
                        std::vector<calib_stereo_residual_t<CAMERA_TYPE> *> &cost_fns,
                        ceres::Problem &problem,
                        PoseLocalParameterization &pose_parameterization,
                        ceres::LossFunction *loss=nullptr) {
  int *cam_res = cam0_params.resolution;

  for (const int tag_id : cam0_aprilgrid.ids) {
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
    for (int corner_id = 0; corner_id < 4; corner_id++) {
      const vec2_t z_C0 = cam0_keypoints[corner_id];
      const vec2_t z_C1 = cam1_keypoints[corner_id];
      const vec3_t r_FFi = object_points[corner_id];
      const auto cost_fn = new calib_stereo_residual_t<CAMERA_TYPE>{
        cam_res, tag_id, corner_id, r_FFi, z_C0, z_C1, covar};
      const auto res_id = problem.AddResidualBlock(cost_fn, // Cost function
                                                   loss,    // Loss function
                                                   T_C0F.param.data(),
                                                   T_C1C0.param.data(),
                                                   cam0_params.param.data(),
                                                   cam1_params.param.data());
      res_ids.push_back(res_id);
      cost_fns.push_back(cost_fn);
    }
  }
  problem.SetParameterization(T_C0F.param.data(), &pose_parameterization);
  problem.SetParameterization(T_C1C0.param.data(), &pose_parameterization);

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
  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_stereo_residual_t<CAMERA_TYPE> *> cost_fns;
  for (size_t i = 0; i < cam0_grids.size(); i++) {
    int retval = process_grid<CAMERA_TYPE>(cam0_grids[i],
                                           cam1_grids[i],
                                           covar,
                                           rel_poses[i],
                                           extrinsic_param,
                                           cam0,
                                           cam1,
                                           res_ids,
                                           cost_fns,
                                           problem,
                                           pose_parameterization,
                                           loss);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  options.check_gradients = true;

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

/* Stereo-Camera Calibration View */
template <typename CAMERA_TYPE>
struct calib_stereo_view_t {
  aprilgrid_t cam0_grid;
  aprilgrid_t cam1_grid;
  pose_t *T_C0F = nullptr;
  pose_t *T_C1C0 = nullptr;
  camera_params_t *cam0 = nullptr;
  camera_params_t *cam1 = nullptr;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_stereo_residual_t<CAMERA_TYPE> *> cost_fns;
};

/* Stereo-Camera Calibration Views */
template <typename CAMERA_TYPE>
using calib_stereo_views_t = std::deque<calib_stereo_view_t<CAMERA_TYPE>>;

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_stereo_view_t<CAMERA_TYPE> &view,
                   std::vector<double> &cam0_errs,
                   std::vector<double> &cam1_errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const auto grid0 = view.cam0_grid;
  const auto grid1 = view.cam1_grid;
  const auto cam0 = view.cam0;
  const auto cam1 = view.cam1;
  const auto T_C0F = view.T_C0F->tf();
  const auto T_C1C0 = view.T_C1C0->tf();

  // Iterate over all tags in AprilGrid
  for (const auto &tag_id : grid0.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_keypoints(grid0, tag_id, cam0_keypoints) != 0) {
      FATAL("Failed to get AprilGrid keypoints!");
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_keypoints(grid1, tag_id, cam1_keypoints) != 0) {
      FATAL("Failed to get AprilGrid keypoints!");
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(grid0, tag_id, object_points) != 0) {
      FATAL("Failed to calculate AprilGrid object points!");
    }

    // Form residual and call the functor for four corners of the tag
    for (int corner_id = 0; corner_id < 4; corner_id++) {
      const vec2_t z_C0 = cam0_keypoints[corner_id];
      const vec2_t z_C1 = cam1_keypoints[corner_id];
      const vec3_t r_FFi = object_points[corner_id];

      const CAMERA_TYPE cam0_geometry(cam0->resolution, cam0->param);
      const CAMERA_TYPE cam1_geometry(cam1->resolution, cam1->param);

      const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi);
      const vec3_t r_C1Fi = tf_point(T_C1C0, r_C0Fi);
      vec2_t z_hat_C0;
      vec2_t z_hat_C1;
      cam0_geometry.project(r_C0Fi, z_hat_C0);
      cam1_geometry.project(r_C1Fi, z_hat_C1);

      const vec2_t r_C0{z_C0 - z_hat_C0};
      const vec2_t r_C1{z_C1 - z_hat_C1};
      cam0_errs.push_back(r_C0.norm());
      cam1_errs.push_back(r_C1.norm());

      if (tags_meta) {
        tags_meta->push_back({tag_id, corner_id});
      }
    }
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_stereo_views_t<CAMERA_TYPE> &views,
                   std::vector<double> &cam0_errs,
                   std::vector<double> &cam1_errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  for (const auto &view : views) {
    reproj_errors(view, cam0_errs, cam1_errs, tags_meta);
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
int filter_views(ceres::Problem &problem,
                 calib_stereo_views_t<CAMERA_TYPE> &views,
                 double threshold=1.2) {
  int nb_outliers = 0;

  // Iterate over views
  for (auto &view : views) {
    auto &grid0 = view.cam0_grid;
    auto &grid1 = view.cam1_grid;
    const auto cam0 = view.cam0;
    const auto cam1 = view.cam1;
    const auto T_C1C0 = view.T_C1C0;
    const auto T_C0F = view.T_C0F;

    // Evaluate reprojection errors
    std::vector<double> cam0_errs;
    std::vector<double> cam1_errs;
    std::vector<std::pair<int, int>> tags_meta;
    reproj_errors<CAMERA_TYPE>(view, cam0_errs, cam1_errs, &tags_meta);

    // Check which AprilGrid tag to remove
    std::set<int> remove_tag_ids;
    for (size_t i = 0; i < tags_meta.size(); i++) {
      auto tag_id = tags_meta[i].first;
      if (cam0_errs[i] > threshold || cam1_errs[i] > threshold) {
        remove_tag_ids.insert(tag_id);
      }
    }

    // Remove residuals that are linked to the tag
    for (const int tag_id : remove_tag_ids) {
      auto res_ids_iter = view.res_ids.begin();
      auto cost_fns_iter = view.cost_fns.begin();

      while (res_ids_iter != view.res_ids.end()) {
        auto res_id = *res_ids_iter;
        auto cost_fn = *cost_fns_iter;

        if (cost_fn->tag_id_ == tag_id) {
          // Remove residual from ceres::Problem and tag id from AprilGrid
          problem.RemoveResidualBlock(res_id);
          aprilgrid_remove(grid0, tag_id);
          aprilgrid_remove(grid1, tag_id);

          // Erase res_id from view
          {
            auto begin = std::begin(view.res_ids);
            auto end = std::end(view.res_ids);
            view.res_ids.erase(std::remove(begin, end, res_id), end);
          }

          // Erase cost_fn from view
          {
            auto begin = std::begin(view.cost_fns);
            auto end = std::end(view.cost_fns);
            view.cost_fns.erase(std::remove(begin, end, cost_fn), end);
          }

          nb_outliers++;
        }

        // Update iterators
        res_ids_iter++;
        cost_fns_iter++;
      }
    }
  }

  // printf("Removed %d outliers!\n", nb_outliers);
  return nb_outliers;
}

/* Estimate Stereo Camera Covariance */
int calib_stereo_covar(camera_params_t &cam0,
                       camera_params_t &cam1,
                       pose_t &extrinsic,
                       ceres::Problem &problem,
                       matx_t &covar);

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
  pose_t *extrinsic = new pose_t{param_counter++, 0, T_C1C0};
  calib_stereo_views_t<CAMERA_TYPE> views;
  double info = -1;
  size_t processed = 0;
  size_t nb_outliers = 0;

  for (const auto &i : indicies) {
    const auto grid0 = grids0[i];
    const auto grid1 = grids1[i];
    const auto ts = grids0[i].timestamp;
    if (grid0.timestamp != grid1.timestamp) {
      LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
      LOG_ERROR("Bad data! Stopping stereo-calibration!");
      return -1;
    } else if (grid0.nb_detections != grid1.nb_detections) {
      LOG_ERROR("cam0_grid.nb_detections != cam1_grid.nb_detections");
      LOG_ERROR("Bad data! Stopping stereo-calibration!");
      return -1;
    }

    // Estimate T_C0F and T_C1F at ts
    // -- T_C0F
    mat4_t T_C0F_;
    const vecx_t cam0_proj = cam0.proj_params();
    const vecx_t cam0_dist = cam0.dist_params();
    aprilgrid_calc_relative_pose(grid0, cam0_proj, cam0_dist, T_C0F_);
    pose_t *cam0_rel_pose = new pose_t{param_counter++, ts, T_C0F_};

    // Create view
    calib_stereo_view_t<CAMERA_TYPE> view;
    view.cam0_grid = grid0;
    view.cam1_grid = grid1;
    view.T_C0F = cam0_rel_pose;
    view.T_C1C0 = extrinsic;
    view.cam0 = &cam0;
    view.cam1 = &cam1;

    // Add AprilGrid to problem
    process_grid<CAMERA_TYPE>(grids0[i], grids1[i],
                              covar,
                              *cam0_rel_pose,
                              *extrinsic,
                              cam0, cam1,
                              view.res_ids,
                              view.cost_fns,
                              problem,
                              pose_parameterization,
                              loss);
    views.push_back(view);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;
    // std::cout << std::endl;

    // Filter views
    nb_outliers = filter_views(problem, views);

    // Evaluate current view
    matx_t covar;
    if (calib_stereo_covar(cam0, cam1, *extrinsic, problem, covar) == 0) {
      const double info_k = covar.trace();
      const double diff = (info - info_k) / info;

      if (info < 0) {
        info = info_k;

      } else if (diff > 0.02) {
        info = info_k;
      } else {
        for (const auto &res_id : views.back().res_ids) {
          problem.RemoveResidualBlock(res_id);
        }
        views.pop_back();
      }
    }

    // Show progress
    reproj_errors(views, data.cam0_errs, data.cam1_errs);

    printf("Processed [%ld / %ld]  ", processed++, indicies.size());
    printf("Total Outliers Removed: %ld ", nb_outliers);
    printf("Batch size: %ld ", views.size());
    printf("Reprojection Error: ");
    printf("\t");
    printf("cam0: [rmse: %.2f,", rmse(data.cam0_errs));
    printf(" mean: %.2f]", mean(data.cam0_errs));
    printf("\t");
    printf("cam1: [rmse: %.2f,", rmse(data.cam1_errs));
    printf(" mean: %.2f]", mean(data.cam1_errs));
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

  // Finish up
  // T_C1C0 = extrinsic->tf();

  print_vector("cam0.proj_params", cam0.proj_params());
  print_vector("cam0.dist_params", cam0.dist_params());
  print_vector("cam1.proj_params", cam1.proj_params());
  print_vector("cam1.dist_params", cam1.dist_params());
  print_matrix("T_C1C0", T_C1C0);

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

int save_results(const std::string &save_path,
                 const camera_params_t &cam0,
                 const camera_params_t &cam1,
                 const mat4_t &T_C1C0,
                 const std::vector<double> &cam0_errs,
                 const std::vector<double> &cam1_errs);

} //  namespace yac
#endif // YAC_CALIB_STEREO_HPP
