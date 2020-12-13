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

  calib_stereo_data_t(const mat2_t &covar_,
                      const aprilgrids_t &cam0_grids_,
                      const aprilgrids_t &cam1_grids_,
                      const camera_params_t &cam0_,
                      const camera_params_t &cam1_)
    : covar{covar_},
      cam0_grids{cam0_grids_},
      cam1_grids{cam1_grids_},
      cam0{cam0_},
      cam1{cam1_} {}
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

/** Stereo camera calibration residual */
template <typename CAMERA_TYPE>
struct reproj_error_with_extrinsics_t
    : public ceres::SizedCostFunction<2, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_index_ = -1;
  int cam_res_[2] = {0, 0};
  int tag_id_ = -1;
  int corner_idx_ = -1;
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  reproj_error_with_extrinsics_t(const int cam_index,
                                 const int cam_res[2],
                                 const int tag_id,
                                 const int corner_idx,
                                 const vec3_t &r_FFi,
                                 const vec2_t &z,
                                 const mat2_t &covar)
      : cam_index_{cam_index},
        cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_idx_{corner_idx},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(2, 6));  // T_C0F
    J_min.push_back(zeros(2, 6));  // T_CnC0
    J_min.push_back(zeros(2, 8));  // cam params
  }

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    // -- Map camera-fiducial pose
    const mat4_t T_C0F = tf(params[0]);
    // -- Map camera-camera pose
    mat4_t T_CnC0 = tf(params[1]);
    if (cam_index_ == 0) {
      T_CnC0 = I(4);
    }
    // -- Map camera params
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    bool valid = true;
    // -- Transform point from fiducial frame to camera-n
    const vec3_t r_CnFi = tf_point(T_CnC0 * T_C0F, r_FFi_);
    // -- Project point from camera frame to image plane
    mat_t<2, 3> cam_Jh;
    vec2_t z_hat;
    const CAMERA_TYPE cam{cam_res_, cam_params};
    if (cam.project(r_CnFi, z_hat, cam_Jh) != 0) {
      valid = false;
    }
    const vec2_t p_CnFi{r_CnFi(0) / r_CnFi(2), r_CnFi(1) / r_CnFi(2)};
    const matx_t cam_J_params = cam.J_params(p_CnFi);

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info_ * (z_ - z_hat);

    // Jacobians
    const matx_t weighted_Jh = -1 * sqrt_info_ * cam_Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        const mat3_t C_C0F = tf_rot(T_C0F);
        J_min[0].block(0, 0, 2, 3) = weighted_Jh * I(3);
        J_min[0].block(0, 3, 2, 3) = weighted_Jh * -skew(C_C0F * r_FFi_);
        if (valid == false) {
          J_min[0].setZero();
        }

        Eigen::Map<mat_t<2, 7, row_major_t>> J0(jacobians[0]);
        J0.setZero();
        J0.block(0, 0, 2, 6) = J_min[0];
      }

      // Jacobians w.r.t T_CnC0
      if (jacobians[1]) {
        if (cam_index_ == 0) {
          // cam0
          J_min[1].setZero();
          Eigen::Map<mat_t<2, 7, row_major_t>> J1(jacobians[1]);
          J1.setZero();

        } else {
          // cam-n
          const mat3_t C_CnC0 = tf_rot(T_CnC0);
          J_min[1].block(0, 0, 2, 3) = weighted_Jh * I(3);
          J_min[1].block(0, 3, 2, 3) = weighted_Jh * -skew(C_CnC0 * r_CnFi);
          if (valid == false) {
            J_min[1].setZero();
          }

          Eigen::Map<mat_t<2, 7, row_major_t>> J1(jacobians[1]);
          J1.setZero();
          J1.block(0, 0, 2, 6) = J_min[1];
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        J_min[2].block(0, 0, 2, 8) = -1 * sqrt_info_ * cam_J_params;
        if (valid == false) {
          J_min[2].setZero();
        }

        Eigen::Map<mat_t<2, 8, row_major_t>> J2(jacobians[2]);
        J2 = J_min[2];
      }
    }

    return true;
  }
};

/* Calibration View */
template <typename CAMERA_TYPE>
struct calib_view_t {
  int camera_index = -1;
  aprilgrid_t grid;
  mat2_t covar;
  pose_t &T_C0F;
  pose_t &T_C1C0;
  camera_params_t &cam;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_with_extrinsics_t<CAMERA_TYPE> *> cost_fns;

  calib_view_t(const int camera_index_,
               const aprilgrid_t &grid_,
               const mat2_t &covar_,
               camera_params_t &cam_,
               pose_t &T_C0F_,
               pose_t &T_C1C0_)
    : camera_index{camera_index_},
      grid{grid_},
      covar{covar_},
      cam{cam_},
      T_C0F{T_C0F_},
      T_C1C0{T_C1C0_} {}

  ~calib_view_t() {
    // for (const auto &cost_fn : cost_fns) {
    //   delete cost_fn;
    // }
  }

};

/* Stereo-Camera Calibration Views */
template <typename CAMERA_TYPE>
using calib_views_t = std::deque<calib_view_t<CAMERA_TYPE>>;

template <typename CAMERA_TYPE>
void save_views(const std::string &save_dir,
                const calib_views_t<CAMERA_TYPE> &views) {
  auto cam_idx = views[0].camera_index;
  auto save_path = save_dir + "/";
  save_path += "cam" + std::to_string(cam_idx) + "-views.csv";

  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#ts,err_x,err_y,reproj_error,tag_id,corner_idx\n");
  for (const auto &view : views) {
    auto ts = view.grid.timestamp;

    std::vector<std::pair<double, double>> residuals;
    std::vector<double> reprojection_errors;
    std::vector<std::pair<int, int>> tags_meta;
    calib_residuals(view, residuals, &tags_meta);
    reproj_errors(view, reprojection_errors);

    for (size_t i = 0; i < residuals.size(); i++) {
      const auto err_x = residuals[i].first;
      const auto err_y = residuals[i].second;
      const auto reproj_err = reprojection_errors[i];
      const auto tag_id = tags_meta[i].first;
      const auto corner_idx = tags_meta[i].second;

      fprintf(csv, "%ld,", ts);
      fprintf(csv, "%f,%f,", err_x, err_y);
      fprintf(csv, "%f,", reproj_err);
      fprintf(csv, "%d,%d\n", tag_id, corner_idx);
    }
  }
  fclose(csv);

}

/* Process AprilGrid - Adding it to the calibration problem */
template <typename CAMERA_TYPE>
static int process_grid(calib_view_t<CAMERA_TYPE> &view,
                        ceres::Problem &problem,
                        PoseLocalParameterization &pose_plus,
                        ceres::LossFunction *loss=nullptr) {
  const mat2_t covar = view.covar;
  const int cam_idx = view.camera_index;
  const int *cam_res = view.cam.resolution;
  const auto &grid = view.grid;
  auto &cam = view.cam;
  auto &T_C0F = view.T_C0F;
  auto &T_C1C0 = view.T_C1C0;

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  view.grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const auto cost_fn = new reproj_error_with_extrinsics_t<CAMERA_TYPE>{
      cam_idx, cam_res, tag_id, corner_idx, r_FFi, z, covar};
    const auto res_id = problem.AddResidualBlock(cost_fn, // Cost function
                                                 loss,    // Loss function
                                                 T_C0F.param.data(),
                                                 T_C1C0.param.data(),
                                                 cam.param.data());

    view.res_ids.push_back(res_id);
    view.cost_fns.push_back(cost_fn);
  }
  problem.SetParameterization(T_C0F.param.data(), &pose_plus);
  problem.SetParameterization(T_C1C0.param.data(), &pose_plus);

  return 0;
}

static pose_t estimate_relative_pose(const int cam_idx,
                                     const aprilgrid_t &grid,
                                     const camera_params_t &cam,
                                     const mat4_t &T_C1C0,
                                     id_t &param_counter) {
  mat4_t rel_pose;
  if (grid.estimate(cam.proj_params(), cam.dist_params(), rel_pose) != 0) {
    FATAL("Failed to estimate relative pose!");
  }
  const auto ts = grid.timestamp;

  if (cam_idx != 0) {
    mat4_t T_C0F = T_C1C0.inverse() * rel_pose;
    return pose_t{param_counter++, ts, T_C0F};
  }

  return pose_t{param_counter++, ts, rel_pose};
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void calib_residuals(const calib_view_t<CAMERA_TYPE> &view,
                     std::vector<std::pair<double, double>> &r,
                     std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const int camera_index = view.camera_index;
  const auto grid = view.grid;
  const auto cam = view.cam;
  const mat4_t T_C0F = view.T_C0F.tf();
  const mat4_t T_C1C0 = view.T_C1C0.tf();

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t cam0_keypoints;
  vec2s_t cam1_keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, cam0_keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = cam0_keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const CAMERA_TYPE cam_geometry(cam.resolution, cam.param);
    const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi);
    vec2_t z_hat;
    if (camera_index == 0) {
      cam_geometry.project(r_C0Fi, z_hat);
    } else if (camera_index == 1) {
      cam_geometry.project(tf_point(T_C1C0, r_C0Fi), z_hat);
    } else {
      FATAL("Invalid camera index [%d]!", camera_index);
    }
    r.push_back({z(0) - z_hat(0), z(1) - z_hat(1)});

    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_view_t<CAMERA_TYPE> &view,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const int camera_index = view.camera_index;
  const auto grid = view.grid;
  const auto cam = view.cam;
  const mat4_t T_C0F = view.T_C0F.tf();
  const mat4_t T_C1C0 = view.T_C1C0.tf();

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t cam0_keypoints;
  vec2s_t cam1_keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, cam0_keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = cam0_keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const CAMERA_TYPE cam_geometry(cam.resolution, cam.param);
    const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi);
    vec2_t z_hat;
    if (camera_index == 0) {
      cam_geometry.project(r_C0Fi, z_hat);
    } else if (camera_index == 1) {
      cam_geometry.project(tf_point(T_C1C0, r_C0Fi), z_hat);
    } else {
      FATAL("Invalid camera index [%d]!", camera_index);
    }

    const vec2_t r{z - z_hat};
    errs.push_back(r.norm());

    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_views_t<CAMERA_TYPE> &views,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  for (const auto &view : views) {
    reproj_errors(view, errs, tags_meta);
  }
}

/* Show calibration results */
template <typename CAMERA_TYPE>
static void show_results(const calib_views_t<CAMERA_TYPE> &cam0_views,
                         const calib_views_t<CAMERA_TYPE> &cam1_views,
                         const mat4_t &T_C1C0) {
  // Show results
  std::vector<double> cam0_errs, cam1_errs;
  reproj_errors<CAMERA_TYPE>(cam0_views, cam0_errs);
  reproj_errors<CAMERA_TYPE>(cam1_views, cam1_errs);

  auto cam0 = cam0_views[0].cam;
  auto cam1 = cam1_views[0].cam;

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
                       mat4_t &T_C1C0) {
  // Initialize stereo-camera intrinsics and extrinsics
  // initialize<CAMERA_TYPE>(cam0_grids, cam1_grids, covar, cam0, cam1, T_C1C0);
  // print_matrix("T_C1C0", T_C1C0);

  // Stereo camera calibration Setup
  id_t param_id = 0;
  T_C1C0 = I(4);
  pose_t extrinsics{param_id++, 0, T_C1C0};
  std::map<timestamp_t, pose_t> rel_poses;

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::LossFunction *loss = nullptr;
  // ceres::LossFunction *loss = new ceres::CauchyLoss(1);
  // ceres::LossFunction *loss = new ceres::HuberLoss(1);
  PoseLocalParameterization pose_plus;

  // Process all aprilgrid data
  calib_views_t<CAMERA_TYPE> cam0_views;
  calib_views_t<CAMERA_TYPE> cam1_views;

  // -- cam0
  for (const auto &grid : cam0_grids) {
    // Estimate T_C0F and add to parameters to be estimated
    const auto ts = grid.timestamp;
    rel_poses[ts] = estimate_relative_pose(0, grid, cam0, T_C1C0, param_id);

    // Add reprojection factors to problem
    cam0_views.emplace_back(0, grid, covar, cam0, rel_poses[ts], extrinsics);
    int retval = process_grid<CAMERA_TYPE>(cam0_views.back(),
                                           problem,
                                           pose_plus,
                                           loss);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }
  // -- cam1
  for (const auto &grid : cam1_grids) {
    // Estimate T_C0F if it does not exist at ts
    const auto ts = grid.timestamp;
    if (rel_poses.count(ts) == 0) {
      rel_poses[ts] = estimate_relative_pose(1, grid, cam1, T_C1C0, param_id);
    }

    // Add reprojection factors to problem
    cam1_views.emplace_back(1, grid, covar, cam1, rel_poses[ts], extrinsics);
    int retval = process_grid<CAMERA_TYPE>(cam1_views.back(),
                                           problem,
                                           pose_plus,
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
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-20;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  // std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Finish up
  T_C1C0 = extrinsics.tf();
  mat4s_t T_C0F;
  mat4s_t T_C1F;
  for (auto kv : rel_poses) {
    auto pose = kv.second;
    T_C0F.emplace_back(pose.tf());
    T_C1F.emplace_back(T_C1C0 * pose.tf());
  }
  if (loss) {
    delete loss;
  }

  // Show results
  show_results<CAMERA_TYPE>(cam0_views, cam1_views, T_C1C0);

  return 0;
}

/* Initialize Stereo-Camera Intrinsics and Extrinsics */
template <typename CAMERA_TYPE>
static void initialize(const aprilgrids_t &grids0,
                       const aprilgrids_t &grids1,
                       const mat2_t &covar,
                       camera_params_t &cam0,
                       camera_params_t &cam1,
                       mat4_t &T_C1C0) {
  // Initialize camera intrinsics
  // -- cam0
  LOG_INFO("Calibrating cam0 intrinsics ...");
  mat4s_t T_C0F;
  if (calib_mono_solve<CAMERA_TYPE>(grids0, covar, cam0, T_C0F) != 0) {
    FATAL("Failed to calibrate cam0 intrinsics!");
  }
  // calib_mono_data_t cam0_data;
  // cam0_data.covar = covar;
  // cam0_data.grids = grids0;
  // cam0_data.cam_params = cam0;
  // if (calib_mono_inc_solve<CAMERA_TYPE>(cam0_data) != 0) {
  //   FATAL("Failed to calibrate cam0 intrinsics!");
  // }
  // cam0 = cam0_data.cam_params;
  // mat4s_t T_C0F = cam0_data.T_CF;
  // -- cam1
  LOG_INFO("Calibrating cam1 intrinsics ...");
  mat4s_t T_C1F;
  if (calib_mono_solve<CAMERA_TYPE>(grids1, covar, cam1, T_C1F) != 0) {
    FATAL("Failed to calibrate cam1 intrinsics!");
  }
  // calib_mono_data_t cam1_data;
  // cam1_data.covar = covar;
  // cam1_data.grids = grids1;
  // cam1_data.cam_params = cam1;
  // if (calib_mono_inc_solve<CAMERA_TYPE>(cam1_data) != 0) {
  //   FATAL("Failed to calibrate cam0 intrinsics!");
  // }
  // cam1 = cam1_data.cam_params;
  // mat4s_t T_C1F = cam1_data.T_CF;

  // // Initialize stereo-camera extrinsics
  // std::vector<double> rx, ry, rz;
  // std::vector<double> roll, pitch, yaw;
  // size_t cam0_idx = 0;
  // size_t cam1_idx = 0;
  // size_t k_end = std::min(T_C0F.size(), T_C1F.size());
  //
  // for (size_t k = 0; k < k_end; k++) {
  //   const auto &grid0 = grids0[cam0_idx];
  //   const auto &grid1 = grids1[cam1_idx];
  //
  //   if (grid0.timestamp == grid1.timestamp) {
  //     const mat4_t T_C1C0_k = T_C1F[cam1_idx] * T_C0F[cam0_idx].inverse();
  //     const vec3_t r_C1C0_k = tf_trans(T_C1C0_k);
  //     const vec3_t rpy_C1C0_k = quat2euler(tf_quat(T_C1C0_k));
  //
  //     rx.push_back(r_C1C0_k(0));
  //     ry.push_back(r_C1C0_k(1));
  //     rz.push_back(r_C1C0_k(2));
  //
  //     roll.push_back(rpy_C1C0_k(0));
  //     pitch.push_back(rpy_C1C0_k(1));
  //     yaw.push_back(rpy_C1C0_k(2));
  //
  //     cam0_idx++;
  //     cam1_idx++;
  //
  //   } else if (grid0.timestamp > grid1.timestamp) {
  //     cam1_idx++;
  //   } else if (grid0.timestamp < grid1.timestamp) {
  //     cam0_idx++;
  //   }
  // }
  //
  // // Take the median and form T_C1C0
  // const vec3_t r_C1C0{median(rx), median(ry), median(rz)};
  // const vec3_t rpy_C1C0{median(roll), median(pitch), median(yaw)};
  // const mat3_t C_C1C0 = euler321(rpy_C1C0);
  // T_C1C0 = tf(C_C1C0, r_C1C0);

  calib_stereo_solve<CAMERA_TYPE>(grids0,
                                  grids1,
                                  covar,
                                  cam0,
                                  cam1,
                                  T_C1C0);
}

template <typename CAMERA_TYPE>
int filter_view(ceres::Problem &problem,
                calib_view_t<CAMERA_TYPE> &view,
                double threshold=1.0) {
  auto &grid = view.grid;
  const auto cam = view.cam;
  int nb_outliers = 0;

  // Evaluate reprojection errors
  std::vector<double> errs;
  std::vector<std::pair<int, int>> tags_meta;
  reproj_errors<CAMERA_TYPE>(view, errs, &tags_meta);

  // Check which AprilGrid tag to remove
  std::set<std::pair<int, int>> outliers;
  for (size_t i = 0; i < tags_meta.size(); i++) {
    auto tag_id = tags_meta[i].first;
    auto corner_idx = tags_meta[i].second;
    if (errs[i] > threshold) {
      outliers.insert({tag_id, corner_idx});
    }
  }

  // Remove residuals that are linked to the tag
  for (const auto &kv : outliers) {
    auto tag_id = kv.first;
    auto corner_idx = kv.second;
    auto res_ids_iter = view.res_ids.begin();
    auto cost_fns_iter = view.cost_fns.begin();

    while (res_ids_iter != view.res_ids.end()) {
      auto res_id = *res_ids_iter;
      auto cost_fn = *cost_fns_iter;

      if (cost_fn->tag_id_ == tag_id && cost_fn->corner_idx_ == corner_idx) {
        // Remove residual from ceres::Problem, tag id and corner_idx from AprilGrid
        problem.RemoveResidualBlock(res_id);
        grid.remove(tag_id, corner_idx);

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
        break;
      }

      // Update iterators
      res_ids_iter++;
      cost_fns_iter++;
    }
  }

  return nb_outliers;
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
int filter_views(ceres::Problem &problem,
                 calib_views_t<CAMERA_TYPE> &views,
                 double threshold=1.0) {
  int nb_outliers = 0;
  for (auto &view : views) {
    nb_outliers += filter_view<CAMERA_TYPE>(problem, view, threshold);
  }

  return nb_outliers;
}

/* Estimate Stereo Camera Covariance */
int calib_stereo_covar(camera_params_t &cam0,
                       camera_params_t &cam1,
                       pose_t &extrinsic,
                       ceres::Problem &problem,
                       matx_t &covar);

/**
 * Find random indicies.
 *
 * Aside from creating a set of random indicies to randomly sample the image
 * frames, the goal is to find a set that yeilds the lowest reprojection error
 * for the first 20 frames. The intuition is to start the incremental solver in
 * a certain state, then add / remove frames that improve the calibration.
 */
template <typename CAMERA_TYPE>
static std::vector<int> find_random_indicies(const int attempts,
                                             const int min_views,
                                             const mat2_t &covar,
                                             const aprilgrids_t &grids0,
                                             const aprilgrids_t &grids1,
                                             camera_params_t &cam0,
                                             camera_params_t &cam1,
                                             pose_t &extrinsics,
                                             bool verbose=false) {
  id_t param_counter = 0;
  int best_idx = 0;
  double best_rmse = 0.0;
  std::vector<int> best_indicies;

  for (int j = 0; j < attempts; j++) {
    // Create random index vector (same length as grids)
    std::vector<int> indicies;
    for (size_t i = 0; i < grids0.size(); i++) indicies.push_back(i);
    std::random_shuffle(std::begin(indicies), std::end(indicies));

    // Views
    calib_views_t<CAMERA_TYPE> cam0_views;
    calib_views_t<CAMERA_TYPE> cam1_views;
    std::deque<pose_t> rel_poses;

    // Evaluate indicies
    for (const auto &i : indicies) {
      const auto grid0 = grids0[i];
      const auto grid1 = grids1[i];
      const auto ts = grids0[i].timestamp;
      if (grid0.timestamp != grid1.timestamp) {
        LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
        FATAL("Bad data! Stopping stereo-calibration!");
      }

      // Estimate T_C0F
      mat4_t T_C0F_;
      const vecx_t cam0_proj = cam0.proj_params();
      const vecx_t cam0_dist = cam0.dist_params();
      grid0.estimate(cam0_proj, cam0_dist, T_C0F_);
      rel_poses.emplace_back(param_counter++, ts, T_C0F_);

      // Create view
      cam0_views.emplace_back(0, grid0, covar, cam0, rel_poses.back(), extrinsics);
      cam1_views.emplace_back(1, grid1, covar, cam1, rel_poses.back(), extrinsics);

      if (cam0_views.size() > min_views) {
        break;
      }
    }

    std::vector<double> cam0_errs;
    std::vector<double> cam1_errs;
    reproj_errors(cam0_views, cam0_errs);
    reproj_errors(cam1_views, cam1_errs);

    if (verbose) {
      printf("[%d]: ", j);
      printf("cam0: [rmse: %.2f,", rmse(cam0_errs));
      printf(" mean: %.2f]", mean(cam0_errs));
      printf("\t");
      printf("cam1: [rmse: %.2f,", rmse(cam1_errs));
      printf(" mean: %.2f]", mean(cam1_errs));
      printf("\n");
    }

    const double cand_rmse = rmse(cam0_errs) + rmse(cam1_errs);
    if (best_indicies.size() == 0 || cand_rmse < best_rmse) {
      best_idx = j;
      best_rmse = cand_rmse;
      best_indicies = indicies;
    }
  }

  if (verbose) {
    printf("best_idx: %d ", best_idx);
    printf("best_rmse: %f ", best_rmse);
    printf("\n");
  }

  return best_indicies;
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
 *
 * This stereo calibrator is a more elabroate version than the standard pipeline.
 * The pipeline is as follows:
 *
 * - Initialize camera intrinsincs by solving standard BA on cam0 and cam1
 *   independently.
 * - Initialize camera extrinsics T_C1C0 using the median of T_C0F and T_C1F,
 *   using the initialized cam0, cam1 intrinsics.
 * - Find random indicies that yields the best `min_views` that yeilds lowest
 *   reprojection error, where `min_views` is a hyper-parameter.
 * - Incrementally solve the stereo-camera intrinsincs and extrinsics
 *   - if views > min_views:
 *     - filter current view whose residal block has a reproj error > threshold
 *     - remove current if uncertainty is not improved by more than 2%
 *   - Stop early if no new frames have been added for N iterations
 * - Final filter all views whose residal block has reproj error > threshold
 * - Final batch optimization
 * - Done
 */
template <typename CAMERA_TYPE>
int calib_stereo_inc_solve(calib_stereo_data_t &data) {
  // Seed random
  srand(time(NULL));

  // Options
  int random_indicies_attempts = 1000;
  size_t filter_min_views = 20;
  double filter_stddev = 5.0;

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

  // Initialize stereo intrinsics and extrinsics
  initialize<CAMERA_TYPE>(grids0, grids1, covar, cam0, cam1, T_C1C0);

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;

  // Process all aprilgrid data
  id_t param_counter = 0;
  std::deque<pose_t> rel_poses;
  pose_t extrinsics{param_counter++, 0, T_C1C0};
  calib_views_t<CAMERA_TYPE> cam0_views;
  calib_views_t<CAMERA_TYPE> cam1_views;
  double info = -1;
  size_t processed = 0;
  size_t nb_outliers = 0;

  // Find best indicies that have the best first 20 views
  auto indicies = find_random_indicies<CAMERA_TYPE>(random_indicies_attempts,
                                                    filter_min_views,
                                                    covar, grids0, grids1,
                                                    cam0, cam1, extrinsics);

  // Incremental solve
  size_t stale_limit = 30;
  size_t stale_counter = 0;

  for (const auto &i : indicies) {
    size_t outliers = 0;
    const auto grid0 = grids0[i];
    const auto grid1 = grids1[i];
    const auto ts = grids0[i].timestamp;
    if (grid0.timestamp != grid1.timestamp) {
      LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
      FATAL("Bad data! Stopping stereo-calibration!");
    }

    // Estimate T_C0F
    mat4_t T_C0F_;
    const vecx_t cam0_proj = cam0.proj_params();
    const vecx_t cam0_dist = cam0.dist_params();
    grid0.estimate(cam0_proj, cam0_dist, T_C0F_);
    rel_poses.emplace_back(param_counter++, ts, T_C0F_);

    // Create view
    cam0_views.emplace_back(0, grid0, covar, cam0, rel_poses.back(), extrinsics);
    cam1_views.emplace_back(1, grid1, covar, cam1, rel_poses.back(), extrinsics);

    // Add view to problem
    process_grid<CAMERA_TYPE>(cam0_views.back(), problem, pose_plus, loss);
    process_grid<CAMERA_TYPE>(cam1_views.back(), problem, pose_plus, loss);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 20;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Filter last view
    if (processed > filter_min_views) {
      // Obtain cam0 and cam1 reprojection errors so far
      std::vector<double> cam0_reproj_errors;
      std::vector<double> cam1_reproj_errors;
      reproj_errors(cam0_views, cam0_reproj_errors);
      reproj_errors(cam1_views, cam1_reproj_errors);
      // Calculate filter threshold for cam0 and cam1
      const double cam0_threshold = filter_stddev * stddev(cam0_reproj_errors);
      const double cam1_threshold = filter_stddev * stddev(cam1_reproj_errors);

      // Filter last view
      outliers += filter_view(problem, cam0_views.back(), cam0_threshold);
      outliers += filter_view(problem, cam1_views.back(), cam1_threshold);
      nb_outliers += outliers;

      // Evaluate current view
      matx_t covar;
      if (calib_stereo_covar(cam0, cam1, extrinsics, problem, covar) == 0) {
        const double info_k = covar.trace();
        const double diff = (info - info_k) / info;

        if (info < 0) {
          info = info_k;

        } else if (diff > 0.02) {
          info = info_k;
          stale_counter = 0;

        } else {
          for (const auto &res_id : cam0_views.back().res_ids) {
            problem.RemoveResidualBlock(res_id);
          }
          for (const auto &res_id : cam1_views.back().res_ids) {
            problem.RemoveResidualBlock(res_id);
          }
          cam0_views.pop_back();
          cam1_views.pop_back();
          stale_counter++;
        }
      }
    }

    // Early stop
    if (stale_counter >= stale_limit) {
      LOG_INFO("stale_counter >= stale_limit");
      LOG_INFO("stopping early!");
      break;
    }

    // Show progress
    reproj_errors(cam0_views, data.cam0_errs);
    reproj_errors(cam1_views, data.cam1_errs);

    printf("Processed [%ld / %ld]  ", processed++, indicies.size());
    printf("Outliers Removed: %ld ", outliers);
    printf("Batch size: %ld ", cam0_views.size() + cam1_views.size());
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
  options.max_num_iterations = 30;
  // options.check_gradients = true;

  // Final filter before last optimization
  // -- Obtain cam0 and cam1 reprojection errors so far
  std::vector<double> cam0_reproj_errors;
  std::vector<double> cam1_reproj_errors;
  reproj_errors(cam0_views, cam0_reproj_errors);
  reproj_errors(cam1_views, cam1_reproj_errors);
  // -- Calculate filter threshold for cam0 and cam1
  const double cam0_threshold = filter_stddev * stddev(cam0_reproj_errors);
  const double cam1_threshold = filter_stddev * stddev(cam1_reproj_errors);
  // -- Filter
  nb_outliers += filter_views(problem, cam0_views, cam0_threshold);
  nb_outliers += filter_views(problem, cam1_views, cam1_threshold);

  // Solve
  ceres::Solver::Summary summary;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-20;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  // Update data
  T_C1C0 = extrinsics.tf();
  T_C0F.clear();
  T_C1F.clear();
  for (auto pose : rel_poses) {
    T_C0F.emplace_back(pose.tf());
    T_C1F.emplace_back(T_C1C0 * pose.tf());
  }

  // Show reuslts
  show_results(cam0_views, cam1_views, T_C1C0);
  save_views("/tmp", cam0_views);
  save_views("/tmp", cam1_views);

  // Clean up
  if (loss) {
    delete loss;
  }

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
