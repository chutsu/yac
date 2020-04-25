#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_mono.hpp"
#include "calib_data.hpp"

namespace yac {


/*****************************************************************************
 *                             STEREO CAMERA
 ****************************************************************************/

/**
 * Stereo camera calibration residual
 */
struct calib_stereo_residual_t {
  calib_params_t cam0_params_;
  calib_params_t cam1_params_;

  real_t z_C0_[2] = {0.0, 0.0};     ///< Measurement from cam0
  real_t z_C1_[2] = {0.0, 0.0};     ///< Measurement from cam1
  real_t p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  calib_stereo_residual_t(const calib_params_t &cam0_params,
                          const calib_params_t &cam1_params,
                          const vec2_t &z_C0,
                          const vec2_t &z_C1,
                          const vec3_t &p_F)
      : cam0_params_{cam0_params}, cam1_params_{cam1_params},
        z_C0_{z_C0(0), z_C0(1)}, z_C1_{z_C1(0), z_C1(1)},
        p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~calib_stereo_residual_t() {}

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const cam0_intrinsics,
                  const T *const cam0_distortion,
                  const T *const cam1_intrinsics,
                  const T *const cam1_distortion,
                  const T *const q_C0C1_,
                  const T *const r_C0C1_,
                  const T *const q_C0F_,
                  const T *const r_C0F_,
                  T *residual) const {
    // Map variables to Eigen
    // -- Fiducial point
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
    // -- cam0 intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam0;
    cam0 << cam0_intrinsics[0], cam0_intrinsics[1], cam0_intrinsics[2], cam0_intrinsics[3],
            cam0_distortion[0], cam0_distortion[1], cam0_distortion[2], cam0_distortion[3];
    // -- cam1 intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam1;
    cam1 << cam1_intrinsics[0], cam1_intrinsics[1], cam1_intrinsics[2], cam1_intrinsics[3],
            cam1_distortion[0], cam1_distortion[1], cam1_distortion[2], cam1_distortion[3];

    // Form transforms
    // clang-format off
    // -- Create transform between fiducial and cam0
    const Eigen::Quaternion<T> q_C0F(q_C0F_[3], q_C0F_[0], q_C0F_[1], q_C0F_[2]);
    const Eigen::Matrix<T, 3, 3> C_C0F = q_C0F.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0F{r_C0F_[0], r_C0F_[1], r_C0F_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0F = tf(C_C0F, r_C0F);
    // -- Create transform between cam0 and cam1
    const Eigen::Quaternion<T> q_C0C1(q_C0C1_[3], q_C0C1_[0], q_C0C1_[1], q_C0C1_[2]);
    const Eigen::Matrix<T, 3, 3> C_C0C1 = q_C0C1.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0C1{r_C0C1_[0], r_C0C1_[1], r_C0C1_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0C1 = tf(C_C0C1, r_C0C1);
    const Eigen::Matrix<T, 4, 4> T_C1C0 = T_C0C1.inverse();
    // clang-format on

    // Project
    // clang-format off
    // -- Project point observed from cam0 to cam0 image plane
    const Eigen::Matrix<T, 3, 1> p_C0 = (T_C0F * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C0_hat;
    const auto cam0_proj = cam0_params_.proj_model;
    const auto cam0_dist = cam0_params_.dist_model;
    if (cam0_proj == "pinhole" && cam0_dist == "radtan4") {
      if (pinhole_radtan4_project(cam0, p_C0, z_C0_hat) != 0) { return false; }
    } else if (cam0_proj == "pinhole" && cam0_dist == "equi4") {
      if (pinhole_equi4_project(cam0, p_C0, z_C0_hat) != 0) { return false; }
    } else {
      FATAL("Unsupported [%s-%s] projection distortion combination!",
            cam0_proj.c_str(),
            cam0_dist.c_str());
    }
    // -- Project point observed from cam0 to cam1 image plane
    const Eigen::Matrix<T, 3, 1> p_C1 = (T_C1C0 * p_C0.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C1_hat;
    const auto cam1_proj = cam1_params_.proj_model;
    const auto cam1_dist = cam1_params_.dist_model;
    if (cam1_proj == "pinhole" && cam1_dist == "radtan4") {
      if (pinhole_radtan4_project(cam1, p_C1, z_C1_hat) != 0) { return false; }
    } else if (cam1_proj == "pinhole" && cam1_dist == "equi4") {
      if (pinhole_equi4_project(cam1, p_C1, z_C1_hat) != 0) { return false; }
    } else {
      FATAL("Unsupported [%s-%s] projection distortion combination!",
            cam1_proj.c_str(),
            cam1_dist.c_str());
    }
    // clang-format on

    // Residual
    // -- cam0 residual
    residual[0] = T(z_C0_[0]) - z_C0_hat(0);
    residual[1] = T(z_C0_[1]) - z_C0_hat(1);
    // -- cam1 residual
    residual[2] = T(z_C1_[0]) - z_C1_hat(0);
    residual[3] = T(z_C1_[1]) - z_C1_hat(1);

    return true;
  }
};

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_aprilgrids,
                       const std::vector<aprilgrid_t> &cam1_aprilgrids,
                       calib_params_t &cam0_params,
                       calib_params_t &cam1_params,
                       mat4_t &T_C0C1,
                       mat4s_t &T_C0F);

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
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
