#ifndef YAC_CALIB_MONO_HPP
#define YAC_CALIB_MONO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"

namespace yac {

/**
 * Calibration mono residual
 */
struct calib_mono_residual_t {
  std::string proj_model_ = "pinhole";
  std::string dist_model_ = "radtan4";
  double z_[2] = {0.0, 0.0};        ///< Measurement
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  calib_mono_residual_t(const std::string &proj_model,
                        const std::string &dist_model,
                        const vec2_t &z,
                        const vec3_t &p_F)
      : proj_model_{proj_model},
        dist_model_{dist_model},
        z_{z(0), z(1)},
        p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~calib_mono_residual_t() {}

  /**
   * Calculate residual (auto diff version)
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
                  const T *const distortion_,
                  const T *const q_CF_,
                  const T *const r_CF_,
                  T *residuals_) const {
    // Map variables to Eigen
    Eigen::Matrix<T, 8, 1> cam_params;
    cam_params << intrinsics_[0], intrinsics_[1], intrinsics_[2], intrinsics_[3],
                  distortion_[0], distortion_[1], distortion_[2], distortion_[3];
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};

    // Form tf
    const Eigen::Quaternion<T> q_CF(q_CF_[3], q_CF_[0], q_CF_[1], q_CF_[2]);
    const Eigen::Matrix<T, 3, 3> C_CF = q_CF.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_CF{r_CF_[0], r_CF_[1], r_CF_[2]};
    Eigen::Matrix<T, 4, 4> T_CF = tf(C_CF, r_CF);

    // Transform and project point to image plane
    const Eigen::Matrix<T, 3, 1> p_C = (T_CF * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_hat;
    if (proj_model_ == "pinhole" && dist_model_ == "radtan4") {
      if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
        return false;
      }
    } else if (proj_model_ == "pinhole" && dist_model_ == "equi4") {
      if (pinhole_equi4_project(cam_params, p_C, z_hat) != 0) {
        return false;
      }
    } else {
      FATAL("Unsupported [%s-%s] projection distortion combination!",
            proj_model_.c_str(), dist_model_.c_str());
    }

    // Residual
    residuals_[0] = T(z_[0]) - z_hat(0);
    residuals_[1] = T(z_[1]) - z_hat(1);

    return true;
  }
};

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const aprilgrids_t &aprilgrids,
                     calib_params_t &calib_params,
                     mat4s_t &T_CF);

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target. This function assumes that the path to `config_file`
 * is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       imshow: true
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.085           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const std::string &config_file);

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_stats(const aprilgrids_t &aprilgrids,
                     const calib_params_t &calib_params,
                     const mat4s_t &poses);

/**
 * Generate poses
 */
mat4s_t calib_generate_poses(const calib_target_t &target);

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
