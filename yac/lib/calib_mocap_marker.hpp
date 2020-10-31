#ifndef YAC_CALIB_MOCAP_MARKER_HPP
#define YAC_CALIB_MOCAP_MARKER_HPP

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "calib_mono.hpp"

namespace yac {

/**
 * MOCAP marker residual
 */
struct mocap_marker_residual_t {
  double z_[2] = {0.0, 0.0};        ///< Measurement from cam0
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point
  std::string proj_model_ = "pinhole";
  std::string dist_model_ = "radtan4";

  mocap_marker_residual_t(const std::string &proj_model,
                          const std::string &dist_model,
                          const vec2_t &z,
                          const vec3_t &p_F)
    : proj_model_{proj_model},
      dist_model_{dist_model},
      z_{z(0), z(1)},
      p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~mocap_marker_residual_t() {}

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
    residual[0] = T(z_[0]) - z_hat(0);
    residual[1] = T(z_[1]) - z_hat(1);

    return true;
  }
};

// /**
//  * Stereo camera calibration residual
//  */
// struct mocap_marker_residual2_t {
//   double z_[2] = {0.0, 0.0};        ///< Measurement from cam0
//   double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point
//   std::string proj_model_ = "pinhole";
//   std::string dist_model_ = "radtan4";
//
//   mocap_marker_residual_t(const std::string &proj_model,
//                           const std::string &dist_model,
//                           const vec2_t &z,
//                           const vec3_t &p_F)
//     : proj_model_{proj_model},
//       dist_model_{dist_model},
//       z_{z(0), z(1)},
//       p_F_{p_F(0), p_F(1), p_F(2)} {}
//
//   ~mocap_marker_residual_t() {}
//
//   bool Evaluate()(double const * const *params,
//                   double *residuals,
//                   double **jacobians) const {
//     // Map optimization variables to Eigen
//     // -- Marker to camera extrinsics pose T_MC
//     const Eigen::Quaternion<T> q_MC(q_MC_[3], q_MC_[0], q_MC_[1], q_MC_[2]);
//     const Eigen::Matrix<T, 3, 1> r_MC{r_MC_[0], r_MC_[1], r_MC_[2]};
//     const Eigen::Matrix<T, 3, 3> C_MC = q_MC.toRotationMatrix();
//     const Eigen::Matrix<T, 4, 4> T_MC = tf(C_MC, r_MC);
//     // -- Marker pose T_WM
//     const Eigen::Quaternion<T> q_WM(q_WM_[3], q_WM_[0], q_WM_[1], q_WM_[2]);
//     const Eigen::Matrix<T, 3, 3> C_WM = q_WM.toRotationMatrix();
//     const Eigen::Matrix<T, 3, 1> r_WM{r_WM_[0], r_WM_[1], r_WM_[2]};
//     const Eigen::Matrix<T, 4, 4> T_WM = tf(C_WM, r_WM);
//     // -- Fiducial pose T_WF
//     const Eigen::Quaternion<T> q_WF(q_WF_[3], q_WF_[0], q_WF_[1], q_WF_[2]);
//     const Eigen::Matrix<T, 3, 3> C_WF = q_WF.toRotationMatrix();
//     const Eigen::Matrix<T, 3, 1> r_WF{r_WF_[0], r_WF_[1], r_WF_[2]};
//     const Eigen::Matrix<T, 4, 4> T_WF = tf(C_WF, r_WF);
//     // -- Camera intrinsics and distortion
//     Eigen::Matrix<T, 8, 1> cam_params;
//     cam_params << intrinsics_[0], intrinsics_[1], intrinsics_[2], intrinsics_[3],
//                   distortion_[0], distortion_[1], distortion_[2], distortion_[3];
//
//     // Project fiducial object point to camera image plane
//     const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
//     const Eigen::Matrix<T, 4, 4> T_CM = T_MC.inverse();
//     const Eigen::Matrix<T, 4, 4> T_MW = T_WM.inverse();
//     const Eigen::Matrix<T, 4, 1> hp_C = T_CM * T_MW * T_WF * p_F.homogeneous();
//     const Eigen::Matrix<T, 3, 1> p_C = hp_C.head(3);
//     Eigen::Matrix<T, 2, 1> z_hat;
//     if (proj_model_ == "pinhole" && dist_model_ == "radtan4") {
//       if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
//         return false;
//       }
//     } else if (proj_model_ == "pinhole" && dist_model_ == "equi4") {
//       if (pinhole_equi4_project(cam_params, p_C, z_hat) != 0) {
//         return false;
//       }
//     } else {
//       FATAL("Unsupported [%s-%s] projection distortion combination!",
//             proj_model_.c_str(), dist_model_.c_str());
//     }
//
//     // Residual
//     residuals[0] = z_[0] - z_hat(0);
//     residuals[1] = z_[1] - z_hat(1);
//
//     return true;
//   }
// };

/**
 * Calibrate mocap marker
 */
int calib_mocap_marker_solve(const aprilgrids_t &aprilgrids,
                             calib_params_t &cam,
                             mat4s_t &T_WM,
                             mat4_t &T_MC,
                             mat4_t &T_WF);

/**
 * Evaluate mocap marker cost
 */
double evaluate_mocap_marker_cost(const aprilgrids_t &aprilgrids,
                                  calib_params_t &cam,
                                  mat4s_t &T_WM,
                                  mat4_t &T_MC);

} //  namespace yac
#endif // YAC_CALIB_MOCAP_MARKER_HPP
