#ifndef YAC_REPROJ_ERROR_HPP
#define YAC_REPROJ_ERROR_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

/** Reprojection Error */
struct reproj_error_t : public ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Data
  // -- Camera
  const camera_geometry_t *cam_geom;
  const int cam_idx;
  const int cam_res[2];
  // -- Fiducial target
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  // -- Measurement
  const vec2_t z;
  // -- Covariance, information and square-root information
  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  /** Constructor */
  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const int cam_idx_,
                 const int cam_res_[2],
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_)
      : cam_geom{cam_geom_}, cam_idx{cam_idx_},
        cam_res{cam_res_[0], cam_res_[1]}, tag_id{tag_id_},
        corner_idx{corner_idx_}, r_FFi{r_FFi_}, z{z_}, covar{covar_},
        info{covar.inverse()}, sqrt_info{info.llt().matrixL().transpose()} {
    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7); // camera-fiducial relative pose
    block_sizes->push_back(7); // camera-camera extrinsics
    block_sizes->push_back(8); // camera parameters
  }

  /** Evaluate */
  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    const mat4_t T_C0F = tf(params[0]);
    const mat4_t T_C0Ci = tf(params[1]);
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    // -- Transform point from fiducial frame to camera-n
    const mat4_t T_CiC0 = T_C0Ci.inverse();
    const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
    // -- Project point from camera frame to image plane
    vec2_t z_hat;
    bool valid = true;
    if (cam_geom->project(cam_res, cam_params, r_CiFi, z_hat) != 0) {
      valid = false;
    }

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info * (z - z_hat);

    // Jacobians
    const matx_t Jh = cam_geom->project_jacobian(cam_params, r_CiFi);
    const matx_t J_cam_params = cam_geom->params_jacobian(cam_params, r_CiFi);
    const matx_t Jh_weighted = -1 * sqrt_info * Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0F = tf_rot(T_C0F);
          J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * r_FFi);
        }
      }

      // Jacobians w.r.t T_C0Ci
      if (jacobians[1]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0Ci = C_CiC0.transpose();
          const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
          J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) =
              -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
        J.setZero();

        if (valid) {
          J.block(0, 0, 2, 8) = -1 * sqrt_info * J_cam_params;
        }
      }
    }

    return true;
  }
};

} // namespace yac
#endif // YAC_REPROJ_ERROR_HPP
