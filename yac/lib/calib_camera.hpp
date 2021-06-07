#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

/** Reprojection Error */
template <typename CAMERA>
struct reproj_error_t : public ceres::SizedCostFunction<2, 7, 7, 8> {
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

  reproj_error_t(const int cam_index,
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
        sqrt_info_{info_.llt().matrixL().transpose()} {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    const mat4_t T_C0F = tf(params[0]);
    const mat4_t T_C0Ci = tf(params[1]);
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    bool valid = true;
    // -- Transform point from fiducial frame to camera-n
    const mat4_t T_CiC0 = T_C0Ci.inverse();
    const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi_);
    // -- Project point from camera frame to image plane
    mat_t<2, 3> cam_Jh;
    vec2_t z_hat;
    const CAMERA cam{cam_res_, cam_params};
    if (cam.project(r_CiFi, z_hat, cam_Jh) != 0) {
      valid = false;
    }
    const vec2_t p_CiFi{r_CiFi(0) / r_CiFi(2), r_CiFi(1) / r_CiFi(2)};
    const matx_t cam_J_params = cam.J_params(p_CiFi);

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info_ * (z_ - z_hat);

    // Jacobians
    const matx_t Jh_weighted = -1 * sqrt_info_ * cam_Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_BF = tf_rot(T_C0F);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();
        J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0 * I(3);
        J.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_BF * r_FFi_);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t T_C0Ci
      if (jacobians[1]) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0Ci = C_CiC0.transpose();
        const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi_);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0 * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
        J.block(0, 0, 2, 8) = -1 * sqrt_info_ * cam_J_params;
        if (valid == false) {
          J.setZero();
        }
      }
    }

    return true;
  }
};

// /* Calibration View */
// struct calib_view_t {
//   int camera_index = -1;
//   aprilgrid_t grid;
//   mat2_t covar;
//   pose_t &T_BF;
//   pose_t &T_C0Ci;
//   camera_params_t &cam;
//
//   std::vector<ceres::ResidualBlockId> res_ids;
//   std::vector<reproj_error_t<CAMERA> *> cost_fns;
//
//   calib_view_t(const int camera_index_,
//                 const aprilgrid_t &grid_,
//                 const mat2_t &covar_,
//                 camera_params_t &cam_,
//                 pose_t &T_BF_,
//                 pose_t &T_C0Ci_)
//     : camera_index{camera_index_},
//       grid{grid_},
//       covar{covar_},
//       cam{cam_},
//       T_BF{T_BF_},
//       T_C0Ci{T_C0Ci_} {}
//
//   ~calib_view_t() {}
// };

struct calib_camera_t {
  calib_data_t &data;
  ceres::Problem *problem;

  calib_camera_t(calib_data_t &data_)
      : data{data_}, problem{data_.problem} {
    // // Setting cam0 to be body frame B
    // extrinsics_t &cam0_exts = data.cam_exts[0];
    // cam0_exts.set_tf(I(3), zeros(3, 1));
    // problem->AddParameterBlock(cam0_exts.param.data(), 7);
    // problem->SetParameterBlockConstant(cam0_exts.param.data());
  }

  ~calib_camera_t() {}

  // static mat4_t estimate_pose(const camera_params_t &cam,
  //                             const aprilgrid_t &grid,
  //                             const mat4_t &T_C0Ci=I(4)) {
  //   mat4_t T_CiF_k;
  //   const int *cam_res = cam.resolution;
  //   const vecx_t proj_params = cam.proj_params();
  //   const vecx_t dist_params = cam.dist_params();
  //   const C cam_geom{cam_res, proj_params, dist_params};
  //   if (grid.estimate(cam_geom, T_CiF_k) != 0) {
  //     FATAL("Failed to estimate relative pose!");
  //   }
  //
  //   const mat4_t T_BF = T_C0Ci * T_CiF_k;
  //   return T_BF;
  // }
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
