#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "camchain.hpp"

namespace yac {


/** Reprojection Error */
struct reproj_error_t : ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const camera_geometry_t *cam_geom;
  const int cam_index;
  const int cam_res[2];
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  const vec2_t z;

  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const int cam_index_,
                 const int cam_res_[2],
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_)
      : cam_geom{cam_geom_},
        cam_index{cam_index_},
        cam_res{cam_res_[0], cam_res_[1]},
        tag_id{tag_id_},
        corner_idx{corner_idx_},
        r_FFi{r_FFi_},
        z{z_},
        covar{covar_},
        info{covar.inverse()},
        sqrt_info{info.llt().matrixL().transpose()} {
    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7);  // camera-fiducial relative pose
    block_sizes->push_back(7);  // camera-camera extrinsics
    block_sizes->push_back(8);  // camera parameters
  }

  bool Evaluate(double const * const *params,
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
          J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
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

/**
 * Initialize camera intrinsics using AprilGrids `grids` observed by the camera,
 * the camera geometry `cam_geom`, and camera parameters `cam_params`.
 */
void initialize_camera(const aprilgrids_t &grids,
                       const camera_geometry_t *cam_geom,
                       camera_params_t &cam_params,
                       const bool verbose=false);

/* Calibration View */
struct calib_view_t {
  aprilgrid_t grid;
  const camera_params_t &cam;
  const pose_t &T_C0F;
  const pose_t &T_C0Ci;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_t *> cost_fns;

  calib_view_t(const aprilgrid_t &grid_,
               const camera_params_t &cam_,
               const pose_t &T_C0F_,
               const pose_t &T_C0Ci_)
    : grid{grid_},
      cam{cam_},
      T_C0F{T_C0F_},
      T_C0Ci{T_C0Ci_} {}
};

/** Camera Calibrator **/
struct calib_camera_t {
  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;

  // State variables
  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsics_t> cam_exts;
  std::map<timestamp_t, pose_t> poses;

  // Data
  std::map<int, camera_geometry_t *> cam_geoms;
  calib_target_t target;
  std::set<timestamp_t> timestamps;
  camera_data_t cam_data;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  calib_camera_t();
  ~calib_camera_t();

  int nb_cams() const;
  aprilgrids_t get_cam_data(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  void add_calib_target(const calib_target_t &target_);
  void add_camera_data(const int cam_idx, const aprilgrids_t &grids);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const bool fixed=false);
  void add_camera_extrinsics(const int cam_idx,
                             const mat4_t &ext=I(4),
                             const bool fixed=false);
  pose_t &add_pose(const timestamp_t &ts,
                   const mat4_t &T,
                   const bool fixed=false);
  void _initialize_intrinsics();
  void _initialize_extrinsics();
  void _setup_problem();
  void solve();
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
