#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "camchain.hpp"

namespace yac {

/**
 * Initialize camera intrinsics using AprilGrids `grids` observed by the camera,
 * the camera geometry `cam_geom`, and camera parameters `cam_params`.
 */
void initialize_camera(const aprilgrids_t &grids,
                       const camera_geometry_t *cam_geom,
                       camera_params_t &cam_params,
                       const bool verbose=false);

/** Reprojection Error */
struct reproj_error_t : ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const camera_params_t &cam_params;
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  const vec2_t z;

  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  reproj_error_t(const camera_params_t &cam_params_,
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_);

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const;
};

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
