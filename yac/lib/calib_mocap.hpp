#ifndef YAC_CALIB_MOCAP_HPP
#define YAC_CALIB_MOCAP_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_camera.hpp"

namespace yac {

// MOCAP VIEW ////////////////////////////////////////////////////////////////

struct mocap_view_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Data
  timestamp_t ts = 0;
  aprilgrid_t grid;
  solver_t *solver = nullptr;
  calib_loss_t *loss = nullptr;
  std::deque<std::shared_ptr<mocap_residual_t>> res_fns;

  camera_geometry_t *cam_geom = nullptr;
  camera_params_t *cam_params = nullptr;
  pose_t *fiducial_pose = nullptr;
  pose_t *mocap_pose = nullptr;
  extrinsics_t *mocap_camera_extrinsics = nullptr;

  mocap_view_t() = delete;
  mocap_view_t(const aprilgrid_t &grid,
               solver_t *solver_,
               calib_loss_t *loss_,
               camera_geometry_t *cam_geom_,
               camera_params_t *cam_params_,
               pose_t *fiducial_pose_,
               pose_t *mocap_pose_,
               extrinsics_t *mocap_camera_extrinsics_);
  ~mocap_view_t() = default;

  int nb_detections() const;
  vec2s_t get_residuals() const;
  std::vector<real_t> get_reproj_errors() const;
  int filter_view(const vec2_t &threshold);
};

//////////////////////////////////////////////////////////////////////////////

// CALIB MOCAP CACHE /////////////////////////////////////////////////////////

struct calib_mocap_cache_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  vecx_t cam_params;
  vecx_t fiducial_pose;
  std::map<timestamp_t, vecx_t> mocap_poses;
  vecx_t mocap_camera_extrinsics;

  calib_mocap_cache_t() = default;
  calib_mocap_cache_t(const camera_params_t &camera_,
                      const pose_t &fiducial_pose_,
                      const std::map<timestamp_t, pose_t> &mocap_poses_,
                      const extrinsics_t &mocap_camera_extrinsics_);
  ~calib_mocap_cache_t() = default;

  void restore(camera_params_t &camera_,
               pose_t &fiducial_pose_,
               std::map<timestamp_t, pose_t> &mocap_poses_,
               extrinsics_t &mocap_camera_extrinsics_);
};

//////////////////////////////////////////////////////////////////////////////

// CALIB MOCAP  //////////////////////////////////////////////////////////////

struct calib_mocap_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Settings
  bool fix_intrinsics = false;
  bool fix_mocap_poses = false;
  bool fix_fiducial_pose = false;
  real_t outlier_threshold = 3.0;
  real_t info_gain_threshold = 0.2;
  bool enable_loss_fn = true;
  std::string loss_fn_type = "CAUCHY";
  double loss_fn_param = 1.0;
  bool enable_shuffle_views = true;
  bool show_progress = true;
  int max_iter = 50;

  // Paths
  std::string config_file;
  std::string data_path;

  // NBV
  real_t info_k = 0.0;
  real_t entropy_k = 0.0;

  // Calibration data
  calib_target_t calib_target;
  std::default_random_engine calib_rng;
  std::map<timestamp_t, aprilgrid_t> calib_grids;
  timestamps_t calib_view_timestamps;
  std::map<timestamp_t, std::shared_ptr<mocap_view_t>> calib_views;
  std::unique_ptr<solver_t> solver;
  std::unique_ptr<calib_loss_t> loss_fn;
  calib_mocap_cache_t cache;

  // Calibration parameters
  std::shared_ptr<camera_geometry_t> camera_geometry;
  camera_params_t camera;
  pose_t fiducial_pose;
  std::map<timestamp_t, pose_t> mocap_poses;
  extrinsics_t mocap_camera_extrinsics;

  calib_mocap_t(const std::string &config_file, const std::string &data_path);

  int get_num_views() const;
  vec2s_t get_residuals() const;
  std::vector<real_t> get_reproj_errors() const;
  mat4_t get_fiducial_pose() const;
  mat4_t get_mocap_camera_extrinsics() const;
  vecx_t get_camera_params() const;

  aprilgrids_t _preprocess(const calib_target_t &calib_target,
                           const std::string &cam_path,
                           const std::string &grid_path);
  void _add_camera(const std::vector<int> &cam_res,
                   const std::string &proj_model,
                   const std::string &dist_model,
                   const vecx_t &proj_params,
                   const vecx_t &dist_params,
                   const bool fix = false);
  void _add_fiducial_pose(const timestamp_t ts,
                          const mat4_t &T_WF,
                          const bool fix = false);
  void _add_mocap_pose(const timestamp_t ts,
                       const mat4_t &T_WM,
                       const bool fix = false);
  void _add_mocap_camera_extrinsics(const mat4_t &T_MC0);
  void _add_view(const aprilgrid_t &grid);
  void _remove_view(const timestamp_t ts);
  int _calc_info(real_t *info, real_t *entropy);
  void _cache_estimates();
  void _restore_estimates();
  void _print_stats(const size_t ts_idx, const size_t nb_timestamps);
  int _filter_last_view();
  int _filter_all_views();

  int solve();
  int solve_nbv();
  void print_settings(FILE *out) const;
  void print_calib_target(FILE *out) const;
  void print_metrics(FILE *out) const;
  void print_fiducial_pose(FILE *out) const;
  void print_mocap_camera_extrinsics(FILE *out) const;
  void print_camera_params(FILE *out) const;
  void print_mocap_poses(FILE *out) const;
  void show_results() const;
  int save_results(const std::string &save_path) const;
};

//////////////////////////////////////////////////////////////////////////////

} //  namespace yac
#endif // YAC_CALIB_MOCAP_HPP
