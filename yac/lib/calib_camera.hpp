#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_errors.hpp"
#include "calib_nbv.hpp"

namespace yac {

// CAMCHAIN ///////////////////////////////////////////////////////////////////j

/** Camera Chain Query Tool */
struct camchain_t {
  std::map<int, std::vector<int>> adj_list;  // CameraChain adjacency list
  std::map<int, std::map<int, mat4_t>> exts; // Extrinsics

  /** Constructor */
  camchain_t(const camera_data_t &cam_data,
             const std::map<int, camera_geometry_t *> &cam_geoms,
             const std::map<int, camera_params_t *> &cam_params);

  void _get_aprilgrids(const timestamp_t &ts,
                       const camera_data_t &cam_data,
                       std::vector<int> &cam_indicies,
                       std::vector<aprilgrid_t> &grids) const;

  /** Insert camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  void insert(const int cam_i, const int cam_j, const mat4_t &T_CiCj);

  /** Check if camera extrinsics `T_CiCj` between `cam_i` and `cam_j` exists */
  bool contains(const int cam_i, const int cam_j) const;

  /** Find camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  int find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const;

  /** Clear camchain */
  void clear();
};

// CALIB VIEW //////////////////////////////////////////////////////////////////

/** Calibration view */
struct calib_view_t {
  // Data
  timestamp_t ts = 0;
  CamIdx2Grids grids;
  fiducial_corners_t *corners = nullptr;

  // Problem
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  CamIdx2ReprojErrors res_fns;
  CamIdx2ReprojErrorIds res_ids;

  // Parameters
  CamIdx2Geometry *cam_geoms = nullptr;
  CamIdx2Parameters *cam_params = nullptr;
  CamIdx2Extrinsics *cam_exts = nullptr;
  pose_t *T_C0F = nullptr;

  calib_view_t(const timestamp_t ts_,
               const CamIdx2Grids &grids_,
               fiducial_corners_t *corners_,
               ceres::Problem *problem_,
               ceres::LossFunction *loss_,
               CamIdx2Geometry *cam_geom_,
               CamIdx2Parameters *cam_params_,
               CamIdx2Extrinsics *cam_exts_,
               pose_t *T_C0F_);
  ~calib_view_t();

  std::vector<int> get_camera_indices() const;
  vec2s_t get_residuals() const;
  vec2s_t get_residuals(const int cam_idx) const;
  std::vector<real_t> get_reproj_errors() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  int filter_view(const real_t reproj_threshold);
  int filter_view(const vec2_t &residual_threshold);
  ceres::ResidualBlockId marginalize(marg_error_t *marg_error);
};

// CALIBRATOR //////////////////////////////////////////////////////////////////

/** Camera Calibrator **/
struct calib_camera_t {
  // Flags
  bool initialized = false;
  bool problem_init = false;
  bool filter_views_init = false;

  // Settings
  // -- General
  bool verbose = true;
  // -- Intrinsics initialization
  bool enable_intrinsics_nbv = false;
  bool enable_intrinsics_outlier_filter = true;
  real_t intrinsics_outlier_threshold = 3.0;
  real_t intrinsics_info_gain_threshold = 0.05;
  int intrinsics_min_nbv_views = 20;
  // -- Extrinsics initialization
  bool enable_extrinsics_outlier_filter = true;
  // -- Final stage settings
  bool enable_nbv = true;
  bool enable_shuffle_views = true;
  bool enable_nbv_filter = true;
  bool enable_outlier_filter = true;
  bool enable_marginalization = false;
  bool enable_early_stopping = false;
  int min_nbv_views = 10;
  real_t outlier_threshold = 3.0;
  real_t info_gain_threshold = 0.2;
  int sliding_window_size = 10;
  int early_stop_threshold = 30;

  // Data
  calib_target_t calib_target;
  std::map<int, real_t> focal_length_init;
  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, std::map<int, aprilgrid_t>> calib_data;
  std::map<int, aprilgrids_t> validation_data;
  int removed_outliers = 0;

  // NBV
  real_t info_k = 0.0;
  real_t valid_error_k = 0.0;

  // Temporary storage
  std::map<int, vec2s_t> batch_residuals;
  std::map<int, vecx_t> cam_params_tmp;
  std::map<int, vecx_t> cam_exts_tmp;
  std::map<timestamp_t, vecx_t> poses_tmp;

  // Problem
  std::default_random_engine calib_rng;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  ceres::ResidualBlockId marg_error_id;
  marg_error_t *marg_error = nullptr;
  PoseLocalParameterization pose_plus;

  // State variables
  fiducial_corners_t corners{calib_target};
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  std::map<timestamp_t, pose_t *> poses;

  // Sliding window
  std::deque<calib_view_t *> calib_views;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructor / Destructor
  calib_camera_t() = delete;
  calib_camera_t(const calib_camera_t &calib_);
  calib_camera_t(const calib_target_t &calib_target_);
  calib_camera_t(const std::string &config_path);
  ~calib_camera_t();

  int nb_cameras() const;
  int nb_views() const;
  aprilgrids_t get_camera_data(const int cam_idx) const;
  std::vector<int> get_camera_indices() const;
  vecx_t get_camera_params(const int cam_idx) const;
  veci2_t get_camera_resolution(const int cam_idx) const;
  std::string get_camera_projection_model(const int cam_idx) const;
  std::string get_camera_distortion_model(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  std::vector<real_t> get_all_reproj_errors();
  std::map<int, std::vector<real_t>> get_reproj_errors();
  std::map<int, vec2s_t> get_residuals();

  void add_camera_data(const int cam_idx, const aprilgrids_t &grids);
  void add_camera_data(const std::map<int, aprilgrids_t> &grids);
  void add_camera_data(const std::map<int, aprilgrids_t> &train_data,
                       const std::map<int, aprilgrids_t> &valid_data);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const bool fixed = false);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const bool fixed = false);
  void add_camera_extrinsics(const int cam_idx,
                             const mat4_t &ext = I(4),
                             const bool fixed = false);
  void add_pose(const timestamp_t ts, const bool fixed = false);
  void add_pose(const int cam_idx,
                const aprilgrid_t &grid,
                const bool fixed = false);
  bool add_view(const std::map<int, aprilgrid_t> &cam_grids);
  void remove_view(const timestamp_t ts);
  void remove_all_views();
  void marginalize();

  real_t _estimate_calib_info();
  int recover_calib_covar(matx_t &calib_covar, bool verbose = true);
  int find_nbv(const std::map<int, mat4s_t> &nbv_poses,
               int &cam_idx,
               int &nbv_idx);

  void _initialize_intrinsics();
  void _initialize_extrinsics();
  int _filter_view(const timestamp_t ts,
                   const std::map<int, vec2_t> &cam_thresholds);
  int _filter_all_views();
  void _cache_estimates();
  void _restore_estimates();
  int _calc_info(real_t *info);
  int _eval_nbv(const timestamp_t ts);
  void _print_stats(const size_t ts_idx, const size_t nb_timestamps);
  void _solve_batch(const bool filter_outliers);
  void _solve_inc();
  void _solve_nbv();

  void solve();
  void print_settings(FILE *out);
  void print_metrics(FILE *out,
                     const std::map<int, std::vector<real_t>> &reproj_errors,
                     const std::vector<real_t> &reproj_errors_all);
  void print_calib_target(FILE *out);
  void print_estimates(FILE *out);
  void show_results();
  int save_results(const std::string &save_path);
  int save_estimates(const std::string &save_path);
  int save_stats(const std::string &save_path);
  real_t inspect(const std::map<int, aprilgrids_t> &cam_data);
};

/** Solve Camera Calibration Problem */
int calib_camera_solve(const std::string &config_path);

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
