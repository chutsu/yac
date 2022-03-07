#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_errors.hpp"
#include "calib_nbv.hpp"
#include "solver.hpp"

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
  solver_t *solver = nullptr;
  CamIdx2ReprojErrors res_fns;

  // Parameters
  CamIdx2Geometry *cam_geoms = nullptr;
  CamIdx2Parameters *cam_params = nullptr;
  CamIdx2Extrinsics *cam_exts = nullptr;
  pose_t *T_C0F = nullptr;

  // Constructor / Destructor
  calib_view_t() = delete;
  calib_view_t(const timestamp_t ts_,
               const CamIdx2Grids &grids_,
               fiducial_corners_t *corners_,
               solver_t *solver_,
               CamIdx2Geometry *cam_geom_,
               CamIdx2Parameters *cam_params_,
               CamIdx2Extrinsics *cam_exts_,
               pose_t *T_C0F_);
  ~calib_view_t();

  int nb_detections() const;
  std::vector<int> get_camera_indices() const;
  vec2s_t get_residuals() const;
  vec2s_t get_residuals(const int cam_idx) const;
  std::vector<real_t> get_reproj_errors() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  int filter_view(const vec2_t &threshold);
  int filter_view(const std::map<int, vec2_t> &thresholds);
  void marginalize(marg_error_t *marg_error);
};

// CALIBRATOR //////////////////////////////////////////////////////////////////

/** Camera Calibrator **/
struct calib_camera_t {
  // Flags
  bool initialized = false;
  bool problem_init = false;
  bool filter_all = true;

  // Settings
  // -- General
  bool verbose = true;
  int max_num_threads = 8;
  bool enable_nbv = true;
  bool enable_shuffle_views = true;
  bool enable_nbv_filter = false;
  bool enable_outlier_filter = false;
  bool enable_marginalization = false;
  bool enable_early_stopping = false;
  int min_nbv_views = 40;
  real_t outlier_threshold = 4.0;
  real_t info_gain_threshold = 0.2;
  int sliding_window_size = 10;
  int early_stop_threshold = 30;

  // Data
  calib_target_t calib_target;
  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, std::map<int, aprilgrid_t>> calib_data;
  std::map<int, aprilgrids_t> validation_data;
  int removed_outliers = 0;

  // Buffers
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buf;
  std::map<int, aprilgrid_t> grid_buf;

  // NBV
  real_t info_k = 0.0;

  std::map<timestamp_t, std::map<int, vecx_t>> cam_estimates;
  std::map<timestamp_t, std::map<int, mat4_t>> exts_estimates;

  std::set<timestamp_t> nbv_timestamps;
  std::map<timestamp_t, std::tuple<real_t, real_t, int>> nbv_costs;
  std::map<timestamp_t, std::map<int, std::vector<real_t>>> nbv_reproj_errors;
  std::map<timestamp_t, bool> nbv_accepted;

  // Temporary storage
  std::map<int, vecx_t> cam_params_tmp;
  std::map<int, vecx_t> cam_exts_tmp;
  std::map<timestamp_t, vecx_t> poses_tmp;

  // Problem
  std::default_random_engine calib_rng;
  solver_t *solver = nullptr;
  marg_error_t *marg_error = nullptr;

  // State variables
  fiducial_corners_t *corners;
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  std::map<timestamp_t, pose_t *> poses;

  // Sliding window
  timestamps_t calib_view_timestamps;
  std::map<timestamp_t, calib_view_t *> calib_views;

  // AprilGrid detector
  std::unique_ptr<aprilgrid_detector_t> detector;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructor / Destructor
  calib_camera_t() = delete;
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
  bool add_measurement(const timestamp_t ts,
                       const int cam_idx,
                       const cv::Mat &cam_img);
  void add_pose(const timestamp_t ts,
                const std::map<int, aprilgrid_t> &cam_grids,
                const bool fixed = false);
  bool add_view(const std::map<int, aprilgrid_t> &cam_grids, const bool force);
  void remove_view(const timestamp_t ts, const bool skip_solve = true);
  void remove_all_views();
  void marginalize();

  int recover_calib_covar(matx_t &calib_covar, bool verbose = false);
  int find_nbv(const std::map<int, mat4s_t> &nbv_poses,
               int &cam_idx,
               int &nbv_idx);

  void _initialize_intrinsics();
  void _initialize_extrinsics();
  int _filter_all_views();
  void _cache_estimates();
  void _restore_estimates();
  int _calc_info(real_t *info);
  int _remove_outliers(const bool filter_all);
  void _track_estimates(const timestamp_t ts);
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

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
