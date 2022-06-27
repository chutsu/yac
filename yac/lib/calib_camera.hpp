#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_residuals.hpp"
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
             const CamIdx2Geometry &cam_geoms,
             const CamIdx2Parameters &cam_params);

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
  CamIdx2ReprojResiduals res_fns;

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
               pose_t *T_C0F_,
               calib_loss_t *loss);
  ~calib_view_t() = default;

  int nb_detections() const;
  std::vector<int> get_camera_indices() const;
  vec2s_t get_residuals() const;
  vec2s_t get_residuals(const int cam_idx) const;
  std::vector<real_t> get_reproj_errors() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  int filter_view(const vec2_t &threshold);
  int filter_view(const std::map<int, vec2_t> &thresholds);
  void marginalize(marg_residual_t *marg_residual);
};

// CALIBRATOR //////////////////////////////////////////////////////////////////

void print_calib_target(FILE *out, const calib_target_t &calib_target);
void print_camera_params(FILE *out,
                         const int cam_idx,
                         const camera_params_t *cam);
void print_estimates(FILE *out,
                     const CamIdx2Parameters &cam_params,
                     const CamIdx2Extrinsics &cam_exts);

/** Camera Calibrator **/
struct calib_camera_t {
  // Flags
  bool filter_all = false;

  // Settings
  // -- General
  bool verbose = true;
  std::string solver_type = "CERES-SOLVER";
  int max_num_threads = 2;
  bool enable_nbv = true;
  bool enable_shuffle_views = true;
  bool enable_nbv_filter = true;
  bool enable_outlier_filter = true;
  bool enable_early_stopping = false;
  bool enable_marginalization = false;
  bool enable_loss_fn = true;
  // std::string loss_fn_type = "BLAKE-ZISSERMAN";
  // double loss_fn_param = 2;
  std::string loss_fn_type = "CAUCHY";
  double loss_fn_param = 1.5;
  int min_nbv_views = 40;
  real_t outlier_threshold = 3.0;
  real_t info_gain_threshold = 0.2;
  int early_stop_threshold = 30;
  int sliding_window_size = 10;

  // Data
  calib_target_t calib_target;
  std::set<timestamp_t> timestamps;
  camera_data_t calib_data;
  std::map<int, aprilgrids_t> validation_data;
  std::map<int, real_t> focal_length_init;
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
  std::shared_ptr<marg_residual_t> marg_residual;
  calib_loss_t *loss_fn = nullptr;

  // State variables
  std::unique_ptr<fiducial_corners_t> corners;
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  StampedPoses poses;

  // Sliding window
  timestamps_t calib_view_timestamps;
  std::set<timestamp_t> filtered_timestamps;
  std::map<timestamp_t, calib_view_t *> calib_views;

  // AprilGrid detector
  std::unique_ptr<aprilgrid_detector_t> detector;

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
  std::vector<real_t> get_all_reproj_errors() const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;
  std::map<int, vec2s_t> get_residuals() const;

  void add_camera_data(const int cam_idx,
                       const aprilgrids_t &grids,
                       const bool init_intrinsics = true);
  void add_camera_data(
      const std::map<int, std::map<timestamp_t, aprilgrid_t>> &grids,
      const bool init_intrinsics = true);
  void add_camera_data(const std::map<int, aprilgrids_t> &grids,
                       const bool init_intrinsics = true);
  void add_camera_data(const std::map<int, aprilgrids_t> &train_data,
                       const std::map<int, aprilgrids_t> &valid_data);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const mat4_t &ext = I(4),
                  const bool fix_intrinsics = false,
                  const bool fix_extrinsics = false);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const mat4_t &ext = I(4),
                  const bool fix_intrinsics = false,
                  const bool fix_extrinsics = false);
  bool add_measurement(const timestamp_t ts,
                       const int cam_idx,
                       const cv::Mat &cam_img,
                       const bool imshow = false);
  void add_pose(const timestamp_t ts,
                const std::map<int, aprilgrid_t> &cam_grids,
                const bool fixed = false);
  bool add_view(const std::map<int, aprilgrid_t> &cam_grids);
  bool add_nbv_view(const std::map<int, aprilgrid_t> &cam_grids);
  void remove_view(const timestamp_t ts);
  void remove_all_views();
  void marginalize();

  int recover_calib_covar(matx_t &calib_covar, bool verbose = false);
  int find_nbv(const std::map<int, mat4s_t> &nbv_poses,
               int &cam_idx,
               int &nbv_idx,
               real_t &nbv_info,
               real_t &info,
               real_t &info_gain);
  int find_nbv_fast(const std::map<int, mat4s_t> &nbv_poses,
                    int &cam_idx,
                    int &nbv_idx,
                    real_t &nbv_info,
                    real_t &info,
                    real_t &info_gain);

  void _initialize_intrinsics();
  void _initialize_extrinsics();
  int _filter_all_views();
  void _cache_estimates();
  void _restore_estimates();
  int _calc_info(real_t *info);
  int _remove_outliers(const bool filter_all);
  void _track_estimates(const timestamp_t ts, const bool view_accepted);
  void _print_stats(const size_t ts_idx, const size_t nb_timestamps);
  void _solve_batch(const bool verbose = false, const int max_iter = 50);
  void _solve_inc();
  void _solve_nbv();
  void _load_views(const std::string &data_path);
  void _load_camera_poses(const std::string &data_path);
  void load_data(const std::string &data_path);
  void solve(const bool skip_init = false);

  void print_settings(FILE *out) const;
  void print_metrics(FILE *out,
                     const std::map<int, std::vector<real_t>> &reproj_errors,
                     const std::vector<real_t> &reproj_errors_all) const;
  void show_results() const;

  int save_results(const std::string &save_path);
  int save_estimates(const std::string &save_path);
  int save_stats(const std::string &save_path);

  real_t inspect(const std::map<int, aprilgrids_t> &cam_data);
};

// NBV EVALUATOR //////////////////////////////////////////////////////////////

/**
 * NBV Evaluator
 */
struct nbv_evaluator_t {
  // Data
  calib_target_t calib_target;
  std::vector<int> cam_indices;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  CamIdx2Geometry cam_geoms;
  std::unique_ptr<fiducial_corners_t> corners;

  // Hessian
  size_t remain_size = 0;
  size_t marg_size = 0;
  ParameterOrder param_order;
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  matx_t H_k;

  nbv_evaluator_t() = delete;
  nbv_evaluator_t(calib_camera_t *calib);
  ~nbv_evaluator_t() = default;

  void form_param_order(const std::unordered_set<calib_residual_t *> &res_fns);
  void form_hessian(const size_t H_size,
                    const std::unordered_set<calib_residual_t *> &res_fns,
                    matx_t &H);
  real_t estimate_log_det_covar(const matx_t &H);
  real_t eval(const mat4_t &T_FC0);
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
