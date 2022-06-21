#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <ceres/ceres.h>

#include "calib_residuals.hpp"
#include "solver.hpp"

namespace yac {

// VISUAL INERTIAL CALIBRATION VIEW ////////////////////////////////////////////

struct calib_vi_view_t {
  // Data
  timestamp_t ts;
  CamIdx2Grids grids;

  // Parameters (internal)
  pose_t pose;
  sb_params_t sb;

  // Parameters (external)
  CamIdx2Geometry &cam_geoms;
  CamIdx2Parameters &cam_params;
  CamIdx2Extrinsics &cam_exts;
  std::shared_ptr<extrinsics_t> imu_exts;
  std::shared_ptr<fiducial_t> fiducial;
  PoseLocalParameterization *pose_plus;

  // Problem
  std::shared_ptr<ceres::Problem> problem;
  // -- Fiducial errors
  CamIdx2FiducialResidualIds fiducial_residual_ids;
  CamIdx2FiducialResiduals fiducial_residuals;
  // -- Imu residual
  ceres::ResidualBlockId imu_residual_id;
  std::shared_ptr<imu_residual_t> imu_residual;

  calib_vi_view_t() = default;
  calib_vi_view_t(const timestamp_t ts_,
                  const CamIdx2Grids &grids_,
                  const mat4_t &T_WS,
                  const vec_t<9> &sb,
                  CamIdx2Geometry &cam_geoms_,
                  CamIdx2Parameters &cam_params_,
                  CamIdx2Extrinsics &cam_exts_,
                  std::shared_ptr<extrinsics_t> imu_exts_,
                  std::shared_ptr<fiducial_t> fiducial_,
                  std::shared_ptr<ceres::Problem> problem_,
                  PoseLocalParameterization *pose_plus);
  ~calib_vi_view_t();

  std::vector<int> get_camera_indices() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;
  std::vector<real_t> get_imu_errors() const;
  int filter_view(const real_t outlier_threshold);
  void form_imu_residual(const imu_params_t &imu_params,
                         const imu_data_t imu_buf,
                         pose_t *pose_j,
                         sb_params_t *sb_j);
  ceres::ResidualBlockId marginalize(marg_residual_t *marg_residual);
};

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

struct calib_vi_t {
  // Flags
  std::mutex mtx;
  bool imu_started = false;
  bool vision_started = false;
  bool initialized = false;
  bool running = false;

  // Settings
  bool verbose = true;
  int max_num_threads = 4;
  int max_iter = 30;
  bool enable_outlier_rejection = false;
  bool enable_marginalization = false;
  double outlier_threshold = 4.0;
  int window_size = 4;
  const real_t img_scale = 0.4;

  // State-Variables
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  std::shared_ptr<extrinsics_t> imu_exts;
  std::shared_ptr<fiducial_t> fiducial;
  std::shared_ptr<time_delay_t> time_delay;

  // Data
  // -- Vision data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buf;
  std::map<int, aprilgrid_t> grid_buf;
  calib_target_t calib_target;
  CamIdx2Grids prev_grids;
  // -- Imu data
  imu_params_t imu_params;
  imu_data_t imu_buf;

  // Problem data
  profiler_t prof;
  size_t calib_view_counter = 0;
  std::deque<std::shared_ptr<calib_vi_view_t>> calib_views;
  ceres::ResidualBlockId marg_residual_id;
  std::shared_ptr<marg_residual_t> marg_residual;

  // Calibration Information
  bool calib_info_ok = false;
  matx_t calib_info;

  // Optimization
  PoseLocalParameterization pose_plus;
  ceres::Problem::Options prob_options;
  std::shared_ptr<ceres::Problem> problem;

  // AprilGrid detector
  std::unique_ptr<aprilgrid_detector_t> detector;

  // Constructor / Destructor
  calib_vi_t(const calib_target_t &calib_target_);
  calib_vi_t(const std::string &config_path);
  ~calib_vi_t() = default;

  void add_imu(const imu_params_t &imu_params_,
               const mat4_t &T_BS,
               const double td = 0.0,
               const bool fix_extrinsics = false,
               const bool fix_time_delay = false);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const mat4_t &T_BCi = I(4),
                  const bool fix_params = true,
                  const bool fix_extrinsics = true);

  int nb_cams() const;
  int nb_views() const;
  std::vector<int> get_camera_indices() const;
  real_t get_camera_rate() const;
  real_t get_imu_rate() const;
  veci2_t get_camera_resolution(const int cam_idx) const;
  vecx_t get_camera_params(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  mat4_t get_imu_extrinsics() const;
  real_t get_imucam_time_delay() const;
  mat4_t get_fiducial_pose() const;
  mat4_t get_imu_pose(const timestamp_t ts) const;
  mat4_t get_imu_pose() const;
  param_t *get_pose_param(const timestamp_t ts) const;
  param_t *get_sb_param(const timestamp_t ts) const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;

  mat4_t estimate_sensor_pose(const CamIdx2Grids &grids);
  void initialize(const CamIdx2Grids &grids, imu_data_t &imu_buf);
  void add_view(const CamIdx2Grids &grids);
  bool add_measurement(const timestamp_t ts,
                       const int cam_idx,
                       const cv::Mat &cam_image);
  void add_measurement(const int cam_idx, const aprilgrid_t &grid);
  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m);
  int recover_calib_covar(matx_t &calib_covar) const;
  int recover_calib_info(matx_t &H) const;
  void marginalize();
  void reset();

  void eval_residuals(ParameterOrder &param_order,
                      std::vector<calib_residual_t *> &res_evaled,
                      size_t &residuals_length,
                      size_t &params_length) const;
  void form_hessian(ParameterOrder &param_order, matx_t &H) const;

  void load_data(const std::string &data_path);
  void solve();
  void show_results();
  int save_results(const std::string &save_path) const;
  void save_estimates(const std::string &dir_path) const;
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
