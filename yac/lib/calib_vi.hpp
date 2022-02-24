#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <ceres/ceres.h>

#include "calib_errors.hpp"

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
  extrinsics_t *imu_exts = nullptr;
  fiducial_t *fiducial = nullptr;
  PoseLocalParameterization *pose_plus = nullptr;

  // Problem
  ceres::Problem *problem = nullptr;
  // -- Fiducial errors
  CamIdx2FiducialErrorIds fiducial_error_ids;
  CamIdx2FiducialErrors fiducial_errors;
  // -- Imu error
  ceres::ResidualBlockId imu_error_id;
  imu_error_t *imu_error = nullptr;

  calib_vi_view_t() = default;
  calib_vi_view_t(const timestamp_t ts_,
                  const CamIdx2Grids &grids_,
                  const mat4_t &T_WS,
                  const vec_t<9> &sb,
                  CamIdx2Geometry &cam_geoms_,
                  CamIdx2Parameters &cam_params_,
                  CamIdx2Extrinsics &cam_exts_,
                  extrinsics_t *imu_exts_,
                  fiducial_t *fiducial_,
                  ceres::Problem *problem_,
                  PoseLocalParameterization *pose_plus);
  ~calib_vi_view_t();

  std::vector<int> get_camera_indices() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;
  std::vector<real_t> get_imu_errors() const;
  int filter_view(const real_t outlier_threshold);
  void form_imu_error(const imu_params_t &imu_params,
                      const imu_data_t &imu_buf,
                      pose_t *pose_j,
                      sb_params_t *sb_j);
  ceres::ResidualBlockId marginalize(marg_error_t *marg_error);
};

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

struct calib_vi_t {
  // Settings
  bool verbose = true;
  int max_num_threads = 4;
  int max_iter = 30;
  bool enable_outlier_rejection = false;
  bool enable_marginalization = false;
  double outlier_threshold = 4.0;
  int window_size = 4;

  // Optimization
  PoseLocalParameterization pose_plus;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;

  // State-Variables
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  extrinsics_t *imu_exts = nullptr;
  fiducial_t *fiducial = nullptr;
  time_delay_t *time_delay = nullptr;

  // Data
  bool initialized = false;
  // -- Vision data
  calib_target_t calib_target;
  std::map<int, std::deque<aprilgrid_t>> grid_buf;
  CamIdx2Grids prev_grids;
  // -- Imu data
  imu_params_t imu_params;
  imu_data_t imu_buf;

  // Problem data
  std::deque<calib_vi_view_t *> calib_views;
  ceres::ResidualBlockId marg_error_id;
  marg_error_t *marg_error = nullptr;

  // AprilGrid detector
  std::unique_ptr<aprilgrid_detector_t> detector;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructor / Destructor
  calib_vi_t(const calib_target_t &calib_target_);
  calib_vi_t(const calib_vi_t &calib);
  calib_vi_t(const std::string &config_path);
  ~calib_vi_t();

  void add_imu(const imu_params_t &imu_params_,
               const mat4_t &T_BS,
               const double td = 0.0,
               const bool fix_extrinsics = false,
               const bool fix_time_delay = false);
  void add_camera(const int cam_idx,
                  const int resolution[2],
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
  veci2_t get_camera_resolution(const int cam_idx) const;
  vecx_t get_camera_params(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  mat4_t get_imu_extrinsics() const;
  real_t get_imucam_time_delay() const;
  mat4_t get_fiducial_pose() const;
  param_t *get_pose_param(const timestamp_t ts) const;
  param_t *get_sb_param(const timestamp_t ts) const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;

  mat4_t estimate_sensor_pose(const CamIdx2Grids &grids);
  void initialize(const CamIdx2Grids &grids, imu_data_t &imu_buf);
  void add_view(const CamIdx2Grids &grids);
  void add_measurement(const int cam_idx, const aprilgrid_t &grid);
  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m);
  int recover_calib_covar(matx_t &calib_covar) const;
  void marginalize();

  void solve();
  void show_results();
  int save_results(const std::string &save_path) const;
  void save_estimates(const std::string &dir_path) const;
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
