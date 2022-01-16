#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <mutex>

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

// Forward declaration
struct fiducial_error_t;

// Typedefs
// clang-format off
using CamIdx2Grids = std::map<int, aprilgrid_t>;
using CamIdx2Geometry = std::map<int, camera_geometry_t *>;
using CamIdx2Parameters = std::map<int, camera_params_t *>;
using CamIdx2Extrinsics = std::map<int, extrinsics_t *>;
using CamIdx2FiducialErrors = std::map<int, std::deque<std::unique_ptr<fiducial_error_t>>>;
using CamIdx2FiducialErrorIds = std::map<int, std::deque<ceres::ResidualBlockId>>;
// clang-format on

// POSE ERROR //////////////////////////////////////////////////////////////////

struct pose_error_t : public ceres::SizedCostFunction<6, 7> {
  pose_t pose_meas_;
  matx_t covar_;
  matx_t info_;
  matx_t sqrt_info_;

  pose_error_t(const pose_t &pose, const matx_t &covar);
  ~pose_error_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// FIDUCIAL ERROR //////////////////////////////////////////////////////////////

struct fiducial_error_t : public ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts_ = 0;

  camera_geometry_t *cam_geom_;
  camera_params_t *cam_params_;
  extrinsics_t *cam_exts_;
  extrinsics_t *imu_exts_;
  fiducial_t *fiducial_;
  pose_t *pose_;

  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_{0.0, 0.0};
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  fiducial_error_t(const timestamp_t &ts,
                   camera_geometry_t *cam_geom,
                   camera_params_t *cam_params,
                   extrinsics_t *cam_exts,
                   extrinsics_t *imu_exts,
                   fiducial_t *fiducial,
                   pose_t *pose,
                   const int tag_id,
                   const int corner_idx,
                   const vec3_t &r_FFi,
                   const vec2_t &z,
                   const mat4_t &T_WF,
                   const mat2_t &covar);
  ~fiducial_error_t() = default;

  int get_residual(vec2_t &r) const;
  int get_reproj_error(real_t &error) const;
  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// INERTIAL ERROR //////////////////////////////////////////////////////////////

#define EST_TIMEDELAY 0

/**
 * Implements a nonlinear IMU factor.
 */
class ImuError : public ::ceres::SizedCostFunction<15, 7, 9, 7, 9> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  size_t state_id = 0;
  timestamp_t imu_t0 = 0;
  timestamp_t imu_t1 = 0;
  size_t pose0_id = 0;
  size_t pose1_id = 0;
  size_t state0_id = 0;
  size_t state1_id = 0;
  size_t timedelay_id = 0;

  mutable mat4_t T_WS_0_last_;
  mutable mat4_t T_WS_1_last_;
  mutable vec_t<9> sb0_last_;
  mutable vec_t<9> sb1_last_;

  ///< The number of residuals
  typedef Eigen::Matrix<double, 15, 15> covariance_t;
  typedef covariance_t information_t;
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;
  typedef Eigen::Matrix<double, 15, 7> jacobian0_t;
  typedef Eigen::Matrix<double, 15, 9> jacobian1_t;

  ImuError() = default;
  ImuError(const imu_data_t &imu_data,
           const imu_params_t &imu_params,
           const timestamp_t &t0,
           const timestamp_t &t1);
  virtual ~ImuError() = default;

  static int propagation(const imu_data_t &imu_data,
                         const imu_params_t &imu_params,
                         mat4_t &T_WS,
                         vec_t<9> &sb,
                         const timestamp_t &t_start,
                         const timestamp_t &t_end,
                         covariance_t *covariance = 0,
                         jacobian_t *jacobian = 0);

  int redoPreintegration(const mat4_t & /*T_WS*/,
                         const vec_t<9> &sb,
                         timestamp_t time,
                         timestamp_t end) const;

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const;

  imu_params_t imu_params_;
  imu_data_t imu_data_;
  timestamp_t t0_;
  timestamp_t t1_;

protected:
  // Preintegration stuff. the mutable is a TERRIBLE HACK, but what can I do.
  ///< Protect access of intermediate results.
  mutable std::mutex preintegrationMutex_;

  // increments (initialise with identity)
  mutable quat_t Delta_q_ = quat_t(1, 0, 0, 0);
  mutable mat3_t C_integral_ = mat3_t::Zero();
  mutable mat3_t C_doubleintegral_ = mat3_t::Zero();
  mutable vec3_t acc_integral_ = vec3_t::Zero();
  mutable vec3_t acc_doubleintegral_ = vec3_t::Zero();

  // Cross matrix accumulatrion
  mutable mat3_t cross_ = mat3_t::Zero();

  // Sub-Jacobians
  mutable mat3_t dalpha_db_g_ = mat3_t::Zero();
  mutable mat3_t dv_db_g_ = mat3_t::Zero();
  mutable mat3_t dp_db_g_ = mat3_t::Zero();

  ///< The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double, 15, 15> P_delta_ =
      Eigen::Matrix<double, 15, 15>::Zero();

  ///< Reference biases that are updated when called redoPreintegration.
  mutable vec_t<9> sb_ref_;

  ///< Keeps track of whether redoPreintegration() needs to be called.
  mutable bool redo_ = true;

  ///< Counts the number of preintegrations for statistics.
  mutable int redoCounter_ = 0;

  // information matrix and its square root form
  mutable information_t information_;
  mutable information_t squareRootInformation_;
};

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
  std::unique_ptr<ImuError> imu_error;

  calib_vi_view_t() = default;
  calib_vi_view_t(const timestamp_t ts_,
                  ceres::Problem *problem_,
                  const CamIdx2Grids &grids_,
                  const mat4_t &T_WS,
                  const vec_t<9> &sb,
                  CamIdx2Geometry &cam_geoms_,
                  CamIdx2Parameters &cam_params_,
                  CamIdx2Extrinsics &cam_exts_,
                  extrinsics_t *imu_exts_,
                  fiducial_t *fiducial_,
                  PoseLocalParameterization *pose_plus);
  ~calib_vi_view_t() = default;

  std::vector<int> get_cam_indices() const;
  std::vector<real_t> get_reproj_errors(const int cam_idx) const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;
  std::vector<real_t> get_imu_errors() const;
  int filter_view(const real_t outlier_threshold);
  void form_imu_error(const imu_params_t &imu_params,
                      const imu_data_t &imu_buf,
                      pose_t *pose_j,
                      sb_params_t *sb_j);
};

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

struct calib_vi_t {
  // Settings
  bool verbose = true;
  double sigma_vision = 1.0;
  int batch_max_iter = 30;
  bool enable_outlier_rejection = true;
  double outlier_threshold = 3.0;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

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
  std::map<int, std::deque<aprilgrid_t>> grid_buf;
  CamIdx2Grids prev_grids;
  // -- Imu data
  imu_params_t imu_params;
  imu_data_t imu_buf;

  // Optimization
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  PoseLocalParameterization pose_plus;
  std::deque<calib_vi_view_t> calib_views;

  // Constructor
  calib_vi_t();
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
  veci2_t get_cam_resolution(const int cam_idx) const;
  vecx_t get_cam_params(const int cam_idx) const;
  mat4_t get_cam_extrinsics(const int cam_idx) const;
  mat4_t get_imu_extrinsics() const;
  mat4_t get_fiducial_pose() const;
  std::map<int, std::vector<real_t>> get_reproj_errors() const;

  mat4_t estimate_sensor_pose(const CamIdx2Grids &grids);
  void initialize(const CamIdx2Grids &grids, imu_data_t &imu_buf);
  void add_view(const CamIdx2Grids &grids);
  void add_measurement(const int cam_idx, const aprilgrid_t &grid);
  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m);
  // int recover_calib_covar(matx_t &calib_covar);

  void solve();
  void show_results();
  int save_results(const std::string &save_path) const;
  void save_estimates(const std::string &dir_path) const;
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
