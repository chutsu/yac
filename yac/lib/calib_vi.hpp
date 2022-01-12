#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <mutex>

#include <ceres/ceres.h>

#include "util/util.hpp"
// #include "calib_data.hpp"
#include "calib_params.hpp"
#include "marg_error.hpp"

namespace yac {

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

  const timestamp_t ts_i_ = 0;
  const timestamp_t ts_j_ = 0;
  const camera_geometry_t *cam_geom_;
  const int cam_idx_;
  const int cam_res_[2] = {0, 0};
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_i_{0.0, 0.0};
  const vec2_t z_j_{0.0, 0.0};
  const vec2_t v_ij_{0.0, 0.0}; // pixel velocity: (z_j - z_i) / dt
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  fiducial_error_t(const timestamp_t &ts_i,
                   const timestamp_t &ts_j,
                   const camera_geometry_t *cam_geom,
                   const int cam_idx,
                   const int cam_res[2],
                   const int tag_id,
                   const int corner_idx,
                   const vec3_t &r_FFi,
                   const vec2_t &z_i,
                   const vec2_t &z_j,
                   const mat4_t &T_WF,
                   const mat2_t &covar);
  ~fiducial_error_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// FIDUCIAL WITH TIME DELAY ERROR //////////////////////////////////////////////

#if FIDUCIAL_PARAMS_SIZE == 2
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 2, 7, 7, 7, 8, 1> {
#elif FIDUCIAL_PARAMS_SIZE == 3
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 3, 7, 7, 7, 8, 1> {
#elif FIDUCIAL_PARAMS_SIZE == 7
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 8, 1> {
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts_i_ = 0;
  const timestamp_t ts_j_ = 0;
  const camera_geometry_t *cam_geom_;
  const int cam_idx_;
  const int cam_res_[2] = {0, 0};
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_i_{0.0, 0.0};
  const vec2_t z_j_{0.0, 0.0};
  const vec2_t v_ij_{0.0, 0.0}; // pixel velocity: (z_j - z_i) / dt
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  fiducial_td_error_t(const timestamp_t &ts_i,
                      const timestamp_t &ts_j,
                      const camera_geometry_t *cam_geom,
                      const int cam_idx,
                      const int cam_res[2],
                      const int tag_id,
                      const int corner_idx,
                      const vec3_t &r_FFi,
                      const vec2_t &z_i,
                      const vec2_t &z_j,
                      const mat4_t &T_WF,
                      const mat2_t &covar);
  ~fiducial_td_error_t() = default;

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
           const timestamp_t &t1) {
    imu_data_ = imu_data;
    imu_params_ = imu_params;
    t0_ = t0;
    t1_ = t1;
  }

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

// APRILGRID BUFFER ////////////////////////////////////////////////////////////

struct aprilgrid_buffer_t {
  int nb_cams = 0;
  std::map<int, std::deque<aprilgrid_t>> buf;

  aprilgrid_buffer_t() = default;
  std::map<int, std::deque<aprilgrid_t>> &data();
  void add(const int cam_idx, const aprilgrid_t &grid);
  bool ready() const;
  aprilgrids_t pop_front();
};

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

struct calib_vi_t {
  // Data
  aprilgrid_buffer_t grids_buf;
  std::map<int, aprilgrids_t> cam_grids;
  std::map<int, aprilgrid_t> grids_prev;
  imu_params_t imu_params;
  imu_data_t imu_buf;
  bool initialized = false;

  // State-Variables
  std::unique_ptr<fiducial_t> fiducial;
  std::unique_ptr<time_delay_t> time_delay;
  std::map<timestamp_t, std::unique_ptr<pose_t>> sensor_poses;
  std::map<timestamp_t, std::unique_ptr<sb_params_t>> speed_biases;
  std::map<int, const camera_geometry_t *> cam_geoms;
  std::map<int, std::unique_ptr<camera_params_t>> cam_params;
  std::map<int, std::unique_ptr<extrinsics_t>> cam_exts;
  std::unique_ptr<extrinsics_t> imu_exts;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Settings
  double sigma_vision = 1.0;
  int batch_max_iter = 30;
  bool enable_outlier_rejection = true;

  // Optimization
  ceres::Problem::Options prob_options;
  std::unique_ptr<ceres::Problem> problem;
  PoseLocalParameterization pose_plus;

  calib_vi_t();
  ~calib_vi_t() = default;

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
                  const bool fix_params = false,
                  const bool fix_extrinsics = false);
  void add_sensor_pose(const timestamp_t ts, const mat4_t &T_WS);
  void add_speed_biases(const timestamp_t ts, const vec_t<9> &sb);
  void add_fiducial_pose(const mat4_t &T_WF);

  int nb_cams();
  vecx_t get_cam_params(const int cam_idx);
  mat4_t get_cam_extrinsics(const int cam_idx);
  mat4_t get_imu_extrinsics();
  mat4_t get_sensor_pose(const int pose_index);
  mat4_t get_fiducial_pose();

  void trim_imu_data(imu_data_t &imu_data, const timestamp_t t1);
  void update_prev_grids(const aprilgrids_t &grids);
  ImuError *add_imu_error(ceres::ResidualBlockId &error_id);
  // void add_reproj_errors(
  //     const int cam_idx,
  //     const aprilgrid_t &grid_j,
  //     std::map<int, std::vector<ceres::ResidualBlockId>> &error_ids,
  //     std::map<int, std::vector<fiducial_td_error_t<pinhole_radtan4_t> *>>
  //         &errors);

  bool fiducial_detected(const aprilgrids_t &grids);
  mat4_t estimate_sensor_pose(const aprilgrids_t &grids);
  void initialize(const timestamp_t &ts,
                  const aprilgrids_t &grids,
                  imu_data_t &imu_buf);
  void add_state(const timestamp_t &ts, const aprilgrids_t &grids);

  void add_measurement(const int cam_idx, const aprilgrid_t &grid);
  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m);
  std::map<int, std::vector<double>> get_camera_errors();

  // void solve(bool verbose = true);
  // void show_results();
  // int recover_calib_covar(matx_t &calib_covar);

  int save_results(const std::string &save_path);
  // void save_poses(const std::string &save_path);
  // void save_speed_biases(const std::string &save_path);
  // void save_cameras(const std::string &save_path);
  // void save_cam_extrinsics(const std::string &save_path);
  // // void save_imu_extrinsics(const std::string &save_path);
  // void save();
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
