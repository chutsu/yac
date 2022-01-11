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

// /* Form initial poses for visual-inertial calibration */
// void calib_vi_init_poses(const calib_target_t &target,
//                          const mat4_t &T_FO,
//                          std::deque<mat4_t> &init_poses);

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

// template <typename CAMERA_TYPE>
// #if FIDUCIAL_PARAMS_SIZE == 2
// struct fiducial_td_error_t
//     : public ceres::SizedCostFunction<2, 2, 7, 7, 7, 8, 1> {
// #elif FIDUCIAL_PARAMS_SIZE == 3
// struct fiducial_td_error_t
//     : public ceres::SizedCostFunction<2, 3, 7, 7, 7, 8, 1> {
// #elif FIDUCIAL_PARAMS_SIZE == 7
// struct fiducial_td_error_t
//     : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 8, 1> {
// #endif
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//   const timestamp_t ts_i_ = 0;
//   const timestamp_t ts_j_ = 0;
//   const int cam_res_[2] = {0, 0};
//   const int tag_id_ = -1;
//   const int corner_idx_ = -1;
//   const vec3_t r_FFi_{0.0, 0.0, 0.0};
//   const vec2_t z_i_{0.0, 0.0};
//   const vec2_t z_j_{0.0, 0.0};
//   const vec2_t v_ij_{0.0, 0.0}; // pixel velocity: (z_j - z_i) / dt
//   const mat4_t T_WF_ = I(4);
//
//   const mat2_t covar_;
//   const mat2_t info_;
//   const mat2_t sqrt_info_;
//
//   fiducial_td_error_t(const timestamp_t &ts_i,
//                     const timestamp_t &ts_j,
//                     const int cam_res[2],
//                     const int tag_id,
//                     const int corner_idx,
//                     const vec3_t &r_FFi,
//                     const vec2_t &z_i,
//                     const vec2_t &z_j,
//                     const mat4_t &T_WF,
//                     const mat2_t &covar)
//       : ts_i_{ts_i}, ts_j_{ts_j}, cam_res_{cam_res[0], cam_res[1]},
//         tag_id_{tag_id}, corner_idx_{corner_idx}, r_FFi_{r_FFi}, z_i_{z_i},
//         z_j_{z_j}, v_ij_{(z_j_ - z_i_) / ns2sec(ts_j_ - ts_i_)}, T_WF_{T_WF},
//         covar_{covar}, info_{covar.inverse()},
//         sqrt_info_{info_.llt().matrixL().transpose()} {
//     assert(ts_i != ts_j && ts_j > ts_i);
//     assert(cam_res[0] != 0 && cam_res[1] != 0);
//     assert(tag_id_ >= 0);
//     assert(corner_idx_ >= 0);
//   }
//
//   ~fiducial_td_error_t() {}
//
//   bool Evaluate(double const *const *params,
//                 double *residuals,
//                 double **jacobians) const {
//     // Pose of fiducial target in world frame
// #if FIDUCIAL_PARAMS_SIZE == 2
//     const double roll = params[0][0];
//     const double pitch = params[0][1];
//     const double yaw = quat2euler(tf_quat(T_WF_))(2);
//     const vec3_t rpy{roll, pitch, yaw};
//     const mat3_t C_WF = euler321(rpy);
//     const vec3_t r_WF = tf_trans(T_WF_);
//     const mat4_t T_WF = tf(C_WF, r_WF);
// #elif FIDUCIAL_PARAMS_SIZE == 3
//     const double roll = params[0][0];
//     const double pitch = params[0][1];
//     const double yaw = params[0][2];
//     const vec3_t rpy{roll, pitch, yaw};
//     const mat3_t C_WF = euler321(rpy);
//     const vec3_t r_WF = tf_trans(T_WF_);
//     const mat4_t T_WF = tf(C_WF, r_WF);
// #elif FIDUCIAL_PARAMS_SIZE == 7
//     const mat4_t T_WF = tf(params[0]);
// #endif
//
//     // Sensor pose, sensor-camera extrinsics, camera parameters
//     const mat4_t T_WS = tf(params[1]);
//     const mat4_t T_BS = tf(params[2]);
//     const mat4_t T_BCi = tf(params[3]);
//     Eigen::Map<const vecx_t> cam_params(params[4], 8);
//     const double td = params[5][0];
//
//     // Transform and project point to image plane
//     bool valid = true;
//     CAMERA_TYPE camera{cam_res_, cam_params};
//     const mat4_t T_CiB = T_BCi.inverse();
//     const mat4_t T_SW = T_WS.inverse();
//     const vec3_t r_CFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
//     mat_t<2, 3> Jh;
//     vec2_t z_hat;
//     if (camera.project(r_CFi, z_hat, Jh) != 0) {
//       valid = false;
//     }
//
//     // Residuals
//     const vec2_t r = sqrt_info_ * ((z_i_ + td * v_ij_) - z_hat);
//     residuals[0] = r(0);
//     residuals[1] = r(1);
//
//     // Jacobians
//     const matx_t Jh_weighted = sqrt_info_ * Jh;
//
//     if (jacobians) {
//       // Jacobians w.r.t. T_WF
//       if (jacobians[0]) {
//         // Form jacobians
// #if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
//         const double cphi = cos(roll);
//         const double sphi = sin(roll);
//         const double ctheta = cos(pitch);
//         const double stheta = sin(pitch);
//         const double cpsi = cos(yaw);
//         const double spsi = sin(yaw);
//
//         const double x = r_FFi_(0);
//         const double y = r_FFi_(1);
//         const double z = r_FFi_(2);
//
//         const vec3_t J_x{y * (sphi * spsi + stheta * cphi * cpsi) +
//                              z * (-sphi * stheta * cpsi + spsi * cphi),
//                          y * (-sphi * cpsi + spsi * stheta * cphi) +
//                              z * (-sphi * spsi * stheta - cphi * cpsi),
//                          y * cphi * ctheta - z * sphi * ctheta};
//         const vec3_t J_y{-x * stheta * cpsi + y * sphi * cpsi * ctheta +
//                              z * cphi * cpsi * ctheta,
//                          -x * spsi * stheta + y * sphi * spsi * ctheta +
//                              z * spsi * cphi * ctheta,
//                          -x * ctheta - y * sphi * stheta - z * stheta *
//                          cphi};
//         const vec3_t J_z{-x * spsi * ctheta +
//                              y * (-sphi * spsi * stheta - cphi * cpsi) +
//                              z * (sphi * cpsi - spsi * stheta * cphi),
//                          x * cpsi * ctheta +
//                              y * (sphi * stheta * cpsi - spsi * cphi) +
//                              z * (sphi * spsi + stheta * cphi * cpsi),
//                          0};
// #endif
//
//         // Fill Jacobian
//         const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
// #if FIDUCIAL_PARAMS_SIZE == 2
//         Eigen::Map<mat_t<2, 2, row_major_t>> J(jacobians[0]);
//         J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
//         J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
// #elif FIDUCIAL_PARAMS_SIZE == 3
//         Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[0]);
//         J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
//         J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
//         J.block(0, 2, 2, 1) = -1 * Jh_weighted * C_CiW * J_z;
// #elif FIDUCIAL_PARAMS_SIZE == 7
//         const mat3_t C_WF = tf_rot(T_WF);
//         Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
//         J.setZero();
//         J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiW * I(3);
//         J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiW * -skew(C_WF *
//         r_FFi_);
// #endif
//         if (valid == false) {
//           J.setZero();
//         }
//       }
//
//       // Jacobians w.r.t T_WS
//       if (jacobians[1]) {
//         const mat3_t C_CiS = tf_rot(T_CiB * T_BS);
//         const mat3_t C_WS = tf_rot(T_WS);
//         const mat3_t C_SW = C_WS.transpose();
//         const mat3_t C_CiW = C_CiS * C_SW;
//         const mat4_t T_SW = T_WS.inverse();
//         const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);
//
//         Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
//         J.setZero();
//         J.block(0, 0, 2, 3) = Jh_weighted * C_CiW * I(3);
//         J.block(0, 3, 2, 3) = Jh_weighted * C_CiW * -skew(C_WS * r_SFi);
//         if (valid == false) {
//           J.setZero();
//         }
//       }
//
//       // Jacobians w.r.t T_BS
//       if (jacobians[2]) {
//         const mat3_t C_CiB = tf_rot(T_CiB);
//         const mat3_t C_BS = tf_rot(T_BS);
//         const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);
//
//         Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
//         J.setZero();
//         J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
//         J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);
//         if (valid == false) {
//           J.setZero();
//         }
//       }
//
//       // Jacobians w.r.t T_BCi
//       if (jacobians[3]) {
//         const mat3_t C_CiB = tf_rot(T_CiB);
//         const mat3_t C_BCi = C_CiB.transpose();
//         const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
//
//         Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[3]);
//         J.setZero();
//         J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiB * I(3);
//         J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiB * -skew(C_BCi *
//         r_CiFi); if (valid == false) {
//           J.setZero();
//         }
//       }
//
//       // Jacobians w.r.t. camera parameters
//       if (jacobians[4]) {
//         const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
//
//         Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[4]);
//         J.block(0, 0, 2, 8) = -1 * sqrt_info_ * camera.J_params(p);
//         if (valid == false) {
//           J.setZero();
//         }
//       }
//
//       // Jacobians w.r.t. time delay
//       if (jacobians[5]) {
//         Eigen::Map<vec2_t> J(jacobians[5]);
//         J = sqrt_info_ * v_ij_;
//         if (valid == false) {
//           J.setZero();
//         }
//       }
//     }
//
//     return true;
//   }
// };

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
  imu_data_t imu_buf;
  bool initialized = false;

  // State-Variables
  fiducial_t *fiducial = nullptr;
  time_delay_t *time_delay = nullptr;
  std::vector<pose_t *> sensor_poses;
  std::vector<sb_params_t *> speed_biases;
  std::map<int, camera_params_t *> cam_params;
  std::map<int, extrinsics_t *> cam_exts;

  // Optimization
  imu_params_t imu_params;
  double sigma_vision = 1.0;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_parameterization;

  int batch_max_iter = 30;
  bool enable_outlier_rejection = true;

  calib_vi_t() {
    // clang-format off
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = new ceres::Problem(prob_options);
    // clang-format on
  }

  ~calib_vi_t() {
    if (fiducial) {
      delete fiducial;
    }
    if (time_delay) {
      delete time_delay;
    }
    for (const auto &pose : sensor_poses) {
      delete pose;
    }
    for (const auto &sb : speed_biases) {
      delete sb;
    }
    for (const auto &kv : cam_params) {
      delete kv.second;
    }
    for (const auto &kv : cam_exts) {
      delete kv.second;
    }
    if (problem) {
      delete problem;
    }
  }

  void add_imu(const imu_params_t &imu_params_, const double td = 0.0);
  void add_camera(const int cam_idx,
                  const int resolution[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const bool fix = false);
  void add_cam_extrinsics(const int cam_idx,
                          const mat4_t &T_BCi,
                          const bool fix = false);
  void add_imu_extrinsics(const mat4_t &T_BS, const bool fix = false);
  void add_sensor_pose(const timestamp_t ts, const mat4_t &T_WS);
  void add_speed_biases(const timestamp_t ts, const vec_t<9> &sb);
  void add_time_delay(const double td);
  void add_fiducial_pose(const mat4_t &T_WF);

  vecx_t get_camera(const int cam_idx);
  mat4_t get_cam_extrinsics(const int cam_idx);
  mat4_t get_sensor_pose(const int pose_index);
  mat4_t get_fiducial_pose();
  int nb_cams();

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

  // clang-format off
  int save_results(const std::string &save_path);
  void save_poses(const std::string &save_path, const std::vector<pose_t *> &poses);
  void save_speed_biases(const std::string &save_path, const std::vector<sb_params_t *> &speed_biases);
  void save_cameras(const std::string &save_path, const std::map<int, camera_params_t *> &cam_params);
  void save_cam_extrinsics(const std::string &save_path);
  // void save_imu_extrinsics(const std::string &save_path);
  void save();
  // clang-format on
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
