#ifndef YAC_IMU_ERROR_HPP
#define YAC_IMU_ERROR_HPP

#include <mutex>

#include <ceres/ceres.h>

#include "core.hpp"
#include "ceres_utils.hpp"
#include "calib_params.hpp"

namespace yac {

#define EST_TIMEDELAY 0

/**
 * Implements a nonlinear IMU factor.
 */
class ImuError
#if EST_TIMEDELAY == 1
    : public ::ceres::SizedCostFunction<
          15 /* number of residuals */,
          7 /* size of first parameter (PoseParameterBlock k) */,
          9 /* size of second parameter (SpeedAndBiasParameterBlock k) */,
          7 /* size of third parameter (PoseParameterBlock k+1) */,
          9 /* size of fourth parameter (SpeedAndBiasParameterBlock k+1) */,
          1 /* size of fifth parameter (TimeDelayParameeterBlock) */> {
#else
    : public ::ceres::SizedCostFunction<
          15 /* number of residuals */,
          7 /* size of first parameter (PoseParameterBlock k) */,
          9 /* size of second parameter (SpeedAndBiasParameterBlock k) */,
          7 /* size of third parameter (PoseParameterBlock k+1) */,
          9 /* size of fourth parameter (SpeedAndBiasParameterBlock k+1) */> {
#endif
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // For debugging
  bool debug_jacobians_ = false;

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
  mutable double time_delay_last_ = 0.0;

  ///< The number of residuals
  typedef Eigen::Matrix<double, 15, 15> covariance_t;
  typedef covariance_t information_t;
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;
  typedef Eigen::Matrix<double, 15, 7> jacobian0_t;
  typedef Eigen::Matrix<double, 15, 9> jacobian1_t;

  ImuError();
  ImuError(const imu_data_t &imu_data,
           const imu_params_t &imu_params,
           const timestamp_t &t_0,
           const timestamp_t &t_1);
  virtual ~ImuError();

  static int propagation(const imu_data_t &imu_data,
                         const imu_params_t &imu_params,
                         mat4_t &T_WS,
                         vec_t<9> &sb,
                         const timestamp_t &t_start,
                         const timestamp_t &t_end,
                         covariance_t *covariance = 0,
                         jacobian_t *jacobian = 0);

  int redoPreintegration(const mat4_t &T_WS,
                         const vec_t<9> &sb,
                         timestamp_t time_start,
                         timestamp_t time_end) const;

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  bool EvaluateWithMinimalJacobians(double const *const *parameters,
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
  mutable Eigen::Matrix<double, 15, 15> P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

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

ImuError::ImuError() {}

ImuError::ImuError(const imu_data_t &imu_data,
                   const imu_params_t &imu_params,
                   const timestamp_t &t0,
                   const timestamp_t &t1) {
  imu_data_ = imu_data;
  imu_params_ = imu_params;
  t0_ = t0;
  t1_ = t1;
}

ImuError::~ImuError() {}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::redoPreintegration(
    const mat4_t & /*T_WS*/,
    const vec_t<9> &sb,
    timestamp_t time,
    timestamp_t end) const {
  // Ensure unique access
  std::lock_guard<std::mutex> lock(preintegrationMutex_);

  // Sanity check:
  assert(imu_data_.timestamps.front() <= time);
  if (!(imu_data_.timestamps.back() >= end))
    return -1; // nothing to do...

  // Increments (initialise with identity)
  Delta_q_ = quat_t(1, 0, 0, 0);
  C_integral_ = mat3_t::Zero();
  C_doubleintegral_ = mat3_t::Zero();
  acc_integral_ = vec3_t::Zero();
  acc_doubleintegral_ = vec3_t::Zero();

  // Cross matrix accumulatrion
  cross_ = mat3_t::Zero();

  // Sub-Jacobians
  dalpha_db_g_ = mat3_t::Zero();
  dv_db_g_ = mat3_t::Zero();
  dp_db_g_ = mat3_t::Zero();

  // Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
	for (size_t k = 0; k < imu_data_.size(); k++) {
		vec3_t omega_S_0 = imu_data_.gyro[k];
		vec3_t acc_S_0 = imu_data_.accel[k];
		vec3_t omega_S_1 = imu_data_.gyro[k + 1];
		vec3_t acc_S_1 = imu_data_.accel[k + 1];

    // Time delta
    timestamp_t nexttime;
    if ((k + 1) == imu_data_.size()) {
      nexttime = end;
    } else {
      nexttime = imu_data_.timestamps[k + 1];
		}
    if ((nexttime - time) < 0) {
      continue;
    }
    double dt = ns2sec(nexttime - time);

    // Interpolate IMU measurements that are beyond the image timestamp
    if (end < nexttime) {
      double interval = ns2sec(nexttime - imu_data_.timestamps[k]);
      nexttime = end;
      dt = ns2sec(nexttime - time);
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    // Check dt is larger than 0
    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    // Interpolate the first IMU measurements
    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(nexttime - imu_data_.timestamps[k]);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }
    // std::cout << "-- " << i << " [" << it->timeStamp << "]" << std::endl;

    // Ensure measurement integrity
    double sigma_g_c = imu_params_.sigma_g_c;
    double sigma_a_c = imu_params_.sigma_a_c;
    // if (fabs(omega_S_0[0]) > imu_params_.g_max ||
    //     fabs(omega_S_0[1]) > imu_params_.g_max ||
    //     fabs(omega_S_0[2]) > imu_params_.g_max ||
    //     fabs(omega_S_1[0]) > imu_params_.g_max ||
    //     fabs(omega_S_1[1]) > imu_params_.g_max ||
    //     fabs(omega_S_1[2]) > imu_params_.g_max) {
    //   sigma_g_c *= 100;
    //   LOG(WARNING) << "gyr saturation";
    // }
    // if (fabs(acc_S_0[0]) > imu_params_.a_max ||
    //     fabs(acc_S_0[1]) > imu_params_.a_max ||
    //     fabs(acc_S_0[2]) > imu_params_.a_max ||
    //     fabs(acc_S_1[0]) > imu_params_.a_max ||
    //     fabs(acc_S_1[1]) > imu_params_.a_max ||
    //     fabs(acc_S_1[2]) > imu_params_.a_max) {
    //   sigma_a_c *= 100;
    //   LOG(WARNING) << "acc saturation";
    // }

    // Actual propagation
    // clang-format off
    // -- Orientation:
    quat_t dq;
    const vec3_t omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - sb.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    quat_t Delta_q_1 = Delta_q_ * dq;
    // -- Rotation matrix integral:
    const mat3_t C = Delta_q_.toRotationMatrix();
    const mat3_t C_1 = Delta_q_1.toRotationMatrix();
    const vec3_t acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - sb.segment<3>(6));
    const mat3_t C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
    const vec3_t acc_integral_1 = acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt;
    // -- Rotation matrix double integral:
    C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral_ += acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;
    // clang-format on

    // Jacobian parts
    // clang-format off
    dalpha_db_g_ += C_1 * rightJacobian(omega_S_true * dt) * dt;
    const mat3_t cross_1 = dq.inverse().toRotationMatrix() * cross_ + rightJacobian(omega_S_true * dt) * dt;
    const mat3_t acc_S_x = skew(acc_S_true);
    mat3_t dv_db_g_1 = dv_db_g_ + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    dp_db_g_ += dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    // clang-format on

    // Covariance propagation
    // clang-format off
    Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
    // Transform
    F_delta.block<3, 3>(0, 3) = -skew(acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
    F_delta.block<3, 3>(0, 6) = I(3) * dt;
    F_delta.block<3, 3>(0, 9) = dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(0, 12) = -C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    F_delta.block<3, 3>(3, 9) = -dt * C_1;
    F_delta.block<3, 3>(6, 3) = -skew(0.5 * (C + C_1) * acc_S_true * dt);
    F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // Add noise. Note that transformations with rotation matrices can be
    // ignored, since the noise is isotropic.
    // F_tot = F_delta*F_tot;
    const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
    P_delta_(3, 3) += sigma2_dalpha;
    P_delta_(4, 4) += sigma2_dalpha;
    P_delta_(5, 5) += sigma2_dalpha;
    const double sigma2_v = dt * sigma_a_c * sigma_a_c;
    P_delta_(6, 6) += sigma2_v;
    P_delta_(7, 7) += sigma2_v;
    P_delta_(8, 8) += sigma2_v;
    const double sigma2_p = 0.5 * dt * dt * sigma2_v;
    P_delta_(0, 0) += sigma2_p;
    P_delta_(1, 1) += sigma2_p;
    P_delta_(2, 2) += sigma2_p;
    const double sigma2_b_g = dt * imu_params_.sigma_gw_c * imu_params_.sigma_gw_c;
    P_delta_(9, 9) += sigma2_b_g;
    P_delta_(10, 10) += sigma2_b_g;
    P_delta_(11, 11) += sigma2_b_g;
    const double sigma2_b_a = dt * imu_params_.sigma_aw_c * imu_params_.sigma_aw_c;
    P_delta_(12, 12) += sigma2_b_a;
    P_delta_(13, 13) += sigma2_b_a;
    P_delta_(14, 14) += sigma2_b_a;
    // clang-format on

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;
    cross_ = cross_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == end)
      break;
  }

  // store the reference (linearisation) point
  sb_ref_ = sb;

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();
  // proto::print_matrix("P_delta", P_delta_);

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();

  // std::cout << squareRootInformation_ << std::endl;
  // exit(0);

  return i;
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::propagation(const imu_data_t &imu_data,
                          const imu_params_t &imu_params,
                          mat4_t &T_WS,
                          vec_t<9> &sb,
                          const timestamp_t &t_start,
                          const timestamp_t &t_end,
                          covariance_t *covariance,
                          jacobian_t *jacobian) {
  // now the propagation
  // timestamp_t time = t_start;
  // timestamp_t end = t_end;
  // printf("time_start: [%ld]\n", time.toNSec());
  // printf("time_end:   [%ld]\n", end.toNSec());
  // printf("imu front:  [%ld]\n", imu_data.front().timeStamp.toNSec());
  // printf("imu back:   [%ld]\n", imu_data.back().timeStamp.toNSec());

  // sanity check:
  assert(imu_data.timestamps.front() <= t_start);
  assert(imu_data.timestamps.back() >= t_end);
  // if (!(imu_data.back().timeStamp >= end))
  //   return -1; // nothing to do...

  // initial condition
  vec3_t r_0 = tf_trans(T_WS);
  quat_t q_WS_0 = tf_quat(T_WS);
  mat3_t C_WS_0 = tf_rot(T_WS);

  // increments (initialise with identity)
  quat_t Delta_q(1, 0, 0, 0);
  mat3_t C_integral = mat3_t::Zero();
  mat3_t C_doubleintegral = mat3_t::Zero();
  vec3_t acc_integral = vec3_t::Zero();
  vec3_t acc_doubleintegral = vec3_t::Zero();

  // cross matrix accumulatrion
  mat3_t cross = mat3_t::Zero();

  // sub-Jacobians
  mat3_t dalpha_db_g = mat3_t::Zero();
  mat3_t dv_db_g = mat3_t::Zero();
  mat3_t dp_db_g = mat3_t::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double, 15, 15> P_delta = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  timestamp_t ts_k = t_start;
	for (size_t k = 0; k < (imu_data.size() - 1); k++) {
		vec3_t omega_S_0 = imu_data.gyro[k];
		vec3_t acc_S_0 = imu_data.accel[k];
		vec3_t omega_S_1 = imu_data.gyro[k + 1];
		vec3_t acc_S_1 = imu_data.accel[k + 1];
		// print_vector("omega_S_0", omega_S_0);
		// print_vector("omega_S_1", omega_S_1);
		// print_vector("acc_S_0", acc_S_0);
		// print_vector("acc_S_1", acc_S_1);

    // Time delta
    timestamp_t ts_kp1;
    if ((k + 1) == imu_data.size()) {
      ts_kp1 = t_end;
    } else {
      ts_kp1 = imu_data.timestamps[k + 1];
		}
    // printf("ts_kp1: %ld\n", ts_kp1);
    // printf("ts_k:     %ld\n", ts_k);
    // printf("diff:     %ld\n", ts_kp1 - ts_k);
    if ((ts_kp1 - ts_k) < 0) {
      continue;
    }
    double dt = ns2sec(ts_kp1 - ts_k);
    // printf("dt: %f\n", dt);

    // If next imu measurement timestamp is after t_end, interpolate gyro and
    // accel at t_end
    if (ts_kp1 > t_end) {
      double interval = ns2sec(ts_kp1 - imu_data.timestamps[k]);
      ts_kp1 = t_end;
      dt = ns2sec(ts_kp1 - ts_k);
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;
    // printf("Delta_t: %f\n", Delta_t);

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(ts_kp1 - ts_k);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_params.sigma_g_c;
    double sigma_a_c = imu_params.sigma_a_c;

    // if (fabs(omega_S_0[0]) > imuParams.g_max ||
    //     fabs(omega_S_0[1]) > imuParams.g_max ||
    //     fabs(omega_S_0[2]) > imuParams.g_max ||
    //     fabs(omega_S_1[0]) > imuParams.g_max ||
    //     fabs(omega_S_1[1]) > imuParams.g_max ||
    //     fabs(omega_S_1[2]) > imuParams.g_max) {
    //   sigma_g_c *= 100;
    //   LOG(WARNING) << "gyr saturation";
    // }
    //
    // if (fabs(acc_S_0[0]) > imuParams.a_max ||
    //     fabs(acc_S_0[1]) > imuParams.a_max ||
    //     fabs(acc_S_0[2]) > imuParams.a_max ||
    //     fabs(acc_S_1[0]) > imuParams.a_max ||
    //     fabs(acc_S_1[1]) > imuParams.a_max ||
    //     fabs(acc_S_1[2]) > imuParams.a_max) {
    //   sigma_a_c *= 100;
    //   LOG(WARNING) << "acc saturation";
    // }

    // actual propagation
    // clang-format off
    // orientation:
    quat_t dq;
    const vec3_t omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - sb.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    quat_t Delta_q_1 = Delta_q * dq;
    // rotation matrix integral:
    const mat3_t C = Delta_q.toRotationMatrix();
    const mat3_t C_1 = Delta_q_1.toRotationMatrix();
    const vec3_t acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - sb.segment<3>(6));
    const mat3_t C_integral_1 = C_integral + 0.5 * (C + C_1) * dt;
    const vec3_t acc_integral_1 = acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
    // rotation matrix double integral:
    C_doubleintegral += C_integral * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral += acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

    // Jacobian parts
    dalpha_db_g += dt * C_1;
    const mat3_t cross_1 = dq.inverse().toRotationMatrix() * cross + rightJacobian(omega_S_true * dt) * dt;
    const mat3_t acc_S_x = skew(acc_S_true);
    mat3_t dv_db_g_1 = dv_db_g + 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
    dp_db_g += dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
    // clang-format on

    // covariance propagation
    // clang-format off
    if (covariance) {
      Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
      // transform
      F_delta.block<3, 3>(0, 3) = -skew( acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
      F_delta.block<3, 3>(0, 6) = I(3) * dt;
      F_delta.block<3, 3>(0, 9) = dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(0, 12) = -C_integral * dt + 0.25 * (C + C_1) * dt * dt;
      F_delta.block<3, 3>(3, 9) = -dt * C_1;
      F_delta.block<3, 3>(6, 3) = -skew(0.5 * (C + C_1) * acc_S_true * dt);
      F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
      P_delta = F_delta * P_delta * F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      // F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(3, 3) += sigma2_dalpha;
      P_delta(4, 4) += sigma2_dalpha;
      P_delta(5, 5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * imu_params.sigma_a_c;
      P_delta(6, 6) += sigma2_v;
      P_delta(7, 7) += sigma2_v;
      P_delta(8, 8) += sigma2_v;
      const double sigma2_p = 0.5 * dt * dt * sigma2_v;
      P_delta(0, 0) += sigma2_p;
      P_delta(1, 1) += sigma2_p;
      P_delta(2, 2) += sigma2_p;
      const double sigma2_b_g = dt * imu_params.sigma_gw_c * imu_params.sigma_gw_c;
      P_delta(9, 9) += sigma2_b_g;
      P_delta(10, 10) += sigma2_b_g;
      P_delta(11, 11) += sigma2_b_g;
      const double sigma2_b_a = dt * imu_params.sigma_aw_c * imu_params.sigma_aw_c;
      P_delta(12, 12) += sigma2_b_a;
      P_delta(13, 13) += sigma2_b_a;
      P_delta(14, 14) += sigma2_b_a;
    }
    // clang-format on

    // memory shift
    Delta_q = Delta_q_1;
    C_integral = C_integral_1;
    acc_integral = acc_integral_1;
    cross = cross_1;
    dv_db_g = dv_db_g_1;
    ts_k = ts_kp1;

    ++i;

    if (ts_kp1 == t_end)
      break;
  }
  // printf("Delta_t: %f\n", Delta_t);
  // print_vector("acc_doubleintegral", acc_doubleintegral);

  // actual propagation output:
  const vec3_t g_W = imu_params.g * vec3_t(0, 0, 6371009).normalized();
	const vec3_t trans_update = r_0 + sb.head<3>() * Delta_t + C_WS_0 * (acc_doubleintegral /*-C_doubleintegral*sb.segment<3>(6)*/) - 0.5 * g_W * Delta_t * Delta_t;
	const quat_t quat_update = q_WS_0 * Delta_q;
	T_WS = tf(quat_update, trans_update);
  sb.head<3>() += C_WS_0 * (acc_integral /*-C_integral*sb.segment<3>(6)*/) - g_W * Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double, 15, 15> &F = *jacobian;
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g, d/db_a
    F.block<3, 3>(0, 3) =
        -skew(C_WS_0 * acc_doubleintegral);
    F.block<3, 3>(0, 6) = I(3) * Delta_t;
    F.block<3, 3>(0, 9) = C_WS_0 * dp_db_g;
    F.block<3, 3>(0, 12) = -C_WS_0 * C_doubleintegral;
    F.block<3, 3>(3, 9) = -C_WS_0 * dalpha_db_g;
    F.block<3, 3>(6, 3) = -skew(C_WS_0 * acc_integral);
    F.block<3, 3>(6, 9) = C_WS_0 * dv_db_g;
    F.block<3, 3>(6, 12) = -C_WS_0 * C_integral;
  }

  // overall covariance, if requested
  if (covariance) {
    Eigen::Matrix<double, 15, 15> &P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double, 15, 15> T = Eigen::Matrix<double, 15, 15>::Identity();
    T.topLeftCorner<3, 3>() = C_WS_0;
    T.block<3, 3>(3, 3) = C_WS_0;
    T.block<3, 3>(6, 6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }

  return i;
}

// This evaluates the error term and additionally computes the Jacobians.
bool ImuError::Evaluate(double const *const *parameters,
                         double *residuals,
                         double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool ImuError::EvaluateWithMinimalJacobians(double const *const *params,
                                            double *residuals,
                                            double **jacobians,
                                            double **jacobiansMinimal) const {
  // Map T_WS_0
  const vec3_t r_WS_0{params[0][0], params[0][1], params[0][2]};
	const quat_t q_WS_0{params[0][6], params[0][3], params[0][4], params[0][5]};
  const mat4_t T_WS_0 = tf(q_WS_0, r_WS_0);

  // Map T_WS_1
  const vec3_t r_WS_1{params[2][0], params[2][1], params[2][2]};
	const quat_t q_WS_1{params[2][6], params[2][3], params[2][4], params[2][5]};
  const mat4_t T_WS_1 = tf(q_WS_1, r_WS_1);

  // Map speed and bias
  vec_t<9> sb0;
  vec_t<9> sb1;
  for (size_t i = 0; i < 9; ++i) {
    sb0(i) = params[1][i];
    sb1(i) = params[3][i];
  }

  // Time delay
#if EST_TIMEDELAY == 1
  const double time_delay = params[4][0];
  time_delay_last_ = time_delay;
#endif

  // Keep track of evaulation values
  T_WS_0_last_ = T_WS_0;
  T_WS_1_last_ = T_WS_1;
  sb0_last_ = sb0;
  sb1_last_ = sb1;

  // Modify time_start and time_end with timedelay if performing numdiff
#if EST_TIMEDELAY == 1
  timestamp_t time_start = t0_;
  timestamp_t time_end = t1_;
  // -- Modify t0 and t1
  const timestamp_t delay_nsec = time_delay * 1e9;
  const timestamp_t t_start_nsec = time_start;
  const timestamp_t t_end_nsec = time_end;
  time_start = t_start_nsec + delay_nsec;
  time_end = t_end_nsec + delay_nsec;
  // -- Bound times to prevent being before or after imu measurements
  if (time_start < imu_data_.timestamps.front()) {
    time_start = imu_data_.timestamps.front();
  }
  if (time_end > imu_data_.timestamps.back()) {
    time_end = imu_data_.timestamps.back();
  }
#else
  timestamp_t time_start = t0_;
  timestamp_t time_end = t1_;
#endif

  // this will NOT be changed:
  const mat3_t C_WS_0 = tf_rot(T_WS_0);
  const mat3_t C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double Delta_t = ns2sec(t1_ - t0_);
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_);
    Delta_b = sb0.tail<6>() - sb_ref_.tail<6>();
  }
  redoPreintegration(T_WS_0, sb0, time_start, time_end);
  Delta_b.setZero();
  // printf("nb imu measurements: %zu\n", imu_data_.size());
  // std::cout << "imu front ts: " << imu_data_.front().timeStamp.toNSec() << std::endl;
  // std::cout << "imu back ts:  " << imu_data_.back().timeStamp.toNSec() << std::endl;
  // std::cout << "time_start:   " << time_start.toNSec() << std::endl;
  // std::cout << "time_end:     " << time_end.toNSec() << std::endl;
  // std::cout << "sb0: " << sb0.transpose() << std::endl;
  // std::cout << "sb1: " << sb1.transpose() << std::endl;
  // redo_ = redo_ || (Delta_b.head<3>().norm() * Delta_t > 0.0001);
  // if (redo_) {
  //   redoPreintegration(T_WS_0, sb0, time_start, time_end);
  //   redoCounter_++;
  //   Delta_b.setZero();
  //   redo_ = false;
  // }

  // Actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_); // this is a bit
                                                            // stupid, but
                                                            // shared read-locks
                                                            // only come in
                                                            // C++14
                                                            // clang-format off
    const vec3_t g_W = imu_params_.g * vec3_t(0, 0, 6371009).normalized();
                                                            // clang-format on

    // Assign Jacobian w.r.t. x0
    // clang-format off
    Eigen::Matrix<double, 15, 15> F0 = Eigen::Matrix<double, 15, 15>::Identity(); // holds for d/db_g, d/db_a
    const vec3_t delta_p_est_W = tf_trans(T_WS_0) - tf_trans(T_WS_1) + sb0.head<3>() * Delta_t - 0.5 * g_W * Delta_t * Delta_t;
    const vec3_t delta_v_est_W = sb0.head<3>() - sb1.head<3>() - g_W * Delta_t;
    const quat_t Dq = deltaQ(-dalpha_db_g_ * Delta_b.head<3>()) * Delta_q_;
    F0.block<3, 3>(0, 0) = C_S0_W;
    F0.block<3, 3>(0, 3) = C_S0_W * skew(delta_p_est_W);
    F0.block<3, 3>(0, 6) = C_S0_W * I(3) * Delta_t;
    F0.block<3, 3>(0, 9) = dp_db_g_;
    F0.block<3, 3>(0, 12) = -C_doubleintegral_;
    F0.block<3, 3>(3, 3) = (plus(Dq * tf_quat(T_WS_1).inverse()) * oplus(tf_quat(T_WS_0))) .topLeftCorner<3, 3>();
    F0.block<3, 3>(3, 9) = (oplus(tf_quat(T_WS_1).inverse() * tf_quat(T_WS_0)) * oplus(Dq)) .topLeftCorner<3, 3>() * (-dalpha_db_g_);
    F0.block<3, 3>(6, 3) = C_S0_W * skew(delta_v_est_W);
    F0.block<3, 3>(6, 6) = C_S0_W;
    F0.block<3, 3>(6, 9) = dv_db_g_;
    F0.block<3, 3>(6, 12) = -C_integral_;
    // clang-format on

    // assign Jacobian w.r.t. x1
    // clang-format off
    Eigen::Matrix<double, 15, 15> F1 = -Eigen::Matrix<double, 15, 15>::Identity(); // holds for the biases
    F1.block<3, 3>(0, 0) = -C_S0_W;
    F1.block<3, 3>(3, 3) = -(plus(Dq) * oplus(tf_quat(T_WS_0)) * plus(tf_quat(T_WS_1).inverse())) .topLeftCorner<3, 3>();
    F1.block<3, 3>(6, 6) = -C_S0_W;
    // clang-format on

    // Overall error vector
    // clang-format off
    Eigen::Matrix<double, 15, 1> error;
    // -- Position
    error.segment<3>(0) = C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3, 6>(0, 9) * Delta_b;
    // -- Orientation
    error.segment<3>(3) = 2 * (Dq * (tf_quat(T_WS_1).inverse() * tf_quat(T_WS_0))) .vec(); // 2*T_WS_0.q()*Dq*T_WS_1.q().inverse();//
    // -- Velocity
    error.segment<3>(6) = C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3, 6>(6, 9) * Delta_b;
    // -- Biases
    error.tail<6>() = sb0.tail<6>() - sb1.tail<6>();
    // clang-format on

    // FATAL("SOMETHING IS WRONG HERE!");
    // squareRootInformation_  = I(15);

    // Error weighting
    // clang-format off
    if (residuals) {
      Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error(residuals);
      weighted_error = squareRootInformation_ * error;
      // print_vector("err", error);
      // r = weighted_error;
      // print_matrix("sqrt_info", squareRootInformation_);
      // exit(0);

      // if (std::isnan(weighted_error(0))) {
      //   printf("pose0_id: %zu\n", pose0_id);
      //   printf("pose1_id: %zu\n", pose1_id);
      //   printf("state0_id: %zu\n", state0_id);
      //   printf("state1_id: %zu\n", state1_id);
      //   printf("timedelay: %zu\n", timedelay_id);
      //   for (const auto &data : imu_data_ ){
      //     std::cout << "gyr: " << data.measurement.gyroscopes.transpose() << std::endl;
      //     std::cout << "acc: " << data.measurement.accelerometers.transpose() << std::endl;
      //   }
      //   std::cout << "residual: " << weighted_error.transpose() << std::endl;
      // }

      // Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error_debug(residuals_debug);
      // weighted_error_debug = squareRootInformation_ * error;
    }
    // clang-format on
    // printf("residual norm: %f\n", (squareRootInformation_ * error).norm());

    // Get the Jacobians
    if (jacobians != NULL) {
      // Sensor pose 0
      // clang-format off
      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_ * F0.block<15, 6>(0, 0);
        // J[0] = J0_minimal;

        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
				J0.block(0, 0, 15, 6) = J0_minimal;

        // // pseudo inverse of the local parametrization Jacobian:
        // Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        // PoseLocalParameterization::liftJacobian(params[0], J_lift.data());
        //
        // // hallucinate Jacobian w.r.t. state
        // Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
        // J0 = J0_minimal * J_lift;

        if (debug_jacobians_) {
          std::cout << "J0:" << std::endl;
          std::cout << J0 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[0] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J0_minimal_mapped(jacobiansMinimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      // clang-format on

      // Speed and Bias 0
      if (jacobians[1] != NULL) {
        // clang-format off
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1 = squareRootInformation_ * F0.block<15, 9>(0, 6);
        // J[1] = J1;

        if (debug_jacobians_) {
          std::cout << "J1:" << std::endl;
          std::cout << J1 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1_minimal_mapped(jacobiansMinimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }
      // clang-format on

      // Sensor pose 1
      // clang-format off
      if (jacobians[2] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J2_minimal = squareRootInformation_ * F1.block<15, 6>(0, 0);

        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J2(jacobians[2]);
        J2.setZero();
				J2.block(0, 0, 15, 6) = J2_minimal;

        // pseudo inverse of the local parametrization Jacobian:
        // Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        // PoseLocalParameterization::liftJacobian(params[2], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        // Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J2(jacobians[2]);
        // J2 = J2_minimal * J_lift;

        if (debug_jacobians_) {
          std::cout << "J2:" << std::endl;
          std::cout << J2 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[2] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J2_minimal_mapped(jacobiansMinimal[2]);
            J2_minimal_mapped = J2_minimal;
          }
        }
      }
      // clang-format on

      // Speed and Bias 1
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3(jacobians[3]);
        J3.setZero();
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);

        if (debug_jacobians_) {
          std::cout << "J3:" << std::endl;
          std::cout << J3 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3_minimal_mapped(jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }

      // Time delay between imu and camera
#if EST_TIMEDELAY == 1
      if (jacobians[4] != NULL) {
        // Calculate numerical differentiation: Basically we are going to
        // resuse the ImuError class and find the jacobians for time_delay.
        // We do this by performing a central difference numerical diff.
        //
        //   (f(time_delay + h) - f(time_delay - h)) / (2 * h)
        //
        // Where f(.) is the residual function, in this case the IMU error.
        // time_delay is the input parameter and h is the num-diff step.
        //
        // In future obviously it would be nice to have a analytical diff, but
        // for now this will have to do.

        // -- Variable used to perturb time_delay (a.k.a num-diff step 'h')
        // Same as the delta in Map::isJacobianCorrect()
        const double delta = 1.0e-8;

        // -- Form parameters
        // clang-format off
        // ---- Sensor poses
        const vec3_t r_WS_0 = tf_trans(T_WS_0);
        const quat_t q_WS_0 = tf_quat(T_WS_0);
        double T_WS_0_data[7] = {r_WS_0(0), r_WS_0(1), r_WS_0(2),
                                 q_WS_0.x(), q_WS_0.y(), q_WS_0.z(), q_WS_0.w()};
        const vec3_t r_WS_1 = tf_trans(T_WS_1);
        const quat_t q_WS_1 = tf_quat(T_WS_1);
        double T_WS_1_data[7] = {r_WS_1(0), r_WS_1(1), r_WS_1(2),
                                 q_WS_1.x(), q_WS_1.y(), q_WS_1.z(), q_WS_1.w()};

        // ---- Speed and biases
        double *speed_bias_0_data = sb0.data();
        double *speed_bias_1_data = sb1.data();

        // ---- Time delays
        const double time_delay_0 = time_delay + delta;
        const double time_delay_1 = time_delay - delta;

        // ---- Params
        // clang-format on
        const double *imu0_params[5] = {T_WS_0_data, speed_bias_0_data, T_WS_1_data, speed_bias_1_data, &time_delay_0};
        const double *imu1_params[5] = {T_WS_0_data, speed_bias_0_data, T_WS_1_data, speed_bias_1_data, &time_delay_1};

        // -- Form IMU errors and evaluate jacobians
        double imu0_residuals[15] = {0};
        double imu1_residuals[15] = {0};
        const ImuError imu0{imu_data_, imu_params_, t0_, t1_};
        const ImuError imu1{imu_data_, imu_params_, t0_, t1_};
        if (imu0.Evaluate(imu0_params, imu0_residuals, nullptr) == false) {
          // eval = false;
          return false;
        }
        if (imu1.Evaluate(imu1_params, imu1_residuals, nullptr) == false) {
          // eval = false;
          return false;
        }

        // -- Perform actual numerical differentiation
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residuals_0(imu0_residuals);
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residuals_1(imu1_residuals);
        const auto J_time_delay = (residuals_1 - residuals_0) * (1 / (2.0 * delta));

        // -- Set time delay jacobian
        Eigen::Map<Eigen::Matrix<double, 15, 1>> J4(jacobians[4]);
        J4 = -1 * J_time_delay;

        if (debug_jacobians_) {
          std::cout << "J4:" << std::endl;
          std::cout << J4 << std::endl;
        }

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[4] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 1>> J4_minimal_mapped(
                jacobiansMinimal[4]);
            J4_minimal_mapped = J4;
          }
        }

      } // jacobian[4]
#endif // if EST_TIMEDELAY == 1

    }   // jacobian
  }     // mutex

  // eval = true;
  return true;
}

} // namespace yac
#endif // YAC_IMU_ERROR_HPP
