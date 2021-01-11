/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#ifndef YAC_IMU_ERROR2_HPP
#define YAC_IMU_ERROR2_HPP

#include <vector>
#include <mutex>
#include <atomic>
#include <ceres/ceres.h>

#include "core.hpp"
#include "ceres_utils.hpp"
#include "calib_params.hpp"

namespace yac {

template<typename VALUE_T>
using AlignedVector = std::vector<VALUE_T, Eigen::aligned_allocator<VALUE_T>>;

/// \brief Implements a nonlinear IMU factor.
class ImuError : public ::ceres::SizedCostFunction<15, 7, 9, 7, 9> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static std::atomic_bool redoPropagationAlways;

  typedef ::ceres::SizedCostFunction<15, 7, 9, 7, 9> base_t;
  static const int kNumResiduals = 15;
  typedef Eigen::Matrix<double, 15, 15> covariance_t;
  typedef covariance_t information_t;
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;
  typedef Eigen::Matrix<double, 15, 7> jacobian0_t;
  typedef Eigen::Matrix<double, 15, 9> jacobian1_t;

  ImuError() {}
  virtual ~ImuError() {}

  ImuError(const imu_data_t &imu_data,
           const imu_params_t &imu_params,
           const timestamp_t &t_0,
           const timestamp_t &t_1);

  static int propagation(const imu_data_t & imu_data,
                         const imu_params_t & imu_params,
                         mat4_t& T_WS,
                         vec_t<9> & sb,
                         const timestamp_t& t_start, const timestamp_t& t_end,
                         covariance_t* covariance = 0,
                         jacobian_t* jacobian = 0);

  int redoPreintegration(const mat4_t& T_WS,
                         const vec_t<9> & sb) const;

  virtual bool Evaluate(double const* const * params, double* residuals,
                        double** jacobians) const;

  bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobiansMinimal) const;

  imu_params_t imu_params_;
  imu_data_t imu_data_;
  timestamp_t t0_;
  timestamp_t t1_;

protected:

  // preintegration stuff. the mutable is a TERRIBLE HACK, but what can I do.
  mutable std::mutex preintegrationMutex_; //< Protect access of intermediate results.
  // increments (initialise with identity)
  mutable Eigen::Quaterniond Delta_q_ = Eigen::Quaterniond(1,0,0,0); ///< Intermediate result
  mutable Eigen::Matrix3d C_integral_ = Eigen::Matrix3d::Zero(); ///< Intermediate result
  mutable Eigen::Matrix3d C_doubleintegral_ = Eigen::Matrix3d::Zero(); ///< Intermediate result
  mutable Eigen::Vector3d acc_integral_ = Eigen::Vector3d::Zero(); ///< Intermediate result
  mutable Eigen::Vector3d acc_doubleintegral_ = Eigen::Vector3d::Zero(); ///< Intermediate result

  // cross matrix accumulatrion
  mutable Eigen::Matrix3d cross_ = Eigen::Matrix3d::Zero(); ///< Intermediate result

  // sub-Jacobians
  mutable Eigen::Matrix3d dalpha_db_g_ = Eigen::Matrix3d::Zero(); ///< Intermediate result
  mutable Eigen::Matrix3d dv_db_g_ = Eigen::Matrix3d::Zero(); ///< Intermediate result
  mutable Eigen::Matrix3d dp_db_g_ = Eigen::Matrix3d::Zero(); ///< Intermediate result

  /// \brief The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double,15,15> P_delta_ = Eigen::Matrix<double,15,15>::Zero();

  /// \brief Reference biases that are updated when called redoPreintegration.
  mutable vec_t<9> sb_ref_ = vec_t<9>::Zero();

  mutable bool redo_ = true; ///< Keeps track of whether or not this redoPreintegration() needs to be called.
  mutable int redoCounter_ = 0; ///< Counts the number of preintegrations for statistics.

  // information matrix and its square root
  mutable information_t information_; ///< The information matrix for this error term.
  mutable information_t squareRootInformation_; ///< The square root information matrix for this error term.

  // for gradient/hessian w.r.t. the sigmas:
  mutable AlignedVector<Eigen::Matrix<double,15,15>> dPdsigma_;
};

std::atomic_bool ImuError::redoPropagationAlways(false);

ImuError::ImuError(const imu_data_t &imu_data,
           	 	 	 	 const imu_params_t &imu_params,
           	 	 	 	 const timestamp_t &t0,
           	 	 	 	 const timestamp_t &t1) {
  imu_data_ = imu_data;
  imu_params_ = imu_params;
  t0_ = t0;
  t1_ = t1;

  // adjust covariance/information etc.
  if(dPdsigma_.size() > 0) {
    P_delta_ = dPdsigma_.at(0)*imu_params_.sigma_g_c*imu_params_.sigma_g_c;
    P_delta_ += dPdsigma_.at(1)*imu_params_.sigma_a_c*imu_params_.sigma_a_c;
    P_delta_ += dPdsigma_.at(2)*imu_params_.sigma_gw_c*imu_params_.sigma_gw_c;
    P_delta_ += dPdsigma_.at(3)*imu_params_.sigma_aw_c*imu_params_.sigma_aw_c;

    // calculate inverse
    information_ = P_delta_.inverse();
    information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

    // square root
    Eigen::LLT<information_t> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();

  }

  // derivatives of covariance matrix w.r.t. sigmas
  dPdsigma_.resize(4);
  for(size_t j=0; j<4; ++j) {
    dPdsigma_.at(j).setZero();
  }
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::propagation(const imu_data_t & imu_data,
                          const imu_params_t & imu_params,
                          mat4_t &T_WS,
                          vec_t<9> &sb,
                          const timestamp_t & t_start,
                          const timestamp_t & t_end, covariance_t* covariance,
                          jacobian_t* jacobian) {

  // now the propagation
  // timestamp_t time = t_start;
  // timestamp_t end = t_end;

  // sanity check:
	assert(imu_data.timestamps.front() <= t_start);
	assert(imu_data.timestamps.back() >= t_end);

  // initial condition
  Eigen::Vector3d r_0 = tf_trans(T_WS);
  Eigen::Quaterniond q_WS_0 = tf_quat(T_WS);
  Eigen::Matrix3d C_WS_0 = tf_rot(T_WS);

  // increments (initialise with identity)
  Eigen::Quaterniond Delta_q(1,0,0,0);
  Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double,15,15> P_delta = Eigen::Matrix<double,15,15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  timestamp_t ts_k = t_start;
	for (size_t k = 0; k < (imu_data.size() - 1); k++) {
		vec3_t omega_S_0 = imu_data.gyro[k];
		vec3_t acc_S_0 = imu_data.accel[k];
		vec3_t omega_S_1 = imu_data.gyro[k + 1];
		vec3_t acc_S_1 = imu_data.accel[k + 1];

    // time delta
    // timestamp_t nexttime;
    // if ((it + 1) == imu_data.end()) {
    //   nexttime = t_end;
    // } else
    //   nexttime = (it + 1)->timeStamp;
    // double dt = (nexttime - time).toSec();

    // Time delta
    timestamp_t ts_kp1;
    if ((k + 1) == imu_data.size()) {
      ts_kp1 = t_end;
    } else {
      ts_kp1 = imu_data.timestamps[k + 1];
		}

    if ((ts_kp1 - ts_k) < 0) {
      continue;
    }
    double dt = ns2sec(ts_kp1 - ts_k);

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

    // if (end < nexttime) {
    //   double interval = (nexttime - it->timeStamp).toSec();
    //   nexttime = t_end;
    //   dt = (nexttime - time).toSec();
    //   const double r = dt / interval;
    //   omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
    //   acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    // }

    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    // if (!hasStarted) {
    //   hasStarted = true;
    //   const double r = dt / (nexttime - it->timeStamp).toSec();
    //   omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
    //   acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    // }

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(ts_kp1 - ts_k);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_params.sigma_g_c;
    double sigma_a_c = imu_params.sigma_a_c;

    // if (fabs(omega_S_0[0]) > imu_params.g_max
    //     || fabs(omega_S_0[1]) > imu_params.g_max
    //     || fabs(omega_S_0[2]) > imu_params.g_max
    //     || fabs(omega_S_1[0]) > imu_params.g_max
    //     || fabs(omega_S_1[1]) > imu_params.g_max
    //     || fabs(omega_S_1[2]) > imu_params.g_max) {
    //   sigma_g_c *= 100;
    //   LOG(WARNING) << "gyr saturation";
    // }
    //
    // if (fabs(acc_S_0[0]) > imu_params.a_max || fabs(acc_S_0[1]) > imu_params.a_max
    //     || fabs(acc_S_0[2]) > imu_params.a_max
    //     || fabs(acc_S_1[0]) > imu_params.a_max
    //     || fabs(acc_S_1[1]) > imu_params.a_max
    //     || fabs(acc_S_1[2]) > imu_params.a_max) {
    //   sigma_a_c *= 100;
    //   LOG(WARNING) << "acc saturation";
    // }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5*(omega_S_0+omega_S_1) - sb.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true = (0.5*(acc_S_0+acc_S_1) - sb.segment<3>(6));
    const Eigen::Matrix3d C_integral_1 = C_integral + 0.5*(C + C_1)*dt;
    const Eigen::Vector3d acc_integral_1 = acc_integral + 0.5*(C + C_1)*acc_S_true*dt;
    // rotation matrix double integral:
    C_doubleintegral += C_integral*dt + 0.25*(C + C_1)*dt*dt;
    acc_doubleintegral += acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt;

    // Jacobian parts
    dalpha_db_g += dt*C_1;
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix()*cross +
        rightJacobian(omega_S_true*dt)*dt;
    const Eigen::Matrix3d acc_S_x = skew(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g + 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
    dp_db_g += dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);

    // covariance propagation
    if (covariance) {
      Eigen::Matrix<double,15,15> F_delta = Eigen::Matrix<double,15,15>::Identity();
      // transform
      F_delta.block<3,3>(0,3) = -skew(acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt);
      F_delta.block<3,3>(0,6) = Eigen::Matrix3d::Identity()*dt;
      F_delta.block<3,3>(0,9) = dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
      F_delta.block<3,3>(0,12) = -C_integral*dt + 0.25*(C + C_1)*dt*dt;
      F_delta.block<3,3>(3,9) = -dt*C_1;
      F_delta.block<3,3>(6,3) = -skew(0.5*(C + C_1)*acc_S_true*dt);
      F_delta.block<3,3>(6,9) = 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
      F_delta.block<3,3>(6,12) = -0.5*(C + C_1)*dt;

      P_delta = F_delta*P_delta*F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be ignored, since the noise is isotropic.
      //F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(3,3) += sigma2_dalpha;
      P_delta(4,4) += sigma2_dalpha;
      P_delta(5,5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * imu_params.sigma_a_c;
      P_delta(6,6) += sigma2_v;
      P_delta(7,7) += sigma2_v;
      P_delta(8,8) += sigma2_v;
      const double sigma2_p = 0.5*dt*dt*sigma2_v;
      P_delta(0,0) += sigma2_p;
      P_delta(1,1) += sigma2_p;
      P_delta(2,2) += sigma2_p;
      const double sigma2_b_g = dt * imu_params.sigma_gw_c * imu_params.sigma_gw_c;
      P_delta(9,9)   += sigma2_b_g;
      P_delta(10,10) += sigma2_b_g;
      P_delta(11,11) += sigma2_b_g;
      const double sigma2_b_a = dt * imu_params.sigma_aw_c * imu_params.sigma_aw_c;
      P_delta(12,12) += sigma2_b_a;
      P_delta(13,13) += sigma2_b_a;
      P_delta(14,14) += sigma2_b_a;
    }

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

  // actual propagation output:
  const Eigen::Vector3d g_W = imu_params.g * Eigen::Vector3d(0, 0, 6371009).normalized();
	const vec3_t trans_update = r_0 + sb.head<3>() * Delta_t + C_WS_0 * (acc_doubleintegral /*-C_doubleintegral*sb.segment<3>(6)*/) - 0.5 * g_W * Delta_t * Delta_t;
	const quat_t quat_update = q_WS_0 * Delta_q;
	T_WS = tf(quat_update, trans_update);
  sb.head<3>() += C_WS_0*(acc_integral/*-C_integral*sb.segment<3>(6)*/)-g_W*Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double,15,15> & F = *jacobian;
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g, d/db_a
    F.block<3,3>(0,3) = -skew(C_WS_0*acc_doubleintegral);
    F.block<3,3>(0,6) = Eigen::Matrix3d::Identity()*Delta_t;
    F.block<3,3>(0,9) = C_WS_0*dp_db_g;
    F.block<3,3>(0,12) = -C_WS_0*C_doubleintegral;
    F.block<3,3>(3,9) = -C_WS_0*dalpha_db_g;
    F.block<3,3>(6,3) = -skew(C_WS_0*acc_integral);
    F.block<3,3>(6,9) = C_WS_0*dv_db_g;
    F.block<3,3>(6,12) = -C_WS_0*C_integral;
  }

  // overall covariance, if requested
  if (covariance) {
    Eigen::Matrix<double,15,15> & P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double,15,15> T = Eigen::Matrix<double,15,15>::Identity();
    T.topLeftCorner<3,3>() = C_WS_0;
    T.block<3,3>(3,3) = C_WS_0;
    T.block<3,3>(6,6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }
  return i;
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::redoPreintegration(const mat4_t& /*T_WS*/,
                                 const vec_t<9> & sb) const {

  // ensure unique access outside this function...

  // now the propagation
  timestamp_t time = t0_;
  timestamp_t end = t1_;

  // sanity check:
  assert(imu_data_.timestamps.front() <=time);
  if (!(imu_data_.timestamps.back() >= end))
    return -1; // nothing to do...

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  C_doubleintegral_ = Eigen::Matrix3d::Zero();
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  cross_ = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  dv_db_g_ = Eigen::Matrix3d::Zero();
  dp_db_g_ = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  // derivatives of covariance matrix w.r.t. sigmas
  dPdsigma_.resize(4);
  for(size_t j=0; j<4; ++j) {
    dPdsigma_.at(j).setZero();
  }

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

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(nexttime - imu_data_.timestamps[k]);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double gyr_sat_mult = 1.0;
    double acc_sat_mult = 1.0;

    // if (fabs(omega_S_0[0]) > imu_params_.g_max
    //     || fabs(omega_S_0[1]) > imu_params_.g_max
    //     || fabs(omega_S_0[2]) > imu_params_.g_max
    //     || fabs(omega_S_1[0]) > imu_params_.g_max
    //     || fabs(omega_S_1[1]) > imu_params_.g_max
    //     || fabs(omega_S_1[2]) > imu_params_.g_max) {
    //   gyr_sat_mult *= 100;
    //   LOG(WARNING)<< "gyr saturation";
    // }

    // if (fabs(acc_S_0[0]) > imu_params_.a_max || fabs(acc_S_0[1]) > imu_params_.a_max
    //     || fabs(acc_S_0[2]) > imu_params_.a_max
    //     || fabs(acc_S_1[0]) > imu_params_.a_max
    //     || fabs(acc_S_1[1]) > imu_params_.a_max
    //     || fabs(acc_S_1[2]) > imu_params_.a_max) {
    //   acc_sat_mult *= 100;
    //   LOG(WARNING)<< "acc saturation";
    // }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1)
        - sb.segment < 3 > (3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true = (0.5 * (acc_S_0 + acc_S_1)
        - sb.segment < 3 > (6));
    const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
    const Eigen::Vector3d acc_integral_1 = acc_integral_
        + 0.5 * (C + C_1) * acc_S_true * dt;
    // rotation matrix double integral:
    C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral_ += acc_integral_ * dt
        + 0.25 * (C + C_1) * acc_S_true * dt * dt;

    // Jacobian parts
    dalpha_db_g_ += C_1 * rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix() * cross_
        + rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d acc_S_x = skew(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g_
        + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    dp_db_g_ += dt * dv_db_g_
        + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);

    // covariance propagation
    Eigen::Matrix<double, 15, 15> F_delta =
        Eigen::Matrix<double, 15, 15>::Identity();
    // transform
    F_delta.block<3, 3>(0, 3) = -skew(
        acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
    F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F_delta.block<3, 3>(0, 9) = dt * dv_db_g_
        + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(0, 12) = -C_integral_ * dt
        + 0.25 * (C + C_1) * dt * dt;
    F_delta.block<3, 3>(3, 9) = -dt * C_1;
    F_delta.block<3, 3>(6, 3) = -skew(
        0.5 * (C + C_1) * acc_S_true * dt);
    F_delta.block<3, 3>(6, 9) = 0.5 * dt
        * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;

    // Q = K * sigma_sq
    Eigen::Matrix<double,15,15> K0 = Eigen::Matrix<double,15,15>::Zero();
    Eigen::Matrix<double,15,15> K1 = Eigen::Matrix<double,15,15>::Zero();
    Eigen::Matrix<double,15,15> K2 = Eigen::Matrix<double,15,15>::Zero();
    Eigen::Matrix<double,15,15> K3 = Eigen::Matrix<double,15,15>::Zero();
    K0.block<3,3>(3,3) = gyr_sat_mult*dt * Eigen::Matrix3d::Identity();
    K1.block<3,3>(0,0) = 0.5 * dt*dt*dt * acc_sat_mult*acc_sat_mult*acc_sat_mult * Eigen::Matrix3d::Identity();
    K1.block<3,3>(6,6) = acc_sat_mult*dt* Eigen::Matrix3d::Identity();
    K2.block<3,3>(9,9) = dt * Eigen::Matrix3d::Identity();
    K3.block<3,3>(12,12) = dt * Eigen::Matrix3d::Identity();
    dPdsigma_.at(0) = F_delta*dPdsigma_.at(0)*F_delta.transpose() + K0;
    dPdsigma_.at(1) = F_delta*dPdsigma_.at(1)*F_delta.transpose() + K1;
    dPdsigma_.at(2) = F_delta*dPdsigma_.at(2)*F_delta.transpose() + K2;
    dPdsigma_.at(3) = F_delta*dPdsigma_.at(3)*F_delta.transpose() + K3;

    //P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // add noise. Note that transformations with rotation matrices can be ignored, since the noise is isotropic.
    //F_tot = F_delta*F_tot;

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;
    cross_ = cross_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == t1_)
      break;

  }

  // store the reference (linearisation) point
  sb_ref_ = sb;

  // get the weighting:
  // enforce symmetric
  for(int j=0; j<4; ++j) {
    dPdsigma_.at(j) = 0.5 * dPdsigma_.at(j) + 0.5 * dPdsigma_.at(j).transpose().eval();
  }
  P_delta_ = dPdsigma_.at(0)*imu_params_.sigma_g_c*imu_params_.sigma_g_c;
  P_delta_ += dPdsigma_.at(1)*imu_params_.sigma_a_c*imu_params_.sigma_a_c;
  P_delta_ += dPdsigma_.at(2)*imu_params_.sigma_gw_c*imu_params_.sigma_gw_c;
  P_delta_ += dPdsigma_.at(3)*imu_params_.sigma_aw_c*imu_params_.sigma_aw_c;

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();

  return i;
}

// void ImuError::setImuParameters(const ImuParameters &imuParameters) {
//
//   imu_params_ = imuParameters;
//
//   // adjust covariance/information etc.
//   if(dPdsigma_.size() > 0) {
//     P_delta_ = dPdsigma_.at(0)*imu_params_.sigma_g_c*imu_params_.sigma_g_c;
//     P_delta_ += dPdsigma_.at(1)*imu_params_.sigma_a_c*imu_params_.sigma_a_c;
//     P_delta_ += dPdsigma_.at(2)*imu_params_.sigma_gw_c*imu_params_.sigma_gw_c;
//     P_delta_ += dPdsigma_.at(3)*imu_params_.sigma_aw_c*imu_params_.sigma_aw_c;
//
//     // calculate inverse
//     information_ = P_delta_.inverse();
//     information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();
//
//     // square root
//     Eigen::LLT<information_t> lltOfInformation(information_);
//     squareRootInformation_ = lltOfInformation.matrixL().transpose();
//
//   }
// }

// This evaluates the error term and additionally computes the Jacobians.
bool ImuError::Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool ImuError::EvaluateWithMinimalJacobians(double const* const * params,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const {
#if 0 // debug disable IMU
Eigen::Map<Eigen::Matrix<double, 15, 1> > weighted_error(residuals);
weighted_error.setZero();
if (jacobians != NULL) {
  if (jacobians[0] != NULL) {
    // Jacobian w.r.t. minimal perturbance
    Eigen::Matrix<double, 15, 6> J0_minimal;
    J0_minimal.setZero();
    Eigen::Map<Eigen::Matrix<double, 15, 7>> J0(jacobians[0]);
    J0.setZero();
    // if requested, provide minimal Jacobians
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J0_minimal_mapped(
            jacobiansMinimal[0]);
        J0_minimal_mapped = J0_minimal;
      }
    }
  }
  if (jacobians[1] != NULL) {
    // Jacobian w.r.t. minimal perturbance
    Eigen::Matrix<double, 15, 9> J1_minimal;
    J1_minimal.setZero();
    Eigen::Map<Eigen::Matrix<double, 15, 9>> J1(jacobians[1]);
    J1.setZero();
    // if requested, provide minimal Jacobians
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J1_minimal_mapped(
            jacobiansMinimal[1]);
        J1_minimal_mapped = J1_minimal;
      }
    }
  }
  if (jacobians[2] != NULL) {
    // Jacobian w.r.t. minimal perturbance
    Eigen::Matrix<double, 15, 6> J2_minimal;
    J2_minimal.setZero();
    Eigen::Map<Eigen::Matrix<double, 15, 7>> J2(jacobians[2]);
    J2.setZero();
    // if requested, provide minimal Jacobians
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[2] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J2_minimal_mapped(
            jacobiansMinimal[2]);
        J2_minimal_mapped = J2_minimal;
      }
    }
  }
  if (jacobians[3] != NULL) {
    // Jacobian w.r.t. minimal perturbance
    Eigen::Matrix<double, 15, 9> J3_minimal;
    J3_minimal.setZero();
    Eigen::Map<Eigen::Matrix<double, 15, 9>> J3(jacobians[3]);
    J3.setZero();
    // if requested, provide minimal Jacobians
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J3_minimal_mapped(
            jacobiansMinimal[3]);
        J3_minimal_mapped = J3_minimal;
      }
    }
  }
}
return true;
#endif

  // Map T_WS_0
  const vec3_t r_WS_0{params[0][0], params[0][1], params[0][2]};
	const quat_t q_WS_0{params[0][6], params[0][3], params[0][4], params[0][5]};
  const mat4_t T_WS_0 = tf(q_WS_0, r_WS_0);

  // Map T_WS_1
  const vec3_t r_WS_1{params[2][0], params[2][1], params[2][2]};
	const quat_t q_WS_1{params[2][6], params[2][3], params[2][4], params[2][5]};
  const mat4_t T_WS_1 = tf(q_WS_1, r_WS_1);

  // get poses
  // const mat4_t T_WS_0(
	// 	Eigen::Vector3d(params[0][0], params[0][1], params[0][2]),
	// 	Eigen::Quaterniond(params[0][6], params[0][3], params[0][4], params[0][5]));
  //
  // const mat4_t T_WS_1(
	// 	Eigen::Vector3d(params[2][0], params[2][1], params[2][2]),
	// 	Eigen::Quaterniond(params[2][6], params[2][3], params[2][4], params[2][5]));

  // get speed and bias
  vec_t<9> sb_0;
  vec_t<9> sb_1;
  for (size_t i = 0; i < 9; ++i) {
    sb_0[i] = params[1][i];
    sb_1[i] = params[3][i];
  }

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = tf_rot(T_WS_0);
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  // const double Delta_t = (t1_ - t0_).toSec();
	assert(t1_ > t0_);
  const double Delta_t = ns2sec(t1_ - t0_);
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_);
    Delta_b = sb_0.tail<6>()
          - sb_ref_.tail<6>();
    redo_ = redo_ || (Delta_b.head<3>().norm() > 0.0003);
    //std::cout << imu_data_.size() << std::endl;
    if ((redo_ && ((imu_data_.size() < 50) || redoPropagationAlways)) || redoCounter_==0) {
      redoPreintegration(T_WS_0, sb_0);
      redoCounter_++;
      Delta_b.setZero();
      redo_ = false;
      //if (redoCounter_ > 1)
      //  std::cout << "pre-integration no. " << redoCounter_ << ", no. measurements = " << imu_data_.size()
      //            << ", always=" << int(redoPropagationAlways)<< std::endl;
      //if (redoCounter_ > 1)
      //std::cout << "pre-integration no. " << redoCounter_ << ", Delta_ba = " << Delta_b.head<3>().norm() << ", no. measurements = " << imu_data_.size() << std::endl;
    }
  }

  // actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_); // this is a bit stupid, but shared read-locks only come in C++14
    const Eigen::Vector3d g_W = imu_params_.g * Eigen::Vector3d(0, 0, 6371009).normalized();

    // assign Jacobian w.r.t. x0
    Eigen::Matrix<double,15,15> F0 =
        Eigen::Matrix<double,15,15>::Identity(); // holds for d/db_g, d/db_a
    const Eigen::Vector3d delta_p_est_W =
        tf_trans(T_WS_0) - tf_trans(T_WS_1) + sb_0.head<3>()*Delta_t - 0.5*g_W*Delta_t*Delta_t;
    const Eigen::Vector3d delta_v_est_W =
        sb_0.head<3>() - sb_1.head<3>() - g_W*Delta_t;
    const Eigen::Quaterniond Dq = deltaQ(-dalpha_db_g_*Delta_b.head<3>())*Delta_q_;
    F0.block<3,3>(0,0) = C_S0_W;
    F0.block<3,3>(0,3) = C_S0_W * skew(delta_p_est_W);
    F0.block<3,3>(0,6) = C_S0_W * Eigen::Matrix3d::Identity()*Delta_t;
    F0.block<3,3>(0,9) = dp_db_g_;
    F0.block<3,3>(0,12) = -C_doubleintegral_;
    F0.block<3,3>(3,3) = (plus(Dq*tf_quat(T_WS_1).inverse()) *
        oplus(tf_quat(T_WS_0))).topLeftCorner<3,3>();
    F0.block<3,3>(3,9) = (oplus(tf_quat(T_WS_1).inverse()*tf_quat(T_WS_0))*
        oplus(Dq)).topLeftCorner<3,3>()*(-dalpha_db_g_);
    F0.block<3,3>(6,3) = C_S0_W * skew(delta_v_est_W);
    F0.block<3,3>(6,6) = C_S0_W;
    F0.block<3,3>(6,9) = dv_db_g_;
    F0.block<3,3>(6,12) = -C_integral_;

    // assign Jacobian w.r.t. x1
    Eigen::Matrix<double,15,15> F1 =
        -Eigen::Matrix<double,15,15>::Identity(); // holds for the biases
    F1.block<3,3>(0,0) = -C_S0_W;
    F1.block<3,3>(3,3) = -(plus(Dq) *
        oplus(tf_quat(T_WS_0)) *
        plus(tf_quat(T_WS_1).inverse())).topLeftCorner<3,3>();
    F1.block<3,3>(6,6) = -C_S0_W;

    // the overall error vector
    Eigen::Matrix<double, 15, 1> error;
    error.segment<3>(0) =  C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3,6>(0,9)*Delta_b;
    error.segment<3>(3) = 2*(Dq*(tf_quat(T_WS_1).inverse()*tf_quat(T_WS_0))).vec(); //2*tf_quat(T_WS_0)*Dq*tf_quat(T_WS_1).inverse();//
    error.segment<3>(6) = C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3,6>(6,9)*Delta_b;
    error.tail<6>() = sb_0.tail<6>() - sb_1.tail<6>();

    // error weighting
    Eigen::Map<Eigen::Matrix<double, 15, 1> > weighted_error(residuals);
    weighted_error = squareRootInformation_ * error;

    // get the Jacobians
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_
            * F0.block<15, 6>(0, 0);

        // // pseudo inverse of the local parametrization Jacobian:
        // Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        // PoseLocalParameterization::liftJacobian(params[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J0(
            jacobians[0]);
        // J0 = J0_minimal * J_lift;
				J0.setZero();
        J0.block(0, 0, 15, 6) = J0_minimal;

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[0] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J0_minimal_mapped(
                jacobiansMinimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J1(
            jacobians[1]);
        J1 = squareRootInformation_ * F0.block<15, 9>(0, 6);

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J1_minimal_mapped(
                jacobiansMinimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }
      if (jacobians[2] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J2_minimal = squareRootInformation_
                    * F1.block<15, 6>(0, 0);

        // // pseudo inverse of the local parametrization Jacobian:
        // Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        // PoseLocalParameterization::liftJacobian(params[2], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J2(
            jacobians[2]);
        // J2 = J2_minimal * J_lift;
				J2.setZero();
        J2.block(0, 0, 15, 6) = J2_minimal;

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[2] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J2_minimal_mapped(
                jacobiansMinimal[2]);
            J2_minimal_mapped = J2_minimal;
          }
        }
      }
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J3(jacobians[3]);
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J3_minimal_mapped(
                jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }
    }
  }

  return true;
}

}  // namespace okvis
#endif /* YAC_IMU_ERROR2_HPP */
