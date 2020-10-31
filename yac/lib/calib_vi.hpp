#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_mono.hpp"
#include "calib_data.hpp"

namespace yac {

/** Inertial residual */
struct calib_imu_residual_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamps_t imu_ts;
  const vec3s_t imu_accel;
  const vec3s_t imu_gyro;
  const vec3_t g{0.0, 0.0, -9.81};

  mat_t<15, 15> P = zeros(15, 15);  // Covariance matrix
  mat_t<12, 12> Q = zeros(12, 12);  // noise matrix
  mat_t<15, 15> F = zeros(15, 15);  // Transition matrix

  // Delta position, velocity and rotation between timestep i and j
  // (i.e start and end of imu measurements)
  vec3_t dp{0.0, 0.0, 0.0};
  vec3_t dv{0.0, 0.0, 0.0};
  quat_t dq{1.0, 0.0, 0.0, 0.0};

  // Accelerometer and gyroscope biases
  vec3_t bg{0.0, 0.0, 0.0};
  vec3_t ba{0.0, 0.0, 0.0};

  calib_imu_residual_t(const id_t id_,
                       const timestamps_t imu_ts_,
                       const vec3s_t imu_accel_,
                       const vec3s_t imu_gyro_ ,
                       const mat_t<15, 15> &info_,
                       const std::vector<param_t *> &params_)
      : factor_t{id_, info_, params_},
        imu_ts{imu_ts_},
        imu_accel{imu_accel_},
        imu_gyro{imu_gyro_} {
    residuals = zeros(15, 1);
    jacobians.push_back(zeros(15, 6));  // T_WS at timestep i
    jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep i
    jacobians.push_back(zeros(15, 6));  // T_WS at timestep j
    jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep j

    propagate(imu_ts_, imu_accel_, imu_gyro_);
  }

  void reset() {
    P = zeros(15, 15);
    F = zeros(15, 15);

    dp = zeros(3);
    dv = zeros(3);
    dq = quat_t{1.0, 0.0, 0.0, 0.0};
    ba = zeros(3);
    bg = zeros(3);
  }

  void propagate(const timestamps_t &ts,
                 const vec3s_t &a_m,
                 const vec3s_t &w_m) {
    assert(ts.size() == a_m.size());
    assert(w_m.size() == a_m.size());

    real_t dt_prev = ns2sec(ts[1] - ts[0]);
    for (size_t i = 0; i < w_m.size(); i++) {
      // Calculate dt
      real_t dt = 0.0;
      if ((i + 1) < w_m.size()) {
        dt = ns2sec(ts[i + 1] - ts[i]);
        dt_prev = dt;
      } else {
        dt = dt_prev;
      }
      // printf("i: %zu\n", i);

      // Update relative position and velocity
      dp = dp + dv * dt + 0.5 * (dq * (a_m[i] - ba)) * dt * dt;
      dv = dv + (dq * (a_m[i] - ba)) * dt;

      // Update relative rotation
      const real_t scalar = 1.0;
      const vec3_t vector = 0.5 * (w_m[i] - bg) * dt;
      const quat_t dq_i{scalar, vector(0), vector(1), vector(2)};
      dq = dq * dq_i;

      // Transition matrix F
      const mat3_t C_ji = dq.toRotationMatrix();
      mat_t<15, 15> F_i = zeros(15, 15);
      F_i.block<3, 3>(0, 3) = I(3);
      F_i.block<3, 3>(3, 6) = -C_ji * skew(a_m[i] - ba);
      F_i.block<3, 3>(3, 9) = -C_ji;
      F_i.block<3, 3>(6, 6) = -skew(w_m[i] - bg);
      F_i.block<3, 3>(6, 12) = -I(3);

      // Input matrix G
      mat_t<15, 12> G_i = zeros(15, 12);
      G_i.block<3, 3>(3, 0) = -C_ji;
      G_i.block<3, 3>(6, 3) = -I(3);
      G_i.block<3, 3>(9, 6) = I(3);
      G_i.block<3, 3>(12, 9) = I(3);

      // Update covariance matrix
      const mat_t<15, 15> I_Fi_dt = (I(15) + F * dt);
      const mat_t<15, 12> Gi_dt = (G_i * dt);
      P = I_Fi_dt * P * I_Fi_dt.transpose() + Gi_dt * Q * Gi_dt.transpose();

      // Update Jacobian
      F = I_Fi_dt * F;
    }
  }

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const q_i_, const T *const r_i_,
                  const T *const sb_i_,
                  const T *const q_j_, const T *const r_j_,
                  const T *const sb_j_,
                  T *residual) const {
    // Map out parameters
    // -- Sensor pose at timestep i
    const mat4_t T_i = tf(params[0]);
    const mat3_t C_i = tf_rot(T_i);
    const mat3_t C_i_inv = C_i.transpose();
    const quat_t q_i = tf_quat(T_i);
    const vec3_t r_i = tf_trans(T_i);
    // -- Speed and bias at timestamp i
    const vec_t<9> sb_i{params[1]->param};
    const vec3_t v_i = sb_i.segment<3>(0);
    const vec3_t ba_i = sb_i.segment<3>(3);
    const vec3_t bg_i = sb_i.segment<3>(6);
    // -- Sensor pose at timestep j
    const mat4_t T_j = tf(params[2]->param);
    const quat_t q_j = tf_quat(T_j);
    const vec3_t r_j = tf_trans(T_j);
    // -- Speed and bias at timestep j
    const vec_t<9> sb_j{params[3]->param};
    const vec3_t v_j = sb_j.segment<3>(0);
    const vec3_t ba_j = sb_j.segment<3>(3);
    const vec3_t bg_j = sb_j.segment<3>(6);

    // Obtain Jacobians for gyro and accel bias
    const mat3_t dp_dbg = F.block<3, 3>(0, 9);
    const mat3_t dp_dba = F.block<3, 3>(0, 12);
    const mat3_t dv_dbg = F.block<3, 3>(3, 9);
    const mat3_t dv_dba = F.block<3, 3>(3, 12);
    const mat3_t dq_dbg = F.block<3, 3>(6, 12);

    // Calculate residuals
    const real_t dt_ij = ns2sec(imu_ts.back() - imu_ts.front());
    const real_t dt_ij_sq = dt_ij * dt_ij;
    const vec3_t dbg = bg_i - bg;
    const vec3_t dba = ba_i - ba;
    const vec3_t alpha = dp + dp_dbg * dbg + dp_dba * dba;
    const vec3_t beta = dv + dv_dbg * dbg + dv_dba * dba;
    const quat_t gamma = dq * quat_delta(dq_dbg * dbg);

    const quat_t q_i_inv = q_i.inverse();
    const quat_t q_j_inv = q_j.inverse();
    const quat_t gamma_inv = gamma.inverse();

    // clang-format off
    residuals << C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha,
                 C_i_inv * (v_j - v_i + g * dt_ij) - beta,
                 2.0 * (gamma_inv * (q_i_inv * q_j)).vec(),
                 ba_j - ba_i,
                 bg_j - bg_i;
    // clang-format on

    return true;
  }
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
