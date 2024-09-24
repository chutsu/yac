#include "InertialError.hpp"

namespace yac {

matx_t InertialError::formQ() {
  matx_t Q = zeros(18, 18);
  Q.block<3, 3>(0, 0) = pow(imu_params_.noise_acc, 2) * I(3);
  Q.block<3, 3>(3, 3) = pow(imu_params_.noise_gyr, 2) * I(3);
  Q.block<3, 3>(6, 6) = pow(imu_params_.noise_acc, 2) * I(3);
  Q.block<3, 3>(9, 9) = pow(imu_params_.noise_gyr, 2) * I(3);
  Q.block<3, 3>(12, 12) = pow(imu_params_.noise_ba, 2) * I(3);
  Q.block<3, 3>(15, 15) = pow(imu_params_.noise_bg, 2) * I(3);
  return Q;
}

matx_t InertialError::formF(const int k,
                            const quat_t &dq_i,
                            const quat_t &dq_j,
                            const vec3_t &ba_i,
                            const vec3_t &bg_i,
                            const double dt) {
  const vec3_t w_k = imu_buffer_.getGyr(k);
  const vec3_t w_kp1 = imu_buffer_.getGyr(k + 1);
  const vec3_t a_k = imu_buffer_.getAcc(k);
  const vec3_t a_kp1 = imu_buffer_.getAcc(k + 1);
  const mat3_t gyr_x = skew(0.5 * (w_k + w_kp1) - bg_i);
  const mat3_t acc_i_x = skew(a_k - ba_i);
  const mat3_t acc_j_x = skew(a_kp1 - ba_i);
  const mat3_t dC_i = dq_i.toRotationMatrix();
  const mat3_t dC_j = dq_j.toRotationMatrix();
  const double dt_sq = dt * dt;

  // Form transition matrix F
  matx_t F = zeros(15, 15);
  // -- F row block 1
  F.block<3, 3>(0, 0) = I(3);
  F.block<3, 3>(0, 3) = -0.25 * dC_i * acc_i_x * dt_sq;
  F.block<3, 3>(0, 3) += -0.25 * dC_j * acc_j_x * (I(3) - gyr_x * dt) * dt_sq;
  F.block<3, 3>(0, 6) = I(3) * dt;
  F.block<3, 3>(0, 9) = -0.25 * (dC_i + dC_j) * dt_sq;
  F.block<3, 3>(0, 12) = 0.25 * -dC_j * acc_j_x * dt_sq * -dt;
  // -- F row block 2
  F.block<3, 3>(3, 3) = I(3) - gyr_x * dt;
  F.block<3, 3>(3, 12) = -I(3) * dt;
  // -- F row block 3
  F.block<3, 3>(6, 3) = -0.5 * dC_i * acc_i_x * dt;
  F.block<3, 3>(6, 3) += -0.5 * dC_j * acc_j_x * (I(3) - gyr_x * dt) * dt;
  F.block<3, 3>(6, 6) = I(3);
  F.block<3, 3>(6, 9) = -0.5 * (dC_i + dC_j) * dt;
  F.block<3, 3>(6, 12) = 0.5 * -dC_j * acc_j_x * dt * -dt;
  // -- F row block 4
  F.block<3, 3>(9, 9) = I(3);
  // -- F row block 5
  F.block<3, 3>(12, 12) = I(3);

  return F;
}

matx_t InertialError::formG(const int k,
                            const quat_t &dq_i,
                            const quat_t &dq_j,
                            const vec3_t &ba_i,
                            const double dt) {
  const vec3_t a_k = imu_buffer_.getAcc(k);
  const mat3_t acc_i_x = skew(a_k - ba_i);
  const mat3_t dC_i = dq_i.toRotationMatrix();
  const mat3_t dC_j = dq_j.toRotationMatrix();
  const double dt_sq = dt * dt;

  matx_t G = zeros(15, 18);
  // -- G row block 1
  G.block<3, 3>(0, 0) = 0.25 * dC_i * dt_sq;
  G.block<3, 3>(0, 3) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
  G.block<3, 3>(0, 6) = 0.25 * dC_j * acc_i_x * dt_sq;
  G.block<3, 3>(0, 9) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
  // -- G row block 2
  G.block<3, 3>(3, 3) = I(3) * dt;
  G.block<3, 3>(3, 9) = I(3) * dt;
  // -- G row block 3
  G.block<3, 3>(6, 0) = 0.5 * dC_i * dt;
  G.block<3, 3>(6, 3) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
  G.block<3, 3>(6, 6) = 0.5 * dC_j * dt;
  G.block<3, 3>(6, 9) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
  // -- G row block 4
  G.block<3, 3>(9, 12) = I(3) * dt;
  // -- G row block 5
  G.block<3, 3>(12, 15) = I(3) * dt;

  return G;
}

void InertialError::propagate() {
  // Noise matrix Q
  const matx_t &Q = formQ();

  // Pre-integrate imu measuremenets
  for (int k = 0; k < (imu_buffer_.getNumMeasurements() - 1); k++) {
    // Timestep
    const timestamp_t ts_i = imu_buffer_.getTimestamp(k);
    const timestamp_t ts_j = imu_buffer_.getTimestamp(k + 1);
    const double dt = ts2sec(ts_j - ts_i);
    const double dt_sq = dt * dt;

    // Setup
    const quat_t dq_i = dq_;
    const vec3_t dr_i = dr_;
    const vec3_t dv_i = dv_;
    const vec3_t ba_i = ba_;
    const vec3_t bg_i = bg_;

    // Gyroscope measurement
    const vec3_t w_k = imu_buffer_.getGyr(k);
    const vec3_t w_kp1 = imu_buffer_.getGyr(k + 1);
    const vec3_t w = 0.5 * (w_k + w_kp1) - bg_i;
    const quat_t dq_perturb{1.0,
                            w.x() * dt / 2.0,
                            w.y() * dt / 2.0,
                            w.z() * dt / 2.0};

    // Accelerometer measurement
    const vec3_t a_k = imu_buffer_.getAcc(k);
    const vec3_t a_kp1 = imu_buffer_.getAcc(k + 1);
    const vec3_t acc_i = dq_i * a_k - ba_i;
    const vec3_t acc_j = (dq_i * dq_perturb) * (a_kp1 - ba_i);
    const vec3_t a = 0.5 * (acc_i + acc_j);

    // Propagate IMU state using mid-point method
    const quat_t dq_j = dq_i * dq_perturb;
    const vec3_t dr_j = dr_i + dv_i * dt + 0.5 * a * dt_sq;
    const vec3_t dv_j = dv_i + a * dt;
    const vec3_t ba_j = ba_i;
    const vec3_t bg_j = bg_i;

    // Continuous time transition matrix F and input matrix G
    const matx_t F = formF(k, dq_i, dq_j, ba_i, bg_i, dt);
    const matx_t G = formG(k, dq_i, dq_j, ba_i, dt);

    // Map results
    dq_ = dq_j;
    dr_ = dr_j;
    dv_ = dv_j;
    ba_ = ba_j;
    bg_ = bg_j;

    // Update
    state_F_ = F * state_F_;
    state_P_ = F * state_P_ * F.transpose() + G * Q * G.transpose();
    Dt_ += dt;
  }

  // Enforce semi-positive-definite on the state covariance matrix
  state_P_ = (state_P_ + state_P_.transpose()) / 2.0;
  sqrt_info_ = state_P_.llt().matrixU();
}

InertialError::InertialError(const std::vector<double *> &param_ptrs,
                             const std::vector<ParamBlock::Type> &param_types,
                             const ImuParams &imu_params,
                             const ImuBuffer &imu_buffer)
    : ResidualBlock{"InertialError", param_ptrs, param_types, 15},
      imu_params_{imu_params}, imu_buffer_{imu_buffer} {
  assert(imu_params_.noise_acc > 0);
  assert(imu_params_.noise_gyr > 0);
  assert(imu_params_.noise_ba > 0);
  assert(imu_params_.noise_bg > 0);
  assert(imu_params_.g.norm() > 0);

  Eigen::Map<const vecx_t> sb_i(param_ptrs[1], 9);
  ba_ = vec3_t{sb_i[3], sb_i[4], sb_i[5]};
  bg_ = vec3_t{sb_i[6], sb_i[7], sb_i[8]};

  propagate();
}

std::shared_ptr<InertialError>
InertialError::create(const ImuParams &imu_params,
                      const ImuBuffer &imu_buffer,
                      double *pose_i,
                      double *sb_i,
                      double *pose_j,
                      double *sb_j) {
  std::vector<double *> param_ptrs = {pose_i, sb_i, pose_j, sb_j};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES,
                                               ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES};
  return std::make_shared<InertialError>(param_ptrs,
                                         param_types,
                                         imu_params,
                                         imu_buffer);
}

bool InertialError::EvaluateWithMinimalJacobians(double const *const *params,
                                                 double *res,
                                                 double **jacs,
                                                 double **min_jacs) const {
  // Map parameters out
  const mat4_t T_i = tf(params[0]);
  Eigen::Map<const vecx_t> sb_i(params[1], 9);
  const mat4_t T_j = tf(params[2]);
  Eigen::Map<const vecx_t> sb_j(params[3], 9);

  const quat_t q_i = tf_quat(T_i);
  const mat3_t C_i = q_i.toRotationMatrix();
  const vec3_t r_i = tf_trans(T_i);
  const vec3_t v_i = sb_i.segment<3>(0);
  const vec3_t ba_i = sb_i.segment<3>(3);
  const vec3_t bg_i = sb_i.segment<3>(6);

  const quat_t q_j = tf_quat(T_j);
  const mat3_t C_j = q_j.toRotationMatrix();
  const vec3_t r_j = tf_trans(T_j);
  const vec3_t v_j = sb_j.segment<3>(0);
  const vec3_t ba_j = sb_j.segment<3>(3);
  const vec3_t bg_j = sb_j.segment<3>(6);

  // Correct the relative position, velocity and orientation
  // -- Extract jacobians from error-state jacobian
  const mat3_t dr_dba = state_F_.block<3, 3>(0, 9);
  const mat3_t dr_dbg = state_F_.block<3, 3>(0, 12);
  const mat3_t dv_dba = state_F_.block<3, 3>(3, 9);
  const mat3_t dv_dbg = state_F_.block<3, 3>(3, 12);
  const mat3_t dq_dbg = state_F_.block<3, 3>(6, 12);
  const vec3_t dba = ba_i - ba_;
  const vec3_t dbg = bg_i - bg_;

  // -- Correct the relative position, velocity and rotation
  dr_ = dr_ + dr_dba * dba + dr_dbg * dbg;
  dv_ = dv_ + dv_dba * dba + dv_dbg * dbg;
  dq_ = dq_ * quat_delta(dq_dbg * dbg);

  // Form residuals
  const vec3_t g = imu_params_.g;
  const double Dt_sq = Dt_ * Dt_;
  const mat3_t C_iT = C_i.transpose();
  vec3_t dr_meas = (C_iT * ((r_j - r_i) - (v_i * Dt_) + (0.5 * g * Dt_sq)));
  vec3_t dv_meas = (C_iT * ((v_j - v_i) + (g * Dt_)));

  const vec3_t err_pos = dr_meas - dr_;
  const vec3_t err_vel = dv_meas - dv_;
  const vec3_t err_rot = 2.0 * (dq_.inverse() * (q_i.inverse() * q_j)).vec();
  const vec3_t err_ba = ba_j - ba_i;
  const vec3_t err_bg = bg_j - bg_i;

  Eigen::Map<vecx_t> r(res, 15);
  r.segment<3>(0) = err_pos;
  r.segment<3>(3) = err_vel;
  r.segment<3>(6) = err_rot;
  r.segment<3>(9) = err_ba;
  r.segment<3>(12) = err_bg;
  r = sqrt_info_ * r;

  // Form Jacobians
  if (jacs == nullptr) {
    return true;
  }

  // Jacobian w.r.t pose i
  if (jacs[0]) {
    const quat_t q_ji{C_j.transpose() * C_i};
    const mat4_t q_left_dq_right = quat_left(q_ji) * quat_right(dq_);
    const mat3_t dtheta_dCi = -(q_left_dq_right).block<3, 3>(1, 1);

    Eigen::Map<mat_t<15, 7, row_major_t>> J(jacs[0]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_iT;         // dr w.r.t r_i
    J.block<3, 3>(0, 3) = skew(dr_meas); // dr w.r.t C_i
    J.block<3, 3>(3, 3) = skew(dv_meas); // dv w.r.t C_i
    J.block<3, 3>(6, 3) = dtheta_dCi;    // dtheta w.r.t C_i
    J = sqrt_info_ * J;

    if (min_jacs && min_jacs[0]) {
      Eigen::Map<mat_t<15, 6, row_major_t>> min_J(min_jacs[0]);
      min_J = J.block<15, 6>(0, 0);
    }
  }

  // Jacobian w.r.t speed and biases i
  if (jacs[1]) {
    const quat_t dq_ji{C_j.transpose() * C_i * dq_.toRotationMatrix()};
    const mat3_t dQ_left_xyz = quat_left(dq_ji).block<3, 3>(1, 1);

    Eigen::Map<mat_t<15, 9, row_major_t>> J(jacs[1]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_i.transpose() * Dt_; // dr w.r.t v_i
    J.block<3, 3>(0, 3) = -dr_dba;                // dr w.r.t ba
    J.block<3, 3>(0, 6) = -dr_dbg;                // dr w.r.t bg
    J.block<3, 3>(3, 0) = -C_i.transpose();       // dv w.r.t v_i
    J.block<3, 3>(3, 3) = -dv_dba;                // dv w.r.t ba
    J.block<3, 3>(3, 6) = -dv_dbg;                // dv w.r.t bg
    J.block<3, 3>(6, 6) = -dQ_left_xyz * dq_dbg;  // dtheta w.r.t C_i
    J.block<3, 3>(9, 3) = -I(3);
    J.block<3, 3>(12, 6) = -I(3);
    J = sqrt_info_ * J;

    if (min_jacs && min_jacs[1]) {
      Eigen::Map<mat_t<15, 9, row_major_t>> min_J(min_jacs[1]);
      min_J = J;
    }
  }

  // Jacobian w.r.t. pose j
  if (jacs[2]) {
    const quat_t error_rot = dq_.inverse() * (q_i.inverse() * q_j);
    const mat3_t dtheta_dCj = quat_left(error_rot).block<3, 3>(1, 1);

    Eigen::Map<mat_t<15, 7, row_major_t>> J(jacs[2]);
    J.setZero();
    J.block<3, 3>(0, 0) = C_i.transpose(); // dr w.r.t r_j
    J.block<3, 3>(6, 3) = dtheta_dCj;      // dtheta w.r.t C_j
    J = sqrt_info_ * J;

    if (min_jacs && min_jacs[2]) {
      Eigen::Map<mat_t<15, 6, row_major_t>> min_J(min_jacs[2]);
      min_J = J.block<15, 6>(0, 0);
    }
  }

  //  Jacobian w.r.t. speed and biases j
  if (jacs[3]) {
    Eigen::Map<mat_t<15, 9, row_major_t>> J(jacs[3]);
    J.setZero();
    J.block<3, 3>(3, 0) = C_i.transpose(); // dv w.r.t v_j
    J.block<3, 3>(9, 3) = I(3);
    J.block<3, 3>(12, 6) = I(3);
    J = sqrt_info_ * J;

    if (min_jacs && min_jacs[3]) {
      Eigen::Map<mat_t<15, 9, row_major_t>> min_J(min_jacs[3]);
      min_J = J;
    }
  }

  return true;
}

} // namespace yac
