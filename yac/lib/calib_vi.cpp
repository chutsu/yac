#include "calib_vi.hpp"

namespace yac {

// POSE ERROR //////////////////////////////////////////////////////////////////

pose_error_t::pose_error_t(const pose_t &pose, const matx_t &covar)
    : pose_meas_{pose}, covar_{covar}, info_{covar.inverse()},
      sqrt_info_{info_.llt().matrixL().transpose()} {}

bool pose_error_t::Evaluate(double const *const *params,
                            double *residuals,
                            double **jacobians) const {
  // Parameters
  mat4_t pose_meas = pose_meas_.tf();
  mat4_t pose_est = tf(params[0]);

  // Error
  const mat4_t dpose = pose_meas * pose_est.inverse();
  const vec3_t dr = tf_trans(pose_meas) - tf_trans(pose_est);
  const mat3_t dC = tf_rot(dpose);
  const quat_t dq{dC};
  const vec3_t dtheta = 2 * vec3_t{dq.x(), dq.y(), dq.z()};
  vec6_t error;
  error.head<3>() = dr;
  error.tail<3>() = dtheta;

  // Residuals
  Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
  r = sqrt_info_ * error;

  // Jacobians
  if (jacobians != NULL && jacobians[0] != NULL) {
    mat_t<6, 6> J_min;
    J_min.setIdentity();
    J_min *= -1.0;
    J_min.block<3, 3>(3, 3) = -plus(dq).topLeftCorner<3, 3>();
    J_min = (sqrt_info_ * J_min).eval();

    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[0]);
    J.setZero();
    J.block(0, 0, 6, 6) = J_min;
  }

  return true;
}

// FIDUCIAL ERROR //////////////////////////////////////////////////////////////

fiducial_error_t::fiducial_error_t(const timestamp_t &ts,
                                   const camera_geometry_t *cam_geom,
                                   const int cam_idx,
                                   const int cam_res[2],
                                   const int tag_id,
                                   const int corner_idx,
                                   const vec3_t &r_FFi,
                                   const vec2_t &z,
                                   const mat4_t &T_WF,
                                   const mat2_t &covar)
    : ts_{ts}, cam_geom_{cam_geom}, cam_idx_{cam_idx},
      cam_res_{cam_res[0], cam_res[1]}, tag_id_{tag_id},
      corner_idx_{corner_idx}, r_FFi_{r_FFi}, z_{z}, T_WF_{T_WF}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {
  assert(ts_i != ts_j && ts_j > ts_i);
  assert(cam_res[0] != 0 && cam_res[1] != 0);
  assert(tag_id_ >= 0);
  assert(corner_idx_ >= 0);

  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // T_WF
  block_sizes->push_back(7); // T_WS
  block_sizes->push_back(7); // T_BS
  block_sizes->push_back(7); // T_BCi
  block_sizes->push_back(8); // camera parameters
}

bool fiducial_error_t::Evaluate(double const *const *params,
                                double *residuals,
                                double **jacobians) const {
  // Map parameters
  // #if FIDUCIAL_PARAMS_SIZE == 2
  //   const double roll = params[0][0];
  //   const double pitch = params[0][1];
  //   const double yaw = quat2euler(tf_quat(T_WF_))(2);
  //   const vec3_t rpy{roll, pitch, yaw};
  //   const mat3_t C_WF = euler321(rpy);
  //   const vec3_t r_WF = tf_trans(T_WF_);
  //   const mat4_t T_WF = tf(C_WF, r_WF);
  // #elif FIDUCIAL_PARAMS_SIZE == 3
  //   const double roll = params[0][0];
  //   const double pitch = params[0][1];
  //   const double yaw = params[0][2];
  //   const vec3_t rpy{roll, pitch, yaw};
  //   const mat3_t C_WF = euler321(rpy);
  //   const vec3_t r_WF = tf_trans(T_WF_);
  //   const mat4_t T_WF = tf(C_WF, r_WF);
  // #elif FIDUCIAL_PARAMS_SIZE == 7
  const mat4_t T_WF = tf(params[0]);
  // #endif
  const mat4_t T_WS = tf(params[1]);
  const mat4_t T_BS = tf(params[2]);
  const mat4_t T_BCi = tf(params[3]);
  Eigen::Map<const vecx_t> cam_params(params[4], 8);

  // Transform and project point to image plane
  bool valid = true;
  const mat4_t T_CiB = T_BCi.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
  vec2_t z_hat;
  if (cam_geom_->project(cam_res_, cam_params, r_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residuals
  const vec2_t r = sqrt_info_ * (z_ - z_hat);
  residuals[0] = r.x();
  residuals[1] = r.y();

  // Jacobians
  const matx_t Jh = cam_geom_->project_jacobian(cam_params, r_CiFi);
  const matx_t Jh_weighted = sqrt_info_ * Jh;

  if (jacobians) {
    // Jacobians w.r.t. T_WF
    if (jacobians[0]) {
      // #if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
      //       const double cphi = cos(roll);
      //       const double sphi = sin(roll);
      //       const double ctheta = cos(pitch);
      //       const double stheta = sin(pitch);
      //       const double cpsi = cos(yaw);
      //       const double spsi = sin(yaw);
      //
      //       const double x = r_FFi_(0);
      //       const double y = r_FFi_(1);
      //       const double z = r_FFi_(2);
      //
      //       const vec3_t J_x{y * (sphi * spsi + stheta * cphi * cpsi) +
      //                            z * (-sphi * stheta * cpsi + spsi * cphi),
      //                        y * (-sphi * cpsi + spsi * stheta * cphi) +
      //                            z * (-sphi * spsi * stheta - cphi * cpsi),
      //                        y * cphi * ctheta - z * sphi * ctheta};
      //       const vec3_t J_y{-x * stheta * cpsi + y * sphi * cpsi * ctheta +
      //                            z * cphi * cpsi * ctheta,
      //                        -x * spsi * stheta + y * sphi * spsi * ctheta +
      //                            z * spsi * cphi * ctheta,
      //                        -x * ctheta - y * sphi * stheta - z * stheta *
      //                        cphi};
      //       const vec3_t J_z{-x * spsi * ctheta +
      //                            y * (-sphi * spsi * stheta - cphi * cpsi) +
      //                            z * (sphi * cpsi - spsi * stheta * cphi),
      //                        x * cpsi * ctheta +
      //                            y * (sphi * stheta * cpsi - spsi * cphi) +
      //                            z * (sphi * spsi + stheta * cphi * cpsi),
      //                        0};
      // #endif

      // Fill Jacobian
      const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
      // #if FIDUCIAL_PARAMS_SIZE == 2
      //       Eigen::Map<mat_t<2, 2, row_major_t>> J(jacobians[0]);
      //       J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      //       J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
      // #elif FIDUCIAL_PARAMS_SIZE == 3
      //       Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[0]);
      //       J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      //       J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
      //       J.block(0, 2, 2, 1) = -1 * Jh_weighted * C_CiW * J_z;
      // #elif FIDUCIAL_PARAMS_SIZE == 7
      const mat3_t C_WF = tf_rot(T_WF);
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiW * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiW * -skew(C_WF * r_FFi_);
      // #endif
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_WS
    if (jacobians[1]) {
      const mat3_t C_CiS = tf_rot(T_CiB * T_BS);
      const mat3_t C_WS = tf_rot(T_WS);
      const mat3_t C_SW = C_WS.transpose();
      const mat3_t C_CiW = C_CiS * C_SW;
      const mat4_t T_SW = T_WS.inverse();
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiW * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiW * -skew(C_WS * r_SFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BS
    if (jacobians[2]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BS = tf_rot(T_BS);
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BCi
    if (jacobians[3]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BCi = C_CiB.transpose();
      const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[3]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiB * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiB * -skew(C_BCi * r_CiFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t. camera parameters
    if (jacobians[4]) {
      const matx_t J_cam = cam_geom_->params_jacobian(cam_params, r_CiFi);
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[4]);
      J.block(0, 0, 2, 8) = -1 * sqrt_info_ * J_cam;

      if (valid == false) {
        J.setZero();
      }
    }

    // // Jacobians w.r.t. time delay
    // if (jacobians[5]) {
    //   Eigen::Map<vec2_t> J(jacobians[5]);
    //   J = sqrt_info_ * v_ij_;
    //   if (valid == false) {
    //     J.setZero();
    //   }
    // }
  }

  return true;
}

// FIDUCIAL WITH TIME DELAY ERROR //////////////////////////////////////////////

fiducial_td_error_t::fiducial_td_error_t(const timestamp_t &ts_i,
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
                                         const mat2_t &covar)
    : ts_i_{ts_i}, ts_j_{ts_j}, cam_geom_{cam_geom}, cam_idx_{cam_idx},
      cam_res_{cam_res[0], cam_res[1]}, tag_id_{tag_id},
      corner_idx_{corner_idx}, r_FFi_{r_FFi}, z_i_{z_i}, z_j_{z_j},
      v_ij_{(z_j_ - z_i_) / ns2sec(ts_j_ - ts_i_)}, T_WF_{T_WF}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixL().transpose()} {
  assert(ts_i != ts_j && ts_j > ts_i);
  assert(cam_res[0] != 0 && cam_res[1] != 0);
  assert(tag_id_ >= 0);
  assert(corner_idx_ >= 0);
}

bool fiducial_td_error_t::Evaluate(double const *const *params,
                                   double *residuals,
                                   double **jacobians) const {
  // Pose of fiducial target in world frame
#if FIDUCIAL_PARAMS_SIZE == 2
  const double roll = params[0][0];
  const double pitch = params[0][1];
  const double yaw = quat2euler(tf_quat(T_WF_))(2);
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF_);
  const mat4_t T_WF = tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 3
  const double roll = params[0][0];
  const double pitch = params[0][1];
  const double yaw = params[0][2];
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF_);
  const mat4_t T_WF = tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 7
  const mat4_t T_WF = tf(params[0]);
#endif

  // Sensor pose, sensor-camera extrinsics, camera parameters
  const mat4_t T_WS = tf(params[1]);
  const mat4_t T_BS = tf(params[2]);
  const mat4_t T_BCi = tf(params[3]);
  Eigen::Map<const vecx_t> cam_params(params[4], 8);
  const double td = params[5][0];

  // Transform and project point to image plane
  bool valid = true;
  const mat4_t T_CiB = T_BCi.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
  vec2_t z_hat;
  if (cam_geom_->project(cam_res_, cam_params, r_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residuals
  const vec2_t r = sqrt_info_ * ((z_i_ + td * v_ij_) - z_hat);
  residuals[0] = r.x();
  residuals[1] = r.y();

  // Jacobians
  const matx_t Jh = cam_geom_->project_jacobian(cam_params, r_CiFi);
  const matx_t Jh_weighted = sqrt_info_ * Jh;

  if (jacobians) {
    // Jacobians w.r.t. T_WF
    if (jacobians[0]) {
      // Form jacobians
#if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
      const double cphi = cos(roll);
      const double sphi = sin(roll);
      const double ctheta = cos(pitch);
      const double stheta = sin(pitch);
      const double cpsi = cos(yaw);
      const double spsi = sin(yaw);

      const double x = r_FFi_(0);
      const double y = r_FFi_(1);
      const double z = r_FFi_(2);

      const vec3_t J_x{y * (sphi * spsi + stheta * cphi * cpsi) +
                           z * (-sphi * stheta * cpsi + spsi * cphi),
                       y * (-sphi * cpsi + spsi * stheta * cphi) +
                           z * (-sphi * spsi * stheta - cphi * cpsi),
                       y * cphi * ctheta - z * sphi * ctheta};
      const vec3_t J_y{-x * stheta * cpsi + y * sphi * cpsi * ctheta +
                           z * cphi * cpsi * ctheta,
                       -x * spsi * stheta + y * sphi * spsi * ctheta +
                           z * spsi * cphi * ctheta,
                       -x * ctheta - y * sphi * stheta - z * stheta * cphi};
      const vec3_t J_z{-x * spsi * ctheta +
                           y * (-sphi * spsi * stheta - cphi * cpsi) +
                           z * (sphi * cpsi - spsi * stheta * cphi),
                       x * cpsi * ctheta +
                           y * (sphi * stheta * cpsi - spsi * cphi) +
                           z * (sphi * spsi + stheta * cphi * cpsi),
                       0};
#endif

      // Fill Jacobian
      const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
#if FIDUCIAL_PARAMS_SIZE == 2
      Eigen::Map<mat_t<2, 2, row_major_t>> J(jacobians[0]);
      J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
#elif FIDUCIAL_PARAMS_SIZE == 3
      Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[0]);
      J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
      J.block(0, 2, 2, 1) = -1 * Jh_weighted * C_CiW * J_z;
#elif FIDUCIAL_PARAMS_SIZE == 7
      const mat3_t C_WF = tf_rot(T_WF);
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiW * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiW * -skew(C_WF * r_FFi_);
#endif
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_WS
    if (jacobians[1]) {
      const mat3_t C_CiS = tf_rot(T_CiB * T_BS);
      const mat3_t C_WS = tf_rot(T_WS);
      const mat3_t C_SW = C_WS.transpose();
      const mat3_t C_CiW = C_CiS * C_SW;
      const mat4_t T_SW = T_WS.inverse();
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();
      J.block(0, 0, 2, 3) = Jh_weighted * C_CiW * I(3);
      J.block(0, 3, 2, 3) = Jh_weighted * C_CiW * -skew(C_WS * r_SFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BS
    if (jacobians[2]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BS = tf_rot(T_BS);
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BCi
    if (jacobians[3]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BCi = C_CiB.transpose();
      const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[3]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiB * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiB * -skew(C_BCi * r_CiFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t. camera parameters
    if (jacobians[4]) {
      const matx_t J_cam = cam_geom_->params_jacobian(cam_params, r_CiFi);
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[4]);
      J.block(0, 0, 2, 8) = -1 * sqrt_info_ * J_cam;

      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t. time delay
    if (jacobians[5]) {
      Eigen::Map<vec2_t> J(jacobians[5]);
      J = sqrt_info_ * v_ij_;
      if (valid == false) {
        J.setZero();
      }
    }
  }

  return true;
}

// INERTIAL ERROR //////////////////////////////////////////////////////////////

ImuError::ImuError(const imu_data_t &imu_data,
                   const imu_params_t &imu_params,
                   const timestamp_t &t0,
                   const timestamp_t &t1) {
  imu_data_ = imu_data;
  imu_params_ = imu_params;
  t0_ = t0;
  t1_ = t1;
}

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
  const vec3_t trans_update =
      r_0 + sb.head<3>() * Delta_t +
      C_WS_0 * (acc_doubleintegral /*-C_doubleintegral*sb.segment<3>(6)*/) -
      0.5 * g_W * Delta_t * Delta_t;
  const quat_t quat_update = q_WS_0 * Delta_q;
  T_WS = tf(quat_update, trans_update);
  sb.head<3>() +=
      C_WS_0 * (acc_integral /*-C_integral*sb.segment<3>(6)*/) - g_W * Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double, 15, 15> &F = *jacobian;
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g,
                     // d/db_a
    F.block<3, 3>(0, 3) = -skew(C_WS_0 * acc_doubleintegral);
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

int ImuError::redoPreintegration(const mat4_t & /*T_WS*/,
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
    dt = 0.005;
    // printf("dt: %f\n", dt);

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

bool ImuError::Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

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

  // Keep track of evaulation values
  T_WS_0_last_ = T_WS_0;
  T_WS_1_last_ = T_WS_1;
  sb0_last_ = sb0;
  sb1_last_ = sb1;
  timestamp_t time_start = t0_;
  timestamp_t time_end = t1_;

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
  int nb_meas = redoPreintegration(T_WS_0, sb0, time_start, time_end);
  bool valid = true;
  if (nb_meas == 0) {
    valid = false;
  }
  Delta_b.setZero();

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

    // Error weighting
    // clang-format off
      Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error(residuals);
      weighted_error = squareRootInformation_ * error;

      // Get the Jacobians
      if (jacobians != NULL) {
        // Sensor pose 0
        // clang-format off
        if (jacobians[0] != NULL) {
          // Jacobian w.r.t. minimal perturbance
          Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_ * F0.block<15, 6>(0, 0);

          Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
          J0.setZero();
          J0.block(0, 0, 15, 6) = J0_minimal;
          if (valid == false) {
            J0.setZero();
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
          if (valid == false) {
            J1.setZero();
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
          if (valid == false) {
            J2.setZero();
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
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3(
            jacobians[3]);
        J3.setZero();
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);
        if (valid == false) {
          J3.setZero();
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                J3_minimal_mapped(jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }

    } // jacobian
  }   // mutex

  // eval = true;
  return true;
}

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

calib_vi_t::calib_vi_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem = new ceres::Problem{prob_options};
}

calib_vi_t::~calib_vi_t() {
  for (auto &[cam_idx, cam] : cam_params) {
    if (cam) {
      delete cam;
    }
  }

  for (auto &[cam_idx, exts] : cam_exts) {
    if (exts) {
      delete exts;
    }
  }

  if (imu_exts) {
    delete imu_exts;
  }

  if (fiducial) {
    delete fiducial;
  }

  if (time_delay) {
    delete time_delay;
  }

  if (problem) {
    delete problem;
  }
}

void calib_vi_t::add_imu(const imu_params_t &imu_params_,
                         const mat4_t &T_BS,
                         const double td,
                         const bool fix_extrinsics,
                         const bool fix_time_delay) {
  // Imu parameters
  imu_params = imu_params_;

  // Imu extrinsics
  imu_exts = new extrinsics_t{T_BS};
  problem->AddParameterBlock(imu_exts->param.data(), 7);
  problem->SetParameterization(imu_exts->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(imu_exts->param.data());
    imu_exts->fixed = true;
  }

  // Imu time delay
  time_delay = new time_delay_t{td};
  problem->AddParameterBlock(time_delay->param.data(), 1);
  if (fix_time_delay) {
    problem->SetParameterBlockConstant(time_delay->param.data());
    time_delay->fixed = true;
  }
}

void calib_vi_t::add_camera(const int cam_idx,
                            const int resolution[2],
                            const std::string &proj_model,
                            const std::string &dist_model,
                            const vecx_t &proj_params,
                            const vecx_t &dist_params,
                            const mat4_t &T_BCi,
                            const bool fix_params,
                            const bool fix_extrinsics) {
  // Camera parameters
  cam_params[cam_idx] = new camera_params_t{cam_idx,
                                            resolution,
                                            proj_model,
                                            dist_model,
                                            proj_params,
                                            dist_params};
  if (fix_params) {
    auto data_ptr = cam_params[cam_idx]->param.data();
    auto block_size = cam_params[cam_idx]->global_size;
    cam_params[cam_idx]->fixed = true;
    problem->AddParameterBlock(data_ptr, block_size);
    problem->SetParameterBlockConstant(data_ptr);
  }

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = &pinhole_radtan4;
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = &pinhole_equi4;
  } else {
    FATAL("Invalid [%s-%s] camera model!",
          proj_model.c_str(),
          dist_model.c_str());
  }

  // Camera extrinsics
  cam_exts[cam_idx] = new extrinsics_t{T_BCi};
  problem->AddParameterBlock(cam_exts[cam_idx]->param.data(), 7);
  problem->SetParameterization(cam_exts[cam_idx]->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(cam_exts[cam_idx]->param.data());
    cam_exts[cam_idx]->fixed = true;
  }
}

int calib_vi_t::nb_cams() { return cam_params.size(); }

veci2_t calib_vi_t::get_cam_resolution(const int cam_idx) {
  auto cam_res = cam_params[cam_idx]->resolution;
  return veci2_t{cam_res[0], cam_res[1]};
}

vecx_t calib_vi_t::get_cam_params(const int cam_idx) {
  return cam_params[cam_idx]->param;
}

mat4_t calib_vi_t::get_cam_extrinsics(const int cam_idx) {
  return cam_exts[cam_idx]->tf();
}

mat4_t calib_vi_t::get_imu_extrinsics() { return imu_exts->tf(); }

// mat4_t calib_vi_t::get_sensor_pose(const int pose_index) {
//   return sensor_poses[pose_index]->tf();
// }

mat4_t calib_vi_t::get_fiducial_pose() { return fiducial->estimate(); }

// void calib_vi_t::add_reproj_errors(
//     const int cam_idx,
//     const aprilgrid_t &grid_j,
//     std::map<int, std::vector<ceres::ResidualBlockId>> &error_ids,
//     std::map<int, std::vector<reproj_error_td_t<pinhole_radtan4_t> *>>
//         &errors) {
//   assert(cam_params.count(cam_idx) > 0);
//   assert(cam_exts.count(cam_idx) > 0);
//   assert(imu_exts != nullptr);
//   assert(sensor_poses.size() >= 1);
//
//   const aprilgrid_t &grid_i = grids_prev[cam_idx];
//   const auto ts_i = grid_i.timestamp;
//   const auto ts_j = grid_j.timestamp;
//   const auto &T_WF = fiducial;
//   const auto &T_WS_i = sensor_poses[sensor_poses.size() - 2];
//   const auto &T_BS = imu_exts;
//   const auto &T_BCi = cam_exts[cam_idx];
//   const auto &cam = cam_params[cam_idx];
//   const int *cam_res = cam->resolution;
//   const mat2_t covar = pow(sigma_vision, 2) * I(2);
//
//   std::vector<int> tag_ids;
//   std::vector<int> corner_indicies;
//   vec2s_t grid_i_keypoints;
//   vec2s_t grid_j_keypoints;
//   vec3s_t object_points;
//   aprilgrid_t::common_measurements(grid_i,
//                                    grid_j,
//                                    tag_ids,
//                                    corner_indicies,
//                                    grid_i_keypoints,
//                                    grid_j_keypoints,
//                                    object_points);
//
//   for (size_t i = 0; i < tag_ids.size(); i++) {
//     const int tag_id = tag_ids[i];
//     const int corner_idx = corner_indicies[i];
//     const vec2_t z_i = grid_i_keypoints[i];
//     const vec2_t z_j = grid_j_keypoints[i];
//     const vec3_t r_FFi = object_points[i];
//
//     auto error = new reproj_error_td_t<pinhole_radtan4_t>(ts_i,
//                                                           ts_j,
//                                                           cam_res,
//                                                           tag_id,
//                                                           corner_idx,
//                                                           r_FFi,
//                                                           z_i,
//                                                           z_j,
//                                                           fiducial->estimate(),
//                                                           covar);
//     auto error_id = problem->AddResidualBlock(error,
//                                               NULL,
//                                               T_WF->param.data(),
//                                               T_WS_i->param.data(),
//                                               T_BS->param.data(),
//                                               T_BCi->param.data(),
//                                               cam->param.data(),
//                                               time_delay->param.data());
//     error_ids[cam_idx].push_back(error_id);
//     errors[cam_idx].push_back(error);
//   }
// }

// ImuError
// *calib_vi_t::add_imu_error(ceres::ResidualBlockId
// &error_id) {
//   pose_t *pose_i =
//   sensor_poses[sensor_poses.size() - 2]; pose_t
//   *pose_j = sensor_poses[sensor_poses.size() -
//   1]; sb_params_t *sb_i =
//   speed_biases[speed_biases.size() - 2];
//   sb_params_t *sb_j =
//   speed_biases[speed_biases.size() - 1]; const
//   auto t0 = pose_i->ts; const auto t1 =
//   pose_j->ts;
//
//   auto error = new ImuError(imu_buf, imu_params,
//   t0, t1); error_id =
//   problem->AddResidualBlock(error,
//                                        NULL,
//                                        pose_i->param.data(),
//                                        sb_i->param.data(),
//                                        pose_j->param.data(),
//                                        sb_j->param.data());
//
//   return error;
// }

mat4_t calib_vi_t::estimate_sensor_pose(const CameraGrids &grids) {
  assert(grids.size() > 0);
  assert(initialized);

  bool detected = false;
  for (const auto &[cam_idx, grid] : grids) {
    if (grid.detected) {
      detected = true;
    }
  }
  if (detected == false) {
    FATAL("No fiducials detected!");
  }

  for (const auto &[cam_idx, grid] : grids) {
    // Skip if not detected
    if (grid.detected == false) {
      continue;
    }

    // Estimate relative pose
    const camera_geometry_t *cam = cam_geoms.at(cam_idx);
    const auto cam_res = get_cam_resolution(cam_idx).data();
    const vecx_t cam_params = get_cam_params(cam_idx);
    mat4_t T_CiF = I(4);
    if (grid.estimate(cam, cam_res, cam_params, T_CiF) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Infer current pose T_WS using T_CiF, T_BCi and T_WF
    const mat4_t T_FCi_k = T_CiF.inverse();
    const mat4_t T_BCi = get_cam_extrinsics(cam_idx);
    const mat4_t T_BS = get_imu_extrinsics();
    const mat4_t T_CiS = T_BCi.inverse() * T_BS;
    const mat4_t T_WF = get_fiducial_pose();
    const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;
    return T_WS_k;
  }

  // Should never reach here
  FATAL("Implementation Error!");
  return zeros(4, 4);
}

void calib_vi_t::initialize(const timestamp_t &ts,
                            const CameraGrids &grids,
                            imu_data_t &imu_buf) {
  assert(grids.size() > 0);
  assert(grids.at(0).detected);

  // Estimate relative pose - T_C0F
  const camera_geometry_t *cam = cam_geoms.at(0);
  const int *cam_res = cam_params.at(0)->resolution;
  const vecx_t params = get_cam_params(0);
  mat4_t T_C0F;
  if (grids.at(0).estimate(cam, cam_res, params, T_C0F) != 0) {
    FATAL("Failed to estimate relative pose!");
    return;
  }

  // Estimate initial IMU attitude
  mat3_t C_WS = imu_buf.initial_attitude();

  // Sensor pose - T_WS
  mat4_t T_WS = tf(C_WS, zeros(3, 1));

  // Fiducial pose - T_WF
  const mat4_t T_BC0 = get_cam_extrinsics(0);
  const mat4_t T_BS = get_imu_extrinsics();
  const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
  mat4_t T_WF = T_WS * T_SC0 * T_C0F;

  // Set:
  // 1. Fiducial target as origin (with r_WF (0, 0, 0))
  // 2. Calculate the sensor pose offset relative to target origin
  const vec3_t offset = -1.0 * tf_trans(T_WF);
  const mat3_t C_WF = tf_rot(T_WF);
  const vec3_t r_WF{0.0, 0.0, 0.0};
  T_WF = tf(C_WF, r_WF);
  T_WS = tf(C_WS, offset);

  // Finish up
  // add_sensor_pose(ts, T_WS);
  // add_speed_biases(ts, zeros(9, 1));
  fiducial = new fiducial_t{T_WF};
  problem->AddParameterBlock(fiducial->param.data(), FIDUCIAL_PARAMS_SIZE);

  LOG_INFO("Initialize:");
  print_matrix("T_WS", T_WS);
  print_matrix("T_WF", T_WF);
  print_matrix("T_BS", T_BS);
  print_matrix("T_BC0", get_cam_extrinsics(0));
  print_matrix("T_BC1", get_cam_extrinsics(1));

  initialized = true;
  prev_grids = grids;
}

void calib_vi_t::add_view(const CameraGrids &grids) {
  // Check aprilgrids, make sure there is atleast 1 detected grid
  bool grid_detected = false;
  timestamp_t grid_ts = 0;
  for (int i = 0; i < nb_cams(); i++) {
    if (grids.at(i).detected) {
      grid_ts = grids.at(i).timestamp;
      grid_detected = true;
      break;
    }
  }
  if (grid_detected == false) {
    return;
  }

  // Estimate current pose
  LOG_INFO("Add view: %ld", grid_ts);
  const mat4_t T_WS_k = estimate_sensor_pose(grids);

  // // Propagate imu measurements to obtain speed and biases
  // const auto t0 = sensor_poses.back()->ts;
  // const auto t1 = ts;
  // mat4_t T_WS = sensor_poses.back()->tf();
  // vec_t<9> sb_k = speed_biases.back()->param;
  // ImuError::propagation(imu_buf, imu_params, T_WS, sb_k, t0, t1);

  // // Infer velocity from two poses T_WS_k and
  // // T_WS_km1
  // const mat4_t T_WS_km1 = tf(sensor_poses.back()->param);
  // const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
  // const vec3_t r_WS_k = tf_trans(T_WS_k);
  // const vec3_t v_WS_k = r_WS_k - r_WS_km1;
  // vec_t<9> sb_k;
  // sb_k.setZero();
  // sb_k << v_WS_k, zeros(3, 1), zeros(3, 1);

  // Add updated sensor pose T_WS_k and speed and biases sb_k Note: instead of
  // using the propagated sensor pose `T_WS`, we are using the provided
  // `T_WS_`, this is because in the scenario where we are using AprilGrid, as
  // a fiducial target, we actually have a better estimation of the sensor
  // pose, as supposed to the imu propagated sensor pose.
  // add_sensor_pose(ts, T_WS_k);
  // add_speed_biases(ts, sb_k);

  // Add inertial factors
  // ceres::ResidualBlockId imu_error_id = nullptr;
  // add_imu_error(imu_error_id);

  // Add vision factors
  // std::map<int,
  // std::vector<ceres::ResidualBlockId>>
  // reproj_error_ids; std::map<int,
  // std::vector<reproj_error_td_t<pinhole_radtan4_t>
  // *>>
  //     reproj_errors;
  // for (int i = 0; i < nb_cams(); i++) {
  //   add_reproj_errors(i, grids[i],
  //   reproj_error_ids, reproj_errors);
  // }

  // Add view to sliding window
  prev_grids = grids;
}

void calib_vi_t::add_measurement(const int cam_idx, const aprilgrid_t &grid) {
  grid_buf[cam_idx].push_back(grid);
}

void calib_vi_t::add_measurement(const timestamp_t imu_ts,
                                 const vec3_t &a_m,
                                 const vec3_t &w_m) {
  imu_buf.add(imu_ts, a_m, w_m);

  if (static_cast<int>(grid_buf.size()) == nb_cams()) {
    // Get camera grids
    CameraGrids grids;
    for (auto &[cam_idx, cam_grids] : grid_buf) {
      grids[cam_idx] = cam_grids.front();
    }
    grid_buf.clear();

    // Initialize T_WS and T_WF
    if (initialized == false) {
      if (grids.at(0).detected && imu_buf.size() > 2) {
        const auto ts = imu_buf.timestamps.front();
        initialize(ts, grids, imu_buf);
      }
      return;
    }

    // Add new view
    add_view(grids);
  }
}

// void calib_vi_t::solve(bool verbose = true) {
//   // Optimize problem - first pass
//   {
//     // LOG_INFO("Optimize problem - first pass");
//     ceres::Solver::Options options;
//     options.minimizer_progress_to_stdout =
//     verbose; options.max_num_iterations =
//     batch_max_iter;
//
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, problem, &summary);
//     std::cout << summary.BriefReport() <<
//     std::endl; if (verbose) {
//       std::cout << summary.BriefReport() <<
//       std::endl; show_results();
//     }
//   }
//
//   // Filter outliers
//   if (enable_outlier_rejection) {
//     // LOG_INFO("Filter outliers");
//     int nb_outliers = 0;
//     int nb_inliers = 0;
//     for (int cam_idx = 0; cam_idx < nb_cams();
//     cam_idx++) {
//       // Obtain all reprojection errors from all
//       views std::vector<double> errs; for (auto
//       &view : sliding_window) {
//         view.calculate_reproj_errors(cam_idx,
//         errs);
//       }
//
//       // Filter outliers from last view
//       const double threshold = 3.0 *
//       stddev(errs); for (auto &view :
//       sliding_window) {
//         nb_outliers += view.filter(cam_idx,
//         threshold); nb_inliers +=
//         view.reproj_errors.size();
//       }
//     }
//     // LOG_INFO("Removed %d outliers!",
//     nb_outliers);
//     // printf("\n");
//
//     // Optimize problem - second pass
//     {
//       // LOG_INFO("Optimize problem - second
//       pass");
//       // Solver options
//       ceres::Solver::Options options;
//       options.minimizer_progress_to_stdout =
//       true; options.max_num_iterations =
//       batch_max_iter;
//
//       // Solve
//       ceres::Solver::Summary summary;
//       ceres::Solve(options, problem, &summary);
//       if (verbose) {
//         std::cout << summary.FullReport() <<
//         std::endl; show_results();
//       }
//     }
//   }
// }
//
// void calib_vi_t::show_results() {
//   // Show results
//   std::map<int, std::vector<double>> cam_errs =
//   get_camera_errors();
//
//   printf("Optimization results:\n");
//   printf("---------------------\n");
//
//   // Stats
//   printf("stats:\n");
//   for (int cam_idx = 0; cam_idx < nb_cams();
//   cam_idx++) {
//     printf("cam%d reprojection error [px]: ",
//     cam_idx); printf("[rmse: %f,",
//     rmse(cam_errs[cam_idx])); printf(" mean:
//     %f,", mean(cam_errs[cam_idx])); printf("
//     median: %f]\n", median(cam_errs[cam_idx]));
//   }
//   printf("\n");
//
//   // Cameras
//   for (int cam_idx = 0; cam_idx < nb_cams();
//   cam_idx++) {
//     printf("cam%d:\n", cam_idx);
//     print_vector("proj_params",
//     cam_params[cam_idx]->proj_params());
//     print_vector("dist_params",
//     cam_params[cam_idx]->dist_params());
//     printf("\n");
//   }
//
//   // Time-delay
//   if (time_delay) {
//     print_vector("time delay (ts_cam = ts_imu +
//     td):", time_delay->param); printf("\n");
//   }
//
//   // Imu extrinsics
//   print_matrix("T_BS", get_imu_extrinsics());
//   print_matrix("T_SB",
//   get_imu_extrinsics().inverse());
//
//   // Camera Extrinsics
//   for (int cam_idx = 1; cam_idx < nb_cams();
//   cam_idx++) {
//     const auto key = "T_C0C" +
//     std::to_string(cam_idx); const mat4_t T_C0Ci
//     = get_cam_extrinsics(cam_idx);
//     print_matrix(key, T_C0Ci);
//
//     {
//       const auto key = "T_C" +
//       std::to_string(cam_idx) + "C0"; const
//       mat4_t T_CiC0 = T_C0Ci.inverse();
//       print_matrix(key, T_CiC0);
//     }
//   }
// }
//
// int calib_vi_t::recover_calib_covar(matx_t
// &calib_covar) {
//   // Recover calibration covariance
//   auto T_BS = imu_exts->param.data();
//   auto td = time_delay->param.data();
//
//   // -- Setup covariance blocks to estimate
//   std::vector<std::pair<const double *, const
//   double *>> covar_blocks;
//   covar_blocks.push_back({T_BS, T_BS});
//   covar_blocks.push_back({td, td});
//
//   // -- Estimate covariance
//   ::ceres::Covariance::Options options;
//   ::ceres::Covariance covar_est(options);
//   if (covar_est.Compute(covar_blocks, problem) ==
//   false) {
//     LOG_ERROR("Failed to estimate covariance!");
//     LOG_ERROR("Maybe Hessian is not full rank?");
//     return -1;
//   }
//
//   // -- Extract covariances sub-blocks
//   Eigen::Matrix<double, 6, 6, Eigen::RowMajor>
//   T_BS_covar; Eigen::Matrix<double, 1, 1,
//   Eigen::RowMajor> td_covar;
//   covar_est.GetCovarianceBlockInTangentSpace(T_BS,
//   T_BS, T_BS_covar.data());
//   covar_est.GetCovarianceBlock(td, td,
//   td_covar.data());
//
//   // -- Form covariance matrix block
//   calib_covar = zeros(7, 7);
//   calib_covar.block(0, 0, 6, 6) = T_BS_covar;
//   calib_covar.block(6, 6, 1, 1) = td_covar;
//
//   // -- Check if calib_covar is full-rank?
//   if (rank(calib_covar) != calib_covar.rows()) {
//     LOG_ERROR("calib_covar is not full rank!");
//     return -1;
//   }
//
//   return 0;
// }

int calib_vi_t::save_results(const std::string &save_path) {
  LOG_INFO("Saved results to [%s]", save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Calibration metrics
  // std::map<int, std::vector<double>> cam_errs = get_camera_errors();
  // fprintf(outfile, "calib_metrics:\n");
  // for (const auto &kv : cam_errs) {
  //   const auto cam_idx = kv.first;
  //   const auto cam_str = "cam" + std::to_string(cam_idx);
  //   const auto cam_errs = kv.second;
  //   fprintf(outfile, "  %s: ", cam_str.c_str());
  //   fprintf(outfile, "[");
  //   fprintf(outfile, "%f, ", rmse(cam_errs));
  //   fprintf(outfile, "%f, ", mean(cam_errs));
  //   fprintf(outfile, "%f, ", median(cam_errs));
  //   fprintf(outfile, "%f", stddev(cam_errs));
  //   fprintf(outfile, "]  # rmse, mean, median, stddev\n");
  // }
  // fprintf(outfile, "\n");
  // fprintf(outfile, "\n");

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = cam_params[cam_idx];
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const std::string proj_params = vec2str(cam->proj_params(), 4);
    const std::string dist_params = vec2str(cam->dist_params(), 4);

    fprintf(outfile, "cam%d:\n", cam_idx);
    fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }
  fprintf(outfile, "\n");

  // // IMU parameters
  // vec3s_t bias_acc;
  // vec3s_t bias_gyr;
  // for (size_t i = 0; i < speed_biases.size(); i++) {
  //   auto sb = speed_biases[i];
  //   bias_gyr.push_back(sb->param.segment<3>(3));
  //   bias_acc.push_back(sb->param.segment<3>(6));
  // }
  // const vec3_t mu_ba = mean(bias_acc);
  // const vec3_t mu_bg = mean(bias_gyr);
  // fprintf(outfile, "imu0:\n");
  // fprintf(outfile, "  rate: %f\n", imu_params.rate);
  // fprintf(outfile, "  sigma_a_c: %f\n", imu_params.sigma_a_c);
  // fprintf(outfile, "  sigma_g_c: %f\n", imu_params.sigma_g_c);
  // fprintf(outfile, "  sigma_aw_c: %f\n", imu_params.sigma_aw_c);
  // fprintf(outfile, "  sigma_gw_c: %f\n", imu_params.sigma_gw_c);
  // fprintf(outfile, "  sigma_ba: %f\n", imu_params.sigma_ba);
  // fprintf(outfile, "  sigma_bg: %f\n", imu_params.sigma_bg);
  // fprintf(outfile, "  bg: [%f, %f, %f]\n", mu_bg(0), mu_bg(1), mu_bg(2));
  // fprintf(outfile, "  ba: [%f, %f, %f]\n", mu_ba(0), mu_ba(1), mu_ba(2));
  // fprintf(outfile, "  g: %f\n", imu_params.g);
  // fprintf(outfile, "\n");
  // fprintf(outfile, "\n");

  // Sensor-Camera extrinsics
  for (int i = 0; i < nb_cams(); i++) {
    // const mat4_t T_BS = get_imu_extrinsics();
    const mat4_t T_BS = I(4);
    const mat4_t T_SB = T_BS.inverse();
    const mat4_t T_BCi = get_cam_extrinsics(i);
    const mat4_t T_SCi = T_SB * T_BCi;

    fprintf(outfile, "T_imu0_cam%d:\n", i);
    fprintf(outfile, "  rows: 4\n");
    fprintf(outfile, "  cols: 4\n");
    fprintf(outfile, "  data: [\n");
    fprintf(outfile, "%s\n", mat2str(T_SCi, "    ").c_str());
    fprintf(outfile, "  ]");
    fprintf(outfile, "\n");
    fprintf(outfile, "\n");
  }
  fprintf(outfile, "\n");

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_cam_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cams() >= 2) {
    for (int i = 1; i < nb_cams(); i++) {
      const mat4_t T_BCi = get_cam_extrinsics(i);
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      const mat4_t T_CiC0 = T_C0Ci.inverse();

      fprintf(outfile, "T_cam0_cam%d:\n", i);
      fprintf(outfile, "  rows: 4\n");
      fprintf(outfile, "  cols: 4\n");
      fprintf(outfile, "  data: [\n");
      fprintf(outfile, "%s\n", mat2str(T_C0Ci, "    ").c_str());
      fprintf(outfile, "  ]");
      fprintf(outfile, "\n");
      fprintf(outfile, "\n");

      fprintf(outfile, "T_cam%d_cam0:\n", i);
      fprintf(outfile, "  rows: 4\n");
      fprintf(outfile, "  cols: 4\n");
      fprintf(outfile, "  data: [\n");
      fprintf(outfile, "%s\n", mat2str(T_CiC0, "    ").c_str());
      fprintf(outfile, "  ]");
      fprintf(outfile, "\n");
      fprintf(outfile, "\n");
    }
  }

  // Finsh up
  fclose(outfile);

  return 0;
}

// void calib_vi_t::save_poses(const std::string
// &save_path) {
//   FILE *csv = fopen(save_path.c_str(), "w");
//   fprintf(csv, "#ts,rx,ry,rz,qw,qx,qy,qz\n");
//   for (size_t i = 0; i < sensor_poses.size();
//   i++) {
//     const auto pose = sensor_poses[i];
//     const auto ts = pose->ts;
//     const vec3_t r = pose->trans();
//     const quat_t q = pose->rot();
//     fprintf(csv, "%ld,", ts);
//     fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
//     fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(),
//     q.y(), q.z());
//   }
//   fclose(csv);
// }
//
// void calib_vi_t::save_speed_biases(const
// std::string &save_path) {
//   FILE *csv = fopen(save_path.c_str(), "w");
//   fprintf(csv, "#ts,");
//   fprintf(csv, "vx,vy,vz,");
//   fprintf(csv, "ba_x,ba_y,ba_z,");
//   fprintf(csv, "bg_x,bg_y,bg_z,\n");
//   for (size_t i = 0; i < speed_biases.size();
//   i++) {
//     const auto sb : speed_biases[i];
//     const auto ts = sb->ts;
//     const vec3_t v = sb->param.segment<3>(0);
//     const vec3_t ba = sb->param.segment<3>(3);
//     const vec3_t bg = sb->param.segment<3>(6);
//     fprintf(csv, "%ld,", ts);
//     fprintf(csv, "%f,%f,%f,", v(0), v(1), v(2));
//     fprintf(csv, "%f,%f,%f,", ba(0), ba(1),
//     ba(2)); fprintf(csv, "%f,%f,%f\n", bg(0),
//     bg(1), bg(2));
//   }
//   fclose(csv);
// }
//
// void calib_vi_t::save_cameras(const std::string
// &save_path) {
//   FILE *csv = fopen(save_path.c_str(), "w");
//   fprintf(csv, "#cam_idx,");
//   fprintf(csv, "resolution,");
//   fprintf(csv, "proj_model,");
//   fprintf(csv, "dist_model,");
//   fprintf(csv, "proj_params,");
//   fprintf(csv, "dist_params\n");
//   for (const auto &kv : cam_params) {
//     const auto cam_idx = kv.first;
//     const auto cam = kv.second;
//     const vecx_t proj = cam->proj_params();
//     const vecx_t dist = cam->dist_params();
//     fprintf(csv, "%d,", cam_idx);
//     fprintf(csv, "%d,%d,", cam->resolution[0],
//     cam->resolution[1]); fprintf(csv, "%s,%s,",
//     cam->proj_model.c_str(),
//     cam->dist_model.c_str()); fprintf(csv,
//     "%f,%f,%f,%f,", proj(0), proj(1), proj(2),
//     proj(3)); fprintf(csv, "%f,%f,%f,%f\n",
//     dist(0), dist(1), dist(2), dist(3));
//   }
//   fclose(csv);
// }
//
// void calib_vi_t::save_cam_extrinsics(const
// std::string &save_path) {
//   FILE *csv = fopen(save_path.c_str(), "w");
//   fprintf(csv,
//   "#cam_idx,cam_idx,rx,ry,rz,qw,qx,qy,qz\n");
//
//   const mat4_t T_BC0 = get_cam_extrinsics(0);
//   const mat4_t T_C0B = T_BC0.inverse();
//
//   for (int i = 0; i < nb_cams(); i++) {
//     const mat4_t T_BCi = get_cam_extrinsics(i);
//     const mat4_t T_C0Ci = T_C0B * T_BCi;
//     const vec3_t r = tf_trans(T_C0Ci);
//     const quat_t q = tf_quat(T_C0Ci);
//     fprintf(csv, "0,");     // cam_idx
//     fprintf(csv, "%d,", i); // cam_idx
//     fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
//     fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(),
//     q.y(), q.z());
//   }
//   fclose(csv);
// }
//
// // void calib_vi_t::save_imu_extrinsics(const
// std::string &save_path) {
// //   FILE *csv = fopen(save_path.c_str(), "w");
// //   fprintf(csv,
// "#imu_idx,cam_idx,rx,ry,rz,qw,qx,qy,qz\n");
// //
// //   const mat4_t T_BS = get_imu_extrinsics();
// //   const mat4_t T_SB = T_BS.inverse();
// //
// //   for (int i = 0; i < nb_cams(); i++) {
// //     const mat4_t T_BCi =
// get_cam_extrinsics(i);
// //     const mat4_t T_SCi = T_SB * T_BCi;
// //
// //     const vec3_t r = tf_trans(T_SCi);
// //     const quat_t q = tf_quat(T_SCi);
// //     fprintf(csv, "0,");     // imu_idx
// //     fprintf(csv, "%d,", i); // cam_idx
// //     fprintf(csv, "%f,%f,%f,", r(0), r(1),
// r(2));
// //     fprintf(csv, "%f,%f,%f,%f\n", q.w(),
// q.x(), q.y(), q.z());
// //   }
// //   fclose(csv);
// // }
//
// void calib_vi_t::save() {
//   save_results("/tmp/calib-stereo_imu.yaml");
//   save_poses("/tmp/sensor_poses.csv");
//   save_speed_biases("/tmp/sensor_speed_biases.csv");
//   save_cameras("/tmp/cameras.csv");
//   save_cam_extrinsics("/tmp/cam_extrinsics.csv");
//   //
//   save_imu_extrinsics("/tmp/imu_extrinsics.csv");
//   save_pose("/tmp/fiducial_pose.csv",
//   get_fiducial_pose());
// }

} // namespace yac
