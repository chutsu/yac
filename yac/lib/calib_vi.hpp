#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "ceres_utils.hpp"
#include "calib_params.hpp"
#include "timeline.hpp"
#include "imu_error.hpp"

namespace yac {

template <typename CAMERA_TYPE>
struct reproj_error_t : public ceres::SizedCostFunction<2, 2, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const int cam_res_[2] = {0, 0};
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_{0.0, 0.0};
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  reproj_error_t(const int cam_res[2],
                 const int tag_id,
                 const int corner_idx,
                 const vec3_t &r_FFi,
                 const vec2_t &z,
                 const mat4_t &T_WF,
                 const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_idx_{corner_idx},
        r_FFi_{r_FFi}, z_{z}, T_WF_{T_WF},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {}

  ~reproj_error_t() {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Pose of fiducial target in world frame
    const double pitch = params[0][1];
    const double roll = params[0][0];
    const double yaw = quat2euler(tf_quat(T_WF_))(2);
    const vec3_t rpy{roll, pitch, yaw};

		const vec3_t r_WF = tf_trans(T_WF_);
    const mat3_t C_WF = euler321(rpy);
    const mat4_t T_WF = tf(C_WF, r_WF);

    // Map optimization variables to Eigen
    // const mat4_t T_WF = tf(params[0]);
    const mat4_t T_WS = tf(params[1]);
    const mat4_t T_SC = tf(params[2]);
    Eigen::Map<const vecx_t> cam_params(params[3], 8);

    // Transform and project point to image plane
    const mat4_t T_SW = T_WS.inverse();
    const mat4_t T_CS = T_SC.inverse();
    const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);
    const vec3_t r_CFi = tf_point(T_CS * T_SW * T_WF, r_FFi_);
    const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
    mat_t<2, 3> Jh;
    matx_t J_params;
    vec2_t z_hat;
    bool valid = true;

    CAMERA_TYPE camera{cam_res_, cam_params};
    if (camera.project(r_CFi, z_hat, Jh) != 0) {
      valid = false;
    }
    J_params = camera.J_params(p);

    // Residuals
    const vec2_t r = sqrt_info_ * (z_ - z_hat);
    residuals[0] = r(0);
    residuals[1] = r(1);

    // Jacobians
    const matx_t Jh_weighted = sqrt_info_ * Jh;
    const mat3_t C_CW = tf_rot(T_CS * T_SW);
    const mat3_t C_SC = tf_rot(T_SC);
    const mat3_t C_CS = C_SC.transpose();

    if (jacobians) {
      // Jacobians w.r.t. T_WF
      if (jacobians[0]) {
        // const mat3_t C_WF = tf_rot(T_WF);
        // Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        // J.setZero();
        // J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CW * I(3);
        // J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CW * -skew(C_WF * r_FFi_);
        // if (valid == false) {
        //   J.setZero();
        // }

				// Form jacobians
        const double cphi = cos(roll);
        const double sphi = sin(roll);
        const double ctheta = cos(pitch);
        const double stheta = sin(pitch);
        const double cpsi = cos(yaw);
        const double spsi = sin(yaw);

        const double x = r_FFi_(0);
        const double y = r_FFi_(1);
        const double z = r_FFi_(2);

        const vec3_t J_x{
          y * (sphi * spsi + stheta * cphi * cpsi) + z * (-sphi * stheta * cpsi + spsi * cphi),
          y * (-sphi * cpsi + spsi * stheta * cphi) + z * (-sphi * spsi * stheta - cphi * cpsi),
          y * cphi * ctheta - z * sphi * ctheta
        };
        const vec3_t J_y{
          -x * stheta * cpsi + y * sphi * cpsi * ctheta + z * cphi * cpsi * ctheta,
          -x * spsi * stheta + y * sphi * spsi * ctheta + z * spsi * cphi * ctheta,
          -x * ctheta - y * sphi * stheta - z * stheta * cphi
        };
        const vec3_t J_z{
          -x * spsi * ctheta + y * (-sphi * spsi * stheta - cphi * cpsi) + z * (sphi * cpsi - spsi * stheta * cphi),
          x * cpsi * ctheta + y * (sphi * stheta * cpsi - spsi * cphi) + z * (sphi * spsi + stheta * cphi * cpsi),
          0
        };

				// Fill Jacobian
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[0]);
        J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CW * J_x;
        J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CW * J_y;
        if (valid == false) {
          J.setZero();
        }

      }

      // Jacobians w.r.t T_WS
      if (jacobians[1]) {
        const mat3_t C_WS = tf_rot(T_WS);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CW * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CW * -skew(C_WS * r_SFi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t T_SC
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CS * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CS * -skew(C_SC * r_CFi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t. camera parameters
      if (jacobians[3]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
        J = -1 * sqrt_info_ * J_params;
        if (valid == false) {
          J.setZero();
        }
      }
    }

    return true;
  }
};

// /** Inertial residual */
// struct imu_error_t : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//   const int imu_index = 0;
//   const imu_params_t imu_params;
//   const imu_data_t imu_data;
//   const vec3_t g{0.0, 0.0, 9.81};
//
//   mat_t<15, 15> J = I(15, 15);  // Jacobians
//   mat_t<15, 15> P = zeros(15, 15);  // Covariance matrix
//   mat_t<12, 12> Q = zeros(12, 12);  // Noise covariance matrix
//   mutable matxs_t J_min;
//
//   // State vector indicies
//   const int POS_IDX = 0;
//   const int ROT_IDX = 3;
//   const int VEL_IDX = 6;
//   const int BA_IDX = 9;
//   const int BG_IDX = 12;
//
//   // Noise vector indicies
//   const int NA_IDX = 0;
//   const int NG_IDX = 3;
//   const int NBA_IDX = 6;
//   const int NBG_IDX = 9;
//
//   // Delta position, velocity and rotation between timestep i and j
//   // (i.e start and end of imu measurements)
//   vec3_t dp{0.0, 0.0, 0.0};
//   vec3_t dv{0.0, 0.0, 0.0};
//   quat_t dq{1.0, 0.0, 0.0, 0.0};
//
//   // Accelerometer and gyroscope biases
//   vec3_t bg{0.0, 0.0, 0.0};
//   vec3_t ba{0.0, 0.0, 0.0};
//
//   imu_error_t(const int imu_index_,
//               const imu_params_t imu_params_,
//               const imu_data_t imu_data_)
//       : imu_index{imu_index_},
//         imu_params{imu_params_},
//         imu_data{imu_data_} {
//     // Setup minimal jacobians
//     J_min.push_back(zeros(15, 6));  // T_WS at timestep i
//     J_min.push_back(zeros(15, 9));  // Speed and bias at timestep i
//     J_min.push_back(zeros(15, 6));  // T_WS at timestep j
//     J_min.push_back(zeros(15, 9));  // Speed and bias at timestep j
//
//     // Form noise covariance matrix Q
//     Q.setZero();
//     Q.block<3, 3>(0, 0) = pow(imu_params_.sigma_a_c, 2) * I(3);
//     Q.block<3, 3>(3, 3) = pow(imu_params_.sigma_g_c, 2) * I(3);
//     Q.block<3, 3>(6, 6) = pow(imu_params_.sigma_aw_c, 2) * I(3);
//     Q.block<3, 3>(9, 9) = pow(imu_params_.sigma_gw_c, 2) * I(3);
//
//     // propagate(imu_data.timestamps, imu_data.accel, imu_data.gyro);
//   }
//
//   void propagate(const imu_data_t &imu_data,
//                  const mat4_t &T_WS_i, const vec_t<9> &speed_biases_i,
//                  mat4_t &T_WS_j, vec_t<9> &speed_biases_j) {
//     const timestamps_t imu_ts = imu_data.timestamps;
//     const vec3s_t imu_gyr = imu_data.gyro;
//     const vec3s_t imu_acc = imu_data.accel;
//     assert(imu_ts.size() > 2);
//     assert(imu_ts.size() == imu_acc.size());
//     assert(imu_gyr.size() == imu_acc.size());
//
//     real_t dt = 0.0;
//     real_t dt_prev = ns2sec(imu_ts[1] - imu_ts[0]);
//     for (size_t k = 0; k < imu_gyr.size(); k++) {
//       // Calculate dt
//       if ((k + 1) < imu_gyr.size()) {
//         dt = ns2sec(imu_ts[k + 1] - imu_ts[k]);
//         dt_prev = dt;
//       } else {
//         dt = dt_prev;
//       }
//
//       // Update relative position and velocity
//       dp = dp + dv * dt + 0.5 * (dq * (imu_acc[k] - ba)) * dt * dt;
//       dv = dv + (dq * (imu_acc[k] - ba)) * dt;
//
//       // Update relative rotation
//       const real_t scalar = 1.0;
//       const vec3_t vector = 0.5 * (imu_gyr[k] - bg) * dt;
//       const quat_t dq_i{scalar, vector(0), vector(1), vector(2)};
//       dq = dq * dq_i;
//
//       // Transition matrix F
//       const mat3_t C_ji = dq.toRotationMatrix();
//       mat_t<15, 15> F_i = zeros(15, 15);
//       F_i.block<3, 3>(POS_IDX, VEL_IDX) = I(3);
//       F_i.block<3, 3>(VEL_IDX, ROT_IDX) = -C_ji * skew(imu_acc[k] - ba);
//       F_i.block<3, 3>(VEL_IDX, BA_IDX) = -C_ji;
//       F_i.block<3, 3>(ROT_IDX, ROT_IDX) = -skew(imu_gyr[k] - bg);
//       F_i.block<3, 3>(ROT_IDX, BG_IDX) = -I(3);
//
//       // printf("k: %ld\n", k);
//       // printf("dt: %f\n", dt);
//       // print_matrix("rot rot", -skew(imu_gyr[k] - bg));
//
//       // Input matrix G
//       mat_t<15, 12> G_i = zeros(15, 12);
//       G_i.block<3, 3>(VEL_IDX, NA_IDX) = -C_ji;
//       G_i.block<3, 3>(ROT_IDX, NG_IDX) = -I(3);
//       G_i.block<3, 3>(BA_IDX, NBA_IDX) = I(3);
//       G_i.block<3, 3>(BG_IDX, NBG_IDX) = I(3);
//
//       // Update jacobian and covariance matrix
//       const mat_t<15, 15> I_Fi_dt = I(15) + dt * F_i;
//       const mat_t<15, 12> Gi_dt = G_i * dt;
//       J = I_Fi_dt * J;
//       P = I_Fi_dt * P * I_Fi_dt.transpose() + Gi_dt * Q * Gi_dt.transpose();
//       // print_matrix("F_i", F_i);
//       // print_matrix("I_Fi_dt", I_Fi_dt);
//       // print_matrix("Gi_dt", Gi_dt);
//       // print_matrix("Q", Q);
//       // print_matrix("P", P);
//     }
//
//     // Return results
//     const vec3_t pos_i = tf_trans(T_WS_i);
//     const quat_t rot_i = tf_quat(T_WS_i);
//     T_WS_j = tf(rot_i * dq, pos_i + dp);
//     speed_biases_j = speed_biases_i;
//     speed_biases_j.head(3) += dv;
//   }
//
//   bool Evaluate(double const * const *params,
//                 double *residuals,
//                 double **jacobians) const {
//     // Map out parameters
//     // -- Sensor pose at timestep i
//     const mat4_t T_i = tf(params[0]);
//     const mat3_t C_i = tf_rot(T_i);
//     const mat3_t C_i_inv = C_i.transpose();
//     const quat_t q_i = tf_quat(T_i);
//     const vec3_t r_i = tf_trans(T_i);
//     // -- Speed and bias at timestamp i
//     const vec_t<9> sb_i{params[1]};
//     const vec3_t v_i = sb_i.segment<3>(0);
//     const vec3_t ba_i = sb_i.segment<3>(3);
//     const vec3_t bg_i = sb_i.segment<3>(6);
//     // -- Sensor pose at timestep j
//     const mat4_t T_j = tf(params[2]);
//     const quat_t q_j = tf_quat(T_j);
//     const vec3_t r_j = tf_trans(T_j);
//     // -- Speed and bias at timestep j
//     const vec_t<9> sb_j{params[3]};
//     const vec3_t v_j = sb_j.segment<3>(0);
//     const vec3_t ba_j = sb_j.segment<3>(3);
//     const vec3_t bg_j = sb_j.segment<3>(6);
//
//     // Obtain Jacobians for gyro and accel bias
//     const mat3_t dp_dba = J.block<3, 3>(POS_IDX, BA_IDX);
//     const mat3_t dp_dbg = J.block<3, 3>(POS_IDX, BG_IDX);
//     const mat3_t dv_dba = J.block<3, 3>(VEL_IDX, BA_IDX);
//     const mat3_t dv_dbg = J.block<3, 3>(VEL_IDX, BG_IDX);
//     const mat3_t dq_dbg = J.block<3, 3>(ROT_IDX, BG_IDX);
//
//     // Calculate square root info
//     const matx_t info{P.inverse()};
//     const matx_t sqrt_info{info.llt().matrixL().transpose()};
//
//     // Calculate residuals
//     const timestamp_t ts_i = imu_data.timestamps.front();
//     const timestamp_t ts_j = imu_data.timestamps.back();
//     const real_t dt_ij = ns2sec(ts_j - ts_i);
//     const real_t dt_ij_sq = dt_ij * dt_ij;
//     const vec3_t dbg = bg_i - bg;
//     const vec3_t dba = ba_i - ba;
//     const vec3_t alpha = dp + dp_dbg * dbg + dp_dba * dba;
//     const vec3_t beta = dv + dv_dbg * dbg + dv_dba * dba;
//     const quat_t gamma = dq * quat_delta(dq_dbg * dbg);
//
//     const quat_t q_i_inv = q_i.inverse();
//     const quat_t q_j_inv = q_j.inverse();
//     const quat_t gamma_inv = gamma.inverse();
//
//     // clang-format off
//     const vec3_t err_trans = C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha;
//     const vec3_t err_rot = 2.0 * (gamma_inv * (q_i_inv * q_j)).vec();
//     const vec3_t err_vel = C_i_inv * (v_j - v_i + g * dt_ij) - beta;
//     const vec3_t err_ba = ba_j - ba_i;
//     const vec3_t err_bg = bg_j - bg_i;
//     Eigen::Map<vec_t<15>> r(residuals);
//     r << err_trans, err_rot, err_vel, err_ba, err_bg;
//     r = sqrt_info * r;
//     // clang-format on
//
//     // print_vector("err_trans", err_trans);
//     // print_vector("err_vel", err_vel);
//     // print_vector("predicted dpos", C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq));
//     // print_vector("measured dpos", alpha);
//     // print_vector("predicted dvel", C_i_inv * (v_j - v_i + g * dt_ij));
//     // print_vector("measured dvel", beta);
//
//     // print_matrix("T_i", T_i);
//     // print_matrix("T_j", T_j);
//     // print_vector("v_i", v_i);
//     // print_vector("v_j", v_j);
//     // print_vector("ba_i", ba_i);
//     // print_vector("ba_j", ba_j);
//     // print_vector("bg_i", bg_i);
//     // print_vector("bg_j", bg_j);
//     // print_vector("err_trans", err_trans);
//     // print_vector("err_rot", err_rot);
//     // print_vector("err_vel", err_vel);
//     // print_vector("err_ba", err_ba);
//     // print_vector("err_bg", err_bg);
//
//     // printf("\n");
//     print_matrix("J", J);
//     print_matrix("P", P);
//     print_matrix("info", info);
//     print_matrix("sqrt_info", sqrt_info);
//
//     // Calculate Jacobians
//     // clang-format off
//     if (jacobians) {
//       // Jacobian w.r.t. pose_i
//       if (jacobians[0]) {
//         J_min[0] = zeros(15, 6);
//         J_min[0].block<3, 3>(POS_IDX, 0) = -C_i_inv;
//         J_min[0].block<3, 3>(POS_IDX, 3) = skew(C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq));
//         J_min[0].block<3, 3>(ROT_IDX, 0) = -quat_mat_xyz(quat_lmul(q_j_inv * q_i) * quat_rmul(gamma));
//         J_min[0].block<3, 3>(VEL_IDX, 3) = skew(C_i_inv * (v_j - v_i + g * dt_ij));
//         J_min[0] = sqrt_info * J_min[0];
//
//         mat_t<6, 7, row_major_t> J_lift;
//         lift_pose_jacobian(q_i, J_lift);
//
//         map_mat_t<15, 7, row_major_t> J0(jacobians[0]);
//         J0 = J_min[0] * J_lift;
//         // J0.setZero();
//         // J0.block<15, 6>(0, 0) = J_min[0];
//       }
//
//       // Jacobian w.r.t. sb_i
//       if (jacobians[1]) {
//         J_min[1] = zeros(15, 9);
//         J_min[1].block<3, 3>(POS_IDX, 0) = -C_i_inv * dt_ij;
//         J_min[1].block<3, 3>(POS_IDX, 3) = -dp_dba;
//         J_min[1].block<3, 3>(POS_IDX, 6) = -dp_dbg;
//         J_min[1].block<3, 3>(VEL_IDX, 0) = -C_i_inv;
//         J_min[1].block<3, 3>(VEL_IDX, 3) = -dv_dba;
//         J_min[1].block<3, 3>(VEL_IDX, 6) = -dv_dbg;
//         J_min[1].block<3, 3>(BA_IDX, 3) = -I(3);
//         J_min[1].block<3, 3>(BG_IDX, 6) = -I(3);
//         J_min[1] = sqrt_info * J_min[1];
//
//         map_mat_t<15, 9, row_major_t> J1(jacobians[1]);
//         J1.setZero();
//         J1 = J_min[1];
//       }
//
//       // Jacobian w.r.t. pose_j
//       if (jacobians[2]) {
//         J_min[2] = zeros(15, 6);
//         J_min[2].block<3, 3>(POS_IDX, 0) = C_i_inv;
//
//         J_min[2].block<3, 3>(ROT_IDX, 3) = quat_lmul_xyz(gamma_inv * q_i_inv * q_j);
//         J_min[2] = sqrt_info * J_min[2];
//
//         mat_t<6, 7, row_major_t> J_lift;
//         lift_pose_jacobian(q_j, J_lift);
//
//         map_mat_t<15, 7, row_major_t> J2(jacobians[2]);
//         J2 = J_min[2] * J_lift;
//         // J2.setZero();
//         // J2.block(0, 0, 15, 6) = J_min[2];
//       }
//
//       // Jacobian w.r.t. sb_j
//       if (jacobians[3]) {
//         // -- Speed and bias at j Jacobian
//         J_min[3] = zeros(15, 9);
//         J_min[3].block<3, 3>(VEL_IDX, 0) = C_i_inv;
//         J_min[3].block<3, 3>(BA_IDX, 3) = I(3);
//         J_min[3].block<3, 3>(BG_IDX, 6) = I(3);
//         J_min[3] = sqrt_info * J_min[3];
//
//         map_mat_t<15, 9, row_major_t> J3(jacobians[3]);
//         J3.setZero();
//         J3 = J_min[3];
//       }
//     }
//     // clang-format on
//
//     return true;
//   }
// };

struct aprilgrid_buffer_t {
  int nb_cams = 0;
  std::map<int, std::deque<aprilgrid_t>> buf;

  std::map<int, std::deque<aprilgrid_t>> &data() {
    return buf;
  }

  void add(const int cam_idx, const aprilgrid_t &grid) {
    buf[cam_idx].push_back(grid);
  }

  bool ready() {
    timestamp_t ts = 0;

    for (int i = 0; i < nb_cams; i++) {
      if (buf[i].size() == 0) {
        return false;
      }

      if (ts == 0) {
        ts = buf[i].front().timestamp;
        continue;
      }

      if (ts != buf[i].front().timestamp) {
        return false;
      }
    }

    return true;
  }

  aprilgrids_t pop_front() {
    aprilgrids_t results;

    for (auto &kv : buf) {
      auto &deque = kv.second;
      results.push_back(deque.front());
      deque.pop_front();
    }

    return results;
  }
};

struct calib_vi_t {
  id_t new_param_id = 0;

  // Calibration data
  timeline_t timeline;

  // Data
  aprilgrid_buffer_t grids_buf;
  std::map<int, aprilgrids_t> cam_grids;
  imu_data_t imu_buf;
  bool initialized = false;

  // Parameters to be optimized
  fiducial_params_t *fiducial = nullptr;
  time_delay_t *time_delay = nullptr;
  std::vector<pose_t *> sensor_poses;
  std::vector<sb_params_t *> speed_biases;
  std::map<int, camera_params_t *> cam_params;
  std::map<int, extrinsics_t *> extrinsics;

  // Optimization
  imu_params_t imu_params;
  double sigma_vision = 1.0;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_parameterization;

	template <typename CAMERA_TYPE>
  struct calib_view_t {
		ceres::Problem *problem = nullptr;
    const int cam_idx = -1;
		const fiducial_params_t *fiducial = nullptr;
    const pose_t *sensor_pose = nullptr;
    const camera_params_t *cam_params = nullptr;
    const extrinsics_t *extrinsics = nullptr;

		std::vector<ceres::ResidualBlockId> reproj_error_ids;
		std::vector<reproj_error_t<CAMERA_TYPE> *> reproj_errors;

		calib_view_t(ceres::Problem *problem_,
		 		 	 	 	 	 const int cam_idx_,
				 	 	 	 	 const fiducial_params_t *fiducial_,
				 	 	 	 	 const pose_t *sensor_pose_,
				 	 	 	 	 const extrinsics_t *extrinsics_,
				 	 	 	 	 const camera_params_t *cam_params_)
			: problem{problem_},
				cam_idx{cam_idx_},
				fiducial{fiducial_},
				sensor_pose{sensor_pose_},
				extrinsics{extrinsics_},
				cam_params{cam_params_} {}

		void add(const ceres::ResidualBlockId &id,
						 reproj_error_t<CAMERA_TYPE> *err) {
			reproj_error_ids.push_back(id);
			reproj_errors.push_back(err);
		}

		void calculate_reproj_errors(std::vector<double> &errs) {
			const mat4_t T_WF = fiducial->estimate();
			const mat4_t T_CiS = extrinsics->tf().inverse();
			const mat4_t T_SW = sensor_pose->tf().inverse();

			for (const auto reproj_error : reproj_errors) {
				const vec2_t z = reproj_error->z_;
				const vec3_t r_FFi = reproj_error->r_FFi_;
				const vec3_t r_CiFi = tf_point(T_CiS * T_SW * T_WF, r_FFi);

				vec2_t z_hat;
				const CAMERA_TYPE cam_geom(cam_params->resolution, cam_params->param);
				cam_geom.project(r_CiFi, z_hat);
				const vec2_t r = z - z_hat;
				errs.push_back(r.norm());
			}
		}

		int filter(const double threshold) {
		  std::vector<double> errs;
      calculate_reproj_errors(errs);

			auto id_iter = reproj_error_ids.begin();
			auto err_iter = reproj_errors.begin();

			int nb_outliers = 0;
			for (size_t i = 0; i < errs.size(); i++) {
				if (errs[i] > threshold) {
					problem->RemoveResidualBlock(*id_iter);
					id_iter = reproj_error_ids.erase(id_iter);
					err_iter = reproj_errors.erase(err_iter);
					nb_outliers++;
				} else {
					id_iter++;
					err_iter++;
				}
			}

			return nb_outliers;
		}
  };
	std::map<int, std::vector<calib_view_t<pinhole_radtan4_t>>> calib_views;

  calib_vi_t() {
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = new ceres::Problem(prob_options);
  }

  ~calib_vi_t() {
    if (fiducial) delete fiducial;
    if (time_delay) delete time_delay;
    for (const auto &pose : sensor_poses) delete pose;
    for (const auto &sb : speed_biases) delete sb;
    for (const auto &kv: cam_params) delete kv.second;
    for (const auto &kv: extrinsics) delete kv.second;
    if (problem) delete problem;
  }

  void add_imu(const imu_params_t &imu_params_, const double td=0.0) {
    imu_params = imu_params_;
    add_time_delay(td);
  }

  void add_camera(const int cam_idx,
                  const int resolution[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const bool fix=false) {
    auto cam = new camera_params_t(new_param_id++,
                                   cam_idx, resolution,
                                   proj_model, dist_model,
                                   proj_params, dist_params);
    cam_params[cam_idx] = cam;
    grids_buf.nb_cams++; // Very important!

    if (fix) {
      problem->AddParameterBlock(cam->param.data(), cam->global_size);
      problem->SetParameterBlockConstant(cam->param.data());
    }
  }

  void add_extrinsics(const int cam_idx,
                      const mat4_t &T_SC,
                      const bool fix=false) {
    auto T_SC_ptr = new extrinsics_t(new_param_id++, T_SC);
    extrinsics[cam_idx] = T_SC_ptr;

    if (fix) {
      problem->SetParameterBlockConstant(T_SC_ptr->param.data());
    }
  }

  void add_sensor_pose(const timestamp_t ts, const mat4_t &T_WS) {
    sensor_poses.push_back(new pose_t{new_param_id++, ts, T_WS});
  }

  void add_speed_biases(const timestamp_t ts, const vec_t<9> &sb) {
    speed_biases.push_back(new sb_params_t{new_param_id++, ts, sb});
  }

  void add_time_delay(const double td) {
    time_delay = new time_delay_t{new_param_id++, td};
  }

  void add_fiducial_pose(const mat4_t &T_WF) {
		fiducial = new fiducial_params_t(new_param_id++, T_WF);
  }

  vecx_t get_camera(const int cam_idx) {
    return cam_params[cam_idx]->param;
  }

  mat4_t get_extrinsic(const int cam_idx) {
    return extrinsics[cam_idx]->tf();
  }

  mat4_t get_sensor_pose(const int pose_index) {
    return sensor_poses[pose_index]->tf();
  }

  mat4_t get_fiducial_pose() {
		fiducial->update();
    return fiducial->estimate();
  }

  int nb_cams() {
    return cam_params.size();
  }

  int estimate_relative_pose(const aprilgrid_t &grid, mat4_t &T_C0F) {
    assert(cam_params.count(0) == 1);

    // Estimate relative pose
    const int *cam_res = cam_params[0]->resolution;
    const auto proj_model = cam_params[0]->proj_model;
    const auto dist_model = cam_params[0]->dist_model;
    const vec4_t proj_params = cam_params[0]->proj_params();
    const vec4_t dist_params = cam_params[0]->dist_params();

    int retval = 0;
    if (proj_model == "pinhole" && dist_model == "radtan4") {
      const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
      retval = grid.estimate(cam, T_C0F);
    } else if (proj_model == "pinhole" && dist_model == "equi4") {
      const pinhole_equi4_t cam{cam_res, proj_params, dist_params};
      retval = grid.estimate(cam, T_C0F);
    } else {
      FATAL("Unsupported projection-distorion type [%s-%s]!",
            proj_model.c_str(), dist_model.c_str());
    }

    return retval;
  }

  void trim_imu_data(imu_data_t &imu_data, const timestamp_t t1) {
    // Makesure the trim timestamp is after the first imu measurement
    if (t1 < imu_data.timestamps.front()) {
      return;
    }

    // Trim IMU measurements
    imu_data_t trimmed_imu_data;
    for (size_t k = 0; k < imu_data.timestamps.size(); k++) {
      const timestamp_t ts = imu_data.timestamps[k];
      if (ts >= t1) {
        const vec3_t acc = imu_data.accel[k];
        const vec3_t gyr = imu_data.gyro[k];
        trimmed_imu_data.add(ts, acc, gyr);
      }
    }

    imu_data = trimmed_imu_data;
  }

  void add_imu_error() {
    pose_t *pose_i = sensor_poses[sensor_poses.size() - 2];
    pose_t *pose_j = sensor_poses[sensor_poses.size() - 1];
    sb_params_t *sb_i = speed_biases[speed_biases.size() - 2];
    sb_params_t *sb_j = speed_biases[speed_biases.size() - 1];
    const auto t0 = pose_i->ts;
    const auto t1 = pose_j->ts;

    auto error = new ImuError(imu_buf, imu_params, t0, t1);
    problem->AddResidualBlock(error,
                              NULL,
                              pose_i->param.data(),
                              sb_i->param.data(),
                              pose_j->param.data(),
                              sb_j->param.data());
    problem->SetParameterization(pose_i->param.data(), &pose_parameterization);
    problem->SetParameterization(pose_j->param.data(), &pose_parameterization);
  }

  void add_reproj_errors(const int cam_idx, const aprilgrid_t &grid) {
    assert(cam_params.count(cam_idx) > 0);
    assert(extrinsics.count(cam_idx) > 0);
    assert(sensor_poses.size() >= 1);

    const auto &T_WF = fiducial;
    const auto &T_WS = sensor_poses.back();
    const auto &T_SC = extrinsics[cam_idx];
    const auto &cam = cam_params[cam_idx];
    const int *cam_res = cam->resolution;
    const mat2_t covar = pow(sigma_vision, 2) * I(2);

		calib_view_t<pinhole_radtan4_t> view{problem, cam_idx, T_WF, T_WS, T_SC, cam};

    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_indicies[i];
      const vec2_t z = keypoints[i];
      const vec3_t r_FFi = object_points[i];

      auto error = new reproj_error_t<pinhole_radtan4_t>(cam_res,
                                                         tag_id,
                                                         corner_idx,
                                                         r_FFi,
                                                         z,
																												 fiducial->estimate(),
                                                         covar);
      auto error_id = problem->AddResidualBlock(error,
                                                NULL,
                                                T_WF->param.data(),
                                                T_WS->param.data(),
                                                T_SC->param.data(),
                                                cam->param.data());

      problem->SetParameterization(T_WS->param.data(), &pose_parameterization);
      problem->SetParameterization(T_SC->param.data(), &pose_parameterization);
			view.add(error_id, error);
    }

    calib_views[cam_idx].push_back(view);
  }

  bool fiducial_detected(const aprilgrids_t &grids) {
    assert(grids.size() > 0);

    bool detected = false;
    for (int i = 0; i < nb_cams(); i++) {
      if (grids[i].detected) {
        detected = true;
      }
    }

    return detected;
  }

  mat4_t estimate_sensor_pose(const aprilgrids_t &grids) {
    assert(grids.size() > 0);
    assert(initialized);

    if (fiducial_detected(grids) == false) {
      FATAL("No fiducials detected!");
    }

    for (int i = 0; i < nb_cams(); i++) {
      // Skip if not detected
      if (grids[i].detected == false) {
        continue;
      }

      // Estimate relative pose
      mat4_t T_CiF = I(4);
      if (estimate_relative_pose(grids[i], T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      // Infer current pose T_WS using T_C0F, T_SC0 and T_WF
      const mat4_t T_FCi_k = T_CiF.inverse();
      const mat4_t T_SCi = get_extrinsic(i);
      const mat4_t T_CiS = T_SCi.inverse();
      const mat4_t T_WF = get_fiducial_pose();
      const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;

      return T_WS_k;
    }

    // Should never reach here
    return zeros(4, 4);
  }

  void initialize(const timestamp_t &ts,
                  const aprilgrids_t &grids,
                  imu_data_t &imu_buf) {
    // Estimate initial IMU attitude
    mat3_t C_WS;
    imu_init_attitude(imu_buf.gyro, imu_buf.accel, C_WS, 5);

    // Add sensor pose - T_WS
    const mat4_t T_WS = tf(C_WS, zeros(3, 1));
    add_sensor_pose(ts, T_WS);
    add_speed_biases(ts, zeros(9, 1));
    LOG_INFO("Sensor pose initialized!");
    print_matrix("T_WS", T_WS);

    // Estimate relative pose
    assert(grids.size() > 0);
    assert(grids[0].detected);
    mat4_t T_C0F = I(4);
    if (estimate_relative_pose(grids[0], T_C0F) != 0) {
      FATAL("Failed to estimate relative pose!");
      return;
    }

    // Add fiducial pose - T_WF
    const mat4_t T_SC0 = get_extrinsic(0);
    const mat4_t T_WF = T_WS * T_SC0 * T_C0F;
    add_fiducial_pose(T_WF);
    for (int i = 0; i < nb_cams(); i++) {
      add_reproj_errors(i, grids[i]);
    }
    LOG_INFO("Fiducial initialized!");
    print_matrix("T_WF", T_WF);

    // Finish up
    initialized = true;
    trim_imu_data(imu_buf, ts);  // Remove imu measurements up to ts
  }

  void add_state(const timestamp_t &ts, const aprilgrids_t &grids) {
    // // Propagate imu measurements to obtain speed and biases
    // const auto t0 = sensor_poses.back()->ts;
    // const auto t1 = ts;
    // mat4_t T_WS = sensor_poses.back()->tf();
    // vec_t<9> sb_k = speed_biases.back()->param;
    // ImuError::propagation(imu_buf, imu_params, T_WS, sb_k, t0, t1);

    // Infer velocity from two poses T_WS_k and T_WS_km1
    const mat4_t T_WS_km1 = tf(sensor_poses.back()->param);
    const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
    const mat4_t T_WS_k = estimate_sensor_pose(grids);
    const vec3_t r_WS_k = tf_trans(T_WS_k);
    const vec3_t v_WS_k = r_WS_k - r_WS_km1;
    vec_t<9> sb_k;
    sb_k.setZero();
    sb_k << v_WS_k, zeros(3, 1), zeros(3, 1);

    // Add updated sensor pose T_WS_k and speed and biases sb_k
    // Note: instead of using the propagated sensor pose `T_WS`, we are using
    // the provided `T_WS_`, this is because in the scenario where we are using
    // AprilGrid, as a fiducial target, we actually have a better estimation of
    // the sensor pose, as supposed to the imu propagated sensor pose.
    add_sensor_pose(ts, T_WS_k);
    add_speed_biases(ts, sb_k);

    // Add error terms
    add_imu_error();
    for (int i = 0; i < nb_cams(); i++) {
      add_reproj_errors(i, grids[i]);
    }

    trim_imu_data(imu_buf, ts);
  }

  void add_measurement(const int cam_idx, const aprilgrid_t &grid) {
    grids_buf.add(cam_idx, grid);
    cam_grids[cam_idx].push_back(grid);
  }

  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m) {
    imu_buf.add(imu_ts, a_m, w_m);

    if (grids_buf.ready()) {
      const auto grids = grids_buf.pop_front();

      // Initialize T_WS and T_WF
      if (initialized == false) {
        if (grids[0].detected && imu_buf.size() > 5) {
          const auto ts = imu_buf.timestamps.front();
          initialize(ts, grids, imu_buf);
        }
        return;
      }

      // Add new state
      timestamp_t grid_ts = 0;
      for (int i = 0; i < nb_cams(); i++) {
        if (grids[i].detected) {
          grid_ts = grids[i].timestamp;
          break;
        }
      }
      if (fiducial_detected(grids) && imu_ts >= grid_ts) {
        const auto ts = imu_buf.timestamps.back();
        add_state(ts, grids);
      }
    }
  }

  void setup(const std::map<int, aprilgrids_t> &cam_grids,
             const imu_data_t &imu_data) {
    // Create timeline
    // -- Add camera events
    for (const auto &kv : cam_grids) {
      const auto cam_idx = kv.first;
      const auto &grids = kv.second;

      for (const auto &grid : grids) {
        const timestamp_t ts = grid.timestamp;
        const timeline_event_t event{ts, cam_idx, grid};
        timeline.add(event);
      }
    }
    // -- Add imu events
    for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
      const timestamp_t ts = imu_data.timestamps[i];
      const vec3_t a = imu_data.accel[i];
      const vec3_t w = imu_data.gyro[i];
      const timeline_event_t event{ts, a, w};
      timeline.add(event);
    }

    // Process timeline
    for (const auto &ts : timeline.timestamps) {
      const auto result = timeline.data.equal_range(ts);
      for (auto it = result.first; it != result.second; it++) {
        const auto event = it->second;

        if (event.type == APRILGRID_EVENT) {
          add_measurement(event.camera_index, event.grid);
        }

        if (event.type == IMU_EVENT) {
          add_measurement(event.ts, event.a_m, event.w_m);
        }
      }
    }
  }

  template <typename CAMERA_TYPE>
  std::vector<double> calculate_reproj_errors(const int cam_idx) {
    assert(cam_grids.size() > cam_idx);
    assert(cam_params.size() > cam_idx);

    // Setup
    const aprilgrids_t &grids0 = cam_grids[0];
    const camera_params_t *cam = cam_params[cam_idx];
    std::vector<double> errs;

    // Fiducial pose
    const mat4_t T_WF = get_fiducial_pose();

    // Sensor-camera extrinsics
    const mat4_t T_CiS = extrinsics[cam_idx]->tf().inverse();

    // Convert sensor_poses to a std::map<timestamp, mat4_t> for easy access
    std::map<timestamp_t, mat4_t> T_SW;
    for (const auto pose : sensor_poses) {
      T_SW[pose->ts] = pose->tf().inverse();
    }

    // Calculate reprojection error
    size_t nb_grids = cam_grids[0].size();
    for (size_t k = 0; k < nb_grids; k++) {
      const auto grid = cam_grids[cam_idx][k];
      const auto ts = grid.timestamp;
      const vec2s_t kpsi = grid.keypoints();
      const vec3s_t pts = grid.object_points();

      // Check if we have a sensor pose estimate for this timestamp
      if (T_SW.count(ts) == 0) {
        continue;
      }

      for (size_t i = 0; i < pts.size(); i++) {
        const vec3_t r_FFi = pts[i];
        const vec3_t r_CiFi = tf_point(T_CiS * T_SW[ts] * T_WF, r_FFi);

        vec2_t z_hat;
        const vec2_t z = kpsi[i];
        const CAMERA_TYPE cam_geom(cam->resolution, cam->param);
        cam_geom.project(r_CiFi, z_hat);
        const vec2_t r = z - z_hat;
        errs.push_back(r.norm());
      }
    }

    return errs;
  }

  std::map<int, std::vector<double>> get_camera_errors() {
    const auto proj_model = cam_params[0]->proj_model;
    const auto dist_model = cam_params[0]->dist_model;

    std::map<int, std::vector<double>> cam_errs;
    for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
      std::vector<double> errs;
      if (proj_model == "pinhole" && dist_model == "radtan4") {
        errs = calculate_reproj_errors<pinhole_radtan4_t>(cam_idx);
      } else if (proj_model == "pinhole" && dist_model == "equi4") {
        errs = calculate_reproj_errors<pinhole_equi4_t>(cam_idx);
      } else {
        FATAL("[%s-%s] unsupported!", proj_model.c_str(), dist_model.c_str());
      }
      cam_errs[cam_idx] = errs;
    }

    return cam_errs;
  }

  void show_results() {
    // Show results
    std::map<int, std::vector<double>> cam_errs = get_camera_errors();

    printf("Optimization results:\n");
    printf("---------------------\n");
    printf("stats:\n");
    printf("cam0 reprojection error [px]: ");
    printf("[rmse: %f,", rmse(cam_errs[0]));
    printf(" mean: %f,", mean(cam_errs[0]));
    printf(" median: %f]\n", median(cam_errs[0]));
    printf("cam1 reprojection error [px]: ");
    printf("[rmse: %f,", rmse(cam_errs[1]));
    printf(" mean: %f,", mean(cam_errs[1]));
    printf(" median: %f]\n", median(cam_errs[1]));
    printf("\n");
    printf("cam0:\n");
    print_vector("proj_params", cam_params[0]->proj_params());
    print_vector("dist_params", cam_params[0]->dist_params());
    printf("\n");
    printf("cam1:\n");
    print_vector("proj_params", cam_params[1]->proj_params());
    print_vector("dist_params", cam_params[1]->dist_params());
    printf("\n");
    if (time_delay) {
      printf("time_delay: %f\n", time_delay->param(0));
      printf("\n");
    }
    const mat4_t T_SC0 = extrinsics[0]->tf();
    const mat4_t T_SC1 = extrinsics[1]->tf();
    print_matrix("T_SC0", T_SC0);
    print_matrix("T_SC1", T_SC1);
    print_matrix("T_C1C0", T_SC1.inverse() * T_SC0);
    printf("\n");
  }

  void solve(bool verbose=true) {
    // Optimize problem - first pass
		{
		  LOG_INFO("Optimize problem - first pass");
		  ceres::Solver::Options options;
		  options.minimizer_progress_to_stdout = true;
		  options.max_num_iterations = 10;
		  ceres::Solver::Summary summary;
		  ceres::Solve(options, problem, &summary);
		  fiducial->update(); // <- Very important!

      std::map<int, std::vector<double>> cam_errs = get_camera_errors();
      printf("stats:\n");
      printf("cam0 reprojection error [px]: ");
      printf("[rmse: %f,", rmse(cam_errs[0]));
      printf(" mean: %f,", mean(cam_errs[0]));
      printf(" median: %f]\n", median(cam_errs[0]));
      printf("cam1 reprojection error [px]: ");
      printf("[rmse: %f,", rmse(cam_errs[1]));
      printf(" mean: %f,", mean(cam_errs[1]));
      printf(" median: %f]\n", median(cam_errs[1]));
      printf("\n");
		}

		// Filter outliers
    LOG_INFO("Filter outliers");
    int nb_outliers = 0;
    int nb_inliers = 0;
		for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
			// Obtain all reprojection errors from all views
			std::vector<double> errs;
			for (auto &view : calib_views[cam_idx]) {
				view.calculate_reproj_errors(errs);
			}

			// Filter outliers from last view
			const double threshold = 3.0 * stddev(errs);
			for (auto &view : calib_views[cam_idx]) {
				nb_outliers += view.filter(threshold);
				nb_inliers += view.reproj_errors.size();
			}
		}
    LOG_INFO("Removed %d outliers!", nb_outliers);
    printf("\n");

    // Optimize problem - second pass
    {
		  LOG_INFO("Optimize problem - second pass");
      // Solver options
      ceres::Solver::Options options;
      options.minimizer_progress_to_stdout = true;
      options.max_num_iterations = 100;
      // options.check_gradients = true;

      // Solve
      ceres::Solver::Summary summary;
      ceres::Solve(options, problem, &summary);
      fiducial->update(); // <- Very important!
      if (verbose) {
        std::cout << summary.FullReport() << std::endl;
        show_results();
      }
    }
  }

  int save_results(const std::string &save_path) {
    // Open results file
    FILE *outfile = fopen(save_path.c_str(), "w");
    if (outfile == NULL) {
      return -1;
    }

    // Calibration metrics
    std::map<int, std::vector<double>> cam_errs = get_camera_errors();
    fprintf(outfile, "calib_metrics:\n");
    for (const auto &kv : cam_errs) {
      const auto cam_idx = kv.first;
      const auto cam_str = "cam" + std::to_string(cam_idx);
      const auto cam_errs = kv.second;
      fprintf(outfile, "  %s: ", cam_str.c_str());
      fprintf(outfile, "[");
      fprintf(outfile, "%f, ", rmse(cam_errs));
      fprintf(outfile, "%f, ", mean(cam_errs));
      fprintf(outfile, "%f, ", median(cam_errs));
      fprintf(outfile, "%f", stddev(cam_errs));
      fprintf(outfile, "]  # rmse, mean, median, stddev\n");
    }
    fprintf(outfile, "\n");
    fprintf(outfile, "\n");

    // Camera parameters
    for (int i = 0; i < nb_cams(); i++) {
      const auto cam = cam_params[i];
      const int *cam_res = cam->resolution;
      const char *proj_model = cam->proj_model.c_str();
      const char *dist_model = cam->dist_model.c_str();
      const std::string proj_params = vec2str(cam->proj_params(), 4);
      const std::string dist_params = vec2str(cam->dist_params(), 4);

      fprintf(outfile, "cam%d\n", i);
      fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
      fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
      fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
      fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
      fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
      fprintf(outfile, "\n");
    }
    fprintf(outfile, "\n");

    // IMU parameters
    fprintf(outfile, "imu0:\n");
    fprintf(outfile, "  rate: %f\n", imu_params.rate);
    fprintf(outfile, "  tau_a: %f\n", imu_params.tau_a);
    fprintf(outfile, "  tau_g: %f\n", imu_params.tau_g);
    fprintf(outfile, "  sigma_g_c: %f\n", imu_params.sigma_g_c);
    fprintf(outfile, "  sigma_a_c: %f\n", imu_params.sigma_a_c);
    fprintf(outfile, "  sigma_gw_c: %f\n", imu_params.sigma_gw_c);
    fprintf(outfile, "  sigma_aw_c: %f\n", imu_params.sigma_aw_c);
    fprintf(outfile, "  g: %f\n", imu_params.g);
    fprintf(outfile, "\n");
    fprintf(outfile, "\n");

    // Sensor-Camera extrinsics
    for (int i = 0; i < nb_cams(); i++) {
      const mat4_t T_SCi = extrinsics[i]->tf();
      fprintf(outfile, "T_SC%d:\n", i);
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
    if (nb_cams() >= 2) {
      for (int i = 1; i < nb_cams(); i++) {
        const mat4_t T_SC0 = extrinsics[0]->tf();
        const mat4_t T_SCi = extrinsics[i]->tf();
        const mat4_t T_C0Ci = T_SC0.inverse() * T_SCi;
        const mat4_t T_CiC0 = T_C0Ci.inverse();

        fprintf(outfile, "T_C0C%d:\n", i);
        fprintf(outfile, "  rows: 4\n");
        fprintf(outfile, "  cols: 4\n");
        fprintf(outfile, "  data: [\n");
        fprintf(outfile, "%s\n", mat2str(T_C0Ci, "    ").c_str());
        fprintf(outfile, "  ]");
        fprintf(outfile, "\n");
        fprintf(outfile, "\n");

        fprintf(outfile, "T_C%dC0:\n", i);
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

  void save() {
    LOG_INFO("Saved results to /tmp/calib_results.yaml");
    save_results("/tmp/calib_results.yaml");

    // Save sensor poses
    {
      FILE *csv = fopen("/tmp/sensor_poses.csv", "w");
      fprintf(csv, "#x,y,z,qw,qx,qy,qz\n");
      for (const auto &pose : sensor_poses) {
        const vec3_t r = pose->trans();
        const quat_t q = pose->rot();
        fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
        fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
      }
      fclose(csv);
    }

    // Save cameras
    {
      FILE *csv = fopen("/tmp/cameras.csv", "w");
      fprintf(csv, "#cam_idx,");
      fprintf(csv, "resolution,");
      fprintf(csv, "proj_model,");
      fprintf(csv, "dist_model,");
      fprintf(csv, "proj_params,");
      fprintf(csv, "dist_params\n");
      for (const auto &kv : cam_params) {
        const auto cam_idx = kv.first;
        const auto cam = kv.second;
        const vecx_t proj = cam->proj_params();
        const vecx_t dist = cam->dist_params();
        fprintf(csv, "%d,", cam_idx);
        fprintf(csv, "%d,%d,", cam->resolution[0], cam->resolution[1]);
        fprintf(csv, "%s,%s,", cam->proj_model.c_str(), cam->dist_model.c_str());
        fprintf(csv, "%f,%f,%f,%f,", proj(0), proj(1), proj(2), proj(3));
        fprintf(csv, "%f,%f,%f,%f\n", dist(0), dist(1), dist(2), dist(3));
      }
      fclose(csv);
    }

    // Save extrinsics
    {
      FILE *csv = fopen("/tmp/extrinsics.csv", "w");
      fprintf(csv, "#x,y,z,qw,qx,qy,qz\n");
      for (const auto &kv : extrinsics) {
        const auto T_SC = kv.second;
        const vec3_t r = T_SC->trans();
        const quat_t q = T_SC->rot();
        fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
        fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
      }
      fclose(csv);
    }

    // Save fiducial_pose
    {
      FILE *csv = fopen("/tmp/fiducial_pose.csv", "w");
      fprintf(csv, "#x,y,z,qw,qx,qy,qz\n");
			const mat4_t T_WF = get_fiducial_pose();
      const vec3_t r = tf_trans(T_WF);
      const quat_t q = tf_quat(T_WF);
      fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
      fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
      fclose(csv);
    }
  }
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
