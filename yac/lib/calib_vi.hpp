#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <deque>
#include <iostream>
#include <memory>
#include <string>

#include <ceres/ceres.h>

// #include "calib_data.hpp"
#include "calib_params.hpp"
#include "imu_error.hpp"
#include "marg_error.hpp"
#include "util/util.hpp"

namespace yac {

// /* Form initial poses for visual-inertial calibration */
// void calib_vi_init_poses(const calib_target_t &target,
//                          const mat4_t &T_FO,
//                          std::deque<mat4_t> &init_poses);

struct pose_error_t : public ceres::SizedCostFunction<6, 7> {
  pose_t pose_meas_;
  matx_t covar_;
  matx_t info_;
  matx_t sqrt_info_;

  pose_error_t(const pose_t &pose, const matx_t &covar)
      : pose_meas_{pose}, covar_{covar}, info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
  }

  ~pose_error_t() = default;

  bool Evaluate(double const *const *params,
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
};

// template <typename CAMERA_TYPE>
// struct reproj_fiducial_error_t
//     : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 8> {
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
//   reproj_fiducial_error_t(const timestamp_t &ts_i,
//                           const timestamp_t &ts_j,
//                           const int cam_res[2],
//                           const int tag_id,
//                           const int corner_idx,
//                           const vec3_t &r_FFi,
//                           const vec2_t &z_i,
//                           const vec2_t &z_j,
//                           const mat4_t &T_WF,
//                           const mat2_t &covar)
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
//   ~reproj_fiducial_error_t() {}
//
//   bool Evaluate(double const *const *params,
//                 double *residuals,
//                 double **jacobians) const {
//     // Pose of fiducial target in world frame
//     const mat4_t T_WF = tf(params[0]);
//
//     // Sensor pose, sensor-camera extrinsics, camera parameters
//     const mat4_t T_WS = tf(params[1]);
//     const mat4_t T_BS = tf(params[2]);
//     const mat4_t T_BCi = tf(params[3]);
//     Eigen::Map<const vecx_t> cam_params(params[4], 8);
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
//     const vec2_t r = sqrt_info_ * (z_i_ - z_hat);
//     residuals[0] = r(0);
//     residuals[1] = r(1);
//
//     // Jacobians
//     const matx_t Jh_weighted = sqrt_info_ * Jh;
//
//     if (jacobians) {
//       // Jacobians w.r.t. T_WF
//       if (jacobians[0]) {
//         // Fill Jacobian
//         const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
//         const mat3_t C_WF = tf_rot(T_WF);
//         Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
//         J.setZero();
//         J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiW * I(3);
//         J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiW * -skew(C_WF *
//         r_FFi_); if (valid == false) {
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
//     }
//
//     return true;
//   }
// };

// template <typename CAMERA_TYPE>
// #if FIDUCIAL_PARAMS_SIZE == 2
// struct reproj_error_td_t
//     : public ceres::SizedCostFunction<2, 2, 7, 7, 7, 8, 1> {
// #elif FIDUCIAL_PARAMS_SIZE == 3
// struct reproj_error_td_t
//     : public ceres::SizedCostFunction<2, 3, 7, 7, 7, 8, 1> {
// #elif FIDUCIAL_PARAMS_SIZE == 7
// struct reproj_error_td_t
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
//   reproj_error_td_t(const timestamp_t &ts_i,
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
//   ~reproj_error_td_t() {}
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
//     const vec3_t err_trans = C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g *
//     dt_ij_sq) - alpha; const vec3_t err_rot = 2.0 * (gamma_inv * (q_i_inv *
//     q_j)).vec(); const vec3_t err_vel = C_i_inv * (v_j - v_i + g * dt_ij) -
//     beta; const vec3_t err_ba = ba_j - ba_i; const vec3_t err_bg = bg_j -
//     bg_i; Eigen::Map<vec_t<15>> r(residuals); r << err_trans, err_rot,
//     err_vel, err_ba, err_bg; r = sqrt_info * r;
//     // clang-format on
//
//     // print_vector("err_trans", err_trans);
//     // print_vector("err_vel", err_vel);
//     // print_vector("predicted dpos", C_i_inv * (r_j - r_i - v_i * dt_ij +
//     0.5 * g * dt_ij_sq));
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
//         J_min[0].block<3, 3>(POS_IDX, 3) = skew(C_i_inv * (r_j - r_i - v_i *
//         dt_ij + 0.5 * g * dt_ij_sq)); J_min[0].block<3, 3>(ROT_IDX, 0) =
//         -quat_mat_xyz(quat_lmul(q_j_inv * q_i) * quat_rmul(gamma));
//         J_min[0].block<3, 3>(VEL_IDX, 3) = skew(C_i_inv * (v_j - v_i + g *
//         dt_ij)); J_min[0] = sqrt_info * J_min[0];
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
//         J_min[2].block<3, 3>(ROT_IDX, 3) = quat_lmul_xyz(gamma_inv * q_i_inv
//         * q_j); J_min[2] = sqrt_info * J_min[2];
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
  //     std::map<int, std::vector<reproj_error_td_t<pinhole_radtan4_t> *>>
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
