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

namespace yac {

template <typename CAMERA_TYPE>
struct reproj_error_t
    : public ceres::SizedCostFunction<2, 7, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  reproj_error_t(const int cam_res[2],
                 const vec3_t &r_FFi,
                 const vec2_t &z,
                 const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(2, 6)); // T_WF
    J_min.push_back(zeros(2, 6)); // T_WS
    J_min.push_back(zeros(2, 6)); // T_SC
    J_min.push_back(zeros(2, 8)); // cam params
  }

  ~reproj_error_t() {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map fiducial pose T_WF
    const mat4_t T_WF = tf(params[0]);
    const mat3_t C_WF = tf_rot(T_WF);
    const quat_t q_WF{C_WF};

    // Map sensor pose T_WS
    const mat4_t T_WS = tf(params[1]);
    const mat4_t T_SW = T_WS.inverse();
    const mat3_t C_WS = tf_rot(T_WS);
    const quat_t q_WS{C_WS};

    // Map sensor-camera pose T_SC
    const mat4_t T_SC = tf(params[2]);
    const mat4_t T_CS = T_SC.inverse();
    const mat3_t C_SC = tf_rot(T_SC);
    const quat_t q_SC{C_SC};

    // Map camera params
    Eigen::Map<const vecx_t> cam_params(params[3], 8);

    // Transform and project point to image plane
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
    const matx_t weighted_Jh = -1 * sqrt_info_ * Jh;
    const mat3_t C_CW = tf_rot(T_CS * T_SW);
    const mat3_t C_CS = C_SC.transpose();

    if (jacobians != NULL) {
      // Jacobians w.r.t. T_WF
      if (jacobians[0] != NULL) {
        J_min[0].block(0, 0, 2, 3) = weighted_Jh * -C_CW * -skew(C_WF * r_FFi_);
        J_min[0].block(0, 3, 2, 3) = weighted_Jh * -C_CW * I(3);
        if (valid == false) {
          J_min[0].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[0], q_WF, jacobians[0]);
      }

      // Jacobians w.r.t T_WS
      if (jacobians[1] != NULL) {
        J_min[1].block(0, 0, 2, 3) = weighted_Jh * -C_CW * -skew(C_WS * r_CFi);
        J_min[1].block(0, 3, 2, 3) = weighted_Jh * -C_CW * I(3);
        if (valid == false) {
          J_min[1].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[1], q_WS, jacobians[1]);
      }

      // Jacobians w.r.t T_SC
      if (jacobians[2] != NULL) {
        J_min[2].block(0, 0, 2, 3) = weighted_Jh * C_CS * skew(C_SC * r_CFi);
        J_min[2].block(0, 3, 2, 3) = weighted_Jh * -C_CS * I(3);
        if (valid == false) {
          J_min[2].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[2], q_SC, jacobians[2]);
      }

      // Jacobians w.r.t. camera parameters
      if (jacobians[3] != NULL) {
        J_min[1] = -1 * sqrt_info_ * J_params;
        if (valid == false) {
          J_min[1].setZero();
        }

        Eigen::Map<mat_t<2, 8, row_major_t>> J1(jacobians[1]);
        J1 = J_min[1];
      }
    }

    return true;
  }
};

/** Inertial residual */
struct imu_error_t : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const int imu_index = 0;
  const imu_params_t imu_params;
  const imu_data_t imu_data;
  const vec3_t g{0.0, 0.0, -9.81};

  mat_t<15, 15> F = I(15, 15);      // Transition matrix
  mat_t<12, 12> Q = zeros(12, 12);  // noise matrix
  mat_t<15, 15> P = zeros(15, 15);  // Covariance matrix
  mutable matxs_t J_min;

  // Parameter order indicies
  const int POS_IDX = 0;
  const int ROT_IDX = 3;
  const int VEL_IDX = 6;
  const int BA_IDX = 9;
  const int BG_IDX = 12;

  // Delta position, velocity and rotation between timestep i and j
  // (i.e start and end of imu measurements)
  vec3_t dp{0.0, 0.0, 0.0};
  vec3_t dv{0.0, 0.0, 0.0};
  quat_t dq{1.0, 0.0, 0.0, 0.0};

  // Accelerometer and gyroscope biases
  vec3_t bg{0.0, 0.0, 0.0};
  vec3_t ba{0.0, 0.0, 0.0};

  imu_error_t(const int imu_index_,
              const imu_params_t imu_params_,
              const imu_data_t imu_data_)
      : imu_index{imu_index_},
        imu_params{imu_params_},
        imu_data{imu_data_} {
    J_min.push_back(zeros(15, 6));  // T_WS at timestep i
    J_min.push_back(zeros(15, 9));  // Speed and bias at timestep i
    J_min.push_back(zeros(15, 6));  // T_WS at timestep j
    J_min.push_back(zeros(15, 9));  // Speed and bias at timestep j
    propagate(imu_data.timestamps, imu_data.accel, imu_data.gyro);
  }

  void propagate(const timestamps_t &ts,
                 const vec3s_t &imu_acc,
                 const vec3s_t &imu_gyr) {
    assert(ts.size() > 2);
    assert(ts.size() == imu_acc.size());
    assert(imu_gyr.size() == imu_acc.size());

    real_t dt = 0.0;
    real_t dt_prev = ns2sec(ts[1] - ts[0]);
    for (size_t k = 0; k < imu_gyr.size(); k++) {
      // Calculate dt
      if ((k + 1) < imu_gyr.size()) {
        dt = ns2sec(ts[k + 1] - ts[k]);
        dt_prev = dt;
      } else {
        dt = dt_prev;
      }

      // Update relative position and velocity
      dp = dp + dv * dt + 0.5 * (dq * (imu_acc[k] - ba)) * dt * dt;
      dv = dv + (dq * (imu_acc[k] - ba)) * dt;

      // Update relative rotation
      const real_t scalar = 1.0;
      const vec3_t vector = 0.5 * (imu_gyr[k] - bg) * dt;
      const quat_t dq_i{scalar, vector(0), vector(1), vector(2)};
      dq = dq * dq_i;

      // Transition matrix F
      const mat3_t C_ji = dq.toRotationMatrix();
      mat_t<15, 15> F_i = zeros(15, 15);
      F_i.block<3, 3>(POS_IDX, VEL_IDX) = I(3);
      F_i.block<3, 3>(ROT_IDX, ROT_IDX) = -skew(imu_gyr[k] - bg);
      F_i.block<3, 3>(ROT_IDX, BG_IDX) = -I(3);
      F_i.block<3, 3>(VEL_IDX, ROT_IDX) = -C_ji * skew(imu_acc[k] - ba);
      F_i.block<3, 3>(VEL_IDX, BA_IDX) = -C_ji;

      printf("k: %ld\n", k);
      printf("dt: %f\n", dt);
      print_matrix("rot rot", -skew(imu_gyr[k] - bg));

      // Input matrix G
      mat_t<15, 12> G_i = zeros(15, 12);
      G_i.block<3, 3>(ROT_IDX, BG_IDX) = -I(3);
      G_i.block<3, 3>(VEL_IDX, BA_IDX) = -C_ji;
      G_i.block<3, 3>(BA_IDX, BA_IDX) = I(3);
      G_i.block<3, 3>(BG_IDX, BG_IDX) = I(3);

      // Update jacobian and covariance matrix
      const mat_t<15, 15> I_Fi_dt = I(15) + dt * F_i;
      const mat_t<15, 12> Gi_dt = G_i * dt;
      F = I_Fi_dt * F;
      P = I_Fi_dt * P * I_Fi_dt.transpose() + Gi_dt * Q * Gi_dt.transpose();
    }
  }

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map out parameters
    // -- Sensor pose at timestep i
    const mat4_t T_i = tf(params[0]);
    const mat3_t C_i = tf_rot(T_i);
    const mat3_t C_i_inv = C_i.transpose();
    const quat_t q_i = tf_quat(T_i);
    const vec3_t r_i = tf_trans(T_i);
    // -- Speed and bias at timestamp i
    const vec_t<9> sb_i{params[1]};
    const vec3_t v_i = sb_i.segment<3>(0);
    const vec3_t ba_i = sb_i.segment<3>(3);
    const vec3_t bg_i = sb_i.segment<3>(6);
    // -- Sensor pose at timestep j
    const mat4_t T_j = tf(params[2]);
    const quat_t q_j = tf_quat(T_j);
    const vec3_t r_j = tf_trans(T_j);
    // -- Speed and bias at timestep j
    const vec_t<9> sb_j{params[3]};
    const vec3_t v_j = sb_j.segment<3>(0);
    const vec3_t ba_j = sb_j.segment<3>(3);
    const vec3_t bg_j = sb_j.segment<3>(6);

    // Obtain Jacobians for gyro and accel bias
    const mat3_t dp_dba = F.block<3, 3>(POS_IDX, BA_IDX);
    const mat3_t dp_dbg = F.block<3, 3>(POS_IDX, BG_IDX);
    const mat3_t dv_dba = F.block<3, 3>(VEL_IDX, BA_IDX);
    const mat3_t dv_dbg = F.block<3, 3>(VEL_IDX, BG_IDX);
    const mat3_t dq_dbg = F.block<3, 3>(ROT_IDX, BG_IDX);

    // Calculate square root info
    const matx_t info{P.inverse()};
    const matx_t sqrt_info{info.llt().matrixL().transpose()};

    // Calculate residuals
    const timestamp_t ts_i = imu_data.timestamps.front();
    const timestamp_t ts_j = imu_data.timestamps.back();
    const real_t dt_ij = ns2sec(ts_j - ts_i);
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
    const vec3_t err_trans = C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha;
    const vec3_t err_rot = 2.0 * (gamma_inv * (q_i_inv * q_j)).vec();
    const vec3_t err_vel = C_i_inv * (v_j - v_i + g * dt_ij) - beta;
    const vec3_t err_ba = ba_j - ba_i;
    const vec3_t err_bg = bg_j - bg_i;
    Eigen::Map<vec_t<15>> r(residuals);
    r << err_trans, err_rot, err_vel, err_ba, err_bg;
    r = sqrt_info * r;
    // clang-format on

    // Calculate Jacobians
    // clang-format off
    if (jacobians) {
      // Jacobian w.r.t. pose_i
      if (jacobians[0]) {
        J_min[0] = zeros(15, 6);
        J_min[0].block<3, 3>(POS_IDX, POS_IDX) = -C_i_inv;
        J_min[0].block<3, 3>(POS_IDX, ROT_IDX) = skew(C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq));
        J_min[0].block<3, 3>(ROT_IDX, ROT_IDX) = -quat_mat_xyz(quat_lmul(q_j_inv * q_i) * quat_rmul(gamma));
        J_min[0].block<3, 3>(VEL_IDX, ROT_IDX) = skew(C_i_inv * (v_j - v_i + g * dt_ij));
        J_min[0] = sqrt_info * J_min[0];

        map_mat_t<15, 7, row_major_t> J0(jacobians[0]);
        J0 = J_min[0];
      }

      // Jacobian w.r.t. sb_i
      if (jacobians[1]) {
        J_min[1] = zeros(15, 9);
        J_min[1].block<3, 3>(POS_IDX, VEL_IDX - VEL_IDX) = -C_i_inv * dt_ij;
        J_min[1].block<3, 3>(POS_IDX, BA_IDX - VEL_IDX) = -dp_dba;
        J_min[1].block<3, 3>(POS_IDX, BG_IDX - VEL_IDX) = -dp_dbg;
        J_min[1].block<3, 3>(VEL_IDX, VEL_IDX - VEL_IDX) = -C_i_inv;
        J_min[1].block<3, 3>(VEL_IDX, BA_IDX - VEL_IDX) = -dv_dba;
        J_min[1].block<3, 3>(VEL_IDX, BG_IDX - VEL_IDX) = -dv_dbg;
        J_min[1].block<3, 3>(BA_IDX, BA_IDX - VEL_IDX) = -I(3);
        J_min[1].block<3, 3>(BG_IDX, BG_IDX - VEL_IDX) = -I(3);
        J_min[1] = sqrt_info * J_min[1];

        map_mat_t<15, 7, row_major_t> J1(jacobians[1]);
        J1 = J_min[1];
      }

      // Jacobian w.r.t. pose_j
      if (jacobians[2]) {
        J_min[2] = zeros(15, 6);
        J_min[2].block<3, 3>(POS_IDX, POS_IDX) = C_i_inv;
        J_min[2].block<3, 3>(ROT_IDX, ROT_IDX) = quat_lmul_xyz(gamma_inv * q_i_inv * q_j_inv);
        J_min[2] = sqrt_info * J_min[2];

        map_mat_t<15, 7, row_major_t> J2(jacobians[2]);
        J2 = J_min[2];
      }

      // Jacobian w.r.t. sb_j
      if (jacobians[3]) {
        // -- Speed and bias at j Jacobian
        J_min[3] = zeros(15, 9);
        J_min[3].block<3, 3>(VEL_IDX, VEL_IDX - VEL_IDX) = C_i_inv;
        J_min[3].block<3, 3>(BA_IDX, BA_IDX - VEL_IDX) = I(3);
        J_min[3].block<3, 3>(BG_IDX, BG_IDX - VEL_IDX) = I(3);
        J_min[3] = sqrt_info * J_min[3];

        map_mat_t<15, 9, row_major_t> J3(jacobians[3]);
        J3 = J_min[3];
      }
    }
    // clang-format on

    return true;
  }
};

struct vi_calibrator_t {
  size_t new_param_id = 0;

  // Calibration data
  timeline_t timeline;

  // Process timeline
  std::map<int, aprilgrid_t> grids_buf;
  imu_data_t imu_buf;
  bool sensor_init = false;
  bool fiducial_init = false;
  mat4_t T_WS_init = I(4);
  mat4_t T_WF_init = I(4);

  // Parameters to be optimized
  pose_t fiducial_pose;
  std::vector<pose_t> sensor_poses;
  std::vector<sb_params_t> speed_biases;
  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsic_t> extrinsics;

  // Optimization
  imu_params_t imu_params;
  double sigma_vision = 0.5;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_parameterization;

  vi_calibrator_t() {
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = new ceres::Problem(prob_options);
  }

  ~vi_calibrator_t() {
    delete problem;
  }

  void add_imu(const imu_params_t &imu_params_) {
    imu_params = imu_params_;
  }

  void add_camera(const int cam_index,
                  const int resolution[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params) {
    camera_params_t cam(new_param_id++, cam_index, resolution,
                        proj_model, dist_model,
                        proj_params, dist_params);
    cam_params[cam_index] = cam;
  }

  void add_extrinsics(const int cam_index, const mat4_t &T_SC) {
    extrinsics[cam_index] = extrinsic_t(new_param_id++, T_SC);
  }

  void add_sensor_pose(const timestamp_t ts, const mat4_t &T_WS) {
    sensor_poses.emplace_back(new_param_id++, ts, T_WS);
  }

  void add_speed_biases(const timestamp_t ts, const vec_t<9> &sb) {
    speed_biases.emplace_back(new_param_id++, ts, sb);
  }

  void add_fiducial_pose(const timestamp_t ts, const mat4_t &T_WF) {
    fiducial_pose = pose_t(new_param_id++, ts, T_WF);
  }

  vecx_t get_camera(const int cam_index) {
    return cam_params[cam_index].param;
  }

  mat4_t get_extrinsic(const int cam_index) {
    return extrinsics[cam_index].tf();
  }

  mat4_t get_sensor_pose(const int pose_index) {
    return sensor_poses[pose_index].tf();
  }

  mat4_t get_fiducial_pose() {
    return fiducial_pose.tf();
  }

  size_t nb_cams() {
    return cam_params.size();
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
      if (ts > t1) {
        const vec3_t acc = imu_data.accel[k];
        const vec3_t gyr = imu_data.gyro[k];
        trimmed_imu_data.add(ts, acc, gyr);
      }
    }

    imu_data = trimmed_imu_data;
  }

  void add_imu_error() {
    double *pose_i = sensor_poses[sensor_poses.size() - 1].param.data();
    double *pose_j = sensor_poses[sensor_poses.size() - 2].param.data();
    double *sb_i = speed_biases[speed_biases.size() - 1].param.data();
    double *sb_j = speed_biases[speed_biases.size() - 2].param.data();

    const int imu_index = 0;
    auto error = new imu_error_t(imu_index, imu_params, imu_buf);
    problem->AddResidualBlock(error, nullptr, pose_i, sb_i, pose_j, sb_j);
  }

  void add_reproj_error(const int cam_index, const aprilgrid_t &grid) {
    const int *cam_res = cam_params[cam_index].resolution;
    const mat2_t covar = pow(sigma_vision, 2) * I(2);
    double *T_WF = fiducial_pose.param.data();
    double *T_WS = sensor_poses[sensor_poses.size() - 1].param.data();
    double *T_SC = extrinsics[cam_index].param.data();
    double *cam = cam_params[cam_index].param.data();

    for (const auto tag_id : grid.ids) {
      vec3s_t object_points;
      aprilgrid_object_points(grid, tag_id, object_points);

      vec2s_t keypoints;
      aprilgrid_get(grid, tag_id, keypoints);

      for (size_t j = 0; j < 4; j++) {
        const vec3_t r_FFi = object_points[j];
        const vec2_t z = keypoints[j];
        auto error = new reproj_error_t<pinhole_radtan4_t>(cam_res, r_FFi, z, covar);
        problem->AddResidualBlock(error, NULL, T_WF, T_WS, T_SC, cam);
      }
    }
  }

  void add_reproj_errors() {
    for (size_t cam_index = 0; cam_index < nb_cams(); cam_index++) {
      add_reproj_error(cam_index, grids_buf.at(cam_index));
    }
  }

  bool add_state(const timestamp_t &ts, const mat4_t &T_WS_k) {
    // Propagate pose and speedAndBias
    // const mat4_t T_WS = tf(sensor_poses.back().param);
    // const vec_t<9> sb = speed_biases.back().param;
    // imu_propagate(imu_data, T_WS, sb);

    // Infer velocity from two poses T_WS_k and T_WS_km1
    const mat4_t T_WS_km1 = tf(sensor_poses.back().param);
    const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
    const vec3_t r_WS_k = tf_trans(T_WS_k);
    const vec3_t v_WS_k = r_WS_k - r_WS_km1;

    vec_t<9> sb_k;
    sb_k << v_WS_k, zeros(3, 1), zeros(3, 1);

    // Add updated sensor pose T_WS and speed and biases sb
    // Note: instead of using the propagated sensor pose `T_WS`, we are using
    // the provided `T_WS_`, this is because in the scenario where we are using
    // AprilGrid, as a fiducial target, we actually have a better estimation of
    // the sensor pose, as supposed to the imu propagated sensor pose.
    add_sensor_pose(ts, T_WS_k);
    add_speed_biases(ts, sb_k);

    // Add error terms
    // add_imu_error();
    add_reproj_errors();

    return true;
  }

  void add_measurement(const int cam_index, const aprilgrid_t &grid) {
    grids_buf[cam_index] = grid;
  }

  void add_measurement(const timestamp_t ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m) {
    imu_buf.add(ts, a_m, w_m);

    if (grids_buf.size() == nb_cams()) {
      // -------------------------- Initialize T_WS ---------------------------
      if (sensor_init == false && imu_buf.size() > 5) {
        // Initialize initial IMU attitude
        mat3_t C_WS;
        imu_init_attitude(imu_buf.gyro, imu_buf.accel, C_WS, 5);

        // Add initial sensor pose
        T_WS_init = tf(C_WS, zeros(3, 1));
        add_sensor_pose(ts, T_WS_init);
        sensor_init = true;
      }

      // Make sure imu data is streaming first
      if (sensor_init == false) {
        LOG_WARN("IMU not initialized yet!");
        grids_buf.clear();
        return;
      }

      // -------------------------- Initialize T_WF ---------------------------
      if (sensor_init && fiducial_init == false && grids_buf[0].detected) {
        // Add T_WF
        const mat4_t T_C0F = grids_buf[0].T_CF;
        const mat4_t T_SC0 = get_extrinsic(0);
        const mat4_t T_WF = T_WS_init * T_SC0 * T_C0F;

        add_fiducial_pose(ts, T_WF);
        fiducial_init = true;
        grids_buf.clear();
        return;
      }

      // Make sure fiducial target is initialized
      if (fiducial_init == false) {
        LOG_WARN("Fiducial not initialized yet!");
        grids_buf.clear();
        return;
      }

      // -------------------------- Add new state - --------------------------
      // Make sure we have enough imu timestamps
      auto grids_ts = grids_buf[0].timestamp;
      if (imu_buf.timestamps.back() < grids_ts) {
        return;
      }
      if (grids_buf[0].detected == false || grids_buf[1].detected == false) {
        grids_buf.clear();
        return;
      }

      // Add states
      const mat4_t T_FC0 = grids_buf[0].T_CF.inverse();
      const mat4_t T_SC0 = get_extrinsic(0);
      const mat4_t T_C0S = T_SC0.inverse();
      const mat4_t T_WF = get_fiducial_pose();
      const mat4_t T_WS = T_WF * T_FC0 * T_C0S;
      add_state(grids_ts, T_WS);

      // Drop oldest IMU measurements and clear aprilgrids
      trim_imu_data(imu_buf, grids_ts);
      grids_buf.clear();
    }
  }

  void setup(const std::map<int, aprilgrids_t> &cam_grids,
              const imu_data_t &imu_data) {
    // Create timeline
    // -- Add camera events
    for (const auto &kv : cam_grids) {
      const auto cam_index = kv.first;
      const auto &grids = kv.second;

      for (const auto &grid : grids) {
        const timestamp_t ts = grid.timestamp;
        const timeline_event_t event{ts, cam_index, grid};
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

  void show_results() {
    // Show results
    // std::vector<double> cam0_errs, cam1_errs;
    // double cam0_rmse, cam1_rmse = 0.0;
    // double cam0_mean, cam1_mean = 0.0;
    // calib_mono_stats<CAMERA_TYPE>(cam0_grids, *cam0, *T_C0F, &cam0_errs, &cam0_rmse, &cam0_mean);
    // calib_mono_stats<CAMERA_TYPE>(cam1_grids, *cam1, *T_C1F, &cam1_errs, &cam1_rmse, &cam1_mean);

    // printf("Optimization results:\n");
    // printf("---------------------\n");
    // print_vector("cam0.proj_params", (*cam0).proj_params());
    // print_vector("cam0.dist_params", (*cam0).dist_params());
    // print_vector("cam1.proj_params", (*cam1).proj_params());
    // print_vector("cam1.dist_params", (*cam1).dist_params());
    // printf("\n");
    // print_matrix("T_C1C0", *T_C1C0);
    // printf("cam0 rms reproj error [px]: %f\n", cam0_rmse);
    // printf("cam1 rms reproj error [px]: %f\n", cam1_rmse);
    // printf("cam0 mean reproj error [px]: %f\n", cam0_mean);
    // printf("cam1 mean reproj error [px]: %f\n", cam1_mean);
    // printf("\n");
  }

  void solve() {
    // Solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.check_gradients = true;

    // Solve
    ceres::Solver::Summary summary;
    // ceres::Solve(options, problem, &summary);
    // std::cout << summary.FullReport() << std::endl;
  }
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
