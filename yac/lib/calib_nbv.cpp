#include "calib_nbv.hpp"

namespace yac {

double info_entropy(const matx_t &covar) {
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(covar);
  const auto eigvals = eig.eigenvalues().array();
  return -1.0 * eigvals.log().sum() / std::log(2);
}

double info_gain(const matx_t &calib_covar, const double info_prev) {
  const double info = info_entropy(calib_covar);
  return 0.5 * (info - info_prev);
}

double shannon_entropy(const matx_t &covar) {
  assert(covar.rows() == covar.cols());
  const auto k = pow(2.0 * M_PI * std::exp(1.0), covar.rows());
  return 0.5 * std::log(k * covar.determinant());
}

void simulate_imu(const ctraj_t &traj,
                  const timestamp_t &ts_start,
                  const timestamp_t &ts_end,
                  const mat4_t &T_BC0,
                  const mat4_t &T_BS,
                  const imu_params_t &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels) {
  // const mat4_t T_C0S = T_BC0.inverse() * T_BS;
  // const mat3_t C_C0S = tf_rot(T_C0S);
  // const vec3_t r_C0S = tf_trans(T_C0S);
  const timestamp_t imu_dt = sec2ts(1.0 / imu_params.rate);
  timestamp_t ts_k = ts_start;
  std::default_random_engine rndeng;

  sim_imu_t sim_imu;
  sim_imu.rate = imu_params.rate;
  // sim_imu.tau_a      = imu_params.tau;
  // sim_imu.tau_g      = imu_params.tau;
  // sim_imu.sigma_g_c  = imu_params.sigma_g_c;
  // sim_imu.sigma_a_c  = imu_params.sigma_a_c;
  // sim_imu.sigma_gw_c = imu_params.sigma_gw_c;
  // sim_imu.sigma_aw_c = imu_params.sigma_aw_c;
  sim_imu.sigma_g_c = 0.0;
  sim_imu.sigma_a_c = 0.0;
  sim_imu.sigma_gw_c = 0.0;
  sim_imu.sigma_aw_c = 0.0;
  sim_imu.g = imu_params.g;

  while (ts_k <= ts_end) {
    // Get camera pose, angular velocity and acceleration in camera frame
    const mat4_t T_WC = ctraj_get_pose(traj, ts_k);
    // const mat3_t C_WC = tf_rot(T_WC);
    const vec3_t v_WC = ctraj_get_velocity(traj, ts_k);
    const vec3_t a_WC = ctraj_get_acceleration(traj, ts_k);
    const vec3_t w_WC = ctraj_get_angular_velocity(traj, ts_k);

    // Convert camera frame to sensor frame
    const mat4_t T_WS_W = T_WC;
    const vec3_t w_WS_W = w_WC;
    const vec3_t a_WS_W = a_WC;
    const vec3_t v_WS_W = v_WC;
    // const mat4_t T_WS_W = T_WC * T_C0S;
    // const vec3_t v_WS_W = v_WC + C_WC * (w_WC.cross(r_C0S));
    // const vec3_t w_WS_W = C_C0S.transpose() * w_WC;
    // const vec3_t a_WS_W = a_WC * C_C0S;

    vec3_t a_WS_S{0.0, 0.0, 0.0};
    vec3_t w_WS_S{0.0, 0.0, 0.0};
    sim_imu_measurement(sim_imu,
                        rndeng,
                        ts_k,
                        T_WS_W,
                        w_WS_W,
                        a_WS_W,
                        a_WS_S,
                        w_WS_S);

    // printf("ts: %ld\n", ts_k);
    // print_matrix("T_WS_W", T_WS_W);
    // print_vector("w_WS_W", w_WS_W);
    // print_vector("a_WS_W", a_WS_W);
    // print_vector("a_WS_S", a_WS_S);
    // print_vector("w_WS_S", w_WS_S);

    imu_time.push_back(ts_k);
    imu_accel.push_back(a_WS_S);
    imu_gyro.push_back(w_WS_S);
    imu_poses.push_back(T_WS_W);
    imu_vels.push_back(v_WS_W);

    ts_k += imu_dt;
  }
}

// void nbt_create_timeline(const timestamps_t &imu_ts,
//                          const vec3s_t &imu_gyr,
//                          const vec3s_t &imu_acc,
//                          const std::vector<aprilgrids_t> grid_data,
//                          timeline_t &timeline) {
//   // -- Add imu events
//   for (size_t i = 0; i < imu_ts.size(); i++) {
//     const timestamp_t ts = imu_ts[i];
//     const vec3_t a_B = imu_acc[i];
//     const vec3_t w_B = imu_gyr[i];
//     const timeline_event_t event{ts, a_B, w_B};
//     timeline.add(event);
//   }
//
//   // -- Add aprilgrids observed from all cameras
//   for (size_t cam_idx = 0; cam_idx < grid_data.size(); cam_idx++) {
//     const auto grids = grid_data[cam_idx];
//     for (const auto &grid : grids) {
//       if (grid.detected == false) {
//         continue;
//       }
//
//       const auto ts = grid.timestamp;
//       const timeline_event_t event{ts, (int) cam_idx, grid};
//       timeline.add(event);
//     }
//   }
// }

} // namespace yac
