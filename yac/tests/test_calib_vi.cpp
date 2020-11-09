#include "munit.hpp"
#include "calib_vi.hpp"

namespace yac {

void imu_propagate(const imu_data_t &imu_data,
                   const vec3_t &g,
                   const vec_t<7> &pose_i,
                   const vec_t<9> &sb_i,
                   vec_t<7> &pose_j,
                   vec_t<9> &sb_j) {
  assert(imu_data.size() > 2);
  auto na = zeros(3, 1);
  auto ng = zeros(3, 1);
  pose_j = pose_i;
  sb_j = sb_i;

  quat_t q_WS{pose_i(3), pose_i(0), pose_i(1), pose_i(2)};
  vec3_t r_WS{pose_i(4), pose_i(5), pose_i(6)};
  vec3_t v_WS = sb_i.segment<3>(0);
  vec3_t ba = sb_i.segment<3>(3);
  vec3_t bg = sb_i.segment<3>(6);

  real_t dt = 0.0;
  for (size_t k = 0; k < imu_data.timestamps.size(); k++) {
    // Calculate dt
    if ((k + 1) < imu_data.timestamps.size()) {
      dt = ns2sec(imu_data.timestamps[k + 1] - imu_data.timestamps[k]);
    }
    const real_t dt_sq = dt * dt;

    // Update position and velocity
    const vec3_t a = (imu_data.accel[k] - ba - na);
    r_WS += v_WS * dt + 0.5 * (q_WS * a) * dt_sq + (0.5 * g * dt_sq);
    v_WS += (q_WS * a) * dt + (g * dt);

    // Update rotation
    const vec3_t w = (imu_data.gyro[k] - bg - ng);
    const real_t scalar = 1.0;
    const vec3_t vector = 0.5 * w * dt;
    q_WS *= quat_t{scalar, vector(0), vector(1), vector(2)};
  }

  // Set results
  pose_j << q_WS.x(), q_WS.y(), q_WS.z(), q_WS.w(), r_WS;
  sb_j.segment(0, 3) = v_WS;
  sb_j.segment(3, 3) = ba;
  sb_j.segment(6, 3) = bg;
}

int test_imu_propagate() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
  sim_data.save("/tmp/sim_data");

  vec_t<7> pose_i;
  vec_t<9> sb_i;
  auto q_WS = sim_data.imu_rot[0];
  auto r_WS = sim_data.imu_pos[0];
  pose_i << q_WS.x(), q_WS.y(), q_WS.z(), q_WS.w(), r_WS;
  sb_i << sim_data.imu_vel[0], zeros(6, 1);

  imu_data_t imu_data;
  imu_data_t imu_buf;
  const vec3_t g{0.0, 0.0, -9.81};

  std::vector<mat4_t> poses;
  std::vector<vec_t<9>> speed_biases;
  poses.emplace_back(tf(q_WS, r_WS));
  speed_biases.emplace_back(sb_i);

  size_t half_index = sim_data.imu_ts.size() / 2.0;
  for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
  // for (size_t k = 0; k < half_index; k++) {
    vec_t<7> pose_j = pose_i;
    vec_t<9> sb_j = sb_i;
    imu_data.add(sim_data.imu_ts[k], sim_data.imu_acc[k], sim_data.imu_gyr[k]);
    imu_buf.add(sim_data.imu_ts[k], sim_data.imu_acc[k], sim_data.imu_gyr[k]);

    if (imu_data.size() > 10) {
      imu_propagate(imu_data, g, pose_i, sb_i, pose_j, sb_j);
      imu_data.clear();
    }

    pose_i = pose_j;
    sb_i = sb_j;

    const quat_t q_WS{pose_i(3), pose_i(0), pose_i(1), pose_i(2)};
    const vec3_t r_WS{pose_i(4), pose_i(5), pose_i(6)};
    poses.emplace_back(tf(q_WS, r_WS));
    speed_biases.emplace_back(sb_i);
  }

  print_matrix("pose_i", poses.front());
  print_matrix("pose_j", poses.back());
  print_vector("sb_i", speed_biases.front());
  print_vector("sb_j", speed_biases.back());
  printf("\n");

  // pose_t pose_i{0, imu_data.timestamps[0], sim_data.imu_poses.front()};
  // sb_params_t sb_i{1, imu_data.timestamps[0], sim_data.imu_vel.front(), zeros(3, 1), zeros(3, 1)};
  // pose_t pose_j{2, imu_data.timestamps.back(), sim_data.imu_poses.back()};
  // sb_params_t sb_j{3, imu_data.timestamps.back(), sim_data.imu_vel.back(), zeros(3, 1), zeros(3, 1)};

  const int imu_index = 0;
  const imu_params_t imu_params;
  timestamps_t sub_imu_timestamps(&sim_data.imu_ts[0], &sim_data.imu_ts[half_index]);
  vec3s_t sub_imu_accel(&sim_data.imu_acc[0], &sim_data.imu_acc[half_index]);
  vec3s_t sub_imu_gyro(&sim_data.imu_gyr[0], &sim_data.imu_gyr[half_index]);

  printf("nb imu ts: %ld\n", sub_imu_timestamps.size());
  printf("nb imu accel: %ld\n", sub_imu_accel.size());
  printf("nb imu gyro: %ld\n", sub_imu_gyro.size());


  imu_error_t residual(imu_index, imu_params, imu_buf);
  // calib_imu_residual_t residual(imu_index,
  //                                sub_imu_timestamps,
  //                                 sub_imu_accel,
  //                                 sub_imu_gyro);

  print_vector("dp", residual.dp);
  print_vector("dv", residual.dv);
  print_quaternion("dq", residual.dq);

  printf("ts end: %f\n", ns2sec(sim_data.imu_ts.back()));
  print_matrix("F", residual.F);

  // print_vector("pose_j", pose_i);
  // print_matrix("T_WS", tf(pose_i));

  // const auto pose_diff = T_WS - T_WS_j;
  // const auto pos_diff = tf_trans(pose_diff);
  // const auto rpy_diff = quat2euler(tf_quat(pose_diff));
  // print_vector("pos diff", pos_diff);
  // print_vector("rpy diff", rpy_diff);
  // MU_CHECK((pos_diff).norm() < 1e-2);
  // MU_CHECK((rpy_diff).norm() < 1e-2);
  // MU_CHECK((sb - sb_j).norm() < 1e-2);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_imu_propagate);
}

} // namespace yav

MU_RUN_TESTS(yac::test_suite);
