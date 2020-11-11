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

  const int imu_index = 0;
  imu_params_t imu_params;
  imu_params.sigma_a_c = 1.86e-03;
  imu_params.sigma_g_c = 1.87e-04;
  imu_params.sigma_aw_c = 4.33e-04;
  imu_params.sigma_gw_c = 2.66e-05;

  imu_data_t imu_buf;

  id_t new_id = 0;
  std::vector<imu_error_t *> imu_errors;
  std::vector<pose_t *> poses;
  std::vector<sb_params_t *> speed_biases;
	ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(prob_options);
  PoseLocalParameterization pose_parameterization;

  const timestamp_t t0 = sim_data.imu_ts[0];
  const mat4_t T_WS_0 = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  const vec3_t vel_0 = sim_data.imu_vel[0];
  const vec3_t ba_0 = zeros(3, 1);
  const vec3_t bg_0 = zeros(3, 1);
  auto pose0 = new pose_t{new_id++, t0, T_WS_0};
  auto sb0 = new sb_params_t{new_id++, t0, vel_0, ba_0, bg_0};
  poses.push_back(pose0);
  speed_biases.push_back(sb0);

  for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
    imu_buf.add(sim_data.imu_ts[k], sim_data.imu_acc[k], sim_data.imu_gyr[k]);

    if (imu_buf.size() > 5) {
      auto imu_error = new imu_error_t{imu_index, imu_params, imu_buf};
			imu_errors.push_back(imu_error);

      pose_t *pose_i = poses.back();
      sb_params_t *sb_i = speed_biases.back();
      const mat4_t T_WS_i = pose_i->tf();
      const vec_t<9> speed_biases_i = sb_i->param;

      mat4_t T_WS_j;
      vec_t<9> speed_biases_j;
      imu_error->propagate(imu_buf,
                           T_WS_i, speed_biases_i,
                           T_WS_j, speed_biases_j);

      const timestamp_t ts = imu_buf.timestamps.back();
      pose_t *pose_j = new pose_t(new_id++, ts, T_WS_j);
      sb_params_t *sb_j = new sb_params_t(new_id++, ts, speed_biases_j);
      poses.push_back(pose_j);
      speed_biases.push_back(sb_j);
      imu_buf.clear();

      problem.AddResidualBlock(imu_error,
                               nullptr,
                               pose_i->param.data(),
                               sb_i->param.data(),
                               pose_j->param.data(),
                               sb_j->param.data());
			break;
    }
  }

	for (const auto pose : poses) {
			problem.SetParameterization(pose->param.data(),
																	&pose_parameterization);
	}

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

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
