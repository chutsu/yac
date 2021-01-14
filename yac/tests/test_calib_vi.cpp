#include "munit.hpp"
#include "euroc.hpp"
#include "calib_mono.hpp"
#include "calib_vi.hpp"

namespace yac {

struct test_data_t {
	const std::string data_path = "/data/euroc/calib/imu_april";
	const std::string grids0_path = data_path + "/grid0/cam0";
	const std::string grids1_path = data_path + "/grid0/cam1";
  euroc_calib_t data{data_path};
  aprilgrids_t grids0;
  aprilgrids_t grids1;
};

test_data_t setup_test_data() {
	test_data_t test_data;
	LOG_INFO("Loading EuRoC data [%s]", test_data.data_path.c_str());

	// preprocess aprilgrids
	LOG_INFO("Preprocessing AprilGrid data ...");
	aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
	const auto grids0_path = test_data.grids0_path;
	const auto grids1_path = test_data.grids1_path;
	if (system(("mkdir -p " + grids0_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids0_path.c_str());
	}
	if (system(("mkdir -p " + grids1_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids1_path.c_str());
	}

	auto timeline = test_data.data.timeline();
	size_t i = 0;
	for (const auto &ts : timeline.timestamps) {
		if (i++ % 100 == 0) {
			printf(".");
			fflush(stdout);
		}

		const auto kv = timeline.data.equal_range(ts);
		for (auto it = kv.first; it != kv.second; it++) {
			const auto event = it->second;
			if (event.type == CAMERA_EVENT) {
				const auto ts = event.ts;
				const int cam_idx = event.camera_index;
				const cv::Mat image = cv::imread(event.image_path);
				const auto grid_fname = std::to_string(ts) + ".csv";

        std::string grid_file;
				if (cam_idx == 0) {
          grid_file = grids0_path + "/" + grid_fname;
        } else if (cam_idx == 1) {
          grid_file = grids1_path + "/" + grid_fname;
        }

        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
					grid = detector.detect(event.ts, image);
					grid.save(grid_file);
        } else {
          grid.load(grid_file);
        }

        switch (cam_idx) {
        case 0: test_data.grids0.push_back(grid); break;
        case 1: test_data.grids1.push_back(grid); break;
        }
			}
		}
	}
	printf("\n");

	return test_data;
}

int test_reproj_error() {
  test_data_t test_data = setup_test_data();

  auto grid_i = test_data.grids0[1];
  auto grid_j = test_data.grids0[2];


  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t kps_i;
  vec2s_t kps_j;
  vec3s_t object_points;
  aprilgrid_t::common_measurements(grid_i, grid_j,
                                   tag_ids,
                                   corner_indicies,
                                   kps_i, kps_j,
                                   object_points);

  int i = 0;
  auto ts_i = grid_i.timestamp;
  auto ts_j = grid_j.timestamp;
  const int tag_id = tag_ids[i];
  const int corner_idx = corner_indicies[i];
  const vec3_t r_FFi = object_points[i];
  const vec2_t z_i = kps_i[i];
  const vec2_t z_j = kps_j[i];
  const mat2_t covar = I(2);

  mat4_t T_WF;
  T_WF << -0.121411, -0.234188, 0.964580, -0.858724,
          0.992576, -0.035726, 0.116261, -0.407895,
          0.007234, 0.971535, 0.236787, -0.486840,
          0.000000, 0.000000, 0.000000, 1.000000;

  mat4_t T_WS;
  T_WS << -0.324045, 0.036747, -0.945328, 0.100000,
          0.036747, 0.998980, 0.026236, 0.200000,
          0.945328, -0.026236, -0.325065, 0.300000,
          0.000000, 0.000000, 0.000000, 1.000000;
  // ^ Note: Due to numerical stability issues the translation component cannot
  // be 0 for checking jacobians

  mat4_t T_SC;
  T_SC << 0.0, -1.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

  const int res[2] = {752, 480};
  const int cam_idx = 0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

  reproj_error_t<pinhole_radtan4_t> err(ts_i, ts_j, res,
                                        tag_id, corner_idx,
                                        r_FFi, z_i, z_j, T_WF,
                                        covar);

  id_t param_counter = 0;
  fiducial_t fiducial{param_counter++, T_WF};
  pose_t sensor_pose{param_counter++, ts_i, T_WS};
  extrinsics_t extrinsics{param_counter++, T_SC};
  camera_params_t cam(param_counter++,
                      cam_idx, res,
                      proj_model, dist_model,
                      proj_params, dist_params);
  time_delay_t td(param_counter++, 0.0);

  std::vector<double *> params = {
    fiducial.param.data(),
    sensor_pose.param.data(),
    extrinsics.param.data(),
    cam.param.data(),
    td.param.data()
  };
  vec2_t r;

  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J2;
  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J3;
  Eigen::Vector2d J4;
  std::vector<double *> jacobians = {
    J0.data(),
    J1.data(),
    J2.data(),
    J3.data(),
    J4.data(),
  };

  // Baseline
  err.Evaluate(params.data(), r.data(), jacobians.data());
  print_vector("r", r);

  double step = 1e-8;
  double threshold = 1e-5;

  // -- Test fiducial jacobian
  {
    mat_t<2, 2> fdiff;

    for (int i = 0; i < 2; i++) {
      fiducial.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      fiducial.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J0", fdiff, J0, threshold, true) == 0);
  }

  // -- Test sensor-pose jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      vec2_t r_fd;
      sensor_pose.perturb(i, 0.5 * step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      sensor_pose.perturb(i, -0.5 * step);

      vec2_t r_bd;
      sensor_pose.perturb(i, -0.5 * step);
      err.Evaluate(params.data(), r_bd.data(), nullptr);
      sensor_pose.perturb(i, 0.5 * step);

      fdiff.col(i) = (r_fd - r_bd) / step;
    }

    const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
  }

  // -- Test extrinsics jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      extrinsics.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      extrinsics.perturb(i, -step);
    }

    const mat_t<2, 6> J2_min = J2.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J2", fdiff, J2_min, threshold, true) == 0);
  }

  // -- Test camera-parameters jacobian
  {
    mat_t<2, 8> fdiff;

    for (int i = 0; i < 8; i++) {
      cam.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J3", fdiff, J3, threshold, true) == 0);
  }


  // -- Test time delay
  {
    mat_t<2, 1> fdiff;

    for (int i = 0; i < 2; i++) {
      td.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      td.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J4", fdiff, J4, threshold, true) == 0);
  }

  return 0;
}

// void imu_propagate(const imu_data_t &imu_data,
//                    const vec3_t &g,
//                    const vec_t<7> &pose_i,
//                    const vec_t<9> &sb_i,
//                    vec_t<7> &pose_j,
//                    vec_t<9> &sb_j) {
//   assert(imu_data.size() > 2);
//   auto na = zeros(3, 1);
//   auto ng = zeros(3, 1);
//   pose_j = pose_i;
//   sb_j = sb_i;
//
//   quat_t q_WS{pose_i(3), pose_i(0), pose_i(1), pose_i(2)};
//   vec3_t r_WS{pose_i(4), pose_i(5), pose_i(6)};
//   vec3_t v_WS = sb_i.segment<3>(0);
//   vec3_t ba = sb_i.segment<3>(3);
//   vec3_t bg = sb_i.segment<3>(6);
//
//   real_t dt = 0.0;
//   for (size_t k = 0; k < imu_data.timestamps.size(); k++) {
//     // Calculate dt
//     if ((k + 1) < imu_data.timestamps.size()) {
//       dt = ns2sec(imu_data.timestamps[k + 1] - imu_data.timestamps[k]);
//     }
//     const real_t dt_sq = dt * dt;
//
//     // Update position and velocity
//     const vec3_t a = (imu_data.accel[k] - ba - na);
//     r_WS += v_WS * dt + 0.5 * (q_WS * a) * dt_sq + (0.5 * g * dt_sq);
//     v_WS += (q_WS * a) * dt + (g * dt);
//
//     // Update rotation
//     const vec3_t w = (imu_data.gyro[k] - bg - ng);
//     const real_t scalar = 1.0;
//     const vec3_t vector = 0.5 * w * dt;
//     q_WS *= quat_t{scalar, vector(0), vector(1), vector(2)};
//   }
//
//   // Set results
//   pose_j << q_WS.x(), q_WS.y(), q_WS.z(), q_WS.w(), r_WS;
//   sb_j.segment(0, 3) = v_WS;
//   sb_j.segment(3, 3) = ba;
//   sb_j.segment(6, 3) = bg;
// }
//
// int test_imu_propagate() {
//   vio_sim_data_t sim_data;
//   sim_circle_trajectory(4.0, sim_data);
//   sim_data.save("/tmp/sim_data");
//
//   const int imu_index = 0;
//   imu_params_t imu_params;
//   imu_params.sigma_a_c = 1.86e-03;
//   imu_params.sigma_g_c = 1.87e-04;
//   imu_params.sigma_aw_c = 4.33e-04;
//   imu_params.sigma_gw_c = 2.66e-05;
//
//   imu_data_t imu_buf;
//
//   id_t new_id = 0;
//   std::vector<imu_error_t *> imu_errors;
//   std::vector<pose_t *> poses;
//   std::vector<sb_params_t *> speed_biases;
// 	ceres::Problem::Options prob_options;
//   prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//   ceres::Problem problem(prob_options);
//   PoseLocalParameterization pose_parameterization;
//
//   const timestamp_t t0 = sim_data.imu_ts[0];
//   const mat4_t T_WS_0 = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
//   const vec3_t vel_0 = sim_data.imu_vel[0];
//   const vec3_t ba_0 = zeros(3, 1);
//   const vec3_t bg_0 = zeros(3, 1);
//   auto pose0 = new pose_t{new_id++, t0, T_WS_0};
//   auto sb0 = new sb_params_t{new_id++, t0, vel_0, ba_0, bg_0};
//   poses.push_back(pose0);
//   speed_biases.push_back(sb0);
//
//   for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
//     imu_buf.add(sim_data.imu_ts[k], sim_data.imu_acc[k], sim_data.imu_gyr[k]);
//
//     if (imu_buf.size() > 5) {
//       auto imu_error = new imu_error_t{imu_index, imu_params, imu_buf};
// 			imu_errors.push_back(imu_error);
//
//       pose_t *pose_i = poses.back();
//       sb_params_t *sb_i = speed_biases.back();
//       const mat4_t T_WS_i = pose_i->tf();
//       const vec_t<9> speed_biases_i = sb_i->param;
//
//       mat4_t T_WS_j;
//       vec_t<9> speed_biases_j;
//       imu_error->propagate(imu_buf,
//                            T_WS_i, speed_biases_i,
//                            T_WS_j, speed_biases_j);
//
//       const timestamp_t ts = imu_buf.timestamps.back();
//       pose_t *pose_j = new pose_t(new_id++, ts, T_WS_j);
//       sb_params_t *sb_j = new sb_params_t(new_id++, ts, speed_biases_j);
//       poses.push_back(pose_j);
//       speed_biases.push_back(sb_j);
//       imu_buf.clear();
//
//       problem.AddResidualBlock(imu_error,
//                                nullptr,
//                                pose_i->param.data(),
//                                sb_i->param.data(),
//                                pose_j->param.data(),
//                                sb_j->param.data());
// 			break;
//     }
//   }
//
// 	for (const auto pose : poses) {
// 			problem.SetParameterization(pose->param.data(),
// 																	&pose_parameterization);
// 	}
//
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = true;
//   options.max_num_iterations = 100;
//   // options.check_gradients = true;
//
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << std::endl;
//   std::cout << std::endl;
//
//   // print_vector("pose_j", pose_i);
//   // print_matrix("T_WS", tf(pose_i));
//
//   // const auto pose_diff = T_WS - T_WS_j;
//   // const auto pos_diff = tf_trans(pose_diff);
//   // const auto rpy_diff = quat2euler(tf_quat(pose_diff));
//   // print_vector("pos diff", pos_diff);
//   // print_vector("rpy diff", rpy_diff);
//   // MU_CHECK((pos_diff).norm() < 1e-2);
//   // MU_CHECK((rpy_diff).norm() < 1e-2);
//   // MU_CHECK((sb - sb_j).norm() < 1e-2);
//
//   return 0;
// }

struct sim_data_t {
  // Camera data
  timestamps_t cam_ts;
  mat4s_t cam_poses_gnd;
  mat4s_t cam_poses;
  aprilgrids_t grids;

  // IMU data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  mat4s_t imu_poses;
  vec3s_t imu_vel;
};



int test_calib_vi_sim() {
  // Camera
  const int res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[1], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const pinhole_radtan4_t camera{res, proj_params, dist_params};

  // IMU
	imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.tau_a = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.tau_g = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.sigma_g_c = 12.0e-4; // Gyro noise density [rad/s/sqrt(Hz)]
  imu_params.sigma_a_c = 8.0e-3;  // Accel noise density [m/s^2/sqrt(Hz)]
  imu_params.sigma_gw_c = 4.0e-6; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  imu_params.sigma_aw_c = 4.0e-5; // Accel drift noise density [m/s^2/sqrt(Hz)]
  imu_params.g = 9.81007;         // Earth's acceleration due to gravity [m/s^2]

	// Extrinsics
	mat4_t T_SC;
  T_SC << 0.0, 0.0, 1.0, 0.0,
          -1.0, 0.0, 0.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

  // AprilGrid Pose
	mat4_t T_WF;
  T_WF << 0.0, 0.0, -1.0, 0.0,
          -1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

  // Simulate
	sim_data_t sim_data;
	const real_t sensor_velocity = 0.3;
	const real_t circle_r = 2.0;
  const real_t circle_dist = 2 * M_PI * circle_r;
  const real_t time_taken = circle_dist / sensor_velocity;
  const real_t f = 1.0 / time_taken;
  const real_t w = -2.0 * M_PI * f;
  const real_t w2 = w * w;

	const int tag_rows = 6;
	const int tag_cols = 6;
	const double tag_size = 0.088;
	const double tag_spacing = 0.3;

  const real_t dt = 1.0 / imu_params.rate;
  const real_t t_end = time_taken;
  real_t t = 0.0;
  real_t theta = deg2rad(180.0);
  real_t yaw = deg2rad(0.0);

  std::default_random_engine rndeng;
  sim_imu_t imu;
  imu.rate = imu_params.rate;
  imu.tau_a = imu_params.tau_a;
  imu.tau_g = imu_params.tau_g;
  imu.sigma_g_c = 0;
  imu.sigma_a_c = 0;
  imu.sigma_gw_c = 0;
  imu.sigma_aw_c = 0;
  // imu.sigma_g_c = imu_params.sigma_g_c;
  // imu.sigma_a_c = imu_params.sigma_a_c;
  // imu.sigma_gw_c = imu_params.sigma_gw_c;
  // imu.sigma_aw_c = imu_params.sigma_aw_c;
  imu.g = imu_params.g;

  const mat4_t T_SC_n = add_noise(T_SC, 0.01, 1.0);
  print_matrix("T_SC", T_SC_n);

  int idx = 0;
  while (t <= t_end) {
    // Form sensor pose
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    const vec3_t r_WS_W{rx, ry, rz};
    const vec3_t rpy_WS{0.0, 0.0, wrapPi(yaw)};
    const mat3_t C_WS_W = euler321(rpy_WS);
    const mat4_t T_WS_W = tf(C_WS_W, r_WS_W);

    // Form sensor velocity
    const real_t vx = -circle_r * w * sin(theta);
    const real_t vy = circle_r * w * cos(theta);
    const real_t vz = 0.0;
    sim_data.imu_vel.emplace_back(vx, vy, vz);

    // Form sensor angular velocity
    const timestamp_t ts_k = t * 1e9;
    const vec3_t w_WS_W{0.0, 0.0, w};

    // Form Sensor acceleration
    const real_t ax = -circle_r * w2 * cos(theta);
    const real_t ay = -circle_r * w2 * sin(theta);
    const real_t az = 0.0;
    const vec3_t a_WS_W{ax, ay, az};

    // Simulate imu measurements
    vec3_t a_WS_S;
    vec3_t w_WS_S;
    sim_imu_measurement(imu,
                        rndeng,
                        ts_k,
                        T_WS_W,
                        w_WS_W,
                        a_WS_W,
                        a_WS_S,
                        w_WS_S);
    sim_data.imu_ts.push_back(ts_k);
    sim_data.imu_acc.push_back(a_WS_S);
    sim_data.imu_gyr.push_back(w_WS_S);
    sim_data.imu_poses.push_back(T_WS_W);

    // Simulate camera measurements
    if ((idx % 5) == 0) {
      const mat4_t T_WC = T_WS_W * T_SC;
      const mat4_t T_CW = T_WC.inverse();
      sim_data.cam_ts.push_back(ts_k);
      sim_data.cam_poses_gnd.push_back(T_WS_W * T_SC);
      sim_data.cam_poses.push_back(T_WC);
      // sim_data.cam_poses.push_back(T_WS_W * T_SC);

      // Check which AprilGrid corner can the camera see
      aprilgrid_t grid{ts_k, tag_rows, tag_cols, tag_size, tag_spacing};
      for (int tag_id = 0; tag_id < (tag_rows * tag_cols); tag_id++) {
        for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
          const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
          const vec3_t r_CFi = tf_point(T_CW * T_WF, r_FFi);
          vec2_t z;
          if (camera.project(r_CFi, z) == 0) {
            grid.add(tag_id, corner_idx, z);
          }
        }
      }
      sim_data.grids.push_back(grid);
    }

    // Update
    t += dt;
    theta += w * dt;  // -ve to go from 180 to -180 in CW fashion
    yaw += w * dt;    // -ve to go from 180 to -180 in CW fashion
    idx++;
  }
	save_poses("/tmp/camera_poses.csv", sim_data.cam_ts, sim_data.cam_poses_gnd);
	save_poses("/tmp/sensor_poses.csv", sim_data.imu_ts, sim_data.imu_poses);
	save_pose("/tmp/extrinsics.csv", T_SC);
	save_pose("/tmp/fiducial.csv", T_WF);
  save_imu_data("/tmp/imu.csv", sim_data.imu_ts, sim_data.imu_acc, sim_data.imu_gyr);

  FILE *csv = fopen("/tmp/imu_vel.csv", "w");
  for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
    const timestamp_t ts = sim_data.imu_ts[k];
    const vec3_t vel = sim_data.imu_vel[k];
		fprintf(csv, "%ld,", ts);
		fprintf(csv, "%f,", vel(0));
		fprintf(csv, "%f,", vel(1));
		fprintf(csv, "%f", vel(2));
		fprintf(csv, "\n");
  }
  fflush(csv);
  fclose(csv);

	// Create timeline
	std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, timeline_event_t> timeline;
	for (size_t k = 0; k < sim_data.cam_ts.size(); k++) {
		const auto ts = sim_data.cam_ts[k];
		timeline_event_t event(ts, 0, sim_data.grids[k]);
		timeline.insert({ts, event});
		timestamps.insert(ts);
	}
	for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
		const auto ts = sim_data.imu_ts[k];
		const auto a_m = sim_data.imu_acc[k];
		const auto w_m = sim_data.imu_gyr[k];
		timeline_event_t event(ts, a_m, w_m);
		timeline.insert({ts, event});
		timestamps.insert(ts);
	}

  // Setup VI calibrator
	calib_vi_t calib;
	calib.add_imu(imu_params);
  calib.add_camera(0, res, proj_model, dist_model, proj_params, dist_params, true);
  calib.add_extrinsics(0, T_SC);

	for (const auto &ts : timestamps) {
		const auto kv = timeline.equal_range(ts);

		for (auto it = kv.first; it != kv.second; it++) {
			const auto event = it->second;
			switch (event.type) {
				case APRILGRID_EVENT: {
					const int cam_idx = event.camera_index;
					calib.add_measurement(cam_idx, event.grid);
					break;
				}
				case IMU_EVENT: {
					const auto ts = event.ts;
					const vec3_t a_m = event.a_m;
					const vec3_t w_m = event.w_m;
					calib.add_measurement(ts, a_m, w_m);
					break;
				}
			}
		}
	}
	calib.solve();
  calib.show_results();

  return 0;
}

int test_calib_vi() {
  test_data_t test_data = setup_test_data();

  // Camera parameters
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  // vec4_t cam0_proj_params{458.501070, 457.166661, 366.159110, 248.254996};
  // vec4_t cam0_dist_params{-0.287362, 0.078245, 0.000138, 0.000161};
  // vec4_t cam1_proj_params{457.001386, 455.589963, 378.700593, 255.177674};
  // vec4_t cam1_dist_params{-0.283609, 0.074904, -0.000109, 0.000191};
  vec4_t cam0_proj_params{458.654, 457.296, 367.215, 248.375};
  vec4_t cam0_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  vec4_t cam1_proj_params{457.587, 456.134, 379.999, 255.238};
  vec4_t cam1_dist_params{-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05};

  // Imu parameters
	imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.tau_a = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.tau_g = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.sigma_a_c = 0.002;       // Accel noise density [m/s^2/sqrt(Hz)]
  imu_params.sigma_aw_c = 0.003;      // Accel drift noise density [m/s^2/sqrt(Hz)]
  imu_params.sigma_g_c = 1.6968e-04;  // Gyro noise density [rad/s/sqrt(Hz)]
  imu_params.sigma_gw_c = 1.9393e-05; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  imu_params.g = 9.81007;         // Earth's acceleration due to gravity [m/s^2]

  // Setup VI calibrator
	calib_vi_t calib;
	calib.add_imu(imu_params);
  calib.add_camera(0, res, proj_model, dist_model, cam0_proj_params, cam0_dist_params, true);
  calib.add_camera(1, res, proj_model, dist_model, cam1_proj_params, cam1_dist_params, true);
	// -- Add sensor-camera extrinsics
	// clang-format off
	mat4_t T_SC0;
  T_SC0 << 0.0, -1.0, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;

	mat4_t T_C1C0;
  T_C1C0 << 0.999997, 0.002169, 0.001346, -0.109879,
            -0.002187, 0.999904, 0.013663, 0.000449,
            -0.001316, -0.013665, 0.999906, -0.000519,
            0.000000, 0.000000, 0.000000, 1.000000;
	mat4_t T_C0C1 = T_C1C0.inverse();

	mat4_t T_SC1 = T_SC0 * T_C0C1;
  // T_SC1 << 0.0, -1.0, 0.0, 0.0,
  //          1.0, 0.0, 0.0, 0.0,
  //          0.0, 0.0, 1.0, 0.0,
  //          0.0, 0.0, 0.0, 1.0;
  calib.add_extrinsics(0, T_SC0);
  calib.add_extrinsics(1, T_SC1);
	// clang-format on

	LOG_INFO("Adding data to problem ...");
	// int counter = 0;
	auto timeline = test_data.data.timeline();
	for (const auto &ts : timeline.timestamps) {
		const auto kv = timeline.data.equal_range(ts);

		for (auto it = kv.first; it != kv.second; it++) {
			const auto event = it->second;

			if (event.type == CAMERA_EVENT) {
				const int cam_idx = event.camera_index;
				const auto ts = event.ts;
				const auto grid_fname = std::to_string(ts) + ".csv";

				std::string grid_fpath;
				if (cam_idx == 0) {
					grid_fpath = test_data.grids0_path + "/" + grid_fname;
				} else if (cam_idx == 1) {
					grid_fpath = test_data.grids1_path + "/" + grid_fname;
        }
				aprilgrid_t grid{grid_fpath};
				calib.add_measurement(cam_idx, grid);

			} else if (event.type == IMU_EVENT) {
				// printf("imu\n");
				const auto ts = event.ts;
				const vec3_t a_m = event.a_m;
				const vec3_t w_m = event.w_m;
				calib.add_measurement(ts, a_m, w_m);
			}
		}
	}
	calib.solve();
	calib.save();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_reproj_error);
  // MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_calib_vi_sim);
  MU_ADD_TEST(test_calib_vi);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
