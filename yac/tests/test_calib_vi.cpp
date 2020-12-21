#include "munit.hpp"
#include "euroc.hpp"
#include "calib_mono.hpp"
#include "calib_vi.hpp"

namespace yac {

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

int test_calib_vi() {
	const std::string data_path = "/data/euroc_mav/imu_april";
	LOG_INFO("Loading EuRoC data [%s]", data_path.c_str());
  euroc_calib_t data(data_path);

	// preprocess aprilgrids
	LOG_INFO("Preprocessing AprilGrid data ...");
	aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
	const std::string grids0_path = data_path + "/grid0/cam0";
	const std::string grids1_path = data_path + "/grid0/cam1";
	if (system(("mkdir -p " + grids0_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids0_path.c_str());
	}
	if (system(("mkdir -p " + grids1_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids1_path.c_str());
	}

	auto timeline = data.timeline();
	size_t i = 0;
  aprilgrids_t grids0;
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

				if (cam_idx == 0 && file_exists(grids0_path + "/" + grid_fname) == false) {
					auto grid = detector.detect(event.ts, image);
					grid.save(grids0_path + "/" + grid_fname);
				} else if (cam_idx == 1 && file_exists(grids1_path + "/" + grid_fname) == false) {
					auto grid = detector.detect(event.ts, image);
					grid.save(grids1_path + "/" + grid_fname);
				}

				if (cam_idx == 0) {
				  aprilgrid_t grid{grids0_path + "/" + grid_fname};
				  if (grid.detected) {
            grids0.push_back(grid);
          }
				}
			}
		}
	}
	printf("\n");

  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  // vec4_t cam0_proj_params{458.641992, 457.348169, 366.001234, 248.234631};
  // vec4_t cam0_dist_params{-0.286772, 0.077026, 0.000240, 0.000036};
  // vec4_t cam1_proj_params{457.212815, 455.865019, 378.604549, 255.328973};
  // vec4_t cam1_dist_params{-0.283342, 0.074053, -0.000056, 0.000052};
  vec4_t cam0_proj_params{458.654, 457.296, 367.215, 248.375};
  vec4_t cam0_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  vec4_t cam1_proj_params{457.587, 456.134, 379.999, 255.238};
  vec4_t cam1_dist_params{-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05};

  // // Initialize camera intrinsics
  // const int res[2] = {752, 480};
  // const std::string proj_model = "pinhole";
  // const std::string dist_model = "radtan4";
  // const double lens_hfov = 90.0;
  // const double lens_vfov = 90.0;
  // const double fx = pinhole_focal(res[0], lens_hfov);
  // const double fy = pinhole_focal(res[1], lens_vfov);
  // const double cx = res[0] / 2.0;
  // const double cy = res[1] / 2.0;
  // vec4_t proj_params{fx, fy, cx, cy};
  // vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  // {
  //   LOG_INFO("Calibrating cam0 intrinsics ...");
  //   camera_params_t cam0{0, 0, res, proj_model, dist_model,
  //                        proj_params, dist_params};
  //
  //   calib_mono_data_t data;
  //   data.covar = I(2);
  //   data.grids = grids0;
  //   data.cam_params = cam0;
  //
  //   if (calib_mono_inc_solve<pinhole_radtan4_t>(data) != 0) {
  //     FATAL("Failed to calibrate cam0 intrinsics!");
  //   }
  //   print_vector("cam0", cam0.param);
  //
  //   proj_params = cam0.proj_params();
  //   dist_params = cam0.dist_params();
  // }

	imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.tau_a = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.tau_g = 3600.0;      // Reversion time constant, currently not in use [s]
  imu_params.sigma_g_c = 12.0e-4; // Gyro noise density [rad/s/sqrt(Hz)]
  imu_params.sigma_a_c = 8.0e-3;  // Accel noise density [m/s^2/sqrt(Hz)]
  imu_params.sigma_gw_c = 4.0e-6; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  imu_params.sigma_aw_c = 4.0e-5; // Accel drift noise density [m/s^2/sqrt(Hz)]
  imu_params.g = 9.81007;         // Earth's acceleration due to gravity [m/s^2]

	calib_vi_t calib;
	calib.add_imu(imu_params);
  calib.add_camera(0, res, proj_model, dist_model, cam0_proj_params, cam0_dist_params, true);
  calib.add_camera(1, res, proj_model, dist_model, cam1_proj_params, cam1_dist_params, true);
  // calib.add_camera(0, res, proj_model, dist_model, proj_params, dist_params);
  // calib.add_camera(1, res, proj_model, dist_model, proj_params, dist_params);
	// -- Add sensor-camera extrinsics
	// clang-format off
	// mat4_t T_SC0;
  // T_SC0 << 0.0, -1.0, 0.0, 0.0,
  //          1.0, 0.0, 0.0, 0.0,
  //          0.0, 0.0, 1.0, 0.0,
  //          0.0, 0.0, 0.0, 1.0;
	// mat4_t T_SC1;
  // T_SC1 << 0.0, -1.0, 0.0, 0.0,
  //          1.0, 0.0, 0.0, 0.0,
  //          0.0, 0.0, 1.0, 0.0,
  //          0.0, 0.0, 0.0, 1.0;
  mat4_t T_SC0;
  T_SC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
          0.0, 0.0, 0.0, 1.0;
	mat4_t T_C1C0;
  T_C1C0 << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
            -0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
            -0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
            0.0, 0.0, 0.0, 1.0;
  mat4_t T_C0C1 = T_C1C0.inverse();
	mat4_t T_SC1 = T_SC0 * T_C0C1;

  calib.add_extrinsics(0, T_SC0);
  calib.add_extrinsics(1, T_SC1);
	// clang-format on

	LOG_INFO("Adding data to problem ...");
	// int counter = 0;
	for (const auto &ts : timeline.timestamps) {
		const auto kv = timeline.data.equal_range(ts);

		for (auto it = kv.first; it != kv.second; it++) {
			const auto event = it->second;

			if (event.type == CAMERA_EVENT) {
				const int cam_idx = event.camera_index;
				const auto ts = event.ts;
				const auto grid_fname = std::to_string(ts) + ".csv";

				std::string grid_fpath;
				if (cam_idx == 0 && file_exists(grids0_path + "/" + grid_fname)) {
					grid_fpath = grids0_path + "/" + grid_fname;
				} else if (cam_idx == 1 && file_exists(grids1_path + "/" + grid_fname)) {
					grid_fpath = grids1_path + "/" + grid_fname;
				} else {
					FATAL("Where is cam%d grid file [%s]?", cam_idx, grid_fname.c_str());
				}

				aprilgrid_t grid{grid_fpath};
				calib.add_measurement(cam_idx, grid);
				printf("%ld grid[%d]\n", grid.timestamp, cam_idx);

			} else if (event.type == IMU_EVENT) {
				// printf("imu\n");
				const auto ts = event.ts;
				const vec3_t a_m = event.a_m;
				const vec3_t w_m = event.w_m;
				calib.add_measurement(ts, a_m, w_m);
				printf("%ld imu\n", ts);
			}
		}
	}
  {
    T_SC0 = calib.get_extrinsic(0);
    T_SC1 = calib.get_extrinsic(1);
    print_matrix("[before opt] T_SC0", T_SC0);
    print_matrix("[before opt] T_SC1", T_SC1);
    print_matrix("[before opt] T_C1C0", T_SC1.inverse() * T_SC0);
    print_vector("[before opt] cam0", calib.get_camera(0));
    print_vector("[before opt] cam1", calib.get_camera(1));
  }
	calib.solve();
	calib.save();

  T_SC0 = calib.get_extrinsic(0);
  T_SC1 = calib.get_extrinsic(1);
	print_matrix("[after opt] T_SC0", T_SC0);
	print_matrix("[after opt] T_SC1", T_SC1);
	print_matrix("[after opt] T_C1C0", T_SC1.inverse() * T_SC0);
	print_vector("[after opt] cam0", calib.get_camera(0));
	print_vector("[after opt] cam1", calib.get_camera(1));

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_calib_vi);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
