#include "calib_mono.hpp"

namespace yac {

static int save_results(const std::string &save_path,
                        const camera_params_t &params,
                        const double rmse,
                        const double mean) {
  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  const int img_w = params.resolution[0];
  const int img_h = params.resolution[1];
  const char *proj_model = params.proj_model.c_str();
  const char *dist_model = params.dist_model.c_str();
  const std::string intrinsics = arr2str(params.proj_params().data(), 4);
  const std::string distortion = arr2str(params.dist_params().data(), 4);

  fprintf(outfile, "calib_results:\n");
  fprintf(outfile, "  rms_reproj_error:  %.2f  # [px]\n", rmse);
  fprintf(outfile, "  mean_reproj_error: %.2f  # [px]\n", mean);

  fprintf(outfile, "cam0:\n");
  fprintf(outfile, "  resolution: [%d, %d]\n", img_w, img_h);
  fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
  fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
  fprintf(outfile, "  proj_params: %s\n", intrinsics.c_str());
  fprintf(outfile, "  dist_params: %s\n", distortion.c_str());

  // Finsh up
  fclose(outfile);

  return 0;
}

int calib_mono_solve(const std::string &config_file) {
  // Calibration config data
  std::string data_path;
  std::string results_fpath;
  bool imshow = true;
  double sigma_vision = 0.0;
  vec2_t resolution{0.0, 0.0};
  real_t lens_hfov = 0.0;
  real_t lens_vfov = 0.0;
  std::string proj_model;
  std::string dist_model;

  // Parse calib config file
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow);
  parse(config, "settings.sigma_vision", sigma_vision);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.lens_hfov", lens_hfov);
  parse(config, "cam0.lens_vfov", lens_vfov);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target in [%s]!", config_file.c_str());
    return -1;
  }

  // Prepare aprilgrid data directory
  const auto grid_data_path = data_path + "/grid0/cam0/data";
  if (dir_exists(grid_data_path) == false) {
    dir_create(grid_data_path);
  }

  // Preprocess calibration data
  const auto cam_data_path = data_path + "/cam0/data";
  int retval = preprocess_camera_data(calib_target,
                                      cam_data_path,
                                      resolution,
                                      lens_hfov,
                                      lens_vfov,
                                      grid_data_path,
                                      imshow);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load calibration data
  aprilgrids_t grids;
  timestamps_t timestamps;
  retval = load_camera_calib_data(grid_data_path, grids, timestamps);
  if (retval != 0) {
    LOG_ERROR("Failed to load camera calibration data!");
    return -1;
  }

  // Setup camera intrinsics and distortion
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {(int) resolution[0], (int) resolution[1]};
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      proj_params, dist_params};

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  const mat2_t covar = I(2) * pow(sigma_vision, 2);
  mat4s_t T_CF;
  std::vector<double> errs;
  double rmse = 0.0;
  double mean = 0.0;

  if (proj_model == "pinhole" && dist_model == "radtan4") {
    calib_mono_solve<pinhole_radtan4_t>(grids, covar, &cam, &T_CF);
    calib_mono_stats<pinhole_radtan4_t>(grids, cam, T_CF, &errs, &rmse, &mean);
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    calib_mono_solve<pinhole_equi4_t>(grids, covar, &cam, &T_CF);
    calib_mono_stats<pinhole_equi4_t>(grids, cam, T_CF, &errs, &rmse, &mean);
  } else {
    LOG_ERROR("[%s-%s] unsupported!", proj_model.c_str(), dist_model.c_str());
    return -1;
  }

  // Save results
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, cam, rmse, mean) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

// mat4_t calib_target_origin(const calib_target_t &target,
//                            const vec2_t &cam_res,
//                            const double hfov) {
// 	const double tag_rows = target.tag_rows;
//   const double tag_cols = target.tag_cols;
//   const double tag_size = target.tag_size;
//   const double tag_spacing = target.tag_spacing;
//
//   // Standard pinhole camera model equation
//   // x = f * X / Z
//   // x / f * X = 1 / Z
//   // Z = f * X / x
//
//   // Calculate target center
//   const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
//   const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
//   const double width = tag_cols * tag_size + spacing_x;
//   const double height = tag_rows * tag_size + spacing_y;
//   const vec2_t center{width / 2.0, height / 2.0};
//
//   // Calculate distance away from target center
//   const double half_width =  width / 2.0;
//   const double half_resolution_x = cam_res(0) / 2.0;
//   const double scale = 0.5; // Target width to be 50% of image space at T_TO
//   const double image_width = cam_res(0);
//   const auto fx = pinhole_focal(image_width, hfov);
//   const auto z_FO = fx * half_width / (half_resolution_x * scale);
//
//   // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
//   const mat3_t C_FO = I(3);
//   const vec3_t r_FO{center(0), center(1), z_FO};
//   const mat4_t T_FO = tf(C_FO, r_FO);
//
//   return T_FO;
// }
//
// mat4s_t calib_generate_poses(const calib_target_t &target) {
//   // const real_t target_width = (target.tag_rows - 1.0) * target.tag_size;
//   // const real_t target_height = (target.tag_cols - 1.0) * target.tag_size;
//   // const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};
//
//   const double tag_rows = target.tag_rows;
//   const double tag_cols = target.tag_cols;
//   const double tag_size = target.tag_size;
//   const double tag_spacing = target.tag_spacing;
//
//   const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
//   const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
//   const double target_width = tag_cols * tag_size + spacing_x;
//   const double target_height = tag_rows * tag_size + spacing_y;
//   const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};
//
//   // Pose settings
//   const auto x_range = linspace(-0.1, 0.1, 5);
//   const auto y_range = linspace(-0.1, 0.1, 5);
//   const auto z_range = linspace(0.3, 0.5, 5);
//
//   // Generate camera positions infront of the AprilGrid target in the target
//   // frame, r_TC.
//   vec3s_t cam_positions;
//   for (const auto &x : x_range) {
//     for (const auto &y : y_range) {
//       for (const auto &z : z_range) {
//         const vec3_t r_TC = vec3_t{x, y, z} + target_center;
//         cam_positions.push_back(r_TC);
//       }
//     }
//   }
//
//   // For each position create a camera pose that "looks at" the AprilGrid
//   // center in the target frame, T_TC.
//   mat4s_t poses;
//   for (const auto &cam_pos : cam_positions) {
//     mat4_t T_TC = lookat(cam_pos, target_center);
//
//     // // Perturb rotation
//     // mat3_t C_TC = tf_rot(T_TC);
//     // const vec3_t rpy{randf(-0.4, 0.4), randf(-0.4, 0.4), randf(-0.4, 0.4)};
//     // const mat3_t C_perturb = euler321(rpy);
//     // C_TC = C_perturb * C_TC;
//     // T_TC.block(0, 0, 3, 3) = C_TC;
//
//     // mat4_t T_TC = tf(I(3), cam_position);
//     poses.push_back(T_TC);
//   }
//
//   return poses;
// }
//
// mat4s_t generate_initial_poses(const calib_target_t &target) {
//   // Calculate target width and height
//   const double tag_rows = target.tag_rows;
//   const double tag_cols = target.tag_cols;
//   const double tag_spacing = target.tag_spacing;
//   const double tag_size = target.tag_size;
//   const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
//   const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
//   const double calib_width = tag_cols * tag_size + spacing_x;
//   const double calib_height = tag_rows * tag_size + spacing_y;
//   const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
//   const double calib_depth = calib_width * 1.2;
//
//   // Generate intial poses
//   mat4s_t poses;
//   // -- First pose is right infront of calibration target
//   const vec3_t calib_start{calib_width / 2.0, calib_height / 2.0, calib_depth};
//   poses.push_back(lookat(calib_start, calib_center));
//   // -- Other 4 are the 4 corners of the aprilgrid
//   for (const auto &x : linspace(0.0, calib_width * 1.5, 2)) {
//     for (const auto &y : linspace(0.0, calib_height * 1.5, 2)) {
//       vec3_t cam_pos{x, y, calib_depth};
//       poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
//     }
//   }
//
//   return poses;
// }
//
// mat4s_t generate_nbv_poses(const calib_target_t &target) {
//   // const vec2_t cam_res{640, 480};
//   // const double hfov = 90.0;
//   // mat4_t T_FO = calib_target_origin(target, cam_res, hfov);
//
//   // Calculate target width and height
//   const double tag_rows = target.tag_rows;
//   const double tag_cols = target.tag_cols;
//   const double tag_spacing = target.tag_spacing;
//   const double tag_size = target.tag_size;
//   const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
//   const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
//   const double calib_width = tag_cols * tag_size + spacing_x;
//   const double calib_height = tag_rows * tag_size + spacing_y;
//   const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
//   const double calib_depth = calib_width * 1.2;
//
//   mat4s_t nbv_poses;
//   for (const auto &x : linspace(0.0, calib_width * 1.5, 5)) {
//     for (const auto &y : linspace(0.0, calib_height * 1.5, 5)) {
//       for (const auto &z : linspace(calib_depth, calib_depth * 1.5, 5)) {
//         vec3_t cam_pos{x, y, z};
//         nbv_poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
//       }
//     }
//   }
//
//   return nbv_poses;
//
//   // // Trajectory parameters
//   // const double rho = (calib_width / 2.0) * 0.9;  // Sphere radius
//   // const double lat_min = deg2rad(0.0);
//   // const double lat_max = deg2rad(360.0);
//   // const double lon_min = deg2rad(0.0);
//   // const double lon_max = deg2rad(80.0);
//
//   // // Form Sphere offset (J) w.r.t. calibration origin (O)
//   // // The sphere origin is at the center of the sphere (duh), but we actually
//   // // want the north pole of the sphere to be the trajectory start point
//   // // therefore we need this offset.
//   // const mat3_t C_OJ = I(3);
//   // const vec3_t r_OJ{0.0, 0.0, -rho};
//   // const mat4_t T_OJ = tf(C_OJ, r_OJ);
//   //
//   // // // Create rotation matrix that converts z-forward to x-forward
//   // // const auto rpy = deg2rad(vec3_t{-90.0, 0.0, -90.0});
//   // // const auto C_BC = euler321(rpy);
//   // // const auto r_BC = zeros(3, 1);
//   // // const auto T_BC = tf(C_BC, r_BC);
//   // // const auto T_CB = T_BC.inverse();
//   //
//   // // Target center (Fc) w.r.t. Target origin (F)
//   // const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};
//   //
//   // // Orbit poses. Imagine a half sphere coming out from the calibration target
//   // // center. The trajectory would go from the pole of the sphere to the sides
//   // // of the sphere. While following the trajectory in a tangent manner the
//   // // camera view focuses on the target center.
//   // mat4s_t nbv_poses;
//   // for (const auto &lat : linspace(lat_min, lat_max, 9)) {
//   //   // Create sphere point and transform it into world frame
//   //   for (const auto &lon : linspace(lon_min, lon_max, 10)) {
//   //     const vec3_t p = sphere(rho, lon, lat);
//   //     const vec4_t hr_FJ = T_FO * T_OJ * p.homogeneous();
//   //     const vec3_t r_FJ = hr_FJ.head(3);
//   //     const mat4_t T_FC = lookat(r_FJ, r_FFc);
//   //
//   //     // Offset from camera (z-forward) to whatever imu frame is forward
//   //     vec3_t rpy{0.0, 0.0, - M_PI / 2.0};
//   //     mat3_t C = euler321(rpy);
//   //     mat4_t T_offset = tf(C, zeros(3, 1));
//   //
//   //     // Form nbv pose
//   //     nbv_poses.emplace_back(T_FC * T_offset);
//   //   }
//   // }
//
//   // return nbv_poses;
// }

// void estimate_covar() {
// 	// Push calibration params to recover covariance
// 	std::vector<std::shared_ptr<ParameterBlock>> recover_params;
// 	recover_params.push_back(T_SC_blocks_[0]);
// 	recover_params.push_back(T_SC_blocks_[1]);
// 	recover_params.push_back(cam_blocks_[0]);
// 	recover_params.push_back(cam_blocks_[1]);
//
// 	std::vector<std::pair<const double *, const double *>> covar_blocks;
// 	for (size_t i = 0; i < recover_params.size(); i++) {
// 		auto param_i = recover_params[i]->parameters();
// 		for (size_t j = i; j < recover_params.size(); j++) {
// 			auto param_j = recover_params[j]->parameters();
// 			covar_blocks.push_back({param_i, param_j});
// 		}
// 	}
//
// 	// Estimate covariance options
// 	::ceres::Covariance::Options options;
// 	options.num_threads = 1;
// 	options.algorithm_type = ::ceres::SPARSE_QR;
//
// 	// Estimate covariance
// 	::ceres::Covariance covar_est(options);
// 	auto problem_ptr = problem_->problem_.get();
// 	if (covar_est.Compute(covar_blocks, problem_ptr) == false) {
// 		LOG_ERROR("Failed to estimate covariance!");
// 		LOG_ERROR("Maybe Hessian is not full rank?");
// 		return -1;
// 	}
//
// 	// Form covariance matrix
// 	calib_covar = zeros(28, 28);
// 	size_t idx_i = 0;
// 	size_t idx_j = 0;
// 	for (size_t i = 0; i < recover_params.size(); i++) {
// 		auto param_i = recover_params[i]->parameters();
// 		auto size_i = recover_params[i]->minimalDimension();
//
// 		for (size_t j = i; j < recover_params.size(); j++) {
// 			auto param_j = recover_params[j]->parameters();
// 			auto size_j = recover_params[j]->minimalDimension();
//
// 			// Get covariance block
// 			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covar_block;
// 			covar_block.resize(size_i, size_j);
// 			covar_est.GetCovarianceBlockInTangentSpace(param_i, param_j, covar_block.data());
//
// 			if (i == j) {
// 				// Diagonal block
// 				calib_covar.block(idx_i, idx_j, size_i, size_j) = covar_block;
// 			} else {
// 				// Off-diagonal block
// 				calib_covar.block(idx_i, idx_j, size_i, size_j) = covar_block;
// 				calib_covar.block(idx_j, idx_i, size_j, size_i) = covar_block.transpose();
// 			}
//
// 			idx_j += size_j;
// 		}
// 		idx_i += size_i;
// 		idx_j = idx_i;
// 	}
//
// 	// Check if calib_covar is full-rank?
// 	if (rank(calib_covar) != calib_covar.rows()) {
// 		LOG_ERROR("calib_covar is not full rank!");
// 		return -1;
// 	}
// }

} //  namespace yac
