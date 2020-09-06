#include "calib_mono.hpp"

namespace yac {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             camera_params_t &cam_params,
                             pose_t &pose,
                             ceres::Problem &problem) {
  for (const auto &tag_id : aprilgrid.ids) {
    // Get keypoints
    vec2s_t keypoints;
    if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp = keypoints[i];
      const auto obj_pt = object_points[i];
      const auto cost_func = new calib_mono_residual_t{cam_params, kp, obj_pt};
      problem.AddResidualBlock(cost_func,
                               NULL,
                               pose.param.data(),
                               cam_params.param.data());
    }
  }

  return 0;
}

class PoseLocalParameterization : public ceres::LocalParameterization {
public:
  PoseLocalParameterization() {}
  virtual ~PoseLocalParameterization() {}

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
		// Form transform
		mat4_t T = tf(x);

		// Update rotation
		vec3_t dalpha{delta[0], delta[1], delta[2]};
		quat_t q = tf_quat(T);
		quat_t dq = quat_delta(dalpha);
		q = dq * q;
		q.normalize();

		// Update translation
		vec3_t dr{delta[3], delta[4], delta[5]};
		vec3_t r = tf_trans(T);
		r = r + dr;

		// Copy results to `x_plus_delta`
		x_plus_delta[0] = q.x();
		x_plus_delta[1] = q.y();
		x_plus_delta[2] = q.z();
		x_plus_delta[3] = q.w();

		x_plus_delta[4] = r(0);
		x_plus_delta[5] = r(1);
		x_plus_delta[6] = r(2);

		return true;
	}

  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
		Jp.setZero();
		Jp.bottomRightCorner<3, 3>().setIdentity();

    mat_t<4, 3> S = zeros(4, 3);
    S(0, 0) = 0.5;
    S(1, 1) = 0.5;
    S(2, 2) = 0.5;

    mat4_t T = tf(x);
    quat_t q = tf_quat(T);
    Jp.block<4, 3>(0, 0) = oplus(q) * S;

		return true;
	}

  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};

int calib_mono_solve(const aprilgrids_t &aprilgrids,
                     camera_params_t &cam_params,
                     mat4s_t &T_CF) {
  // Optimization variables
  std::vector<pose_t> poses;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    poses.emplace_back(i, i, aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  // ceres::EigenQuaternionParameterization quaternion_parameterization;
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   cam_params,
                                   poses[i],
                                   problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem.SetParameterization(poses[i].param.data(),
                                &pose_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Clean up
  T_CF.clear();
  for (auto pose : poses) {
    T_CF.push_back(pose.tf());
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const camera_params_t &params) {
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
  bool imshow = false;
  vec2_t resolution{0.0, 0.0};
  real_t lens_hfov = 0.0;
  real_t lens_vfov = 0.0;
  std::string proj_model;
  std::string dist_model;

  // Parse calib config file
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow, true);
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

  // Setup initial camera intrinsics and distortion for optimization
  // calib_params_t calib_params(proj_model, dist_model,
  //                             resolution(0), resolution(1),
  //                             lens_hfov, lens_vfov);

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
  camera_params_t cam_params{id, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  mat4s_t T_CF;
  if (calib_mono_solve(grids, cam_params, T_CF) != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  // std::cout << calib_params.toString(0) << std::endl;
  calib_mono_stats(grids, cam_params, T_CF);

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  if (save_results(results_fpath, cam_params) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

int calib_mono_stats(const aprilgrids_t &aprilgrids,
                     const camera_params_t &cam_params,
                     const mat4s_t &poses) {
  // Obtain residuals using optimized params
  vec2s_t residuals;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    pose_t pose(i, i, T_CF);

    // Iterate over all tags in AprilGrid
    for (const auto &tag_id : aprilgrid.ids) {
      // Get keypoints
      vec2s_t keypoints;
      if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
        LOG_ERROR("Failed to get AprilGrid keypoints!");
        return -1;
      }

      // Get object points
      vec3s_t object_points;
      if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
        LOG_ERROR("Failed to calculate AprilGrid object points!");
        return -1;
      }

      // Form residual and call the functor for four corners of the tag
      for (size_t j = 0; j < 4; j++) {
        const auto kp = keypoints[j];
        const auto obj_pt = object_points[j];
        const calib_mono_residual_t residual{cam_params, kp, obj_pt};

        std::vector<const double *> params;
				params.push_back(pose.param.data());
				params.push_back(cam_params.param.data());

        vec2_t r;
				residual.Evaluate(params.data(), r.data(), nullptr);
        residuals.push_back(r);
      }
    }
  }

  // Calculate RMSE reprojection error
  real_t err_sum = 0.0;
  for (auto &residual : residuals) {
    const real_t err = residual.norm();
    const real_t err_sq = err * err;
    err_sum += err_sq;
  }
  const real_t err_mean = err_sum / (real_t) residuals.size();
  const real_t rmse = sqrt(err_mean);
  std::cout << "nb_residuals: " << residuals.size() << std::endl;
  std::cout << "rmse reproj error [px]: " << rmse << std::endl;

  return 0;
}

// int calib_mono_estimate_covariance(const aprilgrids_t &aprilgrids,
//                                    calib_params_t &calib_params,
//                                    mat4s_t &T_CF) {
//   // Optimization variables
//   std::vector<calib_pose_t> T_CF_params;
//   for (size_t i = 0; i < aprilgrids.size(); i++) {
//     T_CF_params.emplace_back(aprilgrids[i].T_CF);
//   }
//
//   // Process all aprilgrid data
//   for (size_t i = 0; i < aprilgrids.size(); i++) {
//     int retval = process_aprilgrid(aprilgrids[i],
//                                    calib_params.resolution().data(),
//                                    calib_params.proj_model,
//                                    calib_params.dist_model,
//                                    calib_params.proj_params.data(),
//                                    calib_params.dist_params.data(),
//                                    &T_CF_params[i],
//                                    problem);
//     if (retval != 0) {
//       LOG_ERROR("Failed to add AprilGrid measurements to problem!");
//       return -1;
//     }
//     problem.SetParameterization(T_CF_params[i].q,
//                                 &quaternion_parameterization);
//   }
//
//
//   return 0;
// }


mat4_t calib_target_origin(const calib_target_t &target,
                           const vec2_t &cam_res,
                           const double hfov) {
	const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  // Standard pinhole camera model equation
  // x = f * X / Z
  // x / f * X = 1 / Z
  // Z = f * X / x

  // Calculate target center
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double width = tag_cols * tag_size + spacing_x;
  const double height = tag_rows * tag_size + spacing_y;
  const vec2_t center{width / 2.0, height / 2.0};

  // Calculate distance away from target center
  const double half_width =  width / 2.0;
  const double half_resolution_x = cam_res(0) / 2.0;
  const double scale = 0.5; // Target width to be 50% of image space at T_TO
  const double image_width = cam_res(0);
  const auto fx = pinhole_focal(image_width, hfov);
  const auto z_FO = fx * half_width / (half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  const mat4_t T_FO = tf(C_FO, r_FO);

  return T_FO;
}

mat4s_t calib_generate_poses(const calib_target_t &target) {
  // const real_t target_width = (target.tag_rows - 1.0) * target.tag_size;
  // const real_t target_height = (target.tag_cols - 1.0) * target.tag_size;
  // const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double target_width = tag_cols * tag_size + spacing_x;
  const double target_height = tag_rows * tag_size + spacing_y;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const auto x_range = linspace(-0.1, 0.1, 5);
  const auto y_range = linspace(-0.1, 0.1, 5);
  const auto z_range = linspace(0.3, 0.5, 5);

  // Generate camera positions infront of the AprilGrid target in the target
  // frame, r_TC.
  vec3s_t cam_positions;
  for (const auto &x : x_range) {
    for (const auto &y : y_range) {
      for (const auto &z : z_range) {
        const vec3_t r_TC = vec3_t{x, y, z} + target_center;
        cam_positions.push_back(r_TC);
      }
    }
  }

  // For each position create a camera pose that "looks at" the AprilGrid
  // center in the target frame, T_TC.
  mat4s_t poses;
  for (const auto &cam_pos : cam_positions) {
    mat4_t T_TC = lookat(cam_pos, target_center);

    // // Perturb rotation
    // mat3_t C_TC = tf_rot(T_TC);
    // const vec3_t rpy{randf(-0.4, 0.4), randf(-0.4, 0.4), randf(-0.4, 0.4)};
    // const mat3_t C_perturb = euler321(rpy);
    // C_TC = C_perturb * C_TC;
    // T_TC.block(0, 0, 3, 3) = C_TC;

    // mat4_t T_TC = tf(I(3), cam_position);
    poses.push_back(T_TC);
  }

  return poses;
}

mat4s_t generate_initial_poses(const calib_target_t &target) {
  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  // Generate intial poses
  mat4s_t poses;
  // -- First pose is right infront of calibration target
  const vec3_t calib_start{calib_width / 2.0, calib_height / 2.0, calib_depth};
  poses.push_back(lookat(calib_start, calib_center));
  // -- Other 4 are the 4 corners of the aprilgrid
  for (const auto &x : linspace(0.0, calib_width * 1.5, 2)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 2)) {
      vec3_t cam_pos{x, y, calib_depth};
      poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
    }
  }

  return poses;
}

mat4s_t generate_nbv_poses(const calib_target_t &target) {
  // const vec2_t cam_res{640, 480};
  // const double hfov = 90.0;
  // mat4_t T_FO = calib_target_origin(target, cam_res, hfov);

  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  mat4s_t nbv_poses;
  for (const auto &x : linspace(0.0, calib_width * 1.5, 5)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 5)) {
      for (const auto &z : linspace(calib_depth, calib_depth * 1.5, 5)) {
        vec3_t cam_pos{x, y, z};
        nbv_poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
      }
    }
  }

  return nbv_poses;

  // // Trajectory parameters
  // const double rho = (calib_width / 2.0) * 0.9;  // Sphere radius
  // const double lat_min = deg2rad(0.0);
  // const double lat_max = deg2rad(360.0);
  // const double lon_min = deg2rad(0.0);
  // const double lon_max = deg2rad(80.0);

  // // Form Sphere offset (J) w.r.t. calibration origin (O)
  // // The sphere origin is at the center of the sphere (duh), but we actually
  // // want the north pole of the sphere to be the trajectory start point
  // // therefore we need this offset.
  // const mat3_t C_OJ = I(3);
  // const vec3_t r_OJ{0.0, 0.0, -rho};
  // const mat4_t T_OJ = tf(C_OJ, r_OJ);
  //
  // // // Create rotation matrix that converts z-forward to x-forward
  // // const auto rpy = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  // // const auto C_BC = euler321(rpy);
  // // const auto r_BC = zeros(3, 1);
  // // const auto T_BC = tf(C_BC, r_BC);
  // // const auto T_CB = T_BC.inverse();
  //
  // // Target center (Fc) w.r.t. Target origin (F)
  // const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};
  //
  // // Orbit poses. Imagine a half sphere coming out from the calibration target
  // // center. The trajectory would go from the pole of the sphere to the sides
  // // of the sphere. While following the trajectory in a tangent manner the
  // // camera view focuses on the target center.
  // mat4s_t nbv_poses;
  // for (const auto &lat : linspace(lat_min, lat_max, 9)) {
  //   // Create sphere point and transform it into world frame
  //   for (const auto &lon : linspace(lon_min, lon_max, 10)) {
  //     const vec3_t p = sphere(rho, lon, lat);
  //     const vec4_t hr_FJ = T_FO * T_OJ * p.homogeneous();
  //     const vec3_t r_FJ = hr_FJ.head(3);
  //     const mat4_t T_FC = lookat(r_FJ, r_FFc);
  //
  //     // Offset from camera (z-forward) to whatever imu frame is forward
  //     vec3_t rpy{0.0, 0.0, - M_PI / 2.0};
  //     mat3_t C = euler321(rpy);
  //     mat4_t T_offset = tf(C, zeros(3, 1));
  //
  //     // Form nbv pose
  //     nbv_poses.emplace_back(T_FC * T_offset);
  //   }
  // }

  // return nbv_poses;
}

} //  namespace yac
