#include "calib_mono.hpp"

namespace yac {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             const std::string &proj_model,
                             const std::string &dist_model,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_t *pose,
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

      const auto residual = new calib_mono_residual_t{proj_model, dist_model,
                                                      kp, obj_pt};
      const auto cost_func =
          new ceres::AutoDiffCostFunction<calib_mono_residual_t,
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_CF
                                          3  // Size of: r_CF
                                          >(residual);

      problem.AddResidualBlock(cost_func, // Cost function
                               NULL,      // Loss function
                               intrinsics,
                               distortion,
                               pose->q,
                               pose->r);
    }
  }

  return 0;
}

int calib_mono_solve(const aprilgrids_t &aprilgrids,
                     calib_params_t &calib_params,
                     mat4s_t &T_CF) {
  // Optimization variables
  std::vector<calib_pose_t> T_CF_params;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    T_CF_params.emplace_back(aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   calib_params.proj_model,
                                   calib_params.dist_model,
                                   calib_params.proj_params.data(),
                                   calib_params.dist_params.data(),
                                   &T_CF_params[i],
                                   problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem.SetParameterization(T_CF_params[i].q,
                                &quaternion_parameterization);
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

  // // Estimate covariance matrix
  // std::vector<std::pair<const double*, const double*>> covar_blocks;
  // double *proj_params = calib_params.proj_params.data();
  // covar_blocks.push_back(std::make_pair(proj_params, proj_params));
  // printf("nb covar blocks: %zu\n", covar_blocks.size());
  //
  // ceres::Covariance::Options covar_options;
  // ceres::Covariance covar_est(covar_options);
  // const auto retval = covar_est.Compute(covar_blocks, &problem);
  // if (retval == false) {
  //   printf("Failed to estimate covariance!\n");
  // }
  //
  // double proj_proj_covar[4 * 4] = {0};
  // covar_est.GetCovarianceBlock(proj_params, proj_params, proj_proj_covar);
  // Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> covar_mat(proj_proj_covar);
  // print_matrix("covar_mat", covar_mat);

  // Clean up
  T_CF.clear();
  for (auto pose_param : T_CF_params) {
    T_CF.push_back(pose_param.T());
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const calib_params_t &cam) {
  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  const char *proj_model = cam.proj_model.c_str();
  const char *dist_model = cam.dist_model.c_str();
  const std::string intrinsics = arr2str(cam.proj_params.data(), 4);
  const std::string distortion = arr2str(cam.dist_params.data(), 4);

  fprintf(outfile, "cam0:\n");
  fprintf(outfile, "  resolution: [%d, %d]\n", cam.img_w, cam.img_h);
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
  calib_params_t calib_params(proj_model, dist_model,
                              resolution(0), resolution(1),
                              lens_hfov, lens_vfov);

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  mat4s_t T_CF;
  if (calib_mono_solve(grids, calib_params, T_CF) != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << calib_params.toString(0) << std::endl;
  calib_mono_stats(grids, calib_params, T_CF);

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  if (save_results(results_fpath, calib_params) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

int calib_mono_stats(const aprilgrids_t &aprilgrids,
                     const calib_params_t &calib_params,
                     const mat4s_t &poses) {
  const std::string proj_model = calib_params.proj_model;
  const std::string dist_model = calib_params.dist_model;
  const double *intrinsics = calib_params.proj_params.data();
  const double *distortion = calib_params.dist_params.data();

  // Obtain residuals using optimized params
  vec2s_t residuals;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    const quat_t q_CF = tf_quat(T_CF);
    const vec3_t r_CF = tf_trans(T_CF);

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
        const calib_mono_residual_t residual{proj_model, dist_model,
                                             keypoints[j], object_points[j]};
        real_t res[2] = {0.0, 0.0};
        residual(intrinsics,
                 distortion,
                 q_CF.coeffs().data(),
                 r_CF.data(),
                 res);
        residuals.emplace_back(res[0], res[1]);
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
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return 0;
}

mat4s_t calib_generate_poses(const calib_target_t &target) {
  const real_t target_width = (target.tag_rows - 1.0) * target.tag_size;
  const real_t target_height = (target.tag_cols - 1.0) * target.tag_size;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const auto x_range = linspace(-0.1, 0.1, 5);
  const auto y_range = linspace(-0.1, 0.1, 5);
  const auto z_range = linspace(0.3, 0.5, 5);

  // Generate camera positions infrom of the AprilGrid target in the target
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
  for (const auto cam_position : cam_positions) {
    mat4_t T_TC = lookat(cam_position, target_center);

    // Perturb rotation
    mat3_t C_TC = tf_rot(T_TC);
    const vec3_t rpy{randf(-0.4, 0.4), randf(-0.4, 0.4), randf(-0.4, 0.4)};
    const mat3_t C_perturb = euler321(rpy);
    C_TC = C_perturb * C_TC;
    T_TC.block(0, 0, 3, 3) = C_TC;

    // mat4_t T_TC = tf(I(3), cam_position);
    poses.push_back(T_TC);
  }

  return poses;
}

} //  namespace yac
