#include "yac/calib_camera.hpp"

namespace yac {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

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

int calib_camera_solve(const std::string &config_file) {
  // Calibration config data
  std::string data_path;
  std::string results_fpath;
  bool imshow = false;
  vec2_t resolution{0.0, 0.0};
  real_t lens_hfov = 0.0;
  real_t lens_vfov = 0.0;
  std::string camera_model;
  std::string distortion_model;

  // Parse calib config file
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow, true);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.lens_hfov", lens_hfov);
  parse(config, "cam0.lens_vfov", lens_vfov);
  parse(config, "cam0.camera_model", camera_model);
  parse(config, "cam0.distortion_model", distortion_model);

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
  const int img_w = resolution(0);
  const int img_h = resolution(1);
  const real_t fx = pinhole_focal(img_w, lens_hfov);
  const real_t fy = pinhole_focal(img_h, lens_vfov);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
  pinhole_t<radtan4_t> cam{img_w, img_h, proj_params, dist_params};

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  mat4s_t T_CF;
  if (calib_camera_solve(grids, cam, T_CF) != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << cam << std::endl;
  // std::cout << radtan << std::endl;
  calib_camera_stats<pinhole_radtan4_residual_t>(grids,
                                                 cam.params.data(),
                                                 cam.distortion.params.data(),
                                                 T_CF,
                                                 "");

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  if (save_results(results_fpath, resolution, cam) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

// // aprilgrid_t
// // nbv_create_aprilgrid(const calib_target_t &target,
// //                      const camera_geometry_t<pinhole_t, radtan4_t> &camera,
// //                      const vec2_t &resolution,
// //                      const mat4_t &T_CF) {
// //   // Create AprilGrid
// //   aprilgrid_t grid;
// //   grid.tag_rows = target.tag_rows;
// //   grid.tag_cols = target.tag_cols;
// //   grid.tag_size = target.tag_size;
// //   grid.tag_spacing = target.tag_spacing;
// //
// //   // For every AprilGrid tag
// //   grid.detected = true;
// //   grid.nb_detections = (grid.tag_rows * grid.tag_cols);
// //   for (int i = 0; i < grid.nb_detections; i++) {
// //     // Check if object points are observable
// //     vec3s_t object_points;
// //     aprilgrid_object_points(grid, i, object_points);
// //     vec2s_t observed_keypoints;
// //     vec3s_t observed_points_CF;
// //
// //     for (const auto &p_F : object_points) {
// //       const vec4_t hp_F = p_F.homogeneous();
// //       const vec3_t p_C = (T_CF * hp_F).head(3);
// //       const vec2_t kp = camera_geometry_project(camera, p_C);
// //
// //       const bool x_ok = (kp(0) >= 0 && kp(0) <= resolution(0));
// //       const bool y_ok = (kp(1) >= 0 && kp(1) <= resolution(1));
// //       if (x_ok && y_ok) {
// //         observed_keypoints.push_back(kp);
// //         observed_points_CF.push_back(p_C);
// //       }
// //     }
// //
// //     // Add keypoints and points
// //     if (observed_keypoints.size() == 4) {
// //       grid.ids.push_back(i);
// //       for (size_t i = 0; i < 4; i++) {
// //         grid.keypoints.push_back(observed_keypoints[i]);
// //         grid.points_CF.push_back(observed_points_CF[i]);
// //       }
// //     }
// //   }
// //
// //   // Esimated pose
// //   grid.estimated = true;
// //   grid.T_CF = T_CF;
// //
// //   return grid;
// // }
//
// // void nbv_draw_aprilgrid(const aprilgrid_t grid,
// //                         const pinhole_t &pinhole,
// //                         const radtan4_t &radtan,
// //                         const mat4_t &T_CT,
// //                         cv::Mat &frame) {
// //   const cv::Scalar blue(255, 0, 0);
// //   const cv::Scalar green(0, 255, 0);
// //   const cv::Scalar red(0, 0, 255);
// //
// //   for (const auto &tag_id : grid.ids) {
// //     // Get tag object points
// //     vec3s_t object_points;
// //     aprilgrid_object_points(grid, tag_id, object_points);
// //
// //     // Transform object points to camera frame
// //     const size_t nb_pts = object_points.size();
// //     const matx_t hp_T = vecs2mat(object_points);
// //     const matx_t hp_C = T_CT * hp_T;
// //     const matx_t p_C = hp_C.block(0, 0, 3, nb_pts);
// //
// //     // Draw a rectangle for each tag
// //     for (int i = 0; i < 4; i++) {
// //       cv::Point2f p1;
// //       {
// //         const vec3_t p = p_C.block(0, i, 3, 1);
// //         const vec2_t p_proj{p(0) / p(2), p(1) / p(2)};
// //         const vec2_t pt_D = distort(radtan, p_proj);
// //         const vec2_t pixel = project(pinhole, pt_D);
// //         p1 = cv::Point2f(pixel(0), pixel(1));
// //       }
// //
// //       cv::Point2f p2;
// //       {
// //         const size_t idx = ((i + 1) == 4) ? 0 : i + 1;
// //         const vec3_t p = p_C.block(0, idx, 3, 1);
// //         const vec2_t p_proj{p(0) / p(2), p(1) / p(2)};
// //         const vec2_t pt_D = distort(radtan, p_proj);
// //         const vec2_t pixel = project(pinhole, pt_D);
// //         p2 = cv::Point2f(pixel(0), pixel(1));
// //       }
// //
// //       cv::line(frame, p1, p2, red, 3);
// //     }
// //   }
// // }
// //
// // void nbv_find(const calib_target_t &target,
// //               const aprilgrids_t &aprilgrids,
// //               const pinhole_t &pinhole,
// //               const radtan4_t &radtan,
// //               mat4_t &nbv_pose,
// //               aprilgrid_t &nbv_grid) {
// //   // Evalute NBV
// //   real_t entropy_best = 0.0;
// //   const camera_geometry_t<pinhole_t, radtan4_t> camera{pinhole, radtan};
// //   const vec2_t resolution{pinhole.cx * 2.0, pinhole.cy * 2.0};
// //
// //   // -- Loop over all NBV poses and find one with lowest entropy
// //   for (const auto &T_TC : calib_generate_poses(target)) {
// //     // -- Setup AprilGrids with added NBV grid
// //     aprilgrids_t grids = aprilgrids;
// //     const mat4_t T_CT = T_TC.inverse();
// //     nbv_grid = nbv_create_aprilgrid(target, camera, resolution, T_CT);
// //     grids.push_back(nbv_grid);
// //
// //     // -- Setup camera and distortion model
// //     pinhole_t pinhole_copy = pinhole;
// //     radtan4_t radtan_copy = radtan;
// //     mat4s_t T_CT_ignore;
// //
// //     // -- Evaluate NBV
// //     const real_t entropy =
// //         calib_camera_nbv_solve(grids, pinhole_copy, radtan_copy, T_CT_ignore);
// //     if (entropy < entropy_best) {
// //       entropy_best = entropy;
// //       nbv_pose = T_CT;
// //     }
// //   }
// // }
//
// // static void initialize_intrinsics(cv::VideoCapture &camera,
// //                                   const calib_target_t &target,
// //                                   aprilgrids_t &aprilgrids,
// //                                   pinhole_t &pinhole,
// //                                   radtan4_t &radtan,
// //                                   mat4s_t &T_CT) {
// //   // Guess the camera intrinsics and distortion
// //   cv::Mat frame;
// //   camera.read(frame);
// //   const auto detector = aprilgrid_detector_t();
// //   const real_t fx = pinhole_focal(frame.cols, 120.0);
// //   const real_t fy = pinhole_focal(frame.rows, 120.0);
// //   const real_t cx = frame.cols / 2.0;
// //   const real_t cy = frame.rows / 2.0;
// //   pinhole = pinhole_t{frame.cols, frame.rows, fx, fy, cx, cy};
// //   radtan = radtan4_t{0.0, 0.0, 0.0, 0.0};
// //   const mat3_t K = pinhole.K();
// //   const vec4_t D = radtan.vec();
// //
// //   while (aprilgrids.size() < 5) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     // Detect AprilGrid
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     aprilgrid_detect(grid, detector, frame, K, D);
// //
// //     // Show image and get user input
// //     cv::imshow("Image", frame);
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     } else if (key == 'c' && grid.detected) {
// //       aprilgrids.push_back(grid);
// //     }
// //   }
// //
// //   calib_camera_nbv_solve(aprilgrids, pinhole, radtan, T_CT);
// //   std::cout << "pinhole: " << pinhole << std::endl;
// //   std::cout << "radtan: " << radtan << std::endl;
// // }
//
// static int process_aprilgrid(const aprilgrid_t &aprilgrid,
//                              real_t *intrinsics,
//                              real_t *distortion,
//                              calib_pose_t *pose,
//                              ceres::Problem *problem) {
//   for (const auto &tag_id : aprilgrid.ids) {
//     // Get keypoints
//     vec2s_t keypoints;
//     if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
//       LOG_ERROR("Failed to get AprilGrid keypoints!");
//       return -1;
//     }
//
//     // Get object points
//     vec3s_t object_points;
//     if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
//       LOG_ERROR("Failed to calculate AprilGrid object points!");
//       return -1;
//     }
//
//     // Form residual block
//     for (size_t i = 0; i < 4; i++) {
//       const auto kp = keypoints[i];
//       const auto obj_pt = object_points[i];
//
//       const auto residual = new pinhole_radtan4_residual_t{kp, obj_pt};
//       const auto cost_func =
//           new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t,
//                                           2, // Size of: residual
//                                           4, // Size of: intrinsics
//                                           4, // Size of: distortion
//                                           4, // Size of: q_CF
//                                           3  // Size of: r_CF
//                                           >(residual);
//
//       // const auto cost_func = new intrinsics_residual_t{kp, obj_pt};
//       problem->AddResidualBlock(cost_func, // Cost function
//                                 NULL,      // Loss function
//                                 intrinsics,
//                                 distortion,
//                                 pose->q,
//                                 pose->r);
//     }
//   }
//
//   return 0;
// }
//
// // real_t calib_camera_nbv_solve(aprilgrids_t &aprilgrids,
// //                               pinhole_t &pinhole,
// //                               radtan4_t &radtan,
// //                               mat4s_t &T_CF) {
// //   // Optimization variables
// //   std::vector<calib_pose_t> T_CF_params;
// //   for (size_t i = 0; i < aprilgrids.size(); i++) {
// //     T_CF_params.emplace_back(aprilgrids[i].T_CF);
// //   }
// //
// //   // Setup optimization problem
// //   ceres::Problem::Options problem_options;
// //   problem_options.local_parameterization_ownership =
// //       ceres::DO_NOT_TAKE_OWNERSHIP;
// //   std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
// //   ceres::EigenQuaternionParameterization quaternion_parameterization;
// //
// //   // Process all aprilgrid data
// //   for (size_t i = 0; i < aprilgrids.size(); i++) {
// //     int retval = process_aprilgrid(aprilgrids[i],
// //                                    *pinhole.data,
// //                                    *radtan.data,
// //                                    &T_CF_params[i],
// //                                    problem.get());
// //     if (retval != 0) {
// //       LOG_ERROR("Failed to add AprilGrid measurements to problem!");
// //       return -1;
// //     }
// //     problem->SetParameterization(T_CF_params[i].q,
// //                                  &quaternion_parameterization);
// //   }
// //
// //   // Set solver options
// //   ceres::Solver::Options options;
// //   options.minimizer_progress_to_stdout = false;
// //   options.max_num_iterations = 100;
// //   // options.check_gradients = true;
// //
// //   // Solve
// //   ceres::Solver::Summary summary;
// //   ceres::Solve(options, problem.get(), &summary);
// //
// //   // Estimate covariance
// //   real_t entropy = 0.0;
// //   {
// //     // Estimate pinhole and radtan covariance
// //     std::vector<std::pair<const real_t *, const real_t *>> covar_blocks;
// //     covar_blocks.push_back({*pinhole.data, *pinhole.data});
// //     covar_blocks.push_back({*radtan.data, *radtan.data});
// //     covar_blocks.push_back({*pinhole.data, *radtan.data});
// //     ceres::Covariance::Options options;
// //     ceres::Covariance covar_est(options);
// //     covar_est.Compute(covar_blocks, problem.get());
// //
// //     // Extract pinhole and radtan covariance
// //     real_t pinhole_covar[4 * 4];
// //     real_t radtan_covar[4 * 4];
// //     real_t pinhole_radtan_covar[4 * 4];
// //     covar_est.GetCovarianceBlock(*pinhole.data, *pinhole.data, pinhole_covar);
// //     covar_est.GetCovarianceBlock(*radtan.data, *radtan.data, radtan_covar);
// //     covar_est.GetCovarianceBlock(*pinhole.data,
// //                                  *radtan.data,
// //                                  pinhole_radtan_covar);
// //
// //     // Calculate shannon entropy
// //     matx_t covar;
// //     covar.resize(8, 8);
// //     covar.block(0, 0, 4, 4) = mat4_t{pinhole_covar};
// //     covar.block(0, 4, 4, 4) = mat4_t{pinhole_radtan_covar};
// //     covar.block(4, 4, 4, 4) = mat4_t{radtan_covar};
// //     covar.block(4, 0, 4, 4) = covar.block(0, 4, 4, 4).transpose();
// //     entropy = shannon_entropy(covar);
// //   }
// //
// //   // Update aprilgrid relative pose
// //   for (size_t idx = 0; idx < T_CF.size(); idx++) {
// //     aprilgrids[idx].T_CF = T_CF[idx];
// //   }
// //
// //   // Add results to T_CF
// //   T_CF.clear();
// //   for (auto pose_param : T_CF_params) {
// //     T_CF.push_back(pose_param.T());
// //   }
// //
// //   return entropy;
// // }
//
// // static void validate_calibration(const calib_target_t &target,
// //                                  cv::VideoCapture &camera,
// //                                  pinhole_t &pinhole,
// //                                  radtan4_t &radtan) {
// //   pinhole_radtan4_t camera_geometry{pinhole, radtan};
// //   const mat3_t cam_K = pinhole.K();
// //   const vec4_t cam_D = radtan.vec();
// //   const auto detector = aprilgrid_detector_t();
// //
// //   while (true) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     aprilgrid_detect(grid, detector, frame, cam_K, cam_D);
// //
// //     if (grid.detected) {
// //       cv::Mat validate_frame;
// //       validate_frame = validate_intrinsics(frame,
// //                                            grid.keypoints,
// //                                            grid.points_CF,
// //                                            camera_geometry);
// //       cv::imshow("Image", validate_frame);
// //     } else {
// //       cv::imshow("Image", frame);
// //     }
// //
// //     // Show image and get user input
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     }
// //   }
// // }
//
// // int calib_camera_nbv(const std::string &target_path, const size_t max_frames) {
// //   // Setup calibration target
// //   calib_target_t target;
// //   if (calib_target_load(target, target_path) != 0) {
// //     LOG_ERROR("Failed to load calib target [%s]!", target_path.c_str());
// //     return -1;
// //   }
// //
// //   // Setup camera
// //   cv::VideoCapture camera(2);
// //   if (camera.isOpened() == false) {
// //     return -1;
// //   }
// //   sleep(2);
// //
// //   // Initialize intrinsics
// //   aprilgrids_t aprilgrids;
// //   pinhole_t pinhole;
// //   radtan4_t radtan;
// //   mat4s_t target_poses;
// //   initialize_intrinsics(camera,
// //                         target,
// //                         aprilgrids,
// //                         pinhole,
// //                         radtan,
// //                         target_poses);
// //   LOG_INFO("Initialized intrinsics and distortion!");
// //
// //   // Loop camera feed
// //   const auto detector = aprilgrid_detector_t();
// //   mat4_t nbv_pose;
// //   aprilgrid_t nbv_grid;
// //   nbv_find(target, aprilgrids, pinhole, radtan, nbv_pose, nbv_grid);
// //
// //   while (true) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     // Detect AprilGrid
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     const mat3_t K = pinhole.K();
// //     const vec4_t D{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
// //     aprilgrid_detect(grid, detector, frame, K, D);
// //
// //     // Draw NBV
// //     nbv_draw_aprilgrid(nbv_grid, pinhole, radtan, nbv_pose, frame);
// //
// //     // Calculate position difference between desired and actual
// //     const vec3_t pos_desired = tf_trans(nbv_pose);
// //     const vec3_t pos_actual = tf_trans(grid.T_CF);
// //     const real_t pos_diff = (pos_desired - pos_actual).norm();
// //     bool pos_ok = (pos_diff < 0.03) ? true : false;
// //
// //     // Calculate attitute difference between desired and actual
// //     // const vec3_t rpy_desired = quat2euler(tf_quat(nbv_pose));
// //     // const vec3_t rpy_actual = quat2euler(tf_quat(grid.T_CF));
// //     // const real_t rpy_diff = (rpy_desired - rpy_actual).norm();
// //     // bool rpy_ok = (rad2deg(rpy_diff) < 5.0) ? true : false;
// //     bool rpy_ok = true;
// //
// //     // Update NBV if pose achieved and new AprilGrid, optimize and find NBV
// //     // printf("pos diff [%.2f]\t rpy_diff [%.2f]\n", pos_diff, rpy_diff);
// //     printf("pos diff [%.2f]\n", pos_diff);
// //     if (pos_ok && rpy_ok && grid.detected) {
// //       LOG_INFO("NBV Pose reached!");
// //       aprilgrids.push_back(grid);
// //       calib_camera_nbv_solve(aprilgrids, pinhole, radtan, target_poses);
// //
// //       LOG_INFO("Finding NBV ...");
// //       nbv_find(target, aprilgrids, pinhole, radtan, nbv_pose, nbv_grid);
// //       LOG_INFO("New NBV!");
// //     }
// //
// //     // Termination criteria
// //     if (aprilgrids.size() > max_frames) {
// //       break;
// //     }
// //
// //     // Show image and get user input
// //     cv::imshow("Image", frame);
// //     // cv::Mat frame_flipped;
// //     // cv::flip(frame, frame_flipped, 1);
// //     // cv::imshow("Image", frame_flipped);
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     }
// //   }
// //
// //   std::cout << pinhole << std::endl;
// //   std::cout << radtan << std::endl;
// //   validate_calibration(target, camera, pinhole, radtan);
// //
// //   return 0;
// // }

// int calib_camera_batch(const std::string &target_path,
//                        const size_t max_frames) {
//   // Setup calibration target
//   calib_target_t target;
//   if (calib_target_load(target, target_path) != 0) {
//     LOG_ERROR("Failed to load calib target [%s]!", target_path.c_str());
//     return -1;
//   }
//
//   // Setup camera
//   cv::VideoCapture camera(2);
//   if (camera.isOpened() == false) {
//     return -1;
//   }
//   // sleep(2);
//
//   // Guess the camera intrinsics and distortion
//   cv::Mat frame;
//   camera.read(frame);
//   const auto detector = aprilgrid_detector_t();
//   const real_t fx = pinhole_focal(frame.cols, 120.0);
//   const real_t fy = pinhole_focal(frame.rows, 120.0);
//   const real_t cx = frame.cols / 2.0;
//   const real_t cy = frame.rows / 2.0;
//   pinhole_t pinhole{frame.cols, frame.rows, fx, fy, cx, cy};
//   radtan4_t radtan{0.0, 0.0, 0.0, 0.0};
//   const mat3_t K = pinhole.K();
//   const vec4_t D = radtan.vec();
//
//   aprilgrids_t aprilgrids;
//   while (aprilgrids.size() < max_frames) {
//     // Get image
//     cv::Mat frame;
//     camera.read(frame);
//
//     // Detect AprilGrid
//     aprilgrid_t grid;
//     aprilgrid_set_properties(grid,
//                              target.tag_rows,
//                              target.tag_cols,
//                              target.tag_size,
//                              target.tag_spacing);
//     aprilgrid_detect(grid, detector, frame, K, D);
//
//     // Show image and get user input
//     cv::imshow("Image", frame);
//     char key = (char) cv::waitKey(1);
//     if (key == 'q') {
//       break;
//     } else if (key == 'c' && grid.detected) {
//       aprilgrids.push_back(grid);
//     }
//   }
//
//   mat4s_t target_poses;
//   calib_camera_solve(aprilgrids, pinhole, radtan, target_poses);
//   std::cout << pinhole << std::endl;
//   std::cout << radtan << std::endl;
//   // validate_calibration(target, camera, pinhole, radtan);
//
//   return 0;
// }

int calib_stereo_solve(const std::string &config_file) {
  // Calibration settings
  std::string data_path;
  std::string results_fpath;

  vec2_t cam0_resolution{0.0, 0.0};
  real_t cam0_lens_hfov = 0.0;
  real_t cam0_lens_vfov = 0.0;
  std::string cam0_camera_model;
  std::string cam0_distortion_model;

  vec2_t cam1_resolution{0.0, 0.0};
  real_t cam1_lens_hfov = 0.0;
  real_t cam1_lens_vfov = 0.0;
  std::string cam1_camera_model;
  std::string cam1_distortion_model;

  // Parse calibration config
  config_t config{config_file};

  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "cam0.resolution", cam0_resolution);
  parse(config, "cam0.lens_hfov", cam0_lens_hfov);
  parse(config, "cam0.lens_vfov", cam0_lens_vfov);
  parse(config, "cam0.camera_model", cam0_camera_model);
  parse(config, "cam0.distortion_model", cam0_distortion_model);
  parse(config, "cam1.resolution", cam1_resolution);
  parse(config, "cam1.lens_hfov", cam1_lens_hfov);
  parse(config, "cam1.lens_vfov", cam1_lens_vfov);
  parse(config, "cam1.camera_model", cam1_camera_model);
  parse(config, "cam1.distortion_model", cam1_distortion_model);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", config_file.c_str());
    return -1;
  }

  // Prepare aprilgrid data directory
  const auto cam0_grid_path = data_path + "/grid0/cam0/data";
  if (dir_exists(cam0_grid_path) == false) {
    dir_create(cam0_grid_path);
  }
  const auto cam1_grid_path = data_path + "/grid0/cam1/data";
  if (dir_exists(cam1_grid_path) == false) {
    dir_create(cam1_grid_path);
  }

  // Preprocess calibration data
  int retval = preprocess_stereo_data(calib_target,
                                      data_path + "/cam0/data",
                                      data_path + "/cam1/data",
                                      cam0_resolution,
                                      cam1_resolution,
                                      cam0_lens_hfov,
                                      cam0_lens_vfov,
                                      cam1_lens_hfov,
                                      cam1_lens_vfov,
                                      cam0_grid_path,
                                      cam1_grid_path);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load stereo calibration data
  aprilgrids_t cam0_aprilgrids;
  aprilgrids_t cam1_aprilgrids;
  retval = load_stereo_calib_data(cam0_grid_path,
                                  cam1_grid_path,
                                  cam0_aprilgrids,
                                  cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to load calibration data!");
    return -1;
  }

  // Setup initial cam0 intrinsics and distortion
  // -- cam0
  pinhole_t<radtan4_t> cam0;
  {
    const int img_w = cam0_resolution(0);
    const int img_h = cam0_resolution(1);
    const real_t fx = pinhole_focal(img_w, cam0_lens_hfov);
    const real_t fy = pinhole_focal(img_h, cam0_lens_vfov);
    const real_t cx = img_w / 2.0;
    const real_t cy = img_h / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
    cam0 = pinhole_t<radtan4_t>{img_w, img_h, proj_params, dist_params};
  }
  // -- cam1
  pinhole_t<radtan4_t> cam1;
  {
    const int img_w = cam1_resolution(0);
    const int img_h = cam1_resolution(1);
    const real_t fx = pinhole_focal(img_w, cam1_lens_hfov);
    const real_t fy = pinhole_focal(img_h, cam1_lens_vfov);
    const real_t cx = img_w / 2.0;
    const real_t cy = img_h / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
    cam1 = pinhole_t<radtan4_t>{img_w, img_h, proj_params, dist_params};
  }

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C0C1 = I(4);
  mat4s_t poses;
  retval = calib_stereo_solve(cam0_aprilgrids,
                              cam1_aprilgrids,
                              cam0,
                              cam1,
                              T_C0C1,
                              poses);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  // -- cam0
  std::cout << "cam0:" << std::endl;
  std::cout << cam0 << std::endl;
  std::cout << cam0.distortion << std::endl;
  // -- cam1
  std::cout << "cam1:" << std::endl;
  std::cout << cam1 << std::endl;
  std::cout << cam1.distortion << std::endl;
  // -- cam0-cam1 extrinsics
  std::cout << "T_C0C1:\n" << T_C0C1 << std::endl;
  std::cout << std::endl;

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  retval = save_results(results_fpath,
                        cam0_resolution,
                        cam1_resolution,
                        cam0,
                        cam1,
                        T_C0C1);
  if (retval != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
