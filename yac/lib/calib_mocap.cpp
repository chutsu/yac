#include "calib_mocap.hpp"

namespace yac {

// MOCAP MARKER - CAMERA RESIDUAL //////////////////////////////////////////////

// mocap_residual_t::mocap_residual_t(const camera_geometry_t *cam_geom,
//                                    const vec2_t &z,
//                                    const mat2_t &covar)
//     : cam_geom_{cam_geom_}, z_{z}, covar_{covar}, info_{covar.inverse()},
//       sqrt_info_{info_.llt().matrixU()} {
//   set_num_residuals(2);
//   auto block_sizes = mutable_parameter_block_sizes();
//   block_sizes->push_back(7); // Fiducial pose
//   block_sizes->push_back(7); // Marker pose
//   block_sizes->push_back(7); // Marker-camera extrinsics
// }
//
// bool mocap_residual_t::Evaluate(double const *const *params,
//                                 double *residuals,
//                                 double **jacobians) const {
//   // Map optimization variables to Eigen
//   const mat4_t T_WF = tf(params[0]);
//   const mat4_t T_WM = tf(params[1]);
//   const mat4_t T_MC0 = tf(params[2]);
//   Eigen::Map<const vecx_t> cam_params(params[3], 8);
//
//   // Transform and project point to image plane
//   const mat4_t T_MW = T_WM.inverse();
//   const mat4_t T_C0M = T_MC0.inverse();
//   const vec3_t r_MFi = tf_point(T_MW * T_WF, r_FFi_);
//   const vec3_t r_C0Fi = tf_point(T_C0M * T_MW * T_WF, r_FFi_);
//   const vec2_t p{r_C0Fi(0) / r_C0Fi(2), r_C0Fi(1) / r_C0Fi(2)};
//   mat_t<2, 3> Jh;
//   matx_t J_params;
//   vec2_t z_hat;
//   bool valid = true;
//
//   if (cam_geom_->project(cam_res_, cam_params, r_C0Fi, z_hat) != 0) {
//     valid = false;
//   }
//
//   // Residual
//   const vec2_t r = sqrt_info_ * (z_ - z_hat);
//   residuals[0] = r(0);
//   residuals[1] = r(1);
//
//   // Jacobians
//   const matx_t J_cam_params = cam_geom_->params_jacobian(cam_params, r_C0Fi);
//   const matx_t Jh_weighted = sqrt_info_ * Jh;
//   const mat3_t C_C0W = tf_rot(T_C0M * T_MW);
//   const mat3_t C_MC0 = tf_rot(T_MC0);
//   const mat3_t C_C0M = C_MC0.transpose();
//
//   if (jacobians) {
//     // Jacobians w.r.t T_WF
//     if (jacobians[0]) {
//       const mat3_t C_WF = tf_rot(T_WF);
//
//       Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
//       J.setZero();
//       J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_C0W * I(3);
//       J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_C0W * -skew(C_WF * r_FFi_);
//       if (valid == false) {
//         J.setZero();
//       }
//     }
//
//     // Jacobians w.r.t T_WM
//     if (jacobians[1]) {
//       const mat3_t C_WM = tf_rot(T_WM);
//
//       Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
//       J.setZero();
//       J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0W * I(3);
//       J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0W * -skew(C_WM * r_MFi);
//       if (valid == false) {
//         J.setZero();
//       }
//     }
//
//     // Jacobians w.r.t T_MC0
//     if (jacobians[2]) {
//       Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
//       J.setZero();
//       J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0M * I(3);
//       J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0M * -skew(C_MC0 *
//       r_C0Fi); if (valid == false) {
//         J.setZero();
//       }
//     }
//
//     // Jacobians w.r.t camera params
//     if (jacobians[3]) {
//       Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
//       J = -1 * sqrt_info_ * J_params;
//       if (valid == false) {
//         J.setZero();
//       }
//     }
//   }
//
//   return true;
// }

// MOCAP MARKER - CAMERA CALIBRATION DATA //////////////////////////////////////

aprilgrids_t preprocess_calib_data(const calib_target_t &calib_target,
                                   const std::string &cam_path,
                                   const std::string &grid_path) {
  // Setup AprilGrid detector
  const aprilgrid_detector_t detector(calib_target.tag_rows,
                                      calib_target.tag_cols,
                                      calib_target.tag_size,
                                      calib_target.tag_spacing);

  // Detect AprilGrids
  // -- Get images
  std::vector<std::string> image_paths;
  if (list_dir(cam_path, image_paths) != 0) {
    FATAL("Failed to traverse dir [%s]!", cam_path.c_str());
  }
  std::sort(image_paths.begin(), image_paths.end());
  // -- Detect AprilGrids
  aprilgrids_t grids;
  for (auto &image_path : image_paths) {
    image_path = paths_join(cam_path, image_path);
    const auto ts = std::stoull(parse_fname(image_path));
    const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    const auto grid = detector.detect(ts, image);
    grid.save(grid_path + "/" + std::to_string(ts) + ".csv");
    grids.push_back(grid);
  }

  return grids;
}

static void show_results(const calib_mocap_data_t &data) {
  // Calibration metrics
  std::deque<pose_t> poses;
  const mat4_t T_WF = data.T_WF;
  const mat4_t T_C0M = data.T_MC0.inverse();
  for (size_t k = 0; k < data.grids.size(); k++) {
    const mat4_t T_MW = data.T_WM.at(k).inverse();
    const mat4_t T_C0F = T_C0M * T_MW * T_WF;
    poses.emplace_back(0, T_C0F);
  }

  // // Show results
  // std::vector<double> errs;
  // reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);
  //
  // printf("\n");
  // printf("Optimization results:\n");
  // printf("---------------------\n");
  // printf("nb_points: %ld\n", errs.size());
  // printf("reproj_error [px]: ");
  // printf("[rmse: %f", rmse(errs));
  // printf(" mean: %f", mean(errs));
  // printf(" median: %f]\n", median(errs));
  // printf("\n");
  // print_vector("cam.proj_params", data.cam0.proj_params());
  // print_vector("cam.dist_params", data.cam0.dist_params());
  // printf("\n");
  // print_matrix("T_WF", data.T_WF.tf());
  // print_matrix("T_WM", data.T_WM[0].tf());
  // print_matrix("T_MC0", data.T_MC0.tf());
  //
  // const auto r_MC = tf_trans(data.T_MC0.tf());
  // const auto q_MC = tf_quat(data.T_MC0.tf());
  // printf("r_MC: %f, %f, %f\n", r_MC(0), r_MC(1), r_MC(2));
  // printf("q_MC (x, y, z, w): %f, %f, %f, %f\n",
  //        q_MC.x(),
  //        q_MC.y(),
  //        q_MC.z(),
  //        q_MC.w());
}

calib_mocap_data_t::calib_mocap_data_t(const std::string &config_file) {
  // Load calib config file
  config_t config{config_file};
  parse(config, "settings.fix_intrinsics", fix_intrinsics);
  parse(config, "settings.fix_mocap_poses", fix_mocap_poses);
  parse(config, "settings.fix_fiducial_pose", fix_fiducial_pose);
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.imshow", imshow);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);
  parse(config, "cam0.proj_params", proj_params, true);
  parse(config, "cam0.dist_params", dist_params, true);

  // Setup paths
  results_fpath = paths_join(data_path, "calib_results.yaml");
  cam0_path = data_path + "/cam0/data";
  grid0_path = data_path + "/grid0/cam0/data";
  body0_csv_path = data_path + "/body0/data.csv";
  target0_csv_path = data_path + "/target0/data.csv";

  // Load calibration target
  if (calib_target.load(config_file, "calib_target") != 0) {
    FATAL("Failed to load calib target in [%s]!", config_file.c_str());
  }

  // Initialize camera parameters
  if (yaml_has_key(config, "cam0.proj_params") == false) {
    LOG_INFO("Camera parameters unknown!");
    LOG_INFO("Calibrating camera intrinsics!");
    const auto cam_grids =
        preprocess_calib_data(calib_target, cam0_path, grid0_path);
    calib_camera_t calib{calib_target};
    calib.add_camera_data(0, cam_grids);
    calib.add_camera(0, resolution.data(), proj_model, dist_model);
    calib.solve();
  }

  // Load dataset
  // -- April Grid
  aprilgrids_t grids_raw = load_aprilgrids(grid0_path);
  // -- Mocap marker pose
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_poses(body0_csv_path, body_timestamps, body_poses);
  // -- Synchronize aprilgrids and body poses
  mat4s_t marker_poses;
  lerp_body_poses(grids_raw, body_timestamps, body_poses, grids, marker_poses);
  // -- Fiducial target pose
  T_WF = load_pose(target0_csv_path);
  // -- Marker poses
  for (size_t k = 0; k < grids.size(); k++) {
    T_WM[grids[k].timestamp] = marker_poses[k];
  }
  // -- Mocap marker to camera transform
  mat4_t T_C0F;
  const mat4_t T_FC0 = T_C0F.inverse();
  const mat4_t T_MW = T_WM[grids[0].timestamp].inverse();
  T_MC0 = T_MW * T_WF * T_FC0;
}

camera_params_t calib_mocap_data_t::get_camera_params() const {
  return camera_params_t{0,
                         resolution.data(),
                         proj_model,
                         dist_model,
                         proj_params,
                         dist_params};
}

extrinsics_t calib_mocap_data_t::get_extrinsics() const {
  return extrinsics_t{T_MC0};
}

std::map<timestamp_t, pose_t> calib_mocap_data_t::get_marker_poses() const {
  std::map<timestamp_t, pose_t> marker_poses;
  for (const auto &[ts, marker_pose] : T_WM) {
    marker_poses[ts] = pose_t{0, marker_pose};
  }
  return marker_poses;
}

pose_t calib_mocap_data_t::get_fiducial_pose() const { return pose_t{0, T_WF}; }

int calib_mocap_solve(const calib_mocap_data_t &data) {
  assert(data.grids.size() > 0);
  assert(data.T_WM.size() > 0);
  assert(data.T_WM.size() == data.grids.size());

  // Setup optimization problem
  ceres::Problem::Options options;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(options);
  PoseLocalParameterization pose_plus;

  // Calibration parameters
  std::map<timestamp_t, pose_t> marker_poses = data.get_marker_poses();
  camera_params_t cam = data.get_camera_params();
  pose_t fiducial = data.get_fiducial_pose();
  extrinsics_t extrinsics = data.get_extrinsics();

  // Set ficuial and marker-cam pose parameterization
  problem.AddParameterBlock(fiducial.param.data(), 7);
  problem.AddParameterBlock(extrinsics.param.data(), 7);
  problem.SetParameterization(fiducial.param.data(), &pose_plus);
  problem.SetParameterization(extrinsics.param.data(), &pose_plus);

  // Fix camera parameters
  if (data.fix_intrinsics) {
    problem.SetParameterBlockConstant(cam.param.data());
  }

  // Fix fiducial pose - assumes camera intrincs and PnP is good
  if (data.fix_fiducial_pose) {
    problem.SetParameterBlockConstant(fiducial.param.data());
  }

  // Process all aprilgrid data
  const mat2_t covar = I(2);
  std::vector<ceres::ResidualBlockId> block_ids;
  for (const auto grid : data.grids) {
    const auto ts = grid.timestamp;
    auto &marker_pose = marker_poses[ts];

    // process_aprilgrid<CAMERA_TYPE>(i, covar, data, problem, block_ids);
    problem.AddParameterBlock(marker_pose.param.data(), 7);
    problem.SetParameterization(marker_pose.param.data(), &pose_plus);
    if (data.fix_mocap_poses) {
      problem.SetParameterBlockConstant(marker_pose.param.data());
    }
  }

  // Set solver options
  ceres::Solver::Options solver_options;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.max_num_iterations = 100;

  // Solve
  LOG_INFO("Calibrating mocap-marker to camera extrinsics ...");
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // // Reject outliers
  // LOG_INFO("Rejecting outliers...");
  // std::deque<pose_t> poses;
  // for (int i = 0; i < (int)data.grids.size(); i++) {
  //   const mat4_t T_WF = data.T_WF.tf();
  //   const mat4_t T_C0M = data.T_MC0.tf().inverse();
  //   const mat4_t T_MW = data.T_WM[i].tf().inverse();
  //   const mat4_t T_C0F = T_C0M * T_MW * T_WF;
  //   poses.emplace_back(0, 0, T_C0F);
  // }
  // std::vector<double> errs;
  // reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);
  //
  // const auto nb_res_before = problem.NumResidualBlocks();
  // const auto threshold = 4.0 * stddev(errs);
  // for (int i = 0; i < (int)errs.size(); i++) {
  //   if (errs[i] > threshold) {
  //     problem.RemoveResidualBlock(block_ids[i]);
  //   }
  // }
  // const auto nb_res_after = problem.NumResidualBlocks();
  // const auto res_diff = nb_res_before - nb_res_after;
  // LOG_INFO("Removed: %d residuals out of %d", res_diff, nb_res_before);
  //
  // // Second pass
  // LOG_INFO("Performing second pass ...");
  // ceres::Solve(options, &problem, &summary);
  // // std::cout << summary.FullReport() << std::endl;
  // std::cout << summary.BriefReport() << std::endl;
  // std::cout << std::endl;
  //
  // // Show results
  // show_results<CAMERA_TYPE>(data);
  // save_results<CAMERA_TYPE>(data, data.results_fpath);

  return 0;
}

int calib_mocap_solve(const std::string &config_file) {
  calib_mocap_data_t data{config_file};
  return calib_mocap_solve(data);
}

} //  namespace yac
