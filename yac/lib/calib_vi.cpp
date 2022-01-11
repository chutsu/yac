#include "calib_vi.hpp"

namespace yac {

// void calib_vi_init_poses(const calib_target_t &target,
//                          const mat4_t &T_FO,
//                          std::deque<mat4_t> &poses) {
//   // Tag width
//   const double tag_rows = target.tag_rows;
//   const double tag_cols = target.tag_cols;
//   const double tag_spacing = target.tag_spacing;
//   const double tag_size = target.tag_size;
//   const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
//   const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
//   const double calib_width = tag_cols * tag_size + spacing_x;
//   const double calib_height = tag_rows * tag_size + spacing_y;
//   const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};
//
//   // Create initial poses
//   const auto angle = 30.0;
//   // -- First position (left of calibration origin)
//   {
//     // const vec3_t r_OP{-calib_width / 2.0, 0.0, 0.0};
//     const vec3_t r_OP{0.0, 0.0, 0.0};
//     const vec3_t r_FJ = tf_point(T_FO, r_OP);
//     const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
//     const mat3_t C = euler321(deg2rad(vec3_t{0.0, 0.0, -angle}));
//     const mat4_t T = tf(C, zeros(3, 1));
//     poses.push_back(T_FC0 * T);
//   }
//   // -- Second position (at calibration origin)
//   {
//     // const vec3_t r_OP{calib_width / 2.0, 0.0, 0.0};
//     const vec3_t r_OP{0.0, 0.0, 0.0};
//     const vec3_t r_FJ = tf_point(T_FO, r_OP);
//     const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
//     const mat3_t C = euler321(deg2rad(vec3_t{0.0, 0.0, angle}));
//     const mat4_t T = tf(C, zeros(3, 1));
//     poses.push_back(T_FC0 * T);
//   }
// }

// APRILGRID BUFFER ////////////////////////////////////////////////////////////

std::map<int, std::deque<aprilgrid_t>> &aprilgrid_buffer_t::data() {
  return buf;
}

void aprilgrid_buffer_t::add(const int cam_idx, const aprilgrid_t &grid) {
  buf[cam_idx].push_back(grid);
}

bool aprilgrid_buffer_t::ready() const {
  timestamp_t ts = 0;
  if (nb_cams == 1 && buf.at(0).size() != 0) {
    return true;
  }

  for (int i = 0; i < nb_cams; i++) {
    if (buf.at(i).size() == 0) {
      return false;
    }

    if (ts == 0) {
      ts = buf.at(i).front().timestamp;
      continue;
    }

    if (ts != buf.at(i).front().timestamp) {
      return false;
    }
  }

  return true;
}

aprilgrids_t aprilgrid_buffer_t::pop_front() {
  aprilgrids_t results;

  for (auto &kv : buf) {
    auto &deque = kv.second;
    results.push_back(deque.front());
    deque.pop_front();
  }

  return results;
}

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

void calib_vi_t::add_imu(const imu_params_t &imu_params_, const double td) {
  imu_params = imu_params_;
  add_time_delay(td);
}

void calib_vi_t::add_camera(const int cam_idx,
                            const int resolution[2],
                            const std::string &proj_model,
                            const std::string &dist_model,
                            const vecx_t &proj_params,
                            const vecx_t &dist_params,
                            const bool fix) {
  auto cam = new camera_params_t(cam_idx,
                                 resolution,
                                 proj_model,
                                 dist_model,
                                 proj_params,
                                 dist_params);
  cam_params[cam_idx] = cam;
  grids_buf.nb_cams++; // Very important!

  if (fix) {
    problem->AddParameterBlock(cam->param.data(), cam->global_size);
    problem->SetParameterBlockConstant(cam->param.data());
    cam->fixed = true;
  }
}

void calib_vi_t::add_cam_extrinsics(const int cam_idx,
                                    const mat4_t &T_BCi,
                                    const bool fix) {
  auto cam_ext = new extrinsics_t(T_BCi);
  cam_exts[cam_idx] = cam_ext;
  problem->AddParameterBlock(cam_ext->param.data(), 7);
  problem->SetParameterization(cam_ext->param.data(), &pose_parameterization);

  if (fix) {
    problem->SetParameterBlockConstant(cam_ext->param.data());
    cam_ext->fixed = true;
  }
}

void calib_vi_t::add_sensor_pose(const timestamp_t ts, const mat4_t &T_WS) {
  sensor_poses.push_back(new pose_t{ts, T_WS});
  problem->AddParameterBlock(sensor_poses.back()->param.data(), 7);
  problem->SetParameterization(sensor_poses.back()->param.data(),
                               &pose_parameterization);
}

void calib_vi_t::add_speed_biases(const timestamp_t ts, const vec_t<9> &sb) {
  speed_biases.push_back(new sb_params_t(ts, sb));
  problem->AddParameterBlock(speed_biases.back()->param.data(), 9);
}

void calib_vi_t::add_time_delay(const double td) {
  time_delay = new time_delay_t{td};
  problem->AddParameterBlock(time_delay->param.data(), 1);
}

void calib_vi_t::add_fiducial_pose(const mat4_t &T_WF) {
  fiducial = new fiducial_t(T_WF);
  problem->AddParameterBlock(fiducial->param.data(), FIDUCIAL_PARAMS_SIZE);
}

vecx_t calib_vi_t::get_camera(const int cam_idx) {
  return cam_params[cam_idx]->param;
}

mat4_t calib_vi_t::get_cam_extrinsics(const int cam_idx) {
  return cam_exts[cam_idx]->tf();
}

mat4_t calib_vi_t::get_sensor_pose(const int pose_index) {
  return sensor_poses[pose_index]->tf();
}

mat4_t calib_vi_t::get_fiducial_pose() {
  return fiducial->estimate();
}

int calib_vi_t::nb_cams() {
  return cam_params.size();
}

void calib_vi_t::trim_imu_data(imu_data_t &imu_data, const timestamp_t t1) {
  // Makesure the trim timestamp is after the first imu measurement
  if (t1 < imu_data.timestamps.front()) {
    return;
  }

  // Trim IMU measurements
  imu_data_t trimmed_imu_data;
  for (size_t k = 0; k < imu_data.timestamps.size(); k++) {
    const timestamp_t ts = imu_data.timestamps[k];
    if (ts >= t1) {
      const vec3_t acc = imu_data.accel[k];
      const vec3_t gyr = imu_data.gyro[k];
      trimmed_imu_data.add(ts, acc, gyr);
    }
  }

  imu_data = trimmed_imu_data;
}

void calib_vi_t::update_prev_grids(const aprilgrids_t &grids) {
  // Keep track of old aprilgrids
  grids_prev.clear();
  for (int i = 0; i < nb_cams(); i++) {
    grids_prev[i] = grids[i];
  }
}

ImuError *calib_vi_t::add_imu_error(ceres::ResidualBlockId &error_id) {
  pose_t *pose_i = sensor_poses[sensor_poses.size() - 2];
  pose_t *pose_j = sensor_poses[sensor_poses.size() - 1];
  sb_params_t *sb_i = speed_biases[speed_biases.size() - 2];
  sb_params_t *sb_j = speed_biases[speed_biases.size() - 1];
  const auto t0 = pose_i->ts;
  const auto t1 = pose_j->ts;

  auto error = new ImuError(imu_buf, imu_params, t0, t1);
  error_id = problem->AddResidualBlock(error,
                                       NULL,
                                       pose_i->param.data(),
                                       sb_i->param.data(),
                                       pose_j->param.data(),
                                       sb_j->param.data());

  return error;
}

// void calib_vi_t::add_reproj_errors(
//     const int cam_idx,
//     const aprilgrid_t &grid_j,
//     std::map<int, std::vector<ceres::ResidualBlockId>> &error_ids,
//     std::map<int, std::vector<reproj_error_td_t<pinhole_radtan4_t> *>>
//         &errors) {
//   assert(cam_params.count(cam_idx) > 0);
//   assert(cam_exts.count(cam_idx) > 0);
//   assert(imu_exts != nullptr);
//   assert(sensor_poses.size() >= 1);
//
//   const aprilgrid_t &grid_i = grids_prev[cam_idx];
//   const auto ts_i = grid_i.timestamp;
//   const auto ts_j = grid_j.timestamp;
//   const auto &T_WF = fiducial;
//   const auto &T_WS_i = sensor_poses[sensor_poses.size() - 2];
//   const auto &T_BS = imu_exts;
//   const auto &T_BCi = cam_exts[cam_idx];
//   const auto &cam = cam_params[cam_idx];
//   const int *cam_res = cam->resolution;
//   const mat2_t covar = pow(sigma_vision, 2) * I(2);
//
//   std::vector<int> tag_ids;
//   std::vector<int> corner_indicies;
//   vec2s_t grid_i_keypoints;
//   vec2s_t grid_j_keypoints;
//   vec3s_t object_points;
//   aprilgrid_t::common_measurements(grid_i,
//                                    grid_j,
//                                    tag_ids,
//                                    corner_indicies,
//                                    grid_i_keypoints,
//                                    grid_j_keypoints,
//                                    object_points);
//
//   for (size_t i = 0; i < tag_ids.size(); i++) {
//     const int tag_id = tag_ids[i];
//     const int corner_idx = corner_indicies[i];
//     const vec2_t z_i = grid_i_keypoints[i];
//     const vec2_t z_j = grid_j_keypoints[i];
//     const vec3_t r_FFi = object_points[i];
//
//     auto error = new reproj_error_td_t<pinhole_radtan4_t>(ts_i,
//                                                           ts_j,
//                                                           cam_res,
//                                                           tag_id,
//                                                           corner_idx,
//                                                           r_FFi,
//                                                           z_i,
//                                                           z_j,
//                                                           fiducial->estimate(),
//                                                           covar);
//     auto error_id = problem->AddResidualBlock(error,
//                                               NULL,
//                                               T_WF->param.data(),
//                                               T_WS_i->param.data(),
//                                               T_BS->param.data(),
//                                               T_BCi->param.data(),
//                                               cam->param.data(),
//                                               time_delay->param.data());
//     error_ids[cam_idx].push_back(error_id);
//     errors[cam_idx].push_back(error);
//   }
// }

bool calib_vi_t::fiducial_detected(const aprilgrids_t &grids) {
  assert(grids.size() > 0);

  bool detected = false;
  for (int i = 0; i < nb_cams(); i++) {
    if (grids[i].detected) {
      detected = true;
    }
  }

  return detected;
}

mat4_t calib_vi_t::estimate_sensor_pose(const aprilgrids_t &grids) {
  assert(grids.size() > 0);
  assert(initialized);

  if (fiducial_detected(grids) == false) {
    FATAL("No fiducials detected!");
  }

  for (int i = 0; i < nb_cams(); i++) {
    // Skip if not detected
    if (grids[i].detected == false) {
      continue;
    }

    // Estimate relative pose
    mat4_t T_CiF = I(4);
    // if (estimate_relative_pose(grids[i], T_CiF) != 0) {
    //   FATAL("Failed to estimate relative pose!");
    // }

    // Infer current pose T_WS using T_C0F, T_SC0 and T_WF
    // const mat4_t T_FCi_k = T_CiF.inverse();
    // const mat4_t T_BCi = get_cam_extrinsics(i);
    // const mat4_t T_BS = get_imu_extrinsics();
    // const mat4_t T_CiS = T_BCi.inverse() * T_BS;
    // const mat4_t T_WF = get_fiducial_pose();
    // const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;
    // return T_WS_k;

    return T_CiF;
  }

  // Should never reach here
  FATAL("Implementation Error!");
  return zeros(4, 4);
}

void calib_vi_t::initialize(const timestamp_t &ts,
                            const aprilgrids_t &grids,
                            imu_data_t &imu_buf) {
  // Estimate relative pose - T_C0F
  assert(grids.size() > 0);
  assert(grids[0].detected);
  mat4_t T_C0F = I(4);
  // if (estimate_relative_pose(grids[0], T_C0F) != 0) {
  //   FATAL("Failed to estimate relative pose!");
  //   return;
  // }

  // Estimate initial IMU attitude
  mat3_t C_WS;
  imu_init_attitude(imu_buf.gyro, imu_buf.accel, C_WS, 1);

  // Sensor pose - T_WS
  mat4_t T_WS = tf(C_WS, zeros(3, 1));

  // Fiducial pose - T_WF
  const mat4_t T_BC0 = get_cam_extrinsics(0);
  // const mat4_t T_BS = get_imu_extrinsics();
  // const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
  // mat4_t T_WF = T_WS * T_SC0 * T_C0F;
  mat4_t T_WF = I(4);

  // Set fiducial target as origin (with r_WF (0, 0, 0))
  // and calculate the sensor pose offset relative to target origin
  const vec3_t offset = -1.0 * tf_trans(T_WF);
  const mat3_t C_WF = tf_rot(T_WF);
  const vec3_t r_WF{0.0, 0.0, 0.0};
  T_WF = tf(C_WF, r_WF);
  T_WS = tf(C_WS, offset);

  // Finish up
  add_sensor_pose(ts, T_WS);
  add_speed_biases(ts, zeros(9, 1));
  add_fiducial_pose(T_WF);

  // LOG_INFO("Initialize:");
  // print_matrix("T_WS", T_WS);
  // print_matrix("T_WF", T_WF);
  // print_matrix("T_BS", T_BS);
  // print_matrix("T_BC0", get_cam_extrinsics(0));
  // print_matrix("T_BC1", get_cam_extrinsics(1));

  initialized = true;
  trim_imu_data(imu_buf, ts); // Remove imu measurements up to ts
  update_prev_grids(grids);
}

void calib_vi_t::add_state(const timestamp_t &ts, const aprilgrids_t &grids) {
  // Estimate current pose
  const mat4_t T_WS_k = estimate_sensor_pose(grids);

  // // Propagate imu measurements to obtain speed and biases
  // const auto t0 = sensor_poses.back()->ts;
  // const auto t1 = ts;
  // mat4_t T_WS = sensor_poses.back()->tf();
  // vec_t<9> sb_k = speed_biases.back()->param;
  // ImuError::propagation(imu_buf, imu_params, T_WS, sb_k, t0, t1);

  // Infer velocity from two poses T_WS_k and T_WS_km1
  const mat4_t T_WS_km1 = tf(sensor_poses.back()->param);
  const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
  const vec3_t r_WS_k = tf_trans(T_WS_k);
  const vec3_t v_WS_k = r_WS_k - r_WS_km1;
  vec_t<9> sb_k;
  sb_k.setZero();
  sb_k << v_WS_k, zeros(3, 1), zeros(3, 1);

  // Add updated sensor pose T_WS_k and speed and biases sb_k Note: instead of
  // using the propagated sensor pose `T_WS`, we are using the provided
  // `T_WS_`, this is because in the scenario where we are using AprilGrid, as
  // a fiducial target, we actually have a better estimation of the sensor
  // pose, as supposed to the imu propagated sensor pose.
  add_sensor_pose(ts, T_WS_k);
  add_speed_biases(ts, sb_k);

  // Add inertial factors
  ceres::ResidualBlockId imu_error_id = nullptr;
  ImuError *imu_error = add_imu_error(imu_error_id);

  // Add vision factors
  std::map<int, std::vector<ceres::ResidualBlockId>> reproj_error_ids;
  // std::map<int, std::vector<reproj_error_td_t<pinhole_radtan4_t> *>>
  //     reproj_errors;
  // for (int i = 0; i < nb_cams(); i++) {
  //   add_reproj_errors(i, grids[i], reproj_error_ids, reproj_errors);
  // }

  // // Add view to sliding window
  // const auto &pose_i = sensor_poses[sensor_poses.size() - 2];
  // const auto &pose_j = sensor_poses[sensor_poses.size() - 1];
  // const auto &sb_i = speed_biases[speed_biases.size() - 2];
  // const auto &sb_j = speed_biases[speed_biases.size() - 1];
  // sliding_window.emplace_back(problem,
  //                             fiducial,
  //                             pose_i,
  //                             pose_j,
  //                             sb_i,
  //                             sb_j,
  //                             cam_exts,
  //                             cam_params,
  //                             imu_exts,
  //                             time_delay,
  //                             imu_error_id,
  //                             imu_error,
  //                             reproj_error_ids,
  //                             reproj_errors);
  //
  // trim_imu_data(imu_buf, ts);
  // update_prev_grids(grids);
  //
  // // Solve
  // if (enable_marg && sliding_window.size() >= max_window_size) {
  //   marginalize();
  //   ceres::Solver::Options options;
  //   options.minimizer_progress_to_stdout = true;
  //   options.max_num_iterations = 2;
  //   options.num_threads = 4;
  //   ceres::Solver::Summary summary;
  //   ceres::Solve(options, problem, &summary);
  //
  //   printf("nb_parameter_blocks: %d\n", problem->NumParameterBlocks());
  //   printf("nb_residual_blocks: %d\n", problem->NumResidualBlocks());
  //   std::cout << summary.BriefReport() << std::endl;
  //   std::cout << std::endl;
  // }
}

void calib_vi_t::add_measurement(const int cam_idx, const aprilgrid_t &grid) {
  grids_buf.add(cam_idx, grid);
  cam_grids[cam_idx].push_back(grid);
}

void calib_vi_t::add_measurement(const timestamp_t imu_ts,
                                 const vec3_t &a_m,
                                 const vec3_t &w_m) {
  imu_buf.add(imu_ts, a_m, w_m);

  if (grids_buf.ready()) {
    const auto grids = grids_buf.pop_front();

    // Initialize T_WS and T_WF
    if (initialized == false) {
      if (grids[0].detected && imu_buf.size() > 2) {
        const auto ts = imu_buf.timestamps.front();
        initialize(ts, grids, imu_buf);
      }
      return;
    }

    // Add new state
    timestamp_t grid_ts = 0;
    for (int i = 0; i < nb_cams(); i++) {
      if (grids[i].detected) {
        grid_ts = grids[i].timestamp;
        break;
      }
    }
    if (fiducial_detected(grids) && imu_ts >= grid_ts) {
      const auto ts = imu_buf.timestamps.back();
      add_state(ts, grids);
    }
  }
}

std::map<int, std::vector<double>> calib_vi_t::get_camera_errors() {
  std::map<int, std::vector<double>> cam_errs;

  for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    std::vector<double> errs;
    // for (auto &view : sliding_window) {
    //   view.calculate_reproj_errors(cam_idx, errs);
    // }
    cam_errs[cam_idx] = errs;
  }

  return cam_errs;
}

// void solve(bool verbose = true) {
//   // Optimize problem - first pass
//   {
//     // LOG_INFO("Optimize problem - first pass");
//     ceres::Solver::Options options;
//     options.minimizer_progress_to_stdout = verbose;
//     options.max_num_iterations = batch_max_iter;
//
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, problem, &summary);
//     std::cout << summary.BriefReport() << std::endl;
//     if (verbose) {
//       std::cout << summary.BriefReport() << std::endl;
//       show_results();
//     }
//   }
//
//   // Filter outliers
//   if (enable_outlier_rejection) {
//     // LOG_INFO("Filter outliers");
//     int nb_outliers = 0;
//     int nb_inliers = 0;
//     for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
//       // Obtain all reprojection errors from all views
//       std::vector<double> errs;
//       for (auto &view : sliding_window) {
//         view.calculate_reproj_errors(cam_idx, errs);
//       }
//
//       // Filter outliers from last view
//       const double threshold = 3.0 * stddev(errs);
//       for (auto &view : sliding_window) {
//         nb_outliers += view.filter(cam_idx, threshold);
//         nb_inliers += view.reproj_errors.size();
//       }
//     }
//     // LOG_INFO("Removed %d outliers!", nb_outliers);
//     // printf("\n");
//
//     // Optimize problem - second pass
//     {
//       // LOG_INFO("Optimize problem - second pass");
//       // Solver options
//       ceres::Solver::Options options;
//       options.minimizer_progress_to_stdout = true;
//       options.max_num_iterations = batch_max_iter;
//
//       // Solve
//       ceres::Solver::Summary summary;
//       ceres::Solve(options, problem, &summary);
//       if (verbose) {
//         std::cout << summary.FullReport() << std::endl;
//         show_results();
//       }
//     }
//   }
// }
//
// void show_results() {
//   // Show results
//   std::map<int, std::vector<double>> cam_errs = get_camera_errors();
//
//   printf("Optimization results:\n");
//   printf("---------------------\n");
//
//   // Stats
//   printf("stats:\n");
//   for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
//     printf("cam%d reprojection error [px]: ", cam_idx);
//     printf("[rmse: %f,", rmse(cam_errs[cam_idx]));
//     printf(" mean: %f,", mean(cam_errs[cam_idx]));
//     printf(" median: %f]\n", median(cam_errs[cam_idx]));
//   }
//   printf("\n");
//
//   // Cameras
//   for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
//     printf("cam%d:\n", cam_idx);
//     print_vector("proj_params", cam_params[cam_idx]->proj_params());
//     print_vector("dist_params", cam_params[cam_idx]->dist_params());
//     printf("\n");
//   }
//
//   // Time-delay
//   if (time_delay) {
//     print_vector("time delay (ts_cam = ts_imu + td):", time_delay->param);
//     printf("\n");
//   }
//
//   // Imu extrinsics
//   print_matrix("T_BS", get_imu_extrinsics());
//   print_matrix("T_SB", get_imu_extrinsics().inverse());
//
//   // Camera Extrinsics
//   for (int cam_idx = 1; cam_idx < nb_cams(); cam_idx++) {
//     const auto key = "T_C0C" + std::to_string(cam_idx);
//     const mat4_t T_C0Ci = get_cam_extrinsics(cam_idx);
//     print_matrix(key, T_C0Ci);
//
//     {
//       const auto key = "T_C" + std::to_string(cam_idx) + "C0";
//       const mat4_t T_CiC0 = T_C0Ci.inverse();
//       print_matrix(key, T_CiC0);
//     }
//   }
// }
//
// int calib_vi_t::recover_calib_covar(matx_t &calib_covar) {
//   // Recover calibration covariance
//   auto T_BS = imu_exts->param.data();
//   auto td = time_delay->param.data();
//
//   // -- Setup covariance blocks to estimate
//   std::vector<std::pair<const double *, const double *>> covar_blocks;
//   covar_blocks.push_back({T_BS, T_BS});
//   covar_blocks.push_back({td, td});
//
//   // -- Estimate covariance
//   ::ceres::Covariance::Options options;
//   ::ceres::Covariance covar_est(options);
//   if (covar_est.Compute(covar_blocks, problem) == false) {
//     LOG_ERROR("Failed to estimate covariance!");
//     LOG_ERROR("Maybe Hessian is not full rank?");
//     return -1;
//   }
//
//   // -- Extract covariances sub-blocks
//   Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_BS_covar;
//   Eigen::Matrix<double, 1, 1, Eigen::RowMajor> td_covar;
//   covar_est.GetCovarianceBlockInTangentSpace(T_BS, T_BS, T_BS_covar.data());
//   covar_est.GetCovarianceBlock(td, td, td_covar.data());
//
//   // -- Form covariance matrix block
//   calib_covar = zeros(7, 7);
//   calib_covar.block(0, 0, 6, 6) = T_BS_covar;
//   calib_covar.block(6, 6, 1, 1) = td_covar;
//
//   // -- Check if calib_covar is full-rank?
//   if (rank(calib_covar) != calib_covar.rows()) {
//     LOG_ERROR("calib_covar is not full rank!");
//     return -1;
//   }
//
//   return 0;
// }

int calib_vi_t::save_results(const std::string &save_path) {
  LOG_INFO("Saved results to [%s]", save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Calibration metrics
  std::map<int, std::vector<double>> cam_errs = get_camera_errors();
  fprintf(outfile, "calib_metrics:\n");
  for (const auto &kv : cam_errs) {
    const auto cam_idx = kv.first;
    const auto cam_str = "cam" + std::to_string(cam_idx);
    const auto cam_errs = kv.second;
    fprintf(outfile, "  %s: ", cam_str.c_str());
    fprintf(outfile, "[");
    fprintf(outfile, "%f, ", rmse(cam_errs));
    fprintf(outfile, "%f, ", mean(cam_errs));
    fprintf(outfile, "%f, ", median(cam_errs));
    fprintf(outfile, "%f", stddev(cam_errs));
    fprintf(outfile, "]  # rmse, mean, median, stddev\n");
  }
  fprintf(outfile, "\n");
  fprintf(outfile, "\n");

  // Camera parameters
  for (int i = 0; i < nb_cams(); i++) {
    const auto cam = cam_params[i];
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const std::string proj_params = vec2str(cam->proj_params(), 4);
    const std::string dist_params = vec2str(cam->dist_params(), 4);

    fprintf(outfile, "cam%d:\n", i);
    fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }
  fprintf(outfile, "\n");

  // IMU parameters
  vec3s_t bias_acc;
  vec3s_t bias_gyr;
  for (const sb_params_t *sb : speed_biases) {
    bias_gyr.push_back(sb->param.segment<3>(3));
    bias_acc.push_back(sb->param.segment<3>(6));
  }
  const vec3_t mu_ba = mean(bias_acc);
  const vec3_t mu_bg = mean(bias_gyr);
  fprintf(outfile, "imu0:\n");
  fprintf(outfile, "  rate: %f\n", imu_params.rate);
  fprintf(outfile, "  sigma_a_c: %f\n", imu_params.sigma_a_c);
  fprintf(outfile, "  sigma_g_c: %f\n", imu_params.sigma_g_c);
  fprintf(outfile, "  sigma_aw_c: %f\n", imu_params.sigma_aw_c);
  fprintf(outfile, "  sigma_gw_c: %f\n", imu_params.sigma_gw_c);
  fprintf(outfile, "  sigma_ba: %f\n", imu_params.sigma_ba);
  fprintf(outfile, "  sigma_bg: %f\n", imu_params.sigma_bg);
  fprintf(outfile, "  bg: [%f, %f, %f]\n", mu_bg(0), mu_bg(1), mu_bg(2));
  fprintf(outfile, "  ba: [%f, %f, %f]\n", mu_ba(0), mu_ba(1), mu_ba(2));
  fprintf(outfile, "  g: %f\n", imu_params.g);
  fprintf(outfile, "\n");
  fprintf(outfile, "\n");

  // Sensor-Camera extrinsics
  for (int i = 0; i < nb_cams(); i++) {
    // const mat4_t T_BS = get_imu_extrinsics();
    const mat4_t T_BS = I(4);
    const mat4_t T_SB = T_BS.inverse();
    const mat4_t T_BCi = get_cam_extrinsics(i);
    const mat4_t T_SCi = T_SB * T_BCi;

    fprintf(outfile, "T_imu0_cam%d:\n", i);
    fprintf(outfile, "  rows: 4\n");
    fprintf(outfile, "  cols: 4\n");
    fprintf(outfile, "  data: [\n");
    fprintf(outfile, "%s\n", mat2str(T_SCi, "    ").c_str());
    fprintf(outfile, "  ]");
    fprintf(outfile, "\n");
    fprintf(outfile, "\n");
  }
  fprintf(outfile, "\n");

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_cam_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cams() >= 2) {
    for (int i = 1; i < nb_cams(); i++) {
      const mat4_t T_BCi = get_cam_extrinsics(i);
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      const mat4_t T_CiC0 = T_C0Ci.inverse();

      fprintf(outfile, "T_cam0_cam%d:\n", i);
      fprintf(outfile, "  rows: 4\n");
      fprintf(outfile, "  cols: 4\n");
      fprintf(outfile, "  data: [\n");
      fprintf(outfile, "%s\n", mat2str(T_C0Ci, "    ").c_str());
      fprintf(outfile, "  ]");
      fprintf(outfile, "\n");
      fprintf(outfile, "\n");

      fprintf(outfile, "T_cam%d_cam0:\n", i);
      fprintf(outfile, "  rows: 4\n");
      fprintf(outfile, "  cols: 4\n");
      fprintf(outfile, "  data: [\n");
      fprintf(outfile, "%s\n", mat2str(T_CiC0, "    ").c_str());
      fprintf(outfile, "  ]");
      fprintf(outfile, "\n");
      fprintf(outfile, "\n");
    }
  }

  // Finsh up
  fclose(outfile);

  return 0;
}

void calib_vi_t::save_poses(const std::string &save_path,
                            const std::vector<pose_t *> &poses) {
  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#ts,rx,ry,rz,qw,qx,qy,qz\n");
  for (const auto &pose : poses) {
    const auto ts = pose->ts;
    const vec3_t r = pose->trans();
    const quat_t q = pose->rot();
    fprintf(csv, "%ld,", ts);
    fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
    fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
  }
  fclose(csv);
}

void calib_vi_t::save_speed_biases(
    const std::string &save_path,
    const std::vector<sb_params_t *> &speed_biases) {
  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#ts,");
  fprintf(csv, "vx,vy,vz,");
  fprintf(csv, "ba_x,ba_y,ba_z,");
  fprintf(csv, "bg_x,bg_y,bg_z,\n");
  for (const auto &sb : speed_biases) {
    const auto ts = sb->ts;
    const vec3_t v = sb->param.segment<3>(0);
    const vec3_t ba = sb->param.segment<3>(3);
    const vec3_t bg = sb->param.segment<3>(6);
    fprintf(csv, "%ld,", ts);
    fprintf(csv, "%f,%f,%f,", v(0), v(1), v(2));
    fprintf(csv, "%f,%f,%f,", ba(0), ba(1), ba(2));
    fprintf(csv, "%f,%f,%f\n", bg(0), bg(1), bg(2));
  }
  fclose(csv);
}

void calib_vi_t::save_cameras(
    const std::string &save_path,
    const std::map<int, camera_params_t *> &cam_params) {
  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#cam_idx,");
  fprintf(csv, "resolution,");
  fprintf(csv, "proj_model,");
  fprintf(csv, "dist_model,");
  fprintf(csv, "proj_params,");
  fprintf(csv, "dist_params\n");
  for (const auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = kv.second;
    const vecx_t proj = cam->proj_params();
    const vecx_t dist = cam->dist_params();
    fprintf(csv, "%d,", cam_idx);
    fprintf(csv, "%d,%d,", cam->resolution[0], cam->resolution[1]);
    fprintf(csv, "%s,%s,", cam->proj_model.c_str(), cam->dist_model.c_str());
    fprintf(csv, "%f,%f,%f,%f,", proj(0), proj(1), proj(2), proj(3));
    fprintf(csv, "%f,%f,%f,%f\n", dist(0), dist(1), dist(2), dist(3));
  }
  fclose(csv);
}

void calib_vi_t::save_cam_extrinsics(const std::string &save_path) {
  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#cam_idx,cam_idx,rx,ry,rz,qw,qx,qy,qz\n");

  const mat4_t T_BC0 = get_cam_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();

  for (int i = 0; i < nb_cams(); i++) {
    const mat4_t T_BCi = get_cam_extrinsics(i);
    const mat4_t T_C0Ci = T_C0B * T_BCi;
    const vec3_t r = tf_trans(T_C0Ci);
    const quat_t q = tf_quat(T_C0Ci);
    fprintf(csv, "0,");     // cam_idx
    fprintf(csv, "%d,", i); // cam_idx
    fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
    fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
  }
  fclose(csv);
}

// void calib_vi_t::save_imu_extrinsics(const std::string &save_path) {
//   FILE *csv = fopen(save_path.c_str(), "w");
//   fprintf(csv, "#imu_idx,cam_idx,rx,ry,rz,qw,qx,qy,qz\n");
//
//   const mat4_t T_BS = get_imu_extrinsics();
//   const mat4_t T_SB = T_BS.inverse();
//
//   for (int i = 0; i < nb_cams(); i++) {
//     const mat4_t T_BCi = get_cam_extrinsics(i);
//     const mat4_t T_SCi = T_SB * T_BCi;
//
//     const vec3_t r = tf_trans(T_SCi);
//     const quat_t q = tf_quat(T_SCi);
//     fprintf(csv, "0,");     // imu_idx
//     fprintf(csv, "%d,", i); // cam_idx
//     fprintf(csv, "%f,%f,%f,", r(0), r(1), r(2));
//     fprintf(csv, "%f,%f,%f,%f\n", q.w(), q.x(), q.y(), q.z());
//   }
//   fclose(csv);
// }

void calib_vi_t::save() {
  save_results("/tmp/calib-stereo_imu.yaml");
  save_poses("/tmp/sensor_poses.csv", sensor_poses);
  save_speed_biases("/tmp/sensor_speed_biases.csv", speed_biases);
  save_cameras("/tmp/cameras.csv", cam_params);
  save_cam_extrinsics("/tmp/cam_extrinsics.csv");
  // save_imu_extrinsics("/tmp/imu_extrinsics.csv");
  save_pose("/tmp/fiducial_pose.csv", get_fiducial_pose());
}

} // namespace yac
