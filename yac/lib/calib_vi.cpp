#include "calib_vi.hpp"

namespace yac {

// VISUAL INERTIAL CALIBRATION VIEW ////////////////////////////////////////////

calib_vi_view_t::calib_vi_view_t(const timestamp_t ts_,
                                 ceres::Problem *problem_,
                                 const CamIdx2Grids &grids_,
                                 const mat4_t &T_WS_,
                                 const vec_t<9> &sb_,
                                 CamIdx2Geometry &cam_geoms_,
                                 CamIdx2Parameters &cam_params_,
                                 CamIdx2Extrinsics &cam_exts_,
                                 extrinsics_t *imu_exts_,
                                 fiducial_t *fiducial_,
                                 PoseLocalParameterization *pose_plus)
    : ts{ts_}, grids{grids_}, pose{ts_, T_WS_}, sb{ts_, sb_},
      cam_geoms{cam_geoms_}, cam_params{cam_params_}, cam_exts{cam_exts_},
      imu_exts{imu_exts_}, fiducial{fiducial_}, problem{problem_} {

  // Add pose to problem
  problem->AddParameterBlock(pose.param.data(), 7);
  problem->SetParameterization(pose.param.data(), pose_plus);

  // Add speed and biases to problem
  problem->AddParameterBlock(sb.param.data(), 9);

  // Add fiducial errors
  const mat4_t T_WF = fiducial->estimate();
  const mat2_t covar = I(2);

  for (const auto &[cam_idx, grid] : grids) {
    // Get AprilGrid measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t pts;
    grid.get_measurements(tag_ids, corner_idxs, kps, pts);

    // Add residuals to problem
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_idxs[i];
      const vec2_t z = kps[i];
      const vec3_t r_FFi = pts[i];

      // Form residual
      auto error = std::make_unique<fiducial_error_t>(ts,
                                                      cam_geoms[cam_idx],
                                                      cam_params[cam_idx],
                                                      cam_exts[cam_idx],
                                                      imu_exts,
                                                      fiducial,
                                                      &pose,
                                                      tag_id,
                                                      corner_idx,
                                                      r_FFi,
                                                      z,
                                                      covar);
      fiducial_errors[cam_idx].emplace_back(std::move(error));

      // Add to problem
      auto error_id =
          problem->AddResidualBlock(fiducial_errors[cam_idx].back().get(),
                                    NULL,
                                    fiducial->param.data(),
                                    pose.param.data(),
                                    imu_exts->param.data(),
                                    cam_exts[cam_idx]->param.data(),
                                    cam_params[cam_idx]->param.data());
      fiducial_error_ids[cam_idx].push_back(error_id);
    }
  }
}

std::vector<int> calib_vi_view_t::get_cam_indices() const {
  std::vector<int> cam_idxs;
  for (const auto &[cam_idx, params] : cam_params) {
    UNUSED(params);
    cam_idxs.push_back(cam_idx);
  }

  return cam_idxs;
}

std::vector<real_t>
calib_vi_view_t::get_reproj_errors(const int cam_idx) const {
  std::vector<real_t> cam_errors;

  if (fiducial_errors.count(cam_idx) == 0) {
    return cam_errors;
  }

  for (const auto &error : fiducial_errors.at(cam_idx)) {
    real_t e;
    if (error->get_reproj_error(e) == 0) {
      cam_errors.push_back(e);
    }
  }

  return cam_errors;
}

std::map<int, std::vector<real_t>> calib_vi_view_t::get_reproj_errors() const {
  std::map<int, std::vector<real_t>> cam_errors;

  for (const auto &[cam_idx, errors] : fiducial_errors) {
    for (const auto &error : errors) {
      real_t e;
      if (error->get_reproj_error(e) == 0) {
        cam_errors[cam_idx].push_back(e);
      }
    }
  }

  return cam_errors;
}

int calib_vi_view_t::filter_view(const real_t outlier_threshold) {
  // Get reprojection errors
  const auto reproj_errors = get_reproj_errors();
  std::vector<real_t> reproj_errors_all;
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    extend(reproj_errors_all, cam_errors);
  }

  // Calculate threshold
  const auto error_stddev = stddev(reproj_errors_all);
  const auto threshold = outlier_threshold * error_stddev;
  // const auto threshold = outlier_threshold;

  int nb_inliers = 0;
  int nb_outliers = 0;
  for (const auto cam_idx : get_cam_indices()) {
    auto &cam_errors = fiducial_errors[cam_idx];
    auto &cam_error_ids = fiducial_error_ids[cam_idx];
    auto err_it = cam_errors.begin();
    auto id_it = cam_error_ids.begin();

    while (err_it != cam_errors.end()) {
      auto &fiducial_error = *err_it;
      auto &fiducial_id = *id_it;

      real_t err = 0.0;
      if (fiducial_error->get_reproj_error(err) == 0 && err > threshold) {
        problem->RemoveResidualBlock(fiducial_id);
        err_it = cam_errors.erase(err_it);
        id_it = cam_error_ids.erase(id_it);
        nb_outliers++;
      } else {
        ++err_it;
        ++id_it;
        nb_inliers++;
      }
    }
  }

  // printf("view_ts: %ld\n", ts);
  // printf("error_stddev: %f\n", error_stddev);
  // printf("threshold: %f\n", threshold);
  // printf("nb_inliers: %d\n", nb_inliers);
  // printf("nb_outliers: %d\n", nb_outliers);
  // printf("\n");

  return nb_outliers;
}

void calib_vi_view_t::form_imu_error(const imu_params_t &imu_params,
                                     const imu_data_t &imu_buf,
                                     pose_t *pose_j,
                                     sb_params_t *sb_j) {
  imu_error = std::make_unique<imu_error_t>(imu_params,
                                            imu_buf,
                                            &pose,
                                            &sb,
                                            pose_j,
                                            sb_j);
  imu_error_id = problem->AddResidualBlock(imu_error.get(),
                                           NULL,
                                           pose.param.data(),
                                           sb.param.data(),
                                           pose_j->param.data(),
                                           sb_j->param.data());
}

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

calib_vi_t::calib_vi_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem = new ceres::Problem{prob_options};
}

calib_vi_t::~calib_vi_t() {
  for (auto &[cam_idx, cam] : cam_params) {
    UNUSED(cam_idx);
    if (cam) {
      delete cam;
    }
  }

  for (auto &[cam_idx, exts] : cam_exts) {
    UNUSED(cam_idx);
    if (exts) {
      delete exts;
    }
  }

  if (imu_exts) {
    delete imu_exts;
  }

  if (fiducial) {
    delete fiducial;
  }

  if (time_delay) {
    delete time_delay;
  }

  if (problem) {
    delete problem;
  }
}

void calib_vi_t::add_imu(const imu_params_t &imu_params_,
                         const mat4_t &T_BS,
                         const double td,
                         const bool fix_extrinsics,
                         const bool fix_time_delay) {
  // Imu parameters
  imu_params = imu_params_;

  // Imu extrinsics
  imu_exts = new extrinsics_t{T_BS};
  problem->AddParameterBlock(imu_exts->param.data(), 7);
  problem->SetParameterization(imu_exts->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(imu_exts->param.data());
    imu_exts->fixed = true;
  }

  // Imu time delay
  time_delay = new time_delay_t{td};
  problem->AddParameterBlock(time_delay->param.data(), 1);
  if (fix_time_delay) {
    problem->SetParameterBlockConstant(time_delay->param.data());
    time_delay->fixed = true;
  }
}

void calib_vi_t::add_camera(const int cam_idx,
                            const int resolution[2],
                            const std::string &proj_model,
                            const std::string &dist_model,
                            const vecx_t &proj_params,
                            const vecx_t &dist_params,
                            const mat4_t &T_BCi,
                            const bool fix_params,
                            const bool fix_extrinsics) {
  // Camera parameters
  cam_params[cam_idx] = new camera_params_t{cam_idx,
                                            resolution,
                                            proj_model,
                                            dist_model,
                                            proj_params,
                                            dist_params};
  if (fix_params) {
    auto data_ptr = cam_params[cam_idx]->param.data();
    auto block_size = cam_params[cam_idx]->global_size;
    cam_params[cam_idx]->fixed = true;
    problem->AddParameterBlock(data_ptr, block_size);
    problem->SetParameterBlockConstant(data_ptr);
  }

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = &pinhole_radtan4;
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = &pinhole_equi4;
  } else {
    FATAL("Invalid [%s-%s] camera model!",
          proj_model.c_str(),
          dist_model.c_str());
  }

  // Camera extrinsics
  cam_exts[cam_idx] = new extrinsics_t{T_BCi};
  problem->AddParameterBlock(cam_exts[cam_idx]->param.data(), 7);
  problem->SetParameterization(cam_exts[cam_idx]->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(cam_exts[cam_idx]->param.data());
    cam_exts[cam_idx]->fixed = true;
  }
}

int calib_vi_t::nb_cams() const { return cam_params.size(); }

int calib_vi_t::nb_views() const { return calib_views.size(); }

veci2_t calib_vi_t::get_cam_resolution(const int cam_idx) const {
  auto cam_res = cam_params.at(cam_idx)->resolution;
  return veci2_t{cam_res[0], cam_res[1]};
}

vecx_t calib_vi_t::get_cam_params(const int cam_idx) const {
  return cam_params.at(cam_idx)->param;
}

mat4_t calib_vi_t::get_cam_extrinsics(const int cam_idx) const {
  return cam_exts.at(cam_idx)->tf();
}

mat4_t calib_vi_t::get_imu_extrinsics() const { return imu_exts->tf(); }

mat4_t calib_vi_t::get_fiducial_pose() const { return fiducial->estimate(); }

std::map<int, std::vector<real_t>> calib_vi_t::get_reproj_errors() const {
  std::map<int, std::vector<real_t>> errors;
  for (const auto &view : calib_views) {
    for (const auto cam_idx : view.get_cam_indices()) {
      extend(errors[cam_idx], view.get_reproj_errors(cam_idx));
    }
  }
  return errors;
}

mat4_t calib_vi_t::estimate_sensor_pose(const CamIdx2Grids &grids) {
  assert(grids.size() > 0);
  assert(initialized);

  bool detected = false;
  for (const auto &[cam_idx, grid] : grids) {
    UNUSED(cam_idx);
    if (grid.detected) {
      detected = true;
    }
  }
  if (detected == false) {
    FATAL("No fiducials detected!");
  }

  for (const auto &[cam_idx, grid] : grids) {
    // Skip if not detected
    if (grid.detected == false) {
      continue;
    }

    // Estimate relative pose
    const camera_geometry_t *cam = cam_geoms.at(cam_idx);
    const auto cam_res = get_cam_resolution(cam_idx).data();
    const vecx_t cam_params = get_cam_params(cam_idx);
    mat4_t T_CiF = I(4);
    if (grid.estimate(cam, cam_res, cam_params, T_CiF) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Infer current pose T_WS using T_CiF, T_BCi and T_WF
    const mat4_t T_FCi_k = T_CiF.inverse();
    const mat4_t T_BCi = get_cam_extrinsics(cam_idx);
    const mat4_t T_BS = get_imu_extrinsics();
    const mat4_t T_CiS = T_BCi.inverse() * T_BS;
    const mat4_t T_WF = get_fiducial_pose();
    const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;
    return T_WS_k;
  }

  // Should never reach here
  FATAL("Implementation Error!");
  return zeros(4, 4);
}

void calib_vi_t::initialize(const CamIdx2Grids &grids, imu_data_t &imu_buf) {
  assert(grids.size() > 0);
  assert(grids.at(0).detected);

  // Estimate relative pose - T_C0F
  const camera_geometry_t *cam = cam_geoms.at(0);
  const int *cam_res = cam_params.at(0)->resolution;
  const vecx_t params = get_cam_params(0);
  mat4_t T_C0F;
  if (grids.at(0).estimate(cam, cam_res, params, T_C0F) != 0) {
    FATAL("Failed to estimate relative pose!");
    return;
  }

  // Estimate initial IMU attitude
  mat3_t C_WS = imu_buf.initial_attitude();

  // Sensor pose - T_WS
  mat4_t T_WS = tf(C_WS, zeros(3, 1));

  // Fiducial pose - T_WF
  const mat4_t T_BC0 = get_cam_extrinsics(0);
  const mat4_t T_BS = get_imu_extrinsics();
  const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
  mat4_t T_WF = T_WS * T_SC0 * T_C0F;

  // Set:
  // 1. Fiducial target as origin (with r_WF (0, 0, 0))
  // 2. Calculate the sensor pose offset relative to target origin
  const vec3_t offset = -1.0 * tf_trans(T_WF);
  const mat3_t C_WF = tf_rot(T_WF);
  const vec3_t r_WF{0.0, 0.0, 0.0};
  T_WF = tf(C_WF, r_WF);
  T_WS = tf(C_WS, offset);

  LOG_INFO("Initialize:");
  print_matrix("T_WS", T_WS);
  print_matrix("T_WF", T_WF);
  print_matrix("T_BS", T_BS);
  print_matrix("T_BC0", get_cam_extrinsics(0));
  print_matrix("T_BC1", get_cam_extrinsics(1));

  // Set fiducial
  fiducial = new fiducial_t{T_WF};
  problem->AddParameterBlock(fiducial->param.data(), FIDUCIAL_PARAMS_SIZE);
  if (fiducial->param.size() == 7) {
    problem->SetParameterization(fiducial->param.data(), &pose_plus);
  }

  // First calibration view
  const timestamp_t ts = grids.at(0).timestamp;
  const vec_t<9> sb = zeros(9, 1);
  calib_views.emplace_back(ts,
                           problem,
                           grids,
                           T_WS,
                           sb,
                           cam_geoms,
                           cam_params,
                           cam_exts,
                           imu_exts,
                           fiducial,
                           &pose_plus);

  initialized = true;
  prev_grids = grids;
}

void calib_vi_t::add_view(const CamIdx2Grids &grids) {
  // Pre-check
  if (initialized == false) {
    return;
  }

  // Check aprilgrids, make sure there is atleast 1 detected grid
  bool grid_detected = false;
  timestamp_t grid_ts = 0;
  for (int i = 0; i < nb_cams(); i++) {
    if (grids.at(i).detected) {
      grid_ts = grids.at(i).timestamp;
      grid_detected = true;
      break;
    }
  }
  if (grid_detected == false) {
    return;
  }

  // Estimate current pose
  const mat4_t T_WS_k = estimate_sensor_pose(grids);

  // Get previous calibration view
  auto &view_km1 = calib_views.back();
  const timestamp_t ts_km1 = view_km1.ts;

  // Infer velocity from two poses T_WS_k and T_WS_km1
  const timestamp_t ts_k = grid_ts;
  const real_t dt = ((ts_k - ts_km1) * 1e-9);
  const mat4_t T_WS_km1 = view_km1.pose.tf();
  const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
  const vec3_t r_WS_k = tf_trans(T_WS_k);
  const vec3_t v_WS_k = (r_WS_k - r_WS_km1) / dt;

  // Form current speed and biases vector sb_k
  const vec_t<9> sb_km1 = view_km1.sb.param;
  const vec3_t ba_km1 = sb_km1.segment<3>(3);
  const vec3_t bg_km1 = sb_km1.segment<3>(6);
  vec_t<9> sb_k;
  sb_k << v_WS_k, ba_km1, bg_km1;

  // Form new calibration view
  // Note: instead of using the propagated sensor pose T_WS_k using imu
  // measurements, we are estimating `T_WS_k` via vision. This is because
  // vision estimate is better in this case.
  calib_views.emplace_back(ts_k,
                           problem,
                           grids,
                           T_WS_k,
                           sb_k,
                           cam_geoms,
                           cam_params,
                           cam_exts,
                           imu_exts,
                           fiducial,
                           &pose_plus);

  // Form imu factor between view km1 and k
  auto &view_k = calib_views.back();
  view_km1.form_imu_error(imu_params, imu_buf, &view_k.pose, &view_k.sb);

  prev_grids = grids;
}

void calib_vi_t::add_measurement(const int cam_idx, const aprilgrid_t &grid) {
  grid_buf[cam_idx].push_back(grid);
}

void calib_vi_t::add_measurement(const timestamp_t imu_ts,
                                 const vec3_t &a_m,
                                 const vec3_t &w_m) {
  // Add imu measuremrent
  imu_buf.add(imu_ts, a_m, w_m);

  // Check if we have enough aprilgrids
  if (static_cast<int>(grid_buf.size()) != nb_cams()) {
    return;
  }

  // Get camera grids
  CamIdx2Grids grids;
  for (auto &[cam_idx, cam_grids] : grid_buf) {
    grids[cam_idx] = cam_grids.front();
  }
  grid_buf.clear();

  // Initialize T_WS and T_WF
  if (initialized == false && grids.at(0).detected && imu_buf.size() > 2) {
    initialize(grids, imu_buf);
    return;
  }

  // Add new view
  add_view(grids);
}

void calib_vi_t::solve() {
  // Optimize problem - first pass
  {
    // LOG_INFO("Optimize problem - first pass");
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = verbose;
    options.max_num_iterations = batch_max_iter;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    if (verbose) {
      std::cout << summary.BriefReport() << std::endl << std::endl;
      show_results();
    }
  }

  // Filter outliers
  if (enable_outlier_rejection) {
    LOG_INFO("Filter outliers");
    int nb_outliers = 0;
    for (auto &view : calib_views) {
      nb_outliers += view.filter_view(outlier_threshold);
    }
    LOG_INFO("Removed %d outliers!", nb_outliers);

    // Optimize problem - second pass
    {
      LOG_INFO("Optimize problem - second pass\n");

      // Solve
      ceres::Solver::Options options;
      options.minimizer_progress_to_stdout = true;
      options.max_num_iterations = batch_max_iter;

      ceres::Solver::Summary summary;
      ceres::Solve(options, problem, &summary);
      if (verbose) {
        std::cout << summary.FullReport() << std::endl << std::endl;
        show_results();
      }
    }
  }
}

void calib_vi_t::show_results() {
  // Show results
  printf("Optimization results:\n");
  printf("---------------------\n");

  // Stats - Reprojection Errors
  const auto reproj_errors = get_reproj_errors();
  std::vector<real_t> reproj_errors_all;
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    extend(reproj_errors_all, cam_errors);
  }
  printf("Total reprojection error:\n");
  printf("  rmse:   %.4f # px\n", rmse(reproj_errors_all));
  printf("  mean:   %.4f # px\n", mean(reproj_errors_all));
  printf("  median: %.4f # px\n", median(reproj_errors_all));
  printf("  stddev: %.4f # px\n", stddev(reproj_errors_all));
  printf("\n");

  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    printf("cam[%d] reprojection error:\n", cam_idx);
    printf("  rmse:   %.4f # px\n", rmse(cam_errors));
    printf("  mean:   %.4f # px\n", mean(cam_errors));
    printf("  median: %.4f # px\n", median(cam_errors));
    printf("  stddev: %.4f # px\n", stddev(cam_errors));
    printf("\n");
  }
  printf("\n");

  // Cameras
  for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    printf("cam%d:\n", cam_idx);
    print_vector("  proj_params", cam_params[cam_idx]->proj_params());
    print_vector("  dist_params", cam_params[cam_idx]->dist_params());
    printf("\n");
  }

  // Time-delay
  if (time_delay) {
    print_vector("time delay (ts_cam = ts_imu + td): ", time_delay->param);
    printf("\n");
  }

  // Camera Extrinsics
  for (int cam_idx = 1; cam_idx < nb_cams(); cam_idx++) {
    const auto key = "T_cam0_cam" + std::to_string(cam_idx);
    const mat4_t T_C0Ci = get_cam_extrinsics(cam_idx);
    print_matrix(key, T_C0Ci, "  ");
  }

  // Imu extrinsics
  print_matrix("T_cam0_imu0", get_imu_extrinsics(), "  ");
}

// int calib_vi_t::recover_calib_covar(matx_t &calib_covar) {
//   // Recover calibration covariance
//   auto T_BS = imu_exts->param.data();
//   auto td = time_delay->param.data();
//
//   // -- Setup covariance blocks to estimate
//   std::vector<std::pair<const double *, const
//   double *>> covar_blocks;
//   covar_blocks.push_back({T_BS, T_BS});
//   covar_blocks.push_back({td, td});
//
//   // -- Estimate covariance
//   ::ceres::Covariance::Options options;
//   ::ceres::Covariance covar_est(options);
//   if (covar_est.Compute(covar_blocks, problem) ==
//   false) {
//     LOG_ERROR("Failed to estimate covariance!");
//     LOG_ERROR("Maybe Hessian is not full rank?");
//     return -1;
//   }
//
//   // -- Extract covariances sub-blocks
//   Eigen::Matrix<double, 6, 6, Eigen::RowMajor>
//   T_BS_covar; Eigen::Matrix<double, 1, 1,
//   Eigen::RowMajor> td_covar;
//   covar_est.GetCovarianceBlockInTangentSpace(T_BS,
//   T_BS, T_BS_covar.data());
//   covar_est.GetCovarianceBlock(td, td,
//   td_covar.data());
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

int calib_vi_t::save_results(const std::string &save_path) const {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Stats - Reprojection Errors
  const auto reproj_errors = get_reproj_errors();
  std::vector<real_t> reproj_errors_all;
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    extend(reproj_errors_all, cam_errors);
  }

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Calibration metrics
  fprintf(outfile, "calib_metrics:\n");
  fprintf(outfile, "  total_reproj_error:\n");
  fprintf(outfile, "    rmse:   %f # [px]\n", rmse(reproj_errors_all));
  fprintf(outfile, "    mean:   %f # [px]\n", mean(reproj_errors_all));
  fprintf(outfile, "    median: %f # [px]\n", median(reproj_errors_all));
  fprintf(outfile, "    stddev: %f # [px]\n", stddev(reproj_errors_all));
  fprintf(outfile, "\n");
  for (const auto &[cam_idx, errors] : get_reproj_errors()) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    fprintf(outfile, "  %s_reproj_error:\n", cam_str.c_str());
    fprintf(outfile, "    rmse:   %f # [px]\n", rmse(errors));
    fprintf(outfile, "    mean:   %f # [px]\n", mean(errors));
    fprintf(outfile, "    median: %f # [px]\n", median(errors));
    fprintf(outfile, "    stddev: %f # [px]\n", stddev(errors));
    fprintf(outfile, "\n");
  }
  fprintf(outfile, "\n");

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = cam_params.at(cam_idx);
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const std::string proj_params = vec2str(cam->proj_params(), 4);
    const std::string dist_params = vec2str(cam->dist_params(), 4);

    fprintf(outfile, "cam%d:\n", cam_idx);
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
  for (const auto &view : calib_views) {
    const auto sb = view.sb;
    bias_acc.push_back(sb.param.segment<3>(3));
    bias_gyr.push_back(sb.param.segment<3>(6));
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
  fprintf(outfile, "  g: %f\n", imu_params.g);
  fprintf(outfile, "  ba: [%f, %f, %f]\n", mu_ba.x(), mu_ba.y(), mu_ba.z());
  fprintf(outfile, "  bg: [%f, %f, %f]\n", mu_bg.x(), mu_bg.y(), mu_bg.z());
  fprintf(outfile, "\n");
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
      fprintf(outfile, "  ]\n");
      fprintf(outfile, "\n");
    }
  }

  // Sensor-Camera extrinsics
  const mat4_t T_BS = get_imu_extrinsics();
  const mat4_t T_SB = T_BS.inverse();
  const mat4_t T_BCi = get_cam_extrinsics(0);
  const mat4_t T_SCi = T_SB * T_BCi;
  fprintf(outfile, "T_imu0_cam%d:\n", 0);
  fprintf(outfile, "  rows: 4\n");
  fprintf(outfile, "  cols: 4\n");
  fprintf(outfile, "  data: [\n");
  fprintf(outfile, "%s\n", mat2str(T_SCi, "    ").c_str());
  fprintf(outfile, "  ]\n");
  fprintf(outfile, "\n");

  // Finsh up
  fclose(outfile);

  return 0;
}

void calib_vi_t::save_estimates(const std::string &dir_path) const {
  // Paths
  const auto poses_path = dir_path + "/poses_est.csv";
  const auto sb_path = dir_path + "/speed_biases_est.csv";

  // Setup csv files
  FILE *poses_csv = fopen(poses_path.c_str(), "w");
  FILE *sb_csv = fopen(sb_path.c_str(), "w");

  // Headers
  fprintf(poses_csv, "#ts,rx,ry,rz,qx,qy,qz,qw\n");
  fprintf(sb_csv, "#ts,vx,vy,vz,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n");

  // Write poses and speed and biases
  for (const auto &view : calib_views) {
    // Poses
    const auto pose = view.pose;
    const auto ts = pose.ts;
    const vec3_t r = pose.trans();
    const quat_t q = pose.rot();
    fprintf(poses_csv, "%ld,", ts);
    fprintf(poses_csv, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(poses_csv, "%f,%f,%f,%f\n", q.x(), q.y(), q.z(), q.w());

    // Speed and biases
    const auto sb = view.sb;
    const vec3_t v = sb.param.segment<3>(0);
    const vec3_t ba = sb.param.segment<3>(3);
    const vec3_t bg = sb.param.segment<3>(6);
    fprintf(sb_csv, "%ld,", ts);
    fprintf(sb_csv, "%f,%f,%f,", v.x(), v.y(), v.z());
    fprintf(sb_csv, "%f,%f,%f,", ba.x(), ba.y(), ba.z());
    fprintf(sb_csv, "%f,%f,%f\n", bg.x(), bg.y(), bg.z());
  }

  // Close csv files
  fclose(poses_csv);
  fclose(sb_csv);
}

} // namespace yac
