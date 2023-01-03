#include "calib_mocap.hpp"

namespace yac {

// MOCAP VIEW ////////////////////////////////////////////////////////////////

mocap_view_t::mocap_view_t(const aprilgrid_t &grid_,
                           solver_t *solver_,
                           calib_loss_t *loss_,
                           camera_geometry_t *cam_geom_,
                           camera_params_t *cam_params_,
                           pose_t *fiducial_pose_,
                           pose_t *mocap_pose_,
                           extrinsics_t *mocap_camera_extrinsics_)
    : ts{grid_.timestamp}, grid{grid_}, solver{solver_}, loss{loss_},
      cam_geom{cam_geom_}, cam_params{cam_params_},
      fiducial_pose{fiducial_pose_}, mocap_pose{mocap_pose_},
      mocap_camera_extrinsics{mocap_camera_extrinsics_} {
  if (grid.detected == false) {
    return;
  }

  // Get AprilGrid measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t kps;
  vec3s_t pts;
  grid.get_measurements(tag_ids, corner_idxs, kps, pts);

  // Add mocap residual
  const mat2_t covar = I(2);
  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_idxs[i];
    const vec2_t z = kps[i];
    const vec3_t r_FFi = pts[i];

    auto res_fn = std::make_shared<mocap_residual_t>(grid.timestamp,
                                                     cam_geom,
                                                     cam_params,
                                                     fiducial_pose,
                                                     mocap_pose,
                                                     mocap_camera_extrinsics,
                                                     tag_id,
                                                     corner_idx,
                                                     r_FFi,
                                                     z,
                                                     covar,
                                                     loss_);
    solver->add_residual(res_fn.get());
    res_fns.push_back(res_fn);
  }
}

int mocap_view_t::nb_detections() const { return grid.nb_detections; }

vec2s_t mocap_view_t::get_residuals() const {
  vec2s_t residuals;

  for (auto &res_fn : res_fns) {
    vec2_t r;
    if (res_fn->get_residual(r) == 0) {
      residuals.push_back(r);
    }
  }

  return residuals;
}

std::vector<real_t> mocap_view_t::get_reproj_errors() const {
  std::vector<real_t> reproj_errors;

  for (auto res_fn : res_fns) {
    real_t e;
    if (res_fn->get_reproj_error(e) == 0) {
      reproj_errors.push_back(e);
    }
  }

  return reproj_errors;
}

int mocap_view_t::filter_view(const vec2_t &threshold) {
  // Filter
  const real_t th_x = threshold.x();
  const real_t th_y = threshold.y();
  int nb_inliers = 0;
  int nb_outliers = 0;

  auto res_fns_it = res_fns.begin();
  while (res_fns_it != res_fns.end()) {
    auto res = *res_fns_it;

    vec2_t r{0.0, 0.0};
    if (res->get_residual(r) == 0 && (r.x() > th_x || r.y() > th_y)) {
      // Remove grid measurement
      auto before = grid.nb_detections;
      auto tag_id = res->tag_id;
      auto corner_idx = res->corner_idx;
      grid.remove(tag_id, corner_idx);
      auto after = grid.nb_detections;
      if (grid.has(tag_id, corner_idx)) {
        FATAL("tag_id: %d and corner_idx: %d should not exist!",
              tag_id,
              corner_idx);
      }
      if (before == after) {
        FATAL("before == after!");
      }

      // Remove residual block from ceres::Problem
      solver->remove_residual(res.get());

      res_fns_it = res_fns.erase(res_fns_it);
      nb_outliers++;
    } else {
      res_fns_it++;
      nb_inliers++;
    }
  }

  return nb_outliers;
}

//////////////////////////////////////////////////////////////////////////////

// CALIB MOCAP CACHE /////////////////////////////////////////////////////////

calib_mocap_cache_t::calib_mocap_cache_t(
    const camera_params_t &camera_,
    const pose_t &fiducial_pose_,
    const std::map<timestamp_t, pose_t> &mocap_poses_,
    const extrinsics_t &mocap_camera_extrinsics_) {
  cam_params = camera_.param;
  fiducial_pose = fiducial_pose_.param;
  for (const auto &[ts, mocap_pose] : mocap_poses_) {
    mocap_poses[ts] = mocap_pose.param;
  }
  mocap_camera_extrinsics = mocap_camera_extrinsics_.param;
}

void calib_mocap_cache_t::restore(camera_params_t &camera_,
                                  pose_t &fiducial_pose_,
                                  std::map<timestamp_t, pose_t> &mocap_poses_,
                                  extrinsics_t &mocap_camera_extrinsics_) {
  for (int i = 0; i < 7; i++) {
    camera_.param(i) = cam_params(i);
  }

  for (int i = 0; i < 7; i++) {
    fiducial_pose_.param(i) = fiducial_pose(i);
  }

  for (auto &[ts, mocap_pose] : mocap_poses) {
    for (int i = 0; i < 7; i++) {
      mocap_poses_[ts].param(i) = mocap_pose(i);
    }
  }

  for (int i = 0; i < 7; i++) {
    mocap_camera_extrinsics_.param(i) = mocap_camera_extrinsics(i);
  }
}

//////////////////////////////////////////////////////////////////////////////

// CALIB MOCAP  //////////////////////////////////////////////////////////////

calib_mocap_t::calib_mocap_t(const std::string &config_file_,
                             const std::string &data_path_)
    : config_file{config_file_}, data_path{data_path_},
      calib_rng(std::chrono::system_clock::now().time_since_epoch().count()) {
  // Load calib config file
  std::vector<int> cam_res;
  std::string proj_model;
  std::string dist_model;
  vecx_t proj_params;
  vecx_t dist_params;
  config_t config{config_file};
  parse(config, "settings.fix_intrinsics", fix_intrinsics, true);
  parse(config, "settings.fix_mocap_poses", fix_mocap_poses, true);
  parse(config, "settings.fix_fiducial_pose", fix_fiducial_pose, true);
  parse(config, "settings.outlier_threshold", outlier_threshold, true);
  parse(config, "settings.info_gain_threshold", info_gain_threshold, true);
  parse(config, "settings.enable_loss_fn", enable_loss_fn, true);
  parse(config, "settings.loss_fn_type", loss_fn_type, true);
  parse(config, "settings.loss_fn_param", loss_fn_param, true);
  parse(config, "settings.enable_shuffle_views", enable_shuffle_views, true);
  parse(config, "settings.show_progress", show_progress, true);
  parse(config, "settings.max_iter", max_iter, true);
  parse(config, "cam0.resolution", cam_res);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);
  parse(config, "cam0.proj_params", proj_params, true);
  parse(config, "cam0.dist_params", dist_params, true);

  // Setup paths
  const auto cam0_path = data_path + "/cam0/data";
  const auto grid0_path = data_path + "/grid0/cam0/data";
  const auto body0_csv_path = data_path + "/body0/data.csv";
  const auto target0_csv_path = data_path + "/target0/data.csv";

  // Setup calibration target
  if (calib_target.load(config_file, "calib_target") != 0) {
    FATAL("Failed to load calib target in [%s]!", config_file.c_str());
  }

  // Setup solver
  solver = std::make_unique<ceres_solver_t>();

  // Setup loss function
  if (enable_loss_fn && loss_fn == nullptr) {
    if (loss_fn_type == "BLAKE-ZISSERMAN") {
      loss_fn = std::make_unique<BlakeZissermanLoss>((int)loss_fn_param);
    } else if (loss_fn_type == "CAUCHY") {
      loss_fn = std::make_unique<CauchyLoss>(loss_fn_param);
    } else {
      FATAL("Unsupported loss function type [%s]!", loss_fn_type.c_str());
    }
  }

  // Setup camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    camera_geometry = std::make_shared<pinhole_radtan4_t>();
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    camera_geometry = std::make_shared<pinhole_equi4_t>();
  } else {
    FATAL("Unsupported [%s]-[%s]!", proj_model.c_str(), dist_model.c_str());
  }

  // Preprocess camera data
  const auto cam_grids = _preprocess(calib_target, cam0_path, grid0_path);

  // Initialize / load camera parameters
  if (yaml_has_key(config, "cam0.proj_params") == false) {
    LOG_INFO("Camera parameters unknown!");
    LOG_INFO("Calibrating camera intrinsics!");
    calib_camera_t calib{calib_target};
    calib.add_camera_data(0, cam_grids);
    calib.add_camera(0, cam_res.data(), proj_model, dist_model);
    calib.solve();
    proj_params = calib.get_camera_params(0).head(4);
    dist_params = calib.get_camera_params(0).tail(4);
  }
  _add_camera(cam_res,
              proj_model,
              dist_model,
              proj_params,
              dist_params,
              fix_intrinsics);

  // Load dataset
  // -- Fiducial target pose
  const mat4_t T_WF = load_pose(target0_csv_path);
  _add_fiducial_pose(0, T_WF, fix_fiducial_pose);
  // -- Mocap poses
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_poses(body0_csv_path, body_timestamps, body_poses);
  // -- Synchronize aprilgrids and body poses
  mat4s_t T_WM;
  const aprilgrids_t grids_raw = load_aprilgrids(grid0_path);
  aprilgrids_t grids_lerped;
  lerp_body_poses(grids_raw, body_timestamps, body_poses, grids_lerped, T_WM);
  for (size_t k = 0; k < grids_lerped.size(); k++) {
    _add_mocap_pose(grids_lerped[k].timestamp, T_WM[k], fix_mocap_poses);
  }
  // -- Mocap marker to camera transform
  mat4_t T_MC0;
  for (const auto &grid : grids_lerped) {
    const auto ts = grid.timestamp;
    const mat4_t T_MW = mocap_poses[ts].tf().inverse();

    const auto cam_res = camera.resolution;
    const auto cam_param = camera.param;
    mat4_t T_C0F;
    if (grid.estimate(camera_geometry.get(), cam_res, cam_param, T_C0F) != 0) {
      continue;
    }

    T_MC0 = T_MW * T_WF * T_C0F.inverse();
    calib_grids[ts] = grid;
  }
  _add_mocap_camera_extrinsics(T_MC0);
}

int calib_mocap_t::get_num_views() const { return calib_views.size(); }

std::vector<real_t> calib_mocap_t::get_reproj_errors() const {
  std::vector<real_t> reproj_errors;
  for (const auto &[ts, view] : calib_views) {
    for (const auto e : view->get_reproj_errors()) {
      reproj_errors.push_back(e);
    }
  }
  return reproj_errors;
}

vec2s_t calib_mocap_t::get_residuals() const {
  vec2s_t residuals;

  for (auto &[ts, view] : calib_views) {
    for (const auto &r : view->get_residuals()) {
      residuals.push_back(r);
    }
  }

  return residuals;
}

mat4_t calib_mocap_t::get_fiducial_pose() const { return fiducial_pose.tf(); }

mat4_t calib_mocap_t::get_mocap_camera_extrinsics() const {
  return mocap_camera_extrinsics.tf();
}

vecx_t calib_mocap_t::get_camera_params() const { return camera.param; }

aprilgrids_t calib_mocap_t::_preprocess(const calib_target_t &calib_target,
                                        const std::string &cam_path,
                                        const std::string &grid_path) {
  LOG_INFO("Preprocessing camera data [%s]", cam_path.c_str());

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
    const auto ts_str = std::to_string(ts);
    const auto grid_csv_path = grid_path + "/" + ts_str + ".csv";

    aprilgrid_t grid;
    if (file_exists(grid_csv_path)) {
      grid.load(grid_csv_path);
      if (grid.detected == false) {
        grid.timestamp = ts;
        grid.tag_rows = detector.tag_rows;
        grid.tag_cols = detector.tag_cols;
        grid.tag_size = detector.tag_size;
        grid.tag_spacing = detector.tag_spacing;
      }

    } else {
      const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      grid = detector.detect(ts, image);
      grid.save(grid_csv_path);
    }
    grids.push_back(grid);

    printf(".");
    fflush(stdout);
  }
  printf("\n");
  LOG_INFO("Preprocessed [%ld] AprilGrids!", grids.size());

  return grids;
}

void calib_mocap_t::_add_camera(const std::vector<int> &cam_res,
                                const std::string &proj_model,
                                const std::string &dist_model,
                                const vecx_t &proj_params,
                                const vecx_t &dist_params,
                                const bool fix) {
  camera = camera_params_t{0,
                           cam_res.data(),
                           proj_model,
                           dist_model,
                           proj_params,
                           dist_params,
                           fix};
  solver->add_param(&camera);
}

void calib_mocap_t::_add_fiducial_pose(const timestamp_t ts,
                                       const mat4_t &T_WF,
                                       const bool fix) {
  fiducial_pose = pose_t{ts, T_WF, fix};
  solver->add_param(&fiducial_pose);
}

void calib_mocap_t::_add_mocap_pose(const timestamp_t ts,
                                    const mat4_t &T_WM,
                                    const bool fix) {
  mocap_poses[ts] = pose_t{ts, T_WM, fix};
  solver->add_param(&mocap_poses[ts]);
}

void calib_mocap_t::_add_mocap_camera_extrinsics(const mat4_t &T_MC0) {
  mocap_camera_extrinsics = extrinsics_t{T_MC0};
  solver->add_param(&mocap_camera_extrinsics);
}

void calib_mocap_t::_add_view(const aprilgrid_t &grid) {
  calib_view_timestamps.push_back(grid.timestamp);
  calib_views[grid.timestamp] =
      std::make_shared<mocap_view_t>(grid,
                                     solver.get(),
                                     loss_fn.get(),
                                     camera_geometry.get(),
                                     &camera,
                                     &fiducial_pose,
                                     &mocap_poses[grid.timestamp],
                                     &mocap_camera_extrinsics);
}

void calib_mocap_t::_remove_view(const timestamp_t ts) {
  // Remove pose
  if (mocap_poses.count(ts)) {
    if (solver->has_param(&mocap_poses[ts])) {
      solver->remove_param(&mocap_poses[ts]);
    }
    mocap_poses.erase(ts);
  }

  // Remove view
  if (calib_views.count(ts)) {
    auto view_it = calib_views.find(ts);
    calib_views.erase(view_it);

    auto ts_it = std::find(calib_view_timestamps.begin(),
                           calib_view_timestamps.end(),
                           ts);
    calib_view_timestamps.erase(ts_it);
  }
}

int calib_mocap_t::_calc_info(real_t *info, real_t *entropy) {
  // Form parameter vector
  std::vector<param_t *> params;
  real_t local_param_size = 6;
  params.push_back(&mocap_camera_extrinsics);

  // Estimate the determinant of the marginal covariance matrix
  real_t log_det_covar = 0.0;
  if (solver->estimate_log_det_covar(params, log_det_covar) != 0) {
    return -1;
  }
  if (std::isnan(std::abs(log_det_covar))) {
    return -1;
  }
  *info = log_det_covar / log(2.0);

  // Estimate shannon entropy
  const auto k = pow(2 * M_PI * exp(1), local_param_size);
  *entropy = 0.5 * log(k * exp(log_det_covar));

  return 0;
}

void calib_mocap_t::_cache_estimates() {
  cache = calib_mocap_cache_t(camera,
                              fiducial_pose,
                              mocap_poses,
                              mocap_camera_extrinsics);
}

void calib_mocap_t::_restore_estimates() {
  cache.restore(camera, fiducial_pose, mocap_poses, mocap_camera_extrinsics);
}

void calib_mocap_t::_print_stats(const size_t ts_idx,
                                 const size_t nb_timestamps) {
  // Show general stats
  const real_t progress = ((real_t)ts_idx / nb_timestamps) * 100.0;
  const auto reproj_errors = get_reproj_errors();
  printf("[%.2f%%] ", progress);
  printf("nb_views: %d  ", get_num_views());
  printf("reproj_error: %.4f  ", mean(reproj_errors));
  printf("info_k: %.4f  ", info_k);
  printf("\n");
}

int calib_mocap_t::_filter_last_view() {
  const auto ts = calib_view_timestamps.back();
  const vec2s_t residuals = get_residuals();
  const vec2_t residual_stddev = stddev(residuals);
  const vec2_t thresholds = outlier_threshold * residual_stddev;
  return calib_views[ts]->filter_view(thresholds);
}

int calib_mocap_t::_filter_all_views() {
  const vec2s_t residuals = get_residuals();
  const vec2_t residual_stddev = stddev(residuals);
  const vec2_t thresholds = outlier_threshold * residual_stddev;
  int removed = 0;
  for (auto &[ts, view] : calib_views) {
    removed += view->filter_view(thresholds);
  }
  return removed;
}

int calib_mocap_t::solve() {
  assert(calib_grids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == grids.size());

  // Build problem
  for (const auto &[ts, grid] : calib_grids) {
    _add_view(grid);
  }

  // Solve
  LOG_INFO("Calibrating mocap-marker to camera extrinsics ...");
  solver->solve(max_iter, true, 0);

  // Filter all views
  LOG_INFO("Filtering all views!");
  const int removed = _filter_all_views();
  LOG_INFO("Removed [%d] outliers!\n", removed);

  // Final solve
  LOG_INFO("Solving again!");
  solver->solve(max_iter, true, 0);

  // Show results
  show_results();

  return 0;
}

int calib_mocap_t::solve_nbv() {
  // Pre-process timestamps
  timestamps_t nbv_timestamps;
  for (const auto &[ts, grid] : calib_grids) {
    if (grid.detected) {
      nbv_timestamps.push_back(ts);
    }
  }
  if (nbv_timestamps.size() == 0) {
    FATAL("Implementation Error: No views to process?");
  }

  // Shuffle timestamps
  if (enable_shuffle_views) {
    std::shuffle(nbv_timestamps.begin(), nbv_timestamps.end(), calib_rng);
  }

  int ts_idx = 0;
  int nb_timestamps = nbv_timestamps.size();
  for (const auto &ts : nbv_timestamps) {
    // Add view
    _add_view(calib_grids[ts]);
    _filter_all_views();

    // Keep track of initial values - incase view is rejected
    _cache_estimates();

    // Solve with new view
    solver->solve(30);
    if (get_num_views() < 5) {
      ts_idx++;
      continue;
    }

    // Calculate information gain
    real_t info_kp1 = 0.0;
    real_t entropy_kp1 = 0.0;
    if (_calc_info(&info_kp1, &entropy_kp1) != 0) {
      _restore_estimates();
      ts_idx++;
      continue;
    }

    // Remove view?
    const real_t info_gain = 0.5 * (info_k - info_kp1);
    if (info_gain < info_gain_threshold) {
      _restore_estimates();
      _remove_view(ts);
    } else {
      info_k = info_kp1;
      entropy_k = entropy_kp1;
    }

    // Print stats
    _print_stats(ts_idx++, nb_timestamps);
  }

  // Final refinement
  solver->solve(max_iter, true, 0);
  show_results();

  return 0;
}

void calib_mocap_t::print_settings(FILE *out) const {
  // clang-format off
  fprintf(out, "settings:\n");
  fprintf(out, "  fix_intrinsics: %s\n", fix_intrinsics ? "true" : "false");
  fprintf(out, "  fix_mocap_poses: %s\n", fix_mocap_poses ? "true" : "false");
  fprintf(out, "  fix_fiducial_pose: %s\n", fix_fiducial_pose ? "true" : "false");
  fprintf(out, "  outlier_threshold: %f\n", outlier_threshold);
  fprintf(out, "  info_gain_threshold: %f\n", info_gain_threshold);
  fprintf(out, "  enable_loss_fn: %s\n", enable_loss_fn ? "true" : "false");
  fprintf(out, "  loss_fn_type: \"%s\"\n", loss_fn_type.c_str());
  fprintf(out, "  loss_fn_param: %f\n", loss_fn_param);
  fprintf(out, "  enable_shuffle_views: %s\n", enable_shuffle_views ? "true" : "false");
  fprintf(out, "  show_progress: %s\n", show_progress ? "true" : "false");
  fprintf(out, "  max_iter: %d\n", max_iter);
  fprintf(out, "\n");
  // clang-format on
}

void calib_mocap_t::print_calib_target(FILE *out) const {
  fprintf(out, "calib_target:\n");
  fprintf(out, "  target_type: \"%s\"\n", calib_target.target_type.c_str());
  fprintf(out, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(out, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(out, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(out, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(out, "\n");
}

void calib_mocap_t::print_metrics(FILE *out) const {
  const auto reproj_errors = get_reproj_errors();
  fprintf(out, "total_reproj_error:\n");
  fprintf(out, "  nb_views: %d\n", get_num_views());
  fprintf(out, "  nb_corners: %ld\n", reproj_errors.size());
  fprintf(out, "  rmse:   %.4f # [px]\n", rmse(reproj_errors));
  fprintf(out, "  mean:   %.4f # [px]\n", mean(reproj_errors));
  fprintf(out, "  median: %.4f # [px]\n", median(reproj_errors));
  fprintf(out, "  stddev: %.4f # [px]\n", stddev(reproj_errors));
  fprintf(out, "\n");
}

void calib_mocap_t::print_fiducial_pose(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const mat4_t T_WF = get_fiducial_pose();
  fprintf(out, "T_world_fiducial:\n");
  fprintf(out, "  fixed: %s\n", fiducial_pose.fixed ? "true" : "false");
  fprintf(out, "  rows: 4\n");
  fprintf(out, "  cols: 4\n");
  fprintf(out, "  data: [\n");
  fprintf(out, "%s\n", mat2str(T_WF, "    ", max_digits).c_str());
  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::print_mocap_camera_extrinsics(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const mat4_t T_MC0 = get_mocap_camera_extrinsics();
  const bool fixed = mocap_camera_extrinsics.fixed;
  fprintf(out, "T_mocap_camera:\n");
  fprintf(out, "  fixed: %s\n", fixed ? "true" : "false");
  fprintf(out, "  rows: 4\n");
  fprintf(out, "  cols: 4\n");
  fprintf(out, "  data: [\n");
  fprintf(out, "%s\n", mat2str(T_MC0, "    ", max_digits).c_str());
  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::print_camera_params(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const bool fixed = camera.fixed;
  const int *cam_res = camera.resolution;
  const char *proj_model = camera.proj_model.c_str();
  const char *dist_model = camera.dist_model.c_str();
  const auto proj_params = vec2str(camera.proj_params(), true, max_digits);
  const auto dist_params = vec2str(camera.dist_params(), true, max_digits);

  fprintf(out, "cam%d:\n", 0);
  fprintf(out, "  fixed: %s\n", fixed ? "true" : "false");
  fprintf(out, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
  fprintf(out, "  proj_model: \"%s\"\n", proj_model);
  fprintf(out, "  dist_model: \"%s\"\n", dist_model);
  fprintf(out, "  proj_params: %s\n", proj_params.c_str());
  fprintf(out, "  dist_params: %s\n", dist_params.c_str());
  fprintf(out, "\n");
}

void calib_mocap_t::print_mocap_poses(FILE *out) const {
  fprintf(out, "mocap_poses:\n");
  fprintf(out, "  rows: %d\n", get_num_views());
  fprintf(out, "  cols: 8\n");
  fprintf(out, "\n");
  fprintf(out, "  # ts, rx, ry, rz, qx, qy, qz, qw\n");
  fprintf(out, "  data: [\n");

  auto iter = mocap_poses.begin();
  while (iter != mocap_poses.end()) {
    const auto ts = iter->first;
    const auto pose = iter->second;
    const mat4_t T_C0F = pose.tf();
    const vec3_t r = tf_trans(T_C0F);
    const quat_t q = tf_quat(T_C0F);
    fprintf(out, "    ");
    fprintf(out, "%ld,", ts);
    fprintf(out, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(out, "%f,%f,%f,%f", q.x(), q.y(), q.z(), q.w());

    iter++;
    if (iter != mocap_poses.end()) {
      fprintf(out, ",\n");
    } else {
      fprintf(out, "\n");
    }
  }

  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::show_results() const {
  printf("Optimization results:\n");
  printf("---------------------\n");
  print_settings(stdout);
  print_calib_target(stdout);
  print_metrics(stdout);
  print_camera_params(stdout);
  print_mocap_camera_extrinsics(stdout);
  print_fiducial_pose(stdout);
}

int calib_mocap_t::save_results(const std::string &save_path) const {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  print_settings(outfile);
  print_calib_target(outfile);
  print_metrics(outfile);
  print_mocap_camera_extrinsics(outfile);
  print_camera_params(outfile);
  print_fiducial_pose(outfile);
  print_mocap_poses(outfile);
  fclose(outfile);

  return 0;
}

//////////////////////////////////////////////////////////////////////////////

} //  namespace yac
