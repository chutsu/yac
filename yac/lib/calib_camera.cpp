#include "calib_camera.hpp"

namespace yac {

// CAMCHAIN ////////////////////////////////////////////////////////////////////

camchain_t::camchain_t(const camera_data_t &cam_data,
                       const CamIdx2Geometry &cam_geoms,
                       const std::map<int, camera_params_t *> &cam_params) {
  for (const auto &kv : cam_data) {
    const auto &ts = kv.first;

    // Get all detected AprilGrid at ts
    std::vector<int> cam_indicies;
    std::vector<aprilgrid_t> grids;
    _get_aprilgrids(ts, cam_data, cam_indicies, grids);

    // Check we have more than 1
    if (cam_indicies.size() < 2) {
      continue;
    }

    // Add to camchain if we haven't already
    const int cam_i = cam_indicies[0];
    const auto params_i = cam_params.at(cam_i);
    const auto res_i = params_i->resolution;
    const auto geom_i = cam_geoms.at(cam_i);
    const auto &grid_i = grids[0];

    for (size_t i = 1; i < cam_indicies.size(); i++) {
      const auto cam_j = cam_indicies[i];
      const auto params_j = cam_params.at(cam_j);
      const auto res_j = params_j->resolution;
      const auto geom_j = cam_geoms.at(cam_j);
      const auto &grid_j = grids[i];

      if (contains(cam_i, cam_j) == false) {
        mat4_t T_CiF;
        if (grid_i.estimate(geom_i, res_i, params_i->param, T_CiF) != 0) {
          FATAL("Failed to estimate relative pose!");
        }

        mat4_t T_CjF;
        if (grid_j.estimate(geom_j, res_j, params_j->param, T_CjF) != 0) {
          FATAL("Failed to estimate relative pose!");
        }

        const mat4_t T_CiCj = T_CiF * tf_inv(T_CjF);
        insert(cam_i, cam_j, T_CiCj);
      }
    }
  }
}

void camchain_t::_get_aprilgrids(const timestamp_t &ts,
                                 const camera_data_t &cam_data,
                                 std::vector<int> &cam_indicies,
                                 std::vector<aprilgrid_t> &grids) const {
  // Get all detected AprilGrid at ts
  for (const auto &[cam_idx, grid] : cam_data.at(ts)) {
    // Check aprilgrid is detected
    if (grid.detected == false && grid.nb_detections < 12) {
      continue;
    }

    // Update
    cam_indicies.push_back(cam_idx);
    grids.push_back(grid);
  }
}

void camchain_t::insert(const int cam_i,
                        const int cam_j,
                        const mat4_t &T_CiCj) {
  if (contains(cam_i, cam_j)) {
    return;
  }

  adj_list[cam_i].push_back(cam_j);
  adj_list[cam_j].push_back(cam_i);
  exts[cam_i][cam_j] = T_CiCj;
  exts[cam_j][cam_i] = tf_inv(T_CiCj);
}

bool camchain_t::contains(const int cam_i, const int cam_j) const {
  return (exts.count(cam_i) && exts.at(cam_i).count(cam_j));
}

int camchain_t::find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const {
  // Straight-forward case
  if (cam_i == cam_j) {
    T_CiCj = I(4);
    return 0;
  }

  // Check if we have even inserted the cameras before
  if (exts.count(cam_i) == 0 || exts.count(cam_j) == 0) {
    return -1;
  }

  // Check if we already have the extrinsics pair
  if (contains(cam_i, cam_j)) {
    T_CiCj = exts.at(cam_i).at(cam_j);
    return 0;
  }

  // Iterative BFS - To get path from cam_i to cam_j
  bool found_target = false;
  std::deque<int> queue;
  std::map<int, bool> visited;
  std::map<int, int> path_map;

  queue.push_back(cam_i);
  while (!queue.empty()) {
    const auto parent = queue.front();
    queue.pop_front();
    visited.at(parent) = true;

    for (const int &child : adj_list.at(parent)) {
      if (visited.at(child)) {
        continue;
      }

      queue.push_back(child);
      path_map[child] = parent;

      if (child == cam_j) {
        found_target = true;
        break;
      }
    }
  }

  // Check if we've found the target
  if (found_target == false) {
    return -2;
  }

  // Traverse the path backwards and chain the transforms
  mat4_t T_CjCi = I(4);
  int child = cam_j;
  while (path_map.count(child)) {
    const int parent = path_map[child];
    T_CjCi = T_CjCi * exts.at(child).at(parent);
    child = parent;
  }
  T_CiCj = tf_inv(T_CjCi);

  return 0;
}

void camchain_t::clear() {
  adj_list.clear();
  exts.clear();
}

// CALIB VIEW //////////////////////////////////////////////////////////////////

calib_view_t::calib_view_t(const timestamp_t ts_,
                           const CamIdx2Grids &grids_,
                           fiducial_corners_t *corners_,
                           solver_t *solver_,
                           CamIdx2Geometry *cam_geoms_,
                           CamIdx2Parameters *cam_params_,
                           CamIdx2Extrinsics *cam_exts_,
                           pose_t *T_C0F_)
    : ts{ts_}, grids{grids_}, corners{corners_}, solver{solver_},
      cam_geoms{cam_geoms_},
      cam_params{cam_params_}, cam_exts{cam_exts_}, T_C0F{T_C0F_} {
  const mat2_t covar = I(2);
  for (const auto &[cam_idx, cam_grid] : grids_) {
    if (cam_params->count(cam_idx) == 0) {
      continue;
    }
    if (cam_grid.detected == false) {
      continue;
    }

    // Get AprilGrid measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t pts;
    cam_grid.get_measurements(tag_ids, corner_idxs, kps, pts);

    // Add reprojection errors
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_idxs[i];
      const vec2_t z = kps[i];
      auto p_FFi_ = corners_->get_corner(tag_id, corner_idx);
      auto res_fn = new reproj_residual_t(cam_geoms_->at(cam_idx),
                                          cam_params_->at(cam_idx),
                                          cam_exts_->at(cam_idx),
                                          T_C0F_,
                                          p_FFi_,
                                          z,
                                          covar);
      solver->add_residual(res_fn);
      res_fns[cam_idx].push_back(res_fn);
    }
  }
}

calib_view_t::~calib_view_t() {
  for (auto &[cam_idx, cam_residuals] : res_fns) {
    for (auto res_fn : cam_residuals) {
      delete res_fn;
    }
  }
}

int calib_view_t::nb_detections() const {
  int nb_detections = 0;
  for (const auto &[cam_idx, grid] : grids) {
    nb_detections += grid.nb_detections;
  }
  return nb_detections;
}

std::vector<int> calib_view_t::get_camera_indices() const {
  std::vector<int> camera_indices;
  for (const auto &[cam_idx, cam_param] : *cam_params) {
    camera_indices.push_back(cam_idx);
  }
  return camera_indices;
}

vec2s_t calib_view_t::get_residuals() const {
  vec2s_t residuals;

  for (auto &[cam_idx, cam_residuals] : res_fns) {
    for (auto res_fn : cam_residuals) {
      vec2_t r;
      if (res_fn->get_residual(r) == 0) {
        residuals.push_back(r);
      }
    }
  }

  return residuals;
}

vec2s_t calib_view_t::get_residuals(const int cam_idx) const {
  vec2s_t residuals;

  for (auto &[res_cam_idx, cam_residuals] : res_fns) {
    if (res_cam_idx != cam_idx) {
      continue;
    }

    for (auto res_fn : cam_residuals) {
      vec2_t r;
      if (res_fn->get_residual(r) == 0) {
        residuals.push_back(r);
      }
    }
  }

  return residuals;
}

std::vector<real_t> calib_view_t::get_reproj_errors() const {
  std::vector<real_t> reproj_errors;

  for (auto &[cam_idx, cam_residuals] : res_fns) {
    for (auto res_fn : cam_residuals) {
      real_t e;
      if (res_fn->get_reproj_error(e) == 0) {
        reproj_errors.push_back(e);
      }
    }
  }

  return reproj_errors;
}

std::vector<real_t> calib_view_t::get_reproj_errors(const int cam_idx) const {
  std::vector<real_t> reproj_errors;

  for (auto &[res_cam_idx, cam_residuals] : res_fns) {
    if (res_cam_idx != cam_idx) {
      continue;
    }

    for (auto res_fn : cam_residuals) {
      real_t e;
      if (res_fn->get_reproj_error(e) == 0) {
        reproj_errors.push_back(e);
      }
    }
  }

  return reproj_errors;
}

int calib_view_t::filter_view(const vec2_t &threshold) {
  // Filter
  const real_t th_x = threshold.x();
  const real_t th_y = threshold.y();
  int nb_inliers = 0;
  int nb_outliers = 0;

  for (const auto cam_idx : get_camera_indices()) {
    auto res_fns_it = res_fns[cam_idx].begin();

    while (res_fns_it != res_fns[cam_idx].end()) {
      auto res = *res_fns_it;

      vec2_t r{0.0, 0.0};
      if (res->get_residual(r) == 0 && (r.x() > th_x || r.y() > th_y)) {
        // Remove grid measurement
        auto before = grids[cam_idx].nb_detections;
        auto tag_id = res->p_FFi->tag_id;
        auto corner_idx = res->p_FFi->corner_idx;
        grids[cam_idx].remove(tag_id, corner_idx);
        auto after = grids[cam_idx].nb_detections;
        if (grids[cam_idx].has(tag_id, corner_idx)) {
          FATAL("tag_id: %d and corner_idx: %d should not exist!",
                tag_id,
                corner_idx);
        }
        if (before == after) {
          FATAL("before == after!");
        }

        // Remove residual block from ceres::Problem
        solver->remove_residual(res);
        delete res;

        res_fns_it = res_fns[cam_idx].erase(res_fns_it);
        nb_outliers++;
      } else {
        res_fns_it++;
        nb_inliers++;
      }
    }
  }

  return nb_outliers;
}

int calib_view_t::filter_view(const std::map<int, vec2_t> &thresholds) {
  int nb_outliers = 0;

  for (const auto &[cam_idx, cam_res] : thresholds) {
    nb_outliers += filter_view(cam_res);
  }

  return nb_outliers;
}

void calib_view_t::marginalize(marg_residual_t *marg_residual) {
  // Mark relative pose T_C0F to be marginalized
  T_C0F->marginalize = true;

  // Transfer residuals to marginalization error
  for (auto &[cam_idx, cam_residuals] : res_fns) {
    for (auto &res_fn : cam_residuals) {
      marg_residual->add(res_fn);
    }
  }

  // Marginalize
  std::vector<param_t *> marg_params;
  std::vector<calib_residual_t *> marg_residuals;
  marg_residual->marginalize(marg_params, marg_residuals);
  for (auto param : marg_params) {
    solver->remove_param(param);
  }
  for (auto res_fn : marg_residuals) {
    delete res_fn;
  }
  solver->add_residual(marg_residual);

  // Clear residuals
  res_fns.clear();
  // ^ Important! we don't want to delete the residual blocks when the view
  // is deconstructed, but rather by adding the residual functions to the
  // marginalization error we pass the ownership to marg_residual_t
}

// CALIB CAMERA ////////////////////////////////////////////////////////////////

static void print_calib_target(FILE *out, calib_target_t &calib_target) {
  fprintf(out, "calib_target:\n");
  fprintf(out, "  target_type: %s\n", calib_target.target_type.c_str());
  fprintf(out, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(out, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(out, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(out, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(out, "\n");
}

static void print_camera_params(FILE *out,
                                const int cam_idx,
                                const camera_params_t *cam) {
  const bool max_digits = (out == stdout) ? false : true;
  const int *cam_res = cam->resolution;
  const char *proj_model = cam->proj_model.c_str();
  const char *dist_model = cam->dist_model.c_str();
  const auto proj_params = vec2str(cam->proj_params(), true, max_digits);
  const auto dist_params = vec2str(cam->dist_params(), true, max_digits);

  fprintf(out, "cam%d:\n", cam_idx);
  fprintf(out, "  fixed: %s\n", cam->fixed ? "true" : "false");
  fprintf(out, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
  fprintf(out, "  proj_model: \"%s\"\n", proj_model);
  fprintf(out, "  dist_model: \"%s\"\n", dist_model);
  fprintf(out, "  proj_params: %s\n", proj_params.c_str());
  fprintf(out, "  dist_params: %s\n", dist_params.c_str());
}

static void print_estimates(FILE *out,
                            CamIdx2Parameters &cam_params,
                            CamIdx2Extrinsics &cam_exts) {
  const bool max_digits = (out == stdout) ? false : true;

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = kv.second;
    print_camera_params(out, cam_idx, cam);
    fprintf(out, "\n");
  }

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = cam_exts[0]->tf();
  const mat4_t T_C0B = tf_inv(T_BC0);
  for (auto &kv : cam_exts) {
    const auto cam_idx = kv.first;
    const auto cam_ext = kv.second;
    if (cam_idx == 0) {
      continue;
    }

    const mat4_t T_BCi = cam_ext->tf();
    const mat4_t T_C0Ci = T_C0B * T_BCi;
    fprintf(out, "T_cam0_cam%d:\n", cam_idx);
    fprintf(out, "  fixed: %s\n", cam_ext->fixed ? "true" : "false");
    fprintf(out, "  rows: 4\n");
    fprintf(out, "  cols: 4\n");
    fprintf(out, "  data: [\n");
    fprintf(out, "%s\n", mat2str(T_C0Ci, "    ", max_digits).c_str());
    fprintf(out, "  ]\n");
    fprintf(out, "\n");
  }
}

static void calib_initialize(const calib_target_t &calib_target,
                             const camera_data_t &calib_data,
                             const std::string &solver_type,
                             CamIdx2Geometry &cam_geoms,
                             CamIdx2Parameters &cam_params,
                             CamIdx2Extrinsics &cam_exts,
                             std::map<timestamp_t, pose_t *> &poses) {
  // Setup solver
  solver_t *solver = nullptr;
  if (solver_type == "CERES-SOLVER") {
    solver = new ceres_solver_t();
  } else if (solver_type == "YAC-SOLVER") {
    solver = new yac_solver_t();
  } else {
    FATAL("Unsupported solver type [%s]", solver_type.c_str());
  }

  // Data
  poses.clear();
  std::map<timestamp_t, calib_view_t *> calib_views;
  auto corners = new fiducial_corners_t(calib_target);

  // Setup problem
  // -- Add camera parameters
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    solver->add_param(param);
  }
  // -- Add camera extrinsics
  for (auto &[cam_idx, param] : cam_exts) {
    UNUSED(cam_idx);
    solver->add_param(param);
  }
  // -- Add reprojection errors
  for (auto &[ts, cam_grids] : calib_data) {
    // Find the AprilGrid that has the most observations at timestamp ts
    int best_cam_idx = -1;
    aprilgrid_t best_grid;
    for (const auto &[cam_idx, grid] : cam_grids) {
      if (cam_params.count(cam_idx) == 0) {
        continue;
      }
      if (cam_exts.count(cam_idx) == 0) {
        continue;
      }
      if (grid.detected == false) {
        continue;
      }
      if (grid.nb_detections > best_grid.nb_detections) {
        best_cam_idx = cam_idx;
        best_grid = grid;
      }
    }
    if (best_cam_idx == -1) {
      continue;
    }

    // Estimate relative pose T_C0F
    const auto cam_geom = cam_geoms[best_cam_idx];
    const vecx_t param = cam_params[best_cam_idx]->param;
    const auto res = cam_params[best_cam_idx]->resolution;
    mat4_t T_CiF;
    if (best_grid.estimate(cam_geom, res, param, T_CiF) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Add relative pose T_C0F
    const mat4_t T_C0Ci = cam_exts.at(best_cam_idx)->tf();
    const mat4_t T_C0F = T_C0Ci * T_CiF;
    poses[ts] = new pose_t{ts, T_C0F};
    solver->add_param(poses[ts]);

    // Add calibration view
    calib_views[ts] = new calib_view_t{ts,
                                       cam_grids,
                                       corners,
                                       solver,
                                       &cam_geoms,
                                       &cam_params,
                                       &cam_exts,
                                       poses[ts]};
  }

  // Solve
  // printf("Before:\n");
  // print_estimates(stdout, cam_params, cam_exts);
  // printf("\n");
  solver->solve(50, true, 0);
  // printf("After:\n");
  // print_estimates(stdout, cam_params, cam_exts);
  // printf("\n");

  // Clean up
  for (auto &[ts, view] : calib_views) {
    delete view;
  }
  delete corners;
  delete solver;
}

calib_camera_t::calib_camera_t(const calib_target_t &calib_target_)
    : calib_target{calib_target_},
      calib_rng(std::chrono::system_clock::now().time_since_epoch().count()) {
  // Solver
  if (solver_type == "CERES-SOLVER") {
    solver = new ceres_solver_t();
  } else if (solver_type == "YAC-SOLVER") {
    solver = new yac_solver_t();
  } else {
    FATAL("Unsupported solver type [%s]", solver_type.c_str());
  }

  // AprilGrid
  corners = new fiducial_corners_t(calib_target);
  detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);
}

calib_camera_t::calib_camera_t(const std::string &config_path)
    : calib_rng(std::chrono::system_clock::now().time_since_epoch().count()) {
  // Solver
  if (solver_type == "CERES-SOLVER") {
    solver = new ceres_solver_t();
  } else if (solver_type == "YAC-SOLVER") {
    solver = new yac_solver_t();
  } else {
    FATAL("Unsupported solver type [%s]", solver_type.c_str());
  }

  // Load configuration
  config_t config{config_path};
  // -- Parse settings
  // clang-format off
  parse(config, "settings.verbose", verbose, true);
  parse(config, "settings.max_num_threads", max_num_threads, true);
  parse(config, "settings.enable_nbv", enable_nbv, true);
  parse(config, "settings.enable_shuffle_views", enable_shuffle_views, true);
  parse(config, "settings.enable_nbv_filter", enable_nbv_filter, true);
  parse(config, "settings.enable_marginalization", enable_marginalization, true);
  parse(config, "settings.min_nbv_views", min_nbv_views, true);
  parse(config, "settings.outlier_threshold", outlier_threshold, true);
  parse(config, "settings.info_gain_threshold", info_gain_threshold, true);
  parse(config, "settings.sliding_window_size", sliding_window_size, true);
  // clang-format on
  // -- Parse calibration target
  if (calib_target.load(config_path, "calib_target") != 0) {
    FATAL("Failed to parse calib_target in [%s]!", config_path.c_str());
  }
  // -- Parse camera settings
  for (int cam_idx = 0; cam_idx < 100; cam_idx++) {
    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    if (yaml_has_key(config, cam_str) == 0) {
      continue;
    }

    // Parse
    veci2_t cam_res;
    std::string proj_model;
    std::string dist_model;
    parse(config, cam_str + ".resolution", cam_res);
    parse(config, cam_str + ".proj_model", proj_model);
    parse(config, cam_str + ".dist_model", dist_model);

    if (yaml_has_key(config, cam_str + ".proj_params")) {
      vecx_t k, d;
      parse(config, cam_str + ".proj_params", k);
      parse(config, cam_str + ".dist_params", d);
      add_camera(cam_idx, cam_res.data(), proj_model, dist_model, k, d);
    } else {
      add_camera(cam_idx, cam_res.data(), proj_model, dist_model);
    }
  }
  if (cam_params.size() == 0) {
    FATAL("Failed to parse any camera parameters...");
  }
  // -- Parse camera extrinsics
  for (const auto cam_idx : get_camera_indices()) {
    // // First camera
    // if (cam_idx == 0) {
    //   add_camera_extrinsics(0);
    // }

    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    const std::string key = "T_cam0_" + cam_str;
    if (yaml_has_key(config, key) == 0) {
      // add_camera_extrinsics(cam_idx);
      continue;
    }

    // Extract and add camera extrinsics
    mat4_t T_C0Ci = zeros(4, 4);
    parse(config, key, T_C0Ci);
    cam_exts[cam_idx]->param = tf_vec(T_C0Ci);
  }

  // Load calibration data
  if (yaml_has_key(config, "settings.data_path")) {
    std::string data_path;
    parse(config, "settings.data_path", data_path);

    // Setup camera data paths
    std::map<int, std::string> cam_paths;
    for (size_t cam_idx = 0; cam_idx < cam_params.size(); cam_idx++) {
      const std::string cam_str = "cam" + std::to_string(cam_idx);
      cam_paths[cam_idx] = data_path + "/" + cam_str + "/data";
    }
    if (cam_paths.size() == 0) {
      FATAL("Failed to parse any camera parameters...");
    }

    // Preprocess / load calibration data
    const std::string grids_path = data_path + "/grids0";
    auto data = calib_data_preprocess(calib_target, cam_paths, grids_path);
    if (data.size() == 0) {
      FATAL("Failed to load calib data!");
    }
    add_camera_data(data);
  }

  // AprilGrid
  corners = new fiducial_corners_t(calib_target);
  detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);
}

calib_camera_t::~calib_camera_t() {
  for (auto &[ts, view] : calib_views) {
    delete view;
  }

  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    delete param;
  }

  for (auto &[cam_idx, exts] : cam_exts) {
    UNUSED(cam_idx);
    delete exts;
  }

  if (corners) {
    delete corners;
  }

  for (auto &[ts, pose] : poses) {
    UNUSED(ts);
    delete pose;
  }

  if (solver) {
    delete solver;
  }

  if (marg_residual) {
    delete marg_residual;
  }
}

int calib_camera_t::nb_cameras() const { return cam_params.size(); }

int calib_camera_t::nb_views() const { return calib_views.size(); }

aprilgrids_t calib_camera_t::get_camera_data(const int cam_idx) const {
  aprilgrids_t grids;
  for (const auto &ts : timestamps) {
    if (calib_data.count(ts) == 0) {
      continue;
    }
    if (calib_data.at(ts).count(cam_idx) == 0) {
      continue;
    }
    grids.push_back(calib_data.at(ts).at(cam_idx));
  }
  return grids;
}

std::vector<int> calib_camera_t::get_camera_indices() const {
  std::vector<int> cam_idxs;
  for (const auto &[cam_idx, _] : cam_params) {
    cam_idxs.push_back(cam_idx);
  }
  return cam_idxs;
}

vecx_t calib_camera_t::get_camera_params(const int cam_idx) const {
  return cam_params.at(cam_idx)->param;
}

veci2_t calib_camera_t::get_camera_resolution(const int cam_idx) const {
  auto cam_res = cam_params.at(cam_idx)->resolution;
  return veci2_t{cam_res[0], cam_res[1]};
}

std::string
calib_camera_t::get_camera_projection_model(const int cam_idx) const {
  return cam_params.at(cam_idx)->proj_model;
}

std::string
calib_camera_t::get_camera_distortion_model(const int cam_idx) const {
  return cam_params.at(cam_idx)->dist_model;
}

mat4_t calib_camera_t::get_camera_extrinsics(const int cam_idx) const {
  return cam_exts.at(cam_idx)->tf();
}

std::vector<real_t> calib_camera_t::get_all_reproj_errors() {
  std::vector<real_t> reproj_errors_all;
  for (auto &[ts, view] : calib_views) {
    extend(reproj_errors_all, view->get_reproj_errors());
  }
  return reproj_errors_all;
}

std::map<int, std::vector<real_t>> calib_camera_t::get_reproj_errors() {
  std::map<int, std::vector<real_t>> reproj_errors;

  for (auto &[ts, view] : calib_views) {
    for (auto &cam_idx : view->get_camera_indices()) {
      for (const auto &error : view->get_reproj_errors(cam_idx)) {
        reproj_errors[cam_idx].push_back(error);
      }
    }
  }

  return reproj_errors;
}

std::map<int, vec2s_t> calib_camera_t::get_residuals() {
  std::map<int, vec2s_t> residuals;

  for (auto &[ts, view] : calib_views) {
    for (auto &cam_idx : view->get_camera_indices()) {
      for (const auto &r : view->get_residuals(cam_idx)) {
        residuals[cam_idx].push_back(r);
      }
    }
  }

  return residuals;
}

void calib_camera_t::add_camera_data(const int cam_idx,
                                     const aprilgrids_t &grids) {
  std::vector<real_t> focal_lengths;
  for (const auto &grid : grids) {
    const auto ts = grid.timestamp;
    timestamps.insert(ts);
    calib_data[ts][cam_idx] = grid;

    real_t focal = 0.0;
    if (grid.detected && grid.fully_observable()) {
      if (focal_init(grid, 0, focal) == 0) {
        focal_lengths.push_back(focal);
      }
    }
  }

  if (focal_lengths.size() > 3) {
    focal_length_init[cam_idx] = median(focal_lengths);
    if (cam_params.count(cam_idx)) {
      cam_params[cam_idx]->param[0] = focal_length_init[cam_idx];
      cam_params[cam_idx]->param[1] = focal_length_init[cam_idx];
    }
  }
}

void calib_camera_t::add_camera_data(const std::map<int, aprilgrids_t> &grids) {
  for (const auto &[cam_idx, cam_grids] : grids) {
    add_camera_data(cam_idx, cam_grids);
  }
}

void calib_camera_t::add_camera_data(
    const std::map<int, aprilgrids_t> &train_data,
    const std::map<int, aprilgrids_t> &valid_data) {
  add_camera_data(train_data);
  validation_data = valid_data;
}

void calib_camera_t::add_camera(const int cam_idx,
                                const int cam_res[2],
                                const std::string &proj_model,
                                const std::string &dist_model,
                                const vecx_t &proj_params,
                                const vecx_t &dist_params,
                                const mat4_t &ext,
                                const bool fix_intrinsics,
                                const bool fix_extrinsics) {
  // Camera parameters
  cam_params[cam_idx] = new camera_params_t{cam_idx,
                                            cam_res,
                                            proj_model,
                                            dist_model,
                                            proj_params,
                                            dist_params,
                                            fix_intrinsics};

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = &pinhole_radtan4;
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = &pinhole_equi4;
  } else {
    FATAL("Unsupported [%s]-[%s]!", proj_model.c_str(), dist_model.c_str());
  }

  // Camera extrinsics
  cam_exts[cam_idx] = new extrinsics_t{ext, fix_extrinsics};

  // Add parameter
  solver->add_param(cam_params[cam_idx]);
  solver->add_param(cam_exts[cam_idx]);
}

void calib_camera_t::add_camera(const int cam_idx,
                                const int cam_res[2],
                                const std::string &proj_model,
                                const std::string &dist_model,
                                const mat4_t &ext,
                                const bool fix_intrinsics,
                                const bool fix_extrinsics) {
  // Projection params
  vecx_t proj_params;
  if (proj_model == "pinhole") {
    double fx = pinhole_focal(cam_res[0], 120);
    double fy = pinhole_focal(cam_res[0], 120);
    if (focal_length_init.count(cam_idx)) {
      fx = focal_length_init[cam_idx];
      fy = focal_length_init[cam_idx];
    }
    const double cx = (cam_res[0] - 1) / 2.0;
    const double cy = (cam_res[1] - 1) / 2.0;
    proj_params.resize(4);
    proj_params << fx, fy, cx, cy;
  } else {
    FATAL("Unsupported [%s]!", proj_model.c_str());
  }

  // Distortion params
  vecx_t dist_params;
  if (dist_model == "radtan4" || dist_model == "equi4") {
    dist_params = zeros(4, 1);
  } else {
    FATAL("Unsupported [%s]!", dist_model.c_str());
  }

  // Add camera
  add_camera(cam_idx,
             cam_res,
             proj_model,
             dist_model,
             proj_params,
             dist_params,
             ext,
             fix_intrinsics,
             (cam_idx == 0 || fix_extrinsics));
}

bool calib_camera_t::add_measurement(const timestamp_t ts,
                                     const int cam_idx,
                                     const cv::Mat &cam_img) {
  // Add image to image buffer
  img_buf[cam_idx] = {ts, cam_img};

  // Make sure timestamps in in image buffer all the same
  bool ready = true;
  for (auto &[cam_idx, data] : img_buf) {
    const auto img_ts = data.first;
    if (ts > img_ts) {
      ready = false;
    }
  }
  if (ready == false) {
    return false;
  }

  // Detect AprilGrids
  grid_buf.clear();
  grid_buf = detector->detect(img_buf);

  return true;
}

void calib_camera_t::add_pose(const timestamp_t ts,
                              const std::map<int, aprilgrid_t> &cam_grids,
                              const bool fixed) {
  // Pre-check
  if (poses.count(ts) > 0) {
    FATAL("Pose already exists! Implementation Error!");
    return;
  }

  // Find the AprilGrid that has the most observations at timestamp ts
  int best_cam_idx = 0;
  aprilgrid_t best_grid;
  for (const auto &[cam_idx, grid] : cam_grids) {
    if ((grid.nb_detections > best_grid.nb_detections) &&
        cam_exts.count(cam_idx)) {
      best_cam_idx = cam_idx;
      best_grid = grid;
    }
  }

  // Estimate relative pose T_C0F
  const int cam_idx = best_cam_idx;
  const auto cam_geom = cam_geoms[cam_idx];
  const auto param = cam_params[cam_idx]->param;
  const auto res = cam_params[cam_idx]->resolution;
  mat4_t T_CiF;
  if (best_grid.estimate(cam_geom, res, param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  // Add relative pose T_C0F
  const mat4_t T_C0Ci = get_camera_extrinsics(cam_idx);
  const mat4_t T_C0F = T_C0Ci * T_CiF;
  poses[ts] = new pose_t{ts, T_C0F, fixed};
  solver->add_param(poses[ts]);
}

bool calib_camera_t::add_view(const std::map<int, aprilgrid_t> &cam_grids) {
  // Check if AprilGrid was detected
  bool detected = false;
  timestamp_t ts = 0;
  for (const auto &[cam_idx, grid] : cam_grids) {
    if (grid.detected) {
      detected = true;
      ts = grid.timestamp;
      break;
    }
  }
  if (detected == false) {
    return false;
  }

  // Add to calib data if it does not exist
  if (calib_data.count(ts) == 0) {
    for (const auto &[cam_idx, grid] : cam_grids) {
      if (grid.detected) {
        calib_data[ts][cam_idx] = grid;
      }
    }
  }

  // Add pose
  if (poses.count(ts) == 0) {
    add_pose(ts, cam_grids);
  }
  if (solver->has_param(poses[ts]) == false) {
    solver->add_param(poses[ts]);
  }

  // Add calibration view
  calib_view_timestamps.push_back(ts);
  calib_views[ts] = new calib_view_t{ts,
                                     cam_grids,
                                     corners,
                                     solver,
                                     &cam_geoms,
                                     &cam_params,
                                     &cam_exts,
                                     poses[ts]};

  return true;
}

bool calib_camera_t::add_nbv_view(const std::map<int, aprilgrid_t> &cam_grids) {
  // Add view
  if (add_view(cam_grids) == false) {
    return false;
  }

  // Keep track of initial values - incase view is rejected
  _cache_estimates();

  // Solve with new view
  solver->solve(5);

  // Calculate information gain
  const timestamp_t ts = calib_view_timestamps.back();
  real_t info_kp1 = 0.0;
  if (_calc_info(&info_kp1) != 0) {
    _restore_estimates();
    remove_view(ts);
    return false;
  }
  const real_t info_gain = 0.5 * (info_k - info_kp1);
  // printf("info_gain: %f, info_k: %f, info_kp1: %f\n",
  //        info_gain,
  //        info_k,
  //        info_kp1);

  // Remove view?
  if (info_gain < info_gain_threshold) {
    _restore_estimates();
    remove_view(ts);
    _track_estimates(ts, false);
    return false;
  }

  // Update
  info_k = info_kp1;
  _track_estimates(ts, true);

  return true;
}

void calib_camera_t::remove_view(const timestamp_t ts) {
  // Remove pose
  if (poses.count(ts)) {
    auto pose_ptr = poses[ts];
    if (solver->has_param(pose_ptr)) {
      solver->remove_param(pose_ptr);
    }
    poses.erase(ts);
    delete pose_ptr;
  }

  // Remove view
  if (calib_views.count(ts)) {
    auto view_it = calib_views.find(ts);
    delete view_it->second;
    calib_views.erase(view_it);

    auto ts_it = std::find(calib_view_timestamps.begin(),
                           calib_view_timestamps.end(),
                           ts);
    calib_view_timestamps.erase(ts_it);
  }
}

void calib_camera_t::remove_all_views() {
  timestamps_t view_timestamps;
  for (const auto &[ts, _] : calib_views) {
    UNUSED(_);
    view_timestamps.push_back(ts);
  }
  for (const auto ts : view_timestamps) {
    remove_view(ts);
  }
  removed_outliers = 0;

  // Assert
  if (calib_views.size() != 0) {
    FATAL("calib_views.size() != 0");
  }
  if (solver->num_residuals() != 0) {
    FATAL("solver->num_residuals() != 0");
  }
}

void calib_camera_t::marginalize() {
  // Mark the pose to be marginalized
  auto marg_ts = calib_view_timestamps[0];
  auto view = calib_views[marg_ts];
  poses[marg_ts]->marginalize = true;

  // Form new marg_residual_t
  if (marg_residual == nullptr) {
    // Initialize first marg_residual_t
    marg_residual = new marg_residual_t();

  } else {
    // Add previous marg_residual_t to new
    auto new_marg_residual = new marg_residual_t();
    new_marg_residual->add(marg_residual);

    // Delete old marg_residual_t
    solver->remove_residual(marg_residual);

    // Point to new marg_residual_t
    marg_residual = new_marg_residual;
  }

  // Marginalize view
  view->marginalize(marg_residual);

  // Remove view
  remove_view(marg_ts);
}

int calib_camera_t::find_nbv(const std::map<int, mat4s_t> &nbv_poses,
                             int &cam_idx,
                             int &nbv_idx) {
  // Pre-check
  if (nbv_poses.size() == 0) {
    FATAL("NBV poses empty!?");
  }
  if (nb_views() == 0) {
    FATAL("Calibration problem is empty!?");
  }

  // Current info
  real_t info_k = 0.0;
  if (_calc_info(&info_k) != 0) {
    return -1;
  }

  // Find NBV
  const timestamp_t last_ts = *timestamps.rbegin(); // std::set is orderd
  const timestamp_t nbv_ts = last_ts + 1;
  int best_cam = 0;
  int best_idx = 0;
  real_t best_info = 0;

  int total_nbv_poses = 0;
  for (const auto &[nbv_cam_idx, nbv_cam_poses] : nbv_poses) {
    total_nbv_poses += nbv_cam_poses.size();
  }
  progressbar bar(total_nbv_poses);
  printf("Finding NBV: ");

  for (const auto &[nbv_cam_idx, nbv_cam_poses] : nbv_poses) {
    for (size_t i = 0; i < nbv_cam_poses.size(); i++) {
      // Add NBV pose
      const mat4_t T_FC0 = nbv_cam_poses.at(i);
      const mat4_t T_C0F = tf_inv(T_FC0);
      if (poses.count(nbv_ts) == 0) {
        poses[nbv_ts] = new pose_t{nbv_ts, T_C0F};
        solver->add_param(poses[nbv_ts]);
      }

      // Add NBV view
      std::map<int, aprilgrid_t> cam_grids;
      for (const auto cam_idx : get_camera_indices()) {
        const mat4_t T_C0Ci = cam_exts[cam_idx]->tf();
        cam_grids[cam_idx] = nbv_target_grid(calib_target,
                                             cam_geoms[cam_idx],
                                             cam_params[cam_idx],
                                             nbv_ts,
                                             T_FC0 * T_C0Ci);
      }
      add_view(cam_grids);

      // Evaluate NBV
      real_t info_kp1;
      if (_calc_info(&info_kp1) == 0 && info_kp1 < best_info) {
        best_cam = nbv_cam_idx;
        best_idx = i;
        best_info = info_kp1;
      }

      // Remove view and update
      remove_view(nbv_ts);
      bar.update();
    }
  }
  printf("\n");

  // Return
  cam_idx = best_cam;
  nbv_idx = best_idx;
  const auto info_gain = 0.5 * (info_k - best_info);
  if (info_gain < info_gain_threshold) {
    return -1;
  }

  return 0;
}

void calib_camera_t::_initialize_intrinsics() {
  // Lambda function to extract camera data
  auto extract = [](const camera_data_t &calib_data, const int target_cam_idx) {
    camera_data_t cam_data;
    for (auto &[ts, cam_grids] : calib_data) {
      for (const auto &[cam_idx, grid] : cam_grids) {
        if (cam_idx == target_cam_idx) {
          cam_data[ts] = {{cam_idx, grid}};
        }
      }
    }

    return cam_data;
  };

  // Initialize camera intrinsics
  for (const auto &cam_idx : get_camera_indices()) {
    if (verbose) {
      printf("Initializing cam%d intrinsics ...\n", cam_idx);
    }

    // Fix camera extrinsics
    cam_exts[cam_idx]->fixed = true;
    printf("cam%d ", cam_idx);
    print_vector("initial initial params", cam_params[cam_idx]->param);
    printf("\n");

    // Initialize camera
    std::map<timestamp_t, pose_t *> init_poses;
    calib_initialize(calib_target,
                     extract(calib_data, cam_idx),
                     solver_type,
                     cam_geoms,
                     cam_params,
                     cam_exts,
                     init_poses);
    printf("Initialized to:\n");
    print_camera_params(stdout, cam_idx, cam_params[cam_idx]);
    printf("\n");

    // Clean up
    for (auto &[ts, pose] : init_poses) {
      UNUSED(ts);
      delete pose;
    }

    // Un-fix camera extrinsics
    if (cam_idx != 0) {
      cam_exts[cam_idx]->fixed = false;
    }
  }
}

void calib_camera_t::_initialize_extrinsics() {
  if (nb_cameras() == 1) {
    return;
  }

  if (verbose) {
    printf("Initializing extrinsics ...\n");
  }

  // Initialize camera extrinsics using solve-pnp
  camchain_t camchain{calib_data, cam_geoms, cam_params};
  for (const auto cam_idx : get_camera_indices()) {
    if (get_camera_data(cam_idx).size() == 0) {
      FATAL("No calibration data for cam%d?", cam_idx);
    }

    mat4_t T_C0Ci;
    if (camchain.find(0, cam_idx, T_C0Ci) != 0) {
      FATAL("Failed to initialze T_C0C%d", cam_idx);
    }
    cam_exts[cam_idx]->set_tf(T_C0Ci);
  }

  // Print initial extrinsics
  for (const auto cam_idx : get_camera_indices()) {
    printf("cam%d_", cam_idx);
    print_vector("exts", cam_exts[cam_idx]->param);
  }
  printf("\n");

  // Refine camera extrinsics via joint-optimization
  std::map<timestamp_t, pose_t *> init_poses;
  calib_initialize(calib_target,
                   calib_data,
                   solver_type,
                   cam_geoms,
                   cam_params,
                   cam_exts,
                   init_poses);

  // Print optimized initial extrinsics
  for (const auto cam_idx : get_camera_indices()) {
    printf("cam%d_", cam_idx);
    print_vector("exts", cam_exts[cam_idx]->param);
  }
  printf("\n");

  // Clean up
  for (auto &[ts, pose] : init_poses) {
    UNUSED(ts);
    delete pose;
  }
  init_poses.clear();
}

int calib_camera_t::_filter_all_views() {
  int removed = 0;

  // Calculate reprojection threshold on a per-camera basis
  std::map<int, vec2_t> cam_thresholds;
  std::map<int, vec2s_t> cam_residuals = get_residuals();
  for (const auto cam_idx : get_camera_indices()) {
    const vec2s_t cam_r = cam_residuals[cam_idx];
    const vec2_t cam_stddev = stddev(cam_r);
    cam_thresholds[cam_idx] = outlier_threshold * cam_stddev;
  }

  // Filter views
  for (auto &[ts, view] : calib_views) {
    removed += view->filter_view(cam_thresholds);
  }

  return removed;
}

void calib_camera_t::_cache_estimates() {
  poses_tmp.clear();
  cam_params_tmp.clear();
  cam_exts_tmp.clear();

  for (const auto &[ts, pose] : poses) {
    poses_tmp[ts] = pose->param;
  }
  for (const auto cam_idx : get_camera_indices()) {
    cam_params_tmp[cam_idx] = cam_params[cam_idx]->param;
    cam_exts_tmp[cam_idx] = cam_exts[cam_idx]->param;
  }
}

void calib_camera_t::_restore_estimates() {
  for (const auto &[ts, pose] : poses) {
    if (poses_tmp.count(ts)) {
      pose->param = poses_tmp[ts];
    }
  }
  for (const auto cam_idx : get_camera_indices()) {
    if (cam_params_tmp.count(cam_idx)) {
      cam_params[cam_idx]->param = cam_params_tmp[cam_idx];
    }
    if (cam_exts_tmp.count(cam_idx)) {
      cam_exts[cam_idx]->param = cam_exts_tmp[cam_idx];
    }
  }

  poses_tmp.clear();
  cam_params_tmp.clear();
  cam_exts_tmp.clear();
}

int calib_camera_t::_calc_info(real_t *info) {
  // Form parameter vector
  std::vector<param_t *> params;
  for (const auto &[cam_idx, cam_param] : cam_params) {
    if (cam_param->fixed == false) {
      params.push_back(cam_param);
    }
  }
  for (const auto &[cam_idx, cam_ext] : cam_exts) {
    if (cam_ext->fixed == false) {
      params.push_back(cam_ext);
    }
  }

  // Estimate the determinant of the marginal covariance matrix
  real_t covar_det = 0.0;
  if (solver->estimate_log_covariance_determinant(params, covar_det) != 0) {
    return -1;
  }
  if (std::isnan(std::abs(covar_det))) {
    return -1;
  }
  *info = covar_det / log(2.0);

  return 0;
}

int calib_camera_t::_remove_outliers(const bool filter_all) {
  // Make a copy of the timestamps
  timestamps_t view_timestamps;
  if (filter_all) {
    // Filter all views - in reverse order
    for (int k = calib_view_timestamps.size() - 1; k >= 0; k--) {
      view_timestamps.push_back(calib_view_timestamps[k]);
    }
  } else {
    // Only filter last view
    const auto last_ts = calib_view_timestamps.back();
    if (filtered_timestamps.count(last_ts) == 0) {
      view_timestamps.push_back(last_ts);
      filtered_timestamps.insert(last_ts);
    }
  }

  // Iterate through views in reverse
  if (filter_all) {
    printf("\nFiltering all views: ");
  }

  int nb_views_removed = 0;
  int nb_outliers = 0;
  for (const auto ts : view_timestamps) {
    // Calculate reprojection threshold on a per-camera basis
    std::map<int, vec2_t> cam_thresholds;
    std::map<int, vec2s_t> cam_residuals = get_residuals();
    for (const auto cam_idx : get_camera_indices()) {
      const vec2s_t cam_r = cam_residuals[cam_idx];
      const vec2_t cam_stddev = stddev(cam_r);
      cam_thresholds[cam_idx] = outlier_threshold * cam_stddev;
    }
    if (filter_all) {
      printf(".");
      fflush(stdout);
    }

    // Cache estimates
    _cache_estimates();

    // Filter view
    auto &view = calib_views[ts];
    const auto grids_cache = view->grids;
    int removed = view->filter_view(cam_thresholds);
    if (removed == 0) {
      continue;
    }

    // Get info with filtered view
    solver->solve(5);
    real_t info_before;
    if (_calc_info(&info_before) != 0) {
      printf("Failed to evaluate info!");
      continue;
    }

    // Remove view
    const auto grids = view->grids; // Make a copy of the grids
    remove_view(ts);
    solver->solve(5);

    // Get info with filtered view removed
    real_t info_after;
    if (_calc_info(&info_after) != 0) {
      printf("Failed to evaluate info!");
      add_view(grids_cache);
      _restore_estimates();
      continue;
    }

    // Add the view back?
    const real_t info_gain = 0.5 * (info_after - info_before);
    if (info_gain > info_gain_threshold) {
      // Add view back and restore estimates
      add_view(grids);
      removed_outliers += removed;
      nb_outliers += removed;
    } else {
      nb_views_removed++;
    }

    // Restore estimates
    _restore_estimates();
  }

  // Print stats
  if (filter_all) {
    printf("\n[Stats] kept views: %d, removed views: %d\n\n",
           nb_views(),
           nb_views_removed);
  }

  // Update info_k
  if (view_timestamps.size() && _calc_info(&info_k) != 0) {
    return nb_outliers;
  }

  return nb_outliers;
}

void calib_camera_t::_track_estimates(const timestamp_t ts,
                                      const bool view_accepted) {
  if (nbv_timestamps.count(ts)) {
    return;
  }

  for (auto cam_idx : get_camera_indices()) {
    cam_estimates[ts][cam_idx] = get_camera_params(cam_idx);
    exts_estimates[ts][cam_idx] = get_camera_extrinsics(cam_idx);
  }

  const auto cost_init = solver->initial_cost;
  const auto cost_final = solver->final_cost;
  const auto num_iter = solver->num_iterations;
  nbv_timestamps.insert(ts);
  nbv_costs[ts] = {cost_init, cost_final, num_iter};
  nbv_reproj_errors[ts] = get_reproj_errors();
  nbv_accepted[ts] = view_accepted;
}

void calib_camera_t::_print_stats(const size_t ts_idx,
                                  const size_t nb_timestamps) {
  // Show general stats
  const real_t progress = ((real_t)ts_idx / nb_timestamps) * 100.0;
  const auto reproj_errors_all = get_all_reproj_errors();
  printf("[%.2f%%] ", progress);
  printf("nb_views: %d  ", nb_views());
  printf("reproj_error: %.4f  ", mean(reproj_errors_all));
  printf("info_k: %.4f  ", info_k);
  printf("\n");
  // printf("\n");

  // Show current calibration estimates
  if ((ts_idx % 10) == 0) {
    printf("\n");
    const mat4_t T_BC0 = get_camera_extrinsics(0);
    const mat4_t T_C0B = tf_inv(T_BC0);
    for (const auto cam_idx : get_camera_indices()) {
      const auto param = cam_params[cam_idx]->param;
      printf("cam%d_params: %s\n", cam_idx, vec2str(param).c_str());
    }
    for (const auto cam_idx : get_camera_indices()) {
      if (cam_idx == 0) {
        continue;
      }
      const mat4_t T_BCi = cam_exts[cam_idx]->tf();
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      printf("cam%d_exts: %s\n", cam_idx, vec2str(tf_vec(T_C0Ci)).c_str());
    }
    printf("\n");
  }
}

void calib_camera_t::_solve_batch() {
  for (const auto &ts : timestamps) {
    add_view(calib_data[ts]);
  }
  solver->solve(50, true, 1);
}

void calib_camera_t::_solve_inc() {
  // Solve
  size_t ts_idx = 0;

  for (const auto &ts : timestamps) {
    // Add view
    if (add_view(calib_data[ts]) == false) {
      continue;
    }

    // Solve
    if (static_cast<int>(calib_views.size()) >= sliding_window_size) {
      solver->solve();
      marginalize();
    }

    // Print stats
    if (verbose) {
      _print_stats(ts_idx, timestamps.size());
    }

    // Update
    ts_idx++;
  }

  if (verbose) {
    show_results();
  }
}

void calib_camera_t::_solve_nbv() {
  // Pre-process timestamps
  timestamps_t nbv_timestamps;
  for (const auto ts : timestamps) {
    for (const auto &[cam_idx, grid] : calib_data[ts]) {
      if (grid.detected) {
        nbv_timestamps.push_back(ts);
        break;
      }
    }
  }
  if (nbv_timestamps.size() == 0) {
    FATAL("Implementation Error: No views to process?");
  }

  // Shuffle timestamps
  if (enable_shuffle_views) {
    std::shuffle(nbv_timestamps.begin(), nbv_timestamps.end(), calib_rng);
  }

  // NBV
  size_t ts_idx = 0;
  for (const auto &ts : nbv_timestamps) {
    // Add view
    if (add_nbv_view(calib_data[ts])) {
      // Remove outliers
      if (enable_nbv_filter && nb_views() >= min_nbv_views) {
        _remove_outliers(filter_all);
        filter_all = false;
      }

      // // Marginalize oldest view
      // if (enable_marginalization &&
      //     (calib_views.size() > (size_t)sliding_window_size)) {
      //   marginalize();
      // }
    }

    // Print stats
    if (verbose) {
      _print_stats(ts_idx++, nbv_timestamps.size());
    }
  }

  // Final outlier rejection, then batch solve
  if (enable_outlier_filter) {
    if (verbose) {
      printf("Performing Final Outlier Rejection\n");
    }
    removed_outliers += _remove_outliers(true);
    if (verbose) {
      printf("Removed %d outliers\n", removed_outliers);
    }
  }

  // Final Solve
  // removed_outliers = _filter_all_views();
  solver->solve(10, true, 1);
}

void calib_camera_t::solve() {
  // Print Calibration settings
  if (verbose) {
    print_settings(stdout);
  }

  // Initialize camera intrinsics and extrinsics
  _initialize_intrinsics();
  _initialize_extrinsics();
  if (verbose) {
    printf("Initial camera intrinsics and extrinsics:\n");
    print_estimates(stdout, cam_params, cam_exts);
  }

  // Solve
  if (enable_nbv) {
    printf("Solve NBV:\n");
    solver->algorithm_type = "GAUSS-NEWTON";
    _solve_nbv();
  } else {
    printf("Solve Batch:\n");
    _solve_batch();
  }

  // Show results
  if (verbose) {
    show_results();
  }
}

void calib_camera_t::print_settings(FILE *out) {
  // clang-format off
  fprintf(out, "settings:\n");
  fprintf(out, "  verbose: %s\n", verbose ? "true" : "false");
  fprintf(out, "  solver: %s\n", solver_type.c_str());
  fprintf(out, "  max_num_threads: %d\n", max_num_threads);
  fprintf(out, "  enable_nbv: %s\n", enable_nbv ? "true" : "false");
  fprintf(out, "  enable_shuffle_views: %s\n", enable_shuffle_views ? "true" : "false");
  fprintf(out, "  enable_nbv_filter: %s\n", enable_nbv_filter ? "true" : "false");
  fprintf(out, "  enable_outlier_filter: %s\n", enable_outlier_filter ? "true" : "false");
  fprintf(out, "  enable_marginalization: %s\n", enable_marginalization ? "true" : "false");
  fprintf(out, "  min_nbv_views: %d\n", min_nbv_views);
  fprintf(out, "  outlier_threshold: %f\n", outlier_threshold);
  fprintf(out, "  info_gain_threshold: %f\n", info_gain_threshold);
  fprintf(out, "  sliding_window_size: %d\n", sliding_window_size);
  fprintf(out, "\n");
  // clang-format on
}

void calib_camera_t::print_metrics(
    FILE *out,
    const std::map<int, std::vector<real_t>> &reproj_errors,
    const std::vector<real_t> &reproj_errors_all) {
  fprintf(out, "total_reproj_error:\n");
  fprintf(out, "  rmse:   %.4f # [px]\n", rmse(reproj_errors_all));
  fprintf(out, "  mean:   %.4f # [px]\n", mean(reproj_errors_all));
  fprintf(out, "  median: %.4f # [px]\n", median(reproj_errors_all));
  fprintf(out, "  stddev: %.4f # [px]\n", stddev(reproj_errors_all));
  fprintf(out, "\n");

  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    fprintf(out, "%s_reproj_error:\n", cam_str.c_str());
    fprintf(out, "  rmse:   %.4f # [px]\n", rmse(cam_errors));
    fprintf(out, "  mean:   %.4f # [px]\n", mean(cam_errors));
    fprintf(out, "  median: %.4f # [px]\n", median(cam_errors));
    fprintf(out, "  stddev: %.4f # [px]\n", stddev(cam_errors));
    fprintf(out, "\n");
  }
}

void calib_camera_t::show_results() {
  const auto reproj_errors = get_reproj_errors();
  const auto reproj_errors_all = get_all_reproj_errors();
  printf("Optimization results:\n");
  printf("---------------------\n");
  print_settings(stdout);
  print_metrics(stdout, reproj_errors, reproj_errors_all);
  print_estimates(stdout, cam_params, cam_exts);
}

int calib_camera_t::save_results(const std::string &save_path) {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  const auto reproj_errors = get_reproj_errors();
  const auto reproj_errors_all = get_all_reproj_errors();
  print_settings(outfile);
  print_metrics(outfile, reproj_errors, reproj_errors_all);
  print_calib_target(outfile, calib_target);
  print_estimates(outfile, cam_params, cam_exts);

  // Write estimates to file
  FILE *cam0 = fopen("/tmp/cam0.csv", "w");
  FILE *cam1 = fopen("/tmp/cam1.csv", "w");
  FILE *exts = fopen("/tmp/exts.csv", "w");
  for (const auto ts : nbv_timestamps) {
    auto cam0_param = vec2str(cam_estimates[ts][0], false);
    auto cam1_param = vec2str(cam_estimates[ts][1], false);
    auto exts_param = vec2str(tf_vec(exts_estimates[ts][1]), false);
    fprintf(cam0, "%ld,%s\n", ts, cam0_param.c_str());
    fprintf(cam1, "%ld,%s\n", ts, cam1_param.c_str());
    fprintf(exts, "%ld,%s\n", ts, exts_param.c_str());
  }
  fclose(cam0);
  fclose(cam1);
  fclose(exts);

  // Write convergence to file
  FILE *progress = fopen("/tmp/calib-progress.csv", "w");
  fprintf(progress, "ts,num_iter,cost_start,cost_end,");
  fprintf(progress, "cam0_rmse,cam0_mean,cam0_median,cam0_std,");
  fprintf(progress, "cam1_rmse,cam1_mean,cam1_median,cam1_std,");
  fprintf(progress, "accepted\n");
  for (const auto ts : nbv_timestamps) {
    const auto cost_init = std::get<0>(nbv_costs[ts]);
    const auto cost_final = std::get<1>(nbv_costs[ts]);
    const auto cost_iter = std::get<2>(nbv_costs[ts]);
    const auto reproj_errors = nbv_reproj_errors[ts];
    const auto accepted = nbv_accepted[ts];

    fprintf(progress, "%ld,", ts);
    fprintf(progress, "%d,", cost_iter);
    fprintf(progress, "%f,", cost_init);
    fprintf(progress, "%f,", cost_final);

    if (reproj_errors.count(0)) {
      fprintf(progress, "%f,", rmse(reproj_errors.at(0)));
      fprintf(progress, "%f,", mean(reproj_errors.at(0)));
      fprintf(progress, "%f,", median(reproj_errors.at(0)));
      fprintf(progress, "%f,", stddev(reproj_errors.at(0)));
    } else {
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
    }

    if (reproj_errors.count(1)) {
      fprintf(progress, "%f,", rmse(reproj_errors.at(1)));
      fprintf(progress, "%f,", mean(reproj_errors.at(1)));
      fprintf(progress, "%f,", median(reproj_errors.at(1)));
      fprintf(progress, "%f,", stddev(reproj_errors.at(1)));
    } else {
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
      fprintf(progress, "-1.0,");
    }

    fprintf(progress, "%d\n", accepted);
  }
  fclose(progress);

  return 0;
}

int calib_camera_t::save_estimates(const std::string &save_path) {
  LOG_INFO(KGRN "Saved estimates to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  fprintf(outfile, "#ts,rx,ry,rz,qx,qy,qz,qw\n");
  for (const auto &[ts, pose] : poses) {
    const mat4_t T_C0F = pose->tf();
    const vec3_t r = tf_trans(T_C0F);
    const quat_t q = tf_quat(T_C0F);
    fprintf(outfile, "%ld,", ts);
    fprintf(outfile, "%f,", r.x());
    fprintf(outfile, "%f,", r.y());
    fprintf(outfile, "%f,", r.z());
    fprintf(outfile, "%f,", q.x());
    fprintf(outfile, "%f,", q.y());
    fprintf(outfile, "%f,", q.z());
    fprintf(outfile, "%f\n", q.w());
  }

  return 0;
}

int calib_camera_t::save_stats(const std::string &save_path) {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Header
  fprintf(outfile, "cam_idx,");
  fprintf(outfile, "ts,");
  fprintf(outfile, "tag_id,");
  fprintf(outfile, "corner_idx,");
  fprintf(outfile, "z_x,");
  fprintf(outfile, "z_y,");
  fprintf(outfile, "z_hat_x,");
  fprintf(outfile, "z_hat_y,");
  fprintf(outfile, "rx,");
  fprintf(outfile, "ry\n");

  // Calib views
  for (const auto &[ts, cam_view] : calib_views) {
    for (const auto &[cam_idx, res_fns] : cam_view->res_fns) {
      for (const auto &res_fn : res_fns) {
        const auto corner = res_fn->p_FFi;
        const vec2_t z = res_fn->z;
        const int tag_id = corner->tag_id;
        const int corner_idx = corner->corner_idx;

        vec2_t z_hat = zeros(2, 1);
        vec2_t r = zeros(2, 1);
        if (res_fn->get_residual(z_hat, r) != 0) {
          continue;
        }

        fprintf(outfile, "%d, ", cam_idx);
        fprintf(outfile, "%ld, ", ts);
        fprintf(outfile, "%d, ", tag_id);
        fprintf(outfile, "%d, ", corner_idx);
        fprintf(outfile, "%f, %f, ", z.x(), z.y());
        fprintf(outfile, "%f, %f, ", z_hat.x(), z_hat.y());
        fprintf(outfile, "%f, %f", r.x(), r.y());
        fprintf(outfile, "\n");
      }
    }
  }

  return 0;
}

real_t calib_camera_t::inspect(const std::map<int, aprilgrids_t> &valid_data) {
  // Pre-check
  if (cam_params.size() == 0) {
    FATAL("cam_params.size() == 0");
  } else if (cam_exts.size() == 0) {
    FATAL("cam_exts.size() == 0");
  } else if (cam_exts.size() != cam_params.size()) {
    FATAL("cam_exts.size() != cam_params.size()");
  }

  // Estimate relative poses T_C0F
  std::map<timestamp_t, mat4_t> fiducial_poses;
  for (const auto cam_idx : get_camera_indices()) {
    const auto cam_geom = cam_geoms[cam_idx];
    const auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const auto T_C0Ci = cam_exts[cam_idx]->tf();

    for (size_t k = 0; k < valid_data.at(cam_idx).size(); k++) {
      const auto &grid = valid_data.at(cam_idx)[k];

      // Make sure aprilgrid is detected
      if (grid.detected == false) {
        continue;
      }

      // Check if we already have T_C0F
      if (fiducial_poses.count(grid.timestamp)) {
        continue;
      }

      // Estimate relative pose
      mat4_t T_CiF;
      if (grid.estimate(cam_geom, cam_res, cam_param->param, T_CiF) != 0) {
        continue;
      }

      fiducial_poses[grid.timestamp] = T_C0Ci * T_CiF;
    }
  }

  // Calculate reprojection error
  std::vector<real_t> reproj_errors_all;
  std::map<int, std::vector<real_t>> reproj_errors_cams;
  for (const auto cam_idx : get_camera_indices()) {
    const auto cam_geom = cam_geoms[cam_idx];
    const auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const mat4_t T_CiC0 = tf_inv(cam_exts[cam_idx]->tf());

    for (const auto &grid : valid_data.at(cam_idx)) {
      // Make sure aprilgrid is detected
      if (grid.detected == false) {
        continue;
      }

      // Estimate relative pose
      const timestamp_t ts = grid.timestamp;
      const mat4_t T_C0F = fiducial_poses[ts];
      const mat4_t T_CiF = T_CiC0 * T_C0F;

      // Get AprilGrid measurements
      std::vector<int> tag_ids;
      std::vector<int> corner_idxs;
      vec2s_t kps;
      vec3s_t pts;
      grid.get_measurements(tag_ids, corner_idxs, kps, pts);

      // Add reprojection errors
      for (size_t i = 0; i < tag_ids.size(); i++) {
        const int tag_id = tag_ids[i];
        const int corner_idx = corner_idxs[i];
        const vec2_t z = kps[i];
        const vec3_t p_FFi = grid.object_point(tag_id, corner_idx);
        const vec3_t p_CiFi = tf_point(T_CiF, p_FFi);

        vec2_t z_hat;
        if (cam_geom->project(cam_res, cam_param->param, p_CiFi, z_hat) != 0) {
          continue;
        }
        reproj_errors_cams[cam_idx].push_back((z - z_hat).norm());
        reproj_errors_all.push_back((z - z_hat).norm());
      }
    }
  }

  // Print stats
  if (verbose) {
    printf("\nValidation Statistics:\n");
    printf("----------------------\n");
    print_metrics(stdout, reproj_errors_cams, reproj_errors_all);
  }

  return rmse(reproj_errors_all);
}

} //  namespace yac
