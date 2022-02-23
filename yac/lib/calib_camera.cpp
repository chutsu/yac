#include "calib_camera.hpp"

namespace yac {

// CAMCHAIN ////////////////////////////////////////////////////////////////////

camchain_t::camchain_t(const camera_data_t &cam_data,
                       const std::map<int, camera_geometry_t *> &cam_geoms,
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

        const mat4_t T_CiCj = T_CiF * T_CjF.inverse();
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
  exts[cam_j][cam_i] = T_CiCj.inverse();
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
  T_CiCj = T_CjCi.inverse();

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
                           ceres::Problem *problem_,
                           ceres::LossFunction *loss_,
                           CamIdx2Geometry *cam_geoms_,
                           CamIdx2Parameters *cam_params_,
                           CamIdx2Extrinsics *cam_exts_,
                           pose_t *T_C0F_)
    : ts{ts_}, grids{grids_}, corners{corners_}, problem{problem_}, loss{loss_},
      cam_geoms{cam_geoms_},
      cam_params{cam_params_}, cam_exts{cam_exts_}, T_C0F{T_C0F_} {
  const mat2_t covar = I(2);
  for (const auto &[cam_idx, cam_grid] : grids_) {
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
      auto res_fn = new reproj_error_t(cam_geoms_->at(cam_idx),
                                       cam_params_->at(cam_idx),
                                       cam_exts_->at(cam_idx),
                                       T_C0F_,
                                       p_FFi_,
                                       z,
                                       covar);
      auto res_id = problem->AddResidualBlock(res_fn,
                                              loss,
                                              cam_exts_->at(cam_idx)->data(),
                                              T_C0F_->data(),
                                              p_FFi_->data(),
                                              cam_params->at(cam_idx)->data());
      res_fns[cam_idx].push_back(res_fn);
      res_ids[cam_idx].push_back(res_id);
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
    auto res_ids_it = res_ids[cam_idx].begin();

    while (res_fns_it != res_fns[cam_idx].end()) {
      auto res = *res_fns_it;
      auto res_id = *res_ids_it;

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
        problem->RemoveResidualBlock(res_id);
        delete res;

        res_fns_it = res_fns[cam_idx].erase(res_fns_it);
        res_ids_it = res_ids[cam_idx].erase(res_ids_it);
        nb_outliers++;
      } else {
        ++res_fns_it;
        ++res_ids_it;
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

ceres::ResidualBlockId calib_view_t::marginalize(marg_error_t *marg_error) {
  // Mark relative pose T_C0F to be marginalized
  T_C0F->marginalize = true;

  // Transfer residuals to marginalization error
  for (auto &[cam_idx, cam_residuals] : res_fns) {
    for (auto &res_fn : cam_residuals) {
      marg_error->add(res_fn);
    }
  }
  const auto res_id = marg_error->marginalize(problem);

  // Clear residuals
  res_fns.clear();
  res_ids.clear();
  // ^ Important! we don't want to delete the residual blocks when the view is
  // deconstructed, but rather by adding the residual functions to the
  // marginalization error we pass the ownership to marg_error_t

  return res_id;
}

// CALIB CAMERA ////////////////////////////////////////////////////////////////

calib_camera_t::calib_camera_t(const calib_target_t &calib_target_)
    : calib_target{calib_target_},
      calib_rng(std::chrono::system_clock::now().time_since_epoch().count()) {
  // Ceres-Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);

  // Add fiducial corners to problem
  const int nb_tags = calib_target.tag_rows * calib_target.tag_cols;
  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      auto corner = corners.get_corner(tag_id, corner_idx);
      problem->AddParameterBlock(corner->data(), 3);
      problem->SetParameterBlockConstant(corner->data());
    }
  }

  // AprilGrid detector
  detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);
}

calib_camera_t::calib_camera_t(const std::string &config_path)
    : calib_rng(std::chrono::system_clock::now().time_since_epoch().count()) {
  // Ceres-Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);

  // Load configuration
  config_t config{config_path};
  // -- Parse settings
  // clang-format off
  parse(config, "settings.verbose", verbose);
  parse(config, "settings.max_num_threads", max_num_threads);
  parse(config, "settings.enable_extrinsics_outlier_filter", enable_extrinsics_outlier_filter);
  parse(config, "settings.enable_nbv", enable_nbv);
  parse(config, "settings.enable_shuffle_views", enable_shuffle_views);
  parse(config, "settings.enable_nbv_filter", enable_nbv_filter);
  parse(config, "settings.enable_marginalization", enable_marginalization);
  parse(config, "settings.enable_cross_validation", enable_cross_validation);
  parse(config, "settings.enable_early_stopping", enable_early_stopping);
  parse(config, "settings.min_nbv_views", min_nbv_views);
  parse(config, "settings.outlier_threshold", outlier_threshold);
  parse(config, "settings.info_gain_threshold", info_gain_threshold);
  parse(config, "settings.sliding_window_size", sliding_window_size);
  parse(config, "settings.early_stop_threshold", early_stop_threshold);
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
    // First camera
    if (cam_idx == 0) {
      add_camera_extrinsics(0);
    }

    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    const std::string key = "T_cam0_" + cam_str;
    if (yaml_has_key(config, key) == 0) {
      add_camera_extrinsics(cam_idx);
      continue;
    }

    // Extract and add camera extrinsics
    mat4_t T_C0Ci = zeros(4, 4);
    parse(config, key, T_C0Ci);
    add_camera_extrinsics(cam_idx, T_C0Ci);
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

  // Add fiducial corners to problem
  const int nb_tags = calib_target.tag_rows * calib_target.tag_cols;
  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      auto corner = corners.get_corner(tag_id, corner_idx);
      problem->AddParameterBlock(corner->data(), 3);
      problem->SetParameterBlockConstant(corner->data());
    }
  }

  // AprilGrid detector
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

  for (auto &[ts, pose] : poses) {
    UNUSED(ts);
    delete pose;
  }

  if (loss) {
    delete loss;
  }

  if (problem) {
    delete problem;
  }

  if (marg_error) {
    delete marg_error;
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
                                const bool fixed) {
  // Camera parameters
  cam_params[cam_idx] = new camera_params_t{cam_idx,
                                            cam_res,
                                            proj_model,
                                            dist_model,
                                            proj_params,
                                            dist_params,
                                            fixed};

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = &pinhole_radtan4;
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = &pinhole_equi4;
  } else {
    FATAL("Unsupported [%s]-[%s]!", proj_model.c_str(), dist_model.c_str());
  }

  // Add parameter to problem
  int params_size = proj_params.size() + dist_params.size();
  problem->AddParameterBlock(cam_params[cam_idx]->data(), params_size);
  if (fixed) {
    cam_params[cam_idx]->fixed = true;
    problem->SetParameterBlockConstant(cam_params[cam_idx]->data());
  }
}

void calib_camera_t::add_camera(const int cam_idx,
                                const int cam_res[2],
                                const std::string &proj_model,
                                const std::string &dist_model,
                                const bool fixed) {
  // Projection params
  vecx_t proj_params;
  if (proj_model == "pinhole") {
    double fx = pinhole_focal(cam_res[0], 120);
    double fy = pinhole_focal(cam_res[0], 120);
    if (focal_length_init.count(cam_idx)) {
      fx = focal_length_init[cam_idx];
      fy = focal_length_init[cam_idx];
    }
    const double cx = cam_res[0] / 2.0;
    const double cy = cam_res[1] / 2.0;
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
             fixed);
}

void calib_camera_t::add_camera_extrinsics(const int cam_idx,
                                           const mat4_t &ext,
                                           const bool fixed) {
  if (cam_exts.count(cam_idx) == 0) {
    cam_exts[cam_idx] = new extrinsics_t{ext};
  }

  problem->AddParameterBlock(cam_exts[cam_idx]->data(), 7);
  problem->SetParameterization(cam_exts[cam_idx]->data(), &pose_plus);
  if (cam_idx == 0 || fixed) {
    cam_exts[cam_idx]->fixed = true;
    problem->SetParameterBlockConstant(cam_exts[cam_idx]->data());
  }
}

void calib_camera_t::add_pose(const timestamp_t ts,
                              const std::map<int, aprilgrid_t> &cam_grids,
                              const bool fixed) {
  // Pre-check
  if (poses.count(ts) > 0) {
    // FATAL("Implementation Error!");
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
  poses[ts] = new pose_t{ts, T_C0F};
  problem->AddParameterBlock(poses[ts]->data(), 7);
  problem->SetParameterization(poses[ts]->data(), &pose_plus);
  if (fixed) {
    problem->SetParameterBlockConstant(poses[ts]->data());
  }
}

bool calib_camera_t::add_view(const std::map<int, aprilgrid_t> &cam_grids,
                              const bool force) {
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

  // Add pose
  if (poses.count(ts) == 0) {
    add_pose(ts, cam_grids);
  }

  // Add calibration view
  calib_view_timestamps.push_back(ts);
  calib_views[ts] = new calib_view_t{ts,
                                     cam_grids,
                                     &corners,
                                     problem,
                                     loss,
                                     &cam_geoms,
                                     &cam_params,
                                     &cam_exts,
                                     poses[ts]};
  if (force) {
    return true;
  }

  // Keep track of initial values
  _cache_estimates();

  // Solve with new view
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 10;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

  // Calculate information gain
  real_t info_kp1 = 0.0;
  if (_calc_info(&info_kp1) != 0) {
    _restore_estimates();
    return -1;
  }
  const real_t info_gain = 0.5 * (info_k - info_kp1);

  ceres::Solve(options, problem, &summary);

  // Remove view?
  if (info_gain < info_gain_threshold) {
    _restore_estimates();
    remove_view(ts);

    if (nbv_timestamps.count(ts) == 0) {
      cam_estimates[ts][0] = get_camera_params(0);
      cam_estimates[ts][1] = get_camera_params(1);
      exts_estimates[ts][1] = get_camera_extrinsics(1);

      nbv_timestamps.insert(ts);
      nbv_costs[ts] = {summary.initial_cost,
                       summary.final_cost,
                       summary.num_successful_steps};
      nbv_reproj_errors[ts] = get_reproj_errors();
      nbv_accepted[ts] = false;
    }

    return false;
  }

  // Double check with validation set
  if (enable_cross_validation) {
    // Make sure we actually have validation data
    if (validation_data.size() == 0) {
      FATAL("Validation data is not set!");
    }

    // Get validation error
    calib_camera_t valid{calib_target};
    valid.verbose = false;
    for (const auto cam_idx : get_camera_indices()) {
      const auto cam_param = cam_params[cam_idx];
      const int *cam_res = cam_param->resolution;
      const std::string &proj_model = cam_param->proj_model;
      const std::string &dist_model = cam_param->dist_model;
      const vecx_t &k = cam_param->proj_params();
      const vecx_t &d = cam_param->dist_params();
      valid.add_camera(cam_idx, cam_res, proj_model, dist_model, k, d, true);
      valid.add_camera_extrinsics(cam_idx, cam_exts[cam_idx]->tf());
    }

    // Accept or reject view
    const auto valid_error_kp1 = valid.inspect(validation_data);
    if (fltcmp(valid_error_k, 0.0) == 0 || (valid_error_kp1 < valid_error_k)) {
      valid_error_k = valid_error_kp1;
    } else {
      _restore_estimates();
      remove_view(ts);

      if (nbv_timestamps.count(ts) == 0) {
        cam_estimates[ts][0] = get_camera_params(0);
        cam_estimates[ts][1] = get_camera_params(1);
        exts_estimates[ts][1] = get_camera_extrinsics(1);

        nbv_timestamps.insert(ts);
        nbv_costs[ts] = {summary.initial_cost,
                         summary.final_cost,
                         summary.num_successful_steps};
        nbv_reproj_errors[ts] = get_reproj_errors();
        nbv_accepted[ts] = false;
      }

      return false;
    }
  }

  // Update
  info_k = info_kp1;

  if (nbv_timestamps.count(ts) == 0) {
    cam_estimates[ts][0] = get_camera_params(0);
    cam_estimates[ts][1] = get_camera_params(1);
    exts_estimates[ts][1] = get_camera_extrinsics(1);

    nbv_timestamps.insert(ts);
    nbv_costs[ts] = {summary.initial_cost,
                     summary.final_cost,
                     summary.num_successful_steps};
    nbv_reproj_errors[ts] = get_reproj_errors();
    nbv_accepted[ts] = true;
  }

  return true;
}

void calib_camera_t::remove_view(const timestamp_t ts) {
  // Remove pose
  if (poses.count(ts)) {
    auto pose_ptr = poses[ts];
    if (problem->HasParameterBlock(pose_ptr->param.data())) {
      problem->RemoveParameterBlock(pose_ptr->param.data());
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

  problem_init = false;
  removed_outliers = 0;
}

void calib_camera_t::marginalize() {
  // Mark the pose to be marginalized
  auto marg_ts = calib_view_timestamps[0];
  auto view = calib_views[marg_ts];
  poses[marg_ts]->marginalize = true;

  // Form new marg_error_t
  if (marg_error == nullptr) {
    // Initialize first marg_error_t
    marg_error = new marg_error_t();

  } else {
    // Add previous marg_error_t to new
    auto new_marg_error = new marg_error_t();
    new_marg_error->add(marg_error);

    // Delete old marg_error_t
    problem->RemoveResidualBlock(marg_error_id);

    // Point to new marg_error_t
    marg_error = new_marg_error;
  }

  // Marginalize view
  marg_error_id = view->marginalize(marg_error);

  // Remove view
  remove_view(marg_ts);
}

real_t calib_camera_t::_estimate_calib_info() {
  // Track parameters
  std::vector<calib_error_t *> res_blocks;
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> marg_param_ptrs;
  std::vector<param_t *> remain_camera_param_ptrs;
  std::vector<param_t *> remain_extrinsics_ptrs;
  std::vector<param_t *> remain_fiducial_ptrs;
  for (auto &[ts, view] : calib_views) {
    for (auto &[cam_idx, cam_residuals] : view->res_fns) {
      for (auto &res_block : cam_residuals) {
        for (auto param_block : res_block->param_blocks) {
          // Seen parameter block already
          if (params_seen.count(param_block)) {
            continue;
          }

          // Keep track of parameter block
          if (param_block->type == "camera_params_t") {
            remain_camera_param_ptrs.push_back(param_block);
          } else if (param_block->type == "extrinsics_t") {
            remain_extrinsics_ptrs.push_back(param_block);
          } else if (param_block->type == "fiducial_t") {
            remain_fiducial_ptrs.push_back(param_block);
          } else {
            marg_param_ptrs.push_back(param_block);
          }
          params_seen[param_block] = true;
        }
        res_blocks.push_back(res_block);
      }
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t m = 0;
  size_t r = 0;
  size_t H_idx = 0;
  std::map<param_t *, int> param_index;
  // -- Column indices for parameter blocks to be marginalized
  for (const auto &param_block : marg_param_ptrs) {
    param_index.insert({param_block, H_idx});
    H_idx += param_block->local_size;
    m += param_block->local_size;
  }
  // -- Column indices for parameter blocks to remain
  std::vector<std::vector<param_t *> *> remain_params = {
      &remain_fiducial_ptrs,
      &remain_extrinsics_ptrs,
      &remain_camera_param_ptrs,
  };
  std::vector<param_t *> remain_param_ptrs;
  for (const auto &param_ptrs : remain_params) {
    for (const auto &param_block : *param_ptrs) {
      if (param_block->fixed) {
        continue;
      }
      param_index.insert({param_block, H_idx});
      H_idx += param_block->local_size;
      r += param_block->local_size;
      remain_param_ptrs.push_back(param_block);
    }
  }

  // Form the H and b. Left and RHS of Gauss-Newton.
  // H = J.T * J
  // b = -J.T * e
  const auto local_size = m + r;
  matx_t H = zeros(local_size, local_size);
  vecx_t b = zeros(local_size, 1);

  for (calib_error_t *res_block : res_blocks) {
    // Setup parameter data
    std::vector<double *> param_ptrs;
    for (auto param_block : res_block->param_blocks) {
      param_ptrs.push_back(param_block->data());
    }

    // Setup Jacobians data
    const int r_size = res_block->num_residuals();
    std::vector<matx_row_major_t> jacs;
    std::vector<double *> jac_ptrs;
    for (auto param_block : res_block->param_blocks) {
      jacs.push_back(zeros(r_size, param_block->global_size));
      jac_ptrs.push_back(jacs.back().data());
    }

    // Setup Min-Jacobians data
    std::vector<matx_row_major_t> min_jacs;
    std::vector<double *> min_jac_ptrs;
    for (auto param_block : res_block->param_blocks) {
      min_jacs.push_back(zeros(r_size, param_block->local_size));
      min_jac_ptrs.push_back(min_jacs.back().data());
    }

    // Evaluate residual block
    vecx_t r = zeros(r_size, 1);
    res_block->EvaluateWithMinimalJacobians(param_ptrs.data(),
                                            r.data(),
                                            jac_ptrs.data(),
                                            min_jac_ptrs.data());

    // Fill Hessian
    for (size_t i = 0; i < res_block->param_blocks.size(); i++) {
      const auto &param_i = res_block->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }
      const int idx_i = param_index[param_i];
      const int size_i = param_i->local_size;
      const matx_t J_i = min_jacs[i];

      for (size_t j = i; j < res_block->param_blocks.size(); j++) {
        const auto &param_j = res_block->param_blocks[j];
        if (param_j->fixed) {
          continue;
        }
        const int idx_j = param_index[param_j];
        const int size_j = param_j->local_size;
        const matx_t J_j = min_jacs[j];

        if (i == j) {
          // Form diagonals of H
          H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
        } else {
          // Form off-diagonals of H
          // clang-format off
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() * J_j).transpose();
          // clang-format on
        }
      }

      // RHS of Gauss Newton (i.e. vector b)
      b.segment(idx_i, size_i) += -J_i.transpose() * r;
    }
  }

  // Marginalize
  // -- Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // entry by its reciprocal, leaving the zeros in place, and transposing
  // the resulting matrix.
  // clang-format off
  matx_t Hmm = H.block(0, 0, m, m);
  Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals_inv = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array().inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  // clang-format on
  // -- Calculate Schur's complement
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const vecx_t bmm = b.segment(0, m);
  const vecx_t brr = b.segment(m, r);
  const matx_t H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  const matx_t b_marg = brr - Hrm * Hmm_inv * bmm;

  // Calculate det(inv(H_marg))
  // clang-format off
  {
    const Eigen::SelfAdjointEigenSolver<matx_t> eig(H_marg);
    const auto s = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array(), 0);
    return -1.0 * s.log().sum() / log(2.0);
  }
  // clang-format on
}

int calib_camera_t::recover_calib_covar(matx_t &calib_covar, bool verbose) {
  // Setup covariance blocks to estimate
  int calib_covar_size = 0;
  std::vector<param_t *> param_blocks;
  std::map<param_t *, int> param_indices;
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  // -- Add camera parameter blocks
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    if (param->fixed) {
      continue;
    }
    param_blocks.push_back(param);
    param_indices[param] = calib_covar_size;
    calib_covar_size += param->global_size;
  }
  // -- Add camera extrinsics blocks
  for (auto &[cam_idx, param] : cam_exts) {
    UNUSED(cam_idx);
    if (param->fixed) {
      continue;
    }
    param_blocks.push_back(param);
    param_indices[param] = calib_covar_size;
    calib_covar_size += param->local_size;
  }
  // -- Covariance blocks
  for (size_t i = 0; i < param_blocks.size(); i++) {
    for (size_t j = i; j < param_blocks.size(); j++) {
      auto param_i = param_blocks[i];
      auto param_j = param_blocks[j];
      covar_blocks.push_back({param_i->data(), param_j->data()});
    }
  }

  // Estimate covariance
  ::ceres::Covariance::Options options;
  // options.algorithm_type = ceres::DENSE_SVD;
  ::ceres::Covariance covar_est(options);
  if (covar_est.Compute(covar_blocks, problem) == false) {
    if (verbose) {
      LOG_ERROR("Failed to estimate covariance!");
      LOG_ERROR("Maybe Hessian is not full rank?");
    }
    return -1;
  }
  // -- Form covariance matrix
  calib_covar.resize(calib_covar_size, calib_covar_size);
  calib_covar.setZero();

  for (size_t i = 0; i < param_blocks.size(); i++) {
    for (size_t j = i; j < param_blocks.size(); j++) {
      auto param_i = param_blocks[i];
      auto param_j = param_blocks[j];
      auto ptr_i = param_i->data();
      auto ptr_j = param_j->data();
      auto idx_i = param_indices[param_i];
      auto idx_j = param_indices[param_j];
      auto size_i = param_i->local_size;
      auto size_j = param_j->local_size;

      // Diagonal
      matx_row_major_t block;
      block.resize(size_i, size_j);
      block.setZero();
      auto data = block.data();
      int retval = 0;
      if ((size_i == 6 || size_j == 6)) {
        retval = covar_est.GetCovarianceBlockInTangentSpace(ptr_i, ptr_j, data);
      } else {
        retval = covar_est.GetCovarianceBlock(ptr_i, ptr_j, data);
      }
      calib_covar.block(idx_i, idx_j, size_i, size_j) = block;

      // Off-diagonal
      if (i != j) {
        calib_covar.block(idx_j, idx_i, size_j, size_i) = block.transpose();
      }
    }
  }

  // Check if calib_covar is full-rank?
  if (rank(calib_covar) != calib_covar.rows()) {
    if (verbose) {
      LOG_ERROR("calib_covar is not full rank!");
    }
    return -1;
  }

  return 0;
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
      const mat4_t T_C0F = T_FC0.inverse();
      if (poses.count(nbv_ts) == 0) {
        poses[nbv_ts] = new pose_t{nbv_ts, T_C0F};
        problem->AddParameterBlock(poses[nbv_ts]->data(), 7);
        problem->SetParameterization(poses[nbv_ts]->data(), &pose_plus);
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
      add_view(cam_grids, true);

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
  printf("info_k: %f  ", info_k);
  printf("best_info: %f  ", best_info);
  printf("info_gain: %f\n", info_gain);
  if (info_gain < info_gain_threshold) {
    return -1;
  }

  return 0;
}

void calib_camera_t::_initialize_intrinsics() {
  for (const auto &cam_idx : get_camera_indices()) {
    if (verbose) {
      printf("Initializing cam%d intrinsics ...\n", cam_idx);
    }

    const aprilgrids_t grids = get_camera_data(cam_idx);
    auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const auto proj_model = cam_param->proj_model;
    const auto dist_model = cam_param->dist_model;
    const vecx_t k = cam_param->proj_params();
    const vecx_t d = cam_param->dist_params();

    calib_camera_t calib{calib_target};
    calib.add_camera(0, cam_res, proj_model, dist_model, k, d);
    calib.add_camera_extrinsics(0);
    calib.add_camera_data(0, grids);
    calib.verbose = false;
    calib.enable_nbv = false;
    calib.initialized = true;
    calib.solve();
    cam_params[cam_idx]->param = calib.cam_params[0]->param;
  }
}

void calib_camera_t::_initialize_extrinsics() {
  add_camera_extrinsics(0);
  if (nb_cameras() == 1) {
    return;
  }

  if (verbose) {
    printf("Initializing extrinsics ...\n");
  }

  camchain_t camchain{calib_data, cam_geoms, cam_params};
  for (const auto cam_idx : get_camera_indices()) {
    if (get_camera_data(cam_idx).size() == 0) {
      FATAL("No calibration data for cam%d?", cam_idx);
    }

    mat4_t T_C0Ci;
    if (camchain.find(0, cam_idx, T_C0Ci) != 0) {
      FATAL("Failed to initialze T_C0C%d", cam_idx);
    }
    add_camera_extrinsics(cam_idx, T_C0Ci);
  }

  // Initialize
  if (enable_nbv) {
    _solve_batch(enable_extrinsics_outlier_filter);
    remove_all_views();
  }
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
  // Estimate marginal covariance matrix
  matx_t calib_covar;
  int retval = recover_calib_covar(calib_covar);
  if (retval != 0) {
    return -1;
  }

  // log(det(calib_covar)) / log(2)
  *info = log(calib_covar.determinant()) / log(2.0);

  return 0;
}

int calib_camera_t::_remove_outliers(const bool filter_all) {
  // Make a copy of the timestamps in reverse order
  timestamps_t view_timestamps;
  if (filter_all) {
    // Filter all views
    for (int k = calib_view_timestamps.size() - 1; k >= 0; k--) {
      view_timestamps.push_back(calib_view_timestamps[k]);
    }
  } else {
    // Only filter last view
    view_timestamps.push_back(calib_view_timestamps.back());
  }

  // Iterate through views in reverse
  if (filter_all) {
    printf("\nFiltering all views\n");
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

    // Filter view
    auto &view = calib_views[ts];
    int removed = view->filter_view(cam_thresholds);
    const auto grids = view->grids; // Make a copy of the grids

    // Remove view and check info
    remove_view(ts);
    if (_calc_info(&info_k) != 0) {
      continue;
    }

    // Add the view back
    const auto tmp = enable_cross_validation;
    enable_cross_validation = false;
    if (add_view(grids, false)) {
      removed_outliers += removed;
      nb_outliers += removed;
      if (filter_all) {
        printf("Keep view [%ld], outliers: %d\n", ts, removed);
      }
    } else {
      nb_views_removed++;
      if (filter_all) {
        printf("Remove view [%ld], outliers: %d\n", ts, removed);
      }
    }
    enable_cross_validation = tmp;
  }
  if (filter_all) {
    printf("[Stats] kept views: %d, removed views: %d\n\n",
           nb_views(),
           nb_views_removed);
  }

  // Reset cross validation error if enabled
  if (filter_all && enable_cross_validation) {
    if (validation_data.size() == 0) {
      FATAL("Validation data is not set!");
    }

    calib_camera_t valid{calib_target};
    valid.verbose = false;
    for (const auto cam_idx : get_camera_indices()) {
      const auto cam_param = cam_params[cam_idx];
      const int *cam_res = cam_param->resolution;
      const std::string &proj_model = cam_param->proj_model;
      const std::string &dist_model = cam_param->dist_model;
      const vecx_t &k = cam_param->proj_params();
      const vecx_t &d = cam_param->dist_params();
      valid.add_camera(cam_idx, cam_res, proj_model, dist_model, k, d, true);
      valid.add_camera_extrinsics(cam_idx, cam_exts[cam_idx]->tf());
    }
    valid_error_k = valid.inspect(validation_data);
  }

  return nb_outliers;
}

void calib_camera_t::_print_stats(const size_t ts_idx,
                                  const size_t nb_timestamps) {
  // Show general stats
  const real_t progress = ((real_t)ts_idx / nb_timestamps) * 100.0;
  const auto reproj_errors_all = get_all_reproj_errors();
  printf("[%.2f%%] ", progress);
  printf("nb_views: %d  ", nb_views());
  printf("reproj_error: %.4f  ", rmse(reproj_errors_all));
  printf("info_k: %.4f  ", info_k);
  printf("\n");

  // Show current calibration estimates
  if ((ts_idx % 10) == 0) {
    printf("\n");
    const mat4_t T_BC0 = get_camera_extrinsics(0);
    const mat4_t T_C0B = T_BC0.inverse();
    for (const auto cam_idx : get_camera_indices()) {
      const auto cam_res = cam_params[cam_idx]->resolution;
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

void calib_camera_t::_solve_batch(const bool filter_outliers) {
  // Setup batch problem
  if (problem_init == false) {
    for (const auto ts : timestamps) {
      add_view(calib_data[ts], true);
    }
    problem_init = true;
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 100;
  options.num_threads = max_num_threads;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }

  // Final outlier rejection
  if (filter_outliers) {
    removed_outliers = _filter_all_views();
    if (verbose) {
      printf("Removed %d outliers!\n", removed_outliers);
    }

    // Solve again - second pass
    ceres::Solve(options, problem, &summary);
    if (verbose) {
      std::cout << summary.BriefReport() << std::endl;
      std::cout << std::endl;
    }
  }
}

void calib_camera_t::_solve_inc() {
  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 20;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

  // Solve
  size_t ts_idx = 0;

  for (const auto &ts : timestamps) {
    // Add view
    if (add_view(calib_data[ts], true) == false) {
      continue;
    }

    // Solve
    if (calib_views.size() > 5) {
      marginalize();
    }
    ceres::Solve(options, problem, &summary);

    // Print stats
    if (verbose) {
      _print_stats(ts_idx, timestamps.size());
    }

    // Update
    ts_idx++;
    problem_init = true;
  }

  if (verbose) {
    show_results();
  }
}

void calib_camera_t::_solve_nbv() {
  // Solver options and summary
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

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

  // Shuffle timestamps
  if (enable_shuffle_views) {
    std::shuffle(nbv_timestamps.begin(), nbv_timestamps.end(), calib_rng);
  }

  // NBV
  size_t ts_idx = 0;
  int early_stop_counter = 0;

  for (const auto &ts : nbv_timestamps) {
    // Add view
    if (add_view(calib_data[ts], false) == false) {
      early_stop_counter++;
      _print_stats(ts_idx++, nbv_timestamps.size());
      continue;
    }
    early_stop_counter = 0;

    // Remove outliers
    if (enable_nbv_filter && nb_views() >= min_nbv_views) {
      _remove_outliers(filter_all);
      filter_all = false;
    }

    // Print stats
    if (verbose) {
      _print_stats(ts_idx++, nbv_timestamps.size());
    }

    // Marginalize oldest view
    if (enable_marginalization && calib_views.size() > sliding_window_size) {
      marginalize();
    }

    // Stop early?
    if (enable_early_stopping && early_stop_counter > early_stop_threshold) {
      break;
    }

    // Update
    problem_init = true;
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
  ceres::Solve(options, problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }

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

    fprintf(progress, "%f,", rmse(reproj_errors.at(0)));
    fprintf(progress, "%f,", mean(reproj_errors.at(0)));
    fprintf(progress, "%f,", median(reproj_errors.at(0)));
    fprintf(progress, "%f,", stddev(reproj_errors.at(0)));

    fprintf(progress, "%f,", rmse(reproj_errors.at(1)));
    fprintf(progress, "%f,", mean(reproj_errors.at(1)));
    fprintf(progress, "%f,", median(reproj_errors.at(1)));
    fprintf(progress, "%f,", stddev(reproj_errors.at(1)));

    fprintf(progress, "%d\n", accepted);
  }
  fclose(progress);
}

void calib_camera_t::solve() {
  // Print Calibration settings
  if (verbose) {
    print_settings(stdout);
  }

  // Initialize camera intrinsics and extrinsics
  if (initialized == false) {
    _initialize_intrinsics();
    _initialize_extrinsics();
    initialized = true;
  }

  // Show initializations
  if (verbose) {
    printf("Initial camera intrinsics and extrinsics:\n");
    print_estimates(stdout);
  }

  // Solve
  if (enable_nbv) {
    _solve_nbv();
  } else {
    _solve_batch(enable_outlier_filter);
  }

  // Show results
  if (verbose) {
    show_results();
  }
}

void calib_camera_t::print_settings(FILE *out) {
  // clang-format off
  fprintf(out, "settings:\n");
  fprintf(out, "  # General\n");
  fprintf(out, "  verbose: %d\n", verbose);
  fprintf(out, "  max_num_threads: %d\n", max_num_threads);
  fprintf(out, "\n");
  fprintf(out, "  # Extrinsics initialization\n");
  fprintf(out, "  enable_extrinsics_outlier_filter: %d\n", enable_extrinsics_outlier_filter);
  fprintf(out, "\n");
  fprintf(out, "  # NBV settings\n");
  fprintf(out, "  enable_nbv: %d\n", enable_nbv);
  fprintf(out, "  enable_shuffle_views: %d\n", enable_shuffle_views);
  fprintf(out, "  enable_nbv_filter: %d\n", enable_nbv_filter);
  fprintf(out, "  enable_outlier_filter: %d\n", enable_outlier_filter);
  fprintf(out, "  enable_marginalization: %d\n", enable_marginalization);
  fprintf(out, "  enable_cross_validation: %d\n", enable_cross_validation);
  fprintf(out, "  enable_early_stopping: %d\n", enable_early_stopping);
  fprintf(out, "  min_nbv_views: %d\n", min_nbv_views);
  fprintf(out, "  outlier_threshold: %f\n", outlier_threshold);
  fprintf(out, "  info_gain_threshold: %f\n", info_gain_threshold);
  fprintf(out, "  sliding_window_size: %d\n", sliding_window_size);
  fprintf(out, "  early_stop_threshold: %d\n", early_stop_threshold);
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

void calib_camera_t::print_calib_target(FILE *out) {
  fprintf(out, "calib_target:\n");
  fprintf(out, "  target_type: %s\n", calib_target.target_type.c_str());
  fprintf(out, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(out, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(out, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(out, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(out, "\n");
}

void calib_camera_t::print_estimates(FILE *out) {
  const bool max_digits = (out == stdout) ? false : true;

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = cam_params.at(cam_idx);
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const auto proj_params = vec2str(cam->proj_params(), true, max_digits);
    const auto dist_params = vec2str(cam->dist_params(), true, max_digits);

    fprintf(out, "cam%d:\n", cam_idx);
    fprintf(out, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(out, "  proj_model: \"%s\"\n", proj_model);
    fprintf(out, "  dist_model: \"%s\"\n", dist_model);
    fprintf(out, "  proj_params: %s\n", proj_params.c_str());
    fprintf(out, "  dist_params: %s\n", dist_params.c_str());
    fprintf(out, "\n");
  }

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cameras() >= 2) {
    for (int i = 1; i < nb_cameras(); i++) {
      const mat4_t T_BCi = get_camera_extrinsics(i);
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      const mat4_t T_CiC0 = T_C0Ci.inverse();
      fprintf(out, "T_cam0_cam%d:\n", i);
      fprintf(out, "  rows: 4\n");
      fprintf(out, "  cols: 4\n");
      fprintf(out, "  data: [\n");
      fprintf(out, "%s\n", mat2str(T_C0Ci, "    ", max_digits).c_str());
      fprintf(out, "  ]\n");
      fprintf(out, "\n");
    }
  }
}

void calib_camera_t::show_results() {
  const auto reproj_errors = get_reproj_errors();
  const auto reproj_errors_all = get_all_reproj_errors();
  printf("Optimization results:\n");
  printf("---------------------\n");
  print_settings(stdout);
  print_metrics(stdout, reproj_errors, reproj_errors_all);
  print_estimates(stdout);
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
  print_calib_target(outfile);
  print_estimates(outfile);

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

#pragma omp parallel for shared(fiducial_poses)
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

#pragma omp critical
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
    const mat4_t T_CiC0 = cam_exts[cam_idx]->tf().inverse();

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
