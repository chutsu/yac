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
  for (const auto &kv : cam_data.at(ts)) {
    // Camera index and grid
    const auto cam_idx = kv.first;
    const auto &grid = kv.second;

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

calib_view_t::calib_view_t(ceres::Problem *problem_,
                           ceres::LossFunction *loss_,
                           fiducial_corners_t *corners_,
                           camera_geometry_t *cam_geom_,
                           camera_params_t *cam_params_,
                           pose_t *T_BCi_,
                           pose_t *T_C0F_,
                           const aprilgrid_t &grid_,
                           const mat2_t &covar_)
    : problem{problem_}, loss{loss_}, corners{corners_}, cam_geom{cam_geom_},
      cam_params{cam_params_}, T_BCi{T_BCi_}, T_C0F{T_C0F_}, grid{grid_},
      covar{covar_} {
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
    auto p_FFi_ = corners_->get_corner(tag_id, corner_idx);
    auto res_fn = std::make_shared<reproj_error_t>(cam_geom,
                                                   cam_params,
                                                   T_BCi_,
                                                   T_C0F_,
                                                   p_FFi_,
                                                   z,
                                                   covar);
    auto res_id = problem->AddResidualBlock(res_fn.get(),
                                            loss,
                                            T_BCi_->data(),
                                            T_C0F_->data(),
                                            p_FFi_->data(),
                                            cam_params->data());
    res_fns.push_back(res_fn);
    res_ids.push_back(res_id);
  }
}

vec2s_t calib_view_t::get_residuals() const {
  vec2s_t residuals;

  for (auto res_fn : res_fns) {
    vec2_t r;
    if (res_fn->get_residual(r) == 0) {
      residuals.push_back(r);
    }
  }

  return residuals;
}

std::vector<real_t> calib_view_t::get_reproj_errors() const {
  // Calculate reprojection errors
  std::vector<real_t> reproj_errors;

  for (auto res_fn : res_fns) {
    real_t e;
    if (res_fn->get_reproj_error(e) == 0) {
      reproj_errors.push_back(e);
    }
  }

  return reproj_errors;
}

int calib_view_t::filter_view(const real_t reproj_threshold) {
  // Filter
  int nb_inliers = 0;
  int nb_outliers = 0;
  auto res_fns_it = res_fns.begin();
  auto res_ids_it = res_ids.begin();

  while (res_fns_it != res_fns.end()) {
    auto &res = *res_fns_it;
    auto &res_id = *res_ids_it;

    real_t err = 0.0;
    if (res->get_reproj_error(err) == 0 && err > reproj_threshold) {
      problem->RemoveResidualBlock(res_id);
      res_fns_it = res_fns.erase(res_fns_it);
      res_ids_it = res_ids.erase(res_ids_it);
      nb_outliers++;
    } else {
      ++res_fns_it;
      ++res_ids_it;
      nb_inliers++;
    }
  }

  return nb_outliers;
}

int calib_view_t::filter_view(const vec2_t &residual_threshold) {
  // Filter
  const real_t th_x = residual_threshold.x();
  const real_t th_y = residual_threshold.y();
  int nb_inliers = 0;
  int nb_outliers = 0;
  auto res_fns_it = res_fns.begin();
  auto res_ids_it = res_ids.begin();

  while (res_fns_it != res_fns.end()) {
    auto &res = *res_fns_it;
    auto &res_id = *res_ids_it;

    vec2_t r{0.0, 0.0};
    if (res->get_residual(r) == 0 && (r.x() > th_x || r.y() > th_y)) {
      problem->RemoveResidualBlock(res_id);
      res_fns_it = res_fns.erase(res_fns_it);
      res_ids_it = res_ids.erase(res_ids_it);
      nb_outliers++;
    } else {
      ++res_fns_it;
      ++res_ids_it;
      nb_inliers++;
    }
  }

  return nb_outliers;
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
  // -- Parse calibration target settings
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
    vecx_t proj_params;
    vecx_t dist_params;
    parse(config, cam_str + ".resolution", cam_res);
    parse(config, cam_str + ".proj_model", proj_model);
    parse(config, cam_str + ".dist_model", dist_model);
    parse(config, cam_str + ".proj_params", proj_params, true);
    parse(config, cam_str + ".dist_params", dist_params, true);
    printf("%s\n", cam_str.c_str());
    printf("%s\n", proj_model.c_str());
    printf("%s\n", dist_model.c_str());
    print_vector("proj_params", proj_params);
    print_vector("dist_params", dist_params);
    add_camera(cam_idx,
               cam_res.data(),
               proj_model,
               dist_model,
               proj_params,
               dist_params);
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
      continue;
    }

    // Extract and add camera extrinsics
    mat4_t T_C0Ci = zeros(4, 4);
    parse(config, key, T_C0Ci);
    add_camera_extrinsics(cam_idx, T_C0Ci);
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
}

calib_camera_t::~calib_camera_t() {
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
}

int calib_camera_t::nb_cameras() const { return cam_params.size(); }

int calib_camera_t::nb_views(const int cam_idx) const {
  if (calib_views.count(cam_idx) == 0) {
    return 0;
  }
  return calib_views.at(cam_idx).size();
}

int calib_camera_t::nb_views() const { return static_cast<int>(poses.size()); }

aprilgrids_t calib_camera_t::get_camera_data(const int cam_idx) const {
  aprilgrids_t grids;
  for (const auto &ts : timestamps) {
    if (cam_data.count(ts) == 0) {
      continue;
    }
    if (cam_data.at(ts).count(cam_idx) == 0) {
      continue;
    }
    grids.push_back(cam_data.at(ts).at(cam_idx));
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

  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      if (view) {
        extend(reproj_errors_all, view->get_reproj_errors());
      }
    }
  }

  return reproj_errors_all;
}

std::map<int, std::vector<real_t>> calib_camera_t::get_reproj_errors() {
  std::map<int, std::vector<real_t>> reproj_errors;

  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      if (view) {
        extend(reproj_errors[cam_idx], view->get_reproj_errors());
      }
    }
  }

  return reproj_errors;
}

std::map<int, vec2s_t> calib_camera_t::get_residuals() {
  std::map<int, vec2s_t> residuals;

  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      if (view == nullptr) {
        continue;
      }

      for (const auto r : view->get_residuals()) {
        residuals[cam_idx].push_back(r);
      }
    }
  }

  return residuals;
}

void calib_camera_t::add_camera_data(const std::map<int, aprilgrids_t> &grids) {
  for (const auto &[cam_idx, cam_grids] : grids) {
    add_camera_data(cam_idx, cam_grids);
  }
}

void calib_camera_t::add_camera_data(const int cam_idx,
                                     const aprilgrids_t &grids) {
  std::vector<real_t> focal_lengths;

  for (const auto &grid : grids) {
    const auto ts = grid.timestamp;
    timestamps.insert(ts);
    cam_data[ts][cam_idx] = grid;

    double focal = 0.0;
    if (grid.detected && grid.fully_observable()) {
      if (focal_init(grid, 0, focal) == 0) {
        focal_lengths.push_back(focal);
      }
    }
  }

  if (cam_params.count(cam_idx)) {
    if (focal_lengths.size()) {
      focal_length_init[cam_idx] = median(focal_lengths);
      cam_params[cam_idx]->param[0] = focal_length_init[cam_idx];
      cam_params[cam_idx]->param[1] = focal_length_init[cam_idx];
    }
  }
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

void calib_camera_t::add_pose(const timestamp_t ts, const bool fixed) {
  // Pre-check
  if (poses.count(ts) > 0) {
    FATAL("Implementation Error!");
    return;
  }

  // Find the AprilGrid that has the most observations
  int best_cam_idx = 0;
  aprilgrid_t best_grid;
  for (const auto &[cam_idx, grid] : cam_data.at(ts)) {
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

void calib_camera_t::add_pose(const int cam_idx,
                              const aprilgrid_t &grid,
                              const bool fixed) {
  timestamps.insert(grid.timestamp);
  cam_data[grid.timestamp][cam_idx] = grid;
  add_pose(grid.timestamp, fixed);
}

void calib_camera_t::add_view(const aprilgrid_t &grid,
                              ceres::Problem *problem,
                              ceres::LossFunction *loss,
                              int cam_idx,
                              camera_geometry_t *cam_geom,
                              camera_params_t *cam_params,
                              extrinsics_t *cam_exts,
                              pose_t *rel_pose) {
  const timestamp_t ts = grid.timestamp;
  calib_views[cam_idx][ts] = std::make_unique<calib_view_t>(problem,
                                                            loss,
                                                            &corners,
                                                            cam_geom,
                                                            cam_params,
                                                            cam_exts,
                                                            rel_pose,
                                                            grid);
}

bool calib_camera_t::add_view(const std::map<int, aprilgrid_t> &cam_grids) {
  bool new_view_added = false;

  for (const auto &[cam_idx, grid] : cam_grids) {
    // Check if AprilGrid was detected
    if (grid.detected == false) {
      continue;
    }

    // Add pose
    const auto ts = grid.timestamp;
    if (poses.count(ts) == 0) {
      add_pose(ts);
    }

    // Add calibration view
    add_view(grid,
             problem,
             loss,
             cam_idx,
             cam_geoms[cam_idx],
             cam_params[cam_idx],
             cam_exts[cam_idx],
             poses[ts]);
    new_view_added = true;
  }

  return new_view_added;
}

void calib_camera_t::remove_view(const timestamp_t ts) {
  // Remove view
  for (auto &[cam_idx, cam_views] : calib_views) {
    if (cam_views.count(ts) == 0) {
      continue;
    }

    auto &view = cam_views[ts];
    if (view == nullptr) {
      continue;
    }

    for (auto &res_id : view->res_ids) {
      problem->RemoveResidualBlock(res_id);
    }
    cam_views.erase(ts);
  }

  // Remove pose
  if (poses.count(ts)) {
    auto pose_ptr = poses[ts];
    problem->RemoveParameterBlock(pose_ptr->param.data());
    poses.erase(ts);
    delete pose_ptr;
  }
}

void calib_camera_t::remove_all_views() {
  for (const auto ts : timestamps) {
    remove_view(ts);
  }
  problem_init = false;
}

int calib_camera_t::recover_calib_covar(matx_t &calib_covar, bool verbose) {
  // Setup covariance blocks to estimate
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  int calib_covar_size = 0;
  // -- Add camera parameter blocks
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    if (param->fixed == false) {
      covar_blocks.push_back({param->param.data(), param->param.data()});
      calib_covar_size += param->param.size();
    }
  }
  // -- Add camera extrinsics blocks
  for (auto &[cam_idx, param] : cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    UNUSED(cam_idx);
    if (param->fixed == false) {
      covar_blocks.push_back({param->param.data(), param->param.data()});
      calib_covar_size += 6;
    }
  }

  // Estimate covariance
  ::ceres::Covariance::Options options;
  ::ceres::Covariance covar_est(options);
  if (covar_est.Compute(covar_blocks, problem) == false) {
    if (verbose) {
      LOG_ERROR("Failed to estimate covariance!");
      LOG_ERROR("Maybe Hessian is not full rank?");
    }
    return -1;
  }

  // Extract covariances sub-blocks
  std::map<int, Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> covar_cam_params;
  std::map<int, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covar_cam_exts;
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    if (param->fixed) {
      continue;
    }

    auto param_ptr = param->param.data();
    auto covar_ptr = covar_cam_params[cam_idx].data();
    if (!covar_est.GetCovarianceBlock(param_ptr, param_ptr, covar_ptr)) {
      if (verbose) {
        LOG_ERROR("Failed to estimate cam%d parameter covariance!", cam_idx);
      }
      return -1;
    }
  }
  for (auto &[cam_idx, param] : cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    if (param->fixed) {
      continue;
    }

    auto param_ptr = param->param.data();
    auto covar_ptr = covar_cam_exts[cam_idx].data();
    if (!covar_est.GetCovarianceBlockInTangentSpace(param_ptr,
                                                    param_ptr,
                                                    covar_ptr)) {
      if (verbose) {
        LOG_ERROR("Failed to estimate cam%d extrinsics covariance!", cam_idx);
      }
      return -1;
    }
  }

  // Form covariance matrix block
  calib_covar = zeros(calib_covar_size, calib_covar_size);
  int idx = 0;
  for (auto &[cam_idx, covar] : covar_cam_params) {
    calib_covar.block(idx, idx, 8, 8) = covar;
    idx += 8;
  }
  for (auto &[cam_idx, covar] : covar_cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    calib_covar.block(idx, idx, 6, 6) = covar;
    idx += 6;
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

  // Current Entropy
  real_t entropy_k = 0.0;
  {
    matx_t calib_covar;
    if (recover_calib_covar(calib_covar, true) != 0) {
      return -1;
    }
    entropy_k = shannon_entropy(calib_covar);
  }

  // Find NBV
  const timestamp_t last_ts = *timestamps.rbegin(); // std::set is orderd
  const timestamp_t nbv_ts = last_ts + 1;
  int best_cam = 0;
  int best_idx = 0;
  real_t best_entropy = 0;

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
      for (const auto cam_idx : get_camera_indices()) {
        const mat4_t T_C0Ci = cam_exts[cam_idx]->tf();
        const auto &grid = nbv_target_grid(calib_target,
                                           cam_geoms[cam_idx],
                                           cam_params[cam_idx],
                                           nbv_ts,
                                           T_FC0 * T_C0Ci);

        add_view(grid,
                 problem,
                 loss,
                 cam_idx,
                 cam_geoms[cam_idx],
                 cam_params[cam_idx],
                 cam_exts[cam_idx],
                 poses[nbv_ts]);
      }

      // Evaluate NBV
      // -- Estimate calibration covariance
      bar.update();
      matx_t calib_covar;
      if (recover_calib_covar(calib_covar, true) != 0) {
        remove_view(nbv_ts);
        continue;
      }
      remove_view(nbv_ts);
      // -- Calculate entropy
      const real_t entropy = shannon_entropy(calib_covar);
      if (entropy < best_entropy) {
        best_cam = nbv_cam_idx;
        best_idx = i;
        best_entropy = entropy;
      }
    }
  }
  printf("\n");

  // Return
  cam_idx = best_cam;
  nbv_idx = best_idx;

  const auto info_gain = 0.5 * (entropy_k - best_entropy);
  printf("best_entropy: %f\n", best_entropy);
  printf("entropy_k: %f\n", entropy_k);
  printf("info_gain: %f\n", info_gain);
  if (info_gain < info_gain_threshold) {
    return 1;
  } else {
    entropy_k = best_entropy;
  }

  return 0;
}

int calib_camera_t::find_nbv(std::vector<timestamp_t> &nbv_timestamps,
                             real_t &best_entropy,
                             timestamp_t &best_nbv_ts) {
  progressbar bar(nbv_timestamps.size());
  printf("Finding NBV: ");

  best_entropy = 0.0;
  best_nbv_ts = 0;
  auto it = nbv_timestamps.begin();

  while (it != nbv_timestamps.end()) {
    // Timestamp
    auto nbv_ts = *it++;

    // Add view
    bool new_view_added = false;
    for (const auto &[cam_idx, grid] : cam_data[nbv_ts]) {
      // Check if AprilGrid was detected
      if (grid.detected == false) {
        continue;
      }

      // Add pose
      if (poses.count(nbv_ts) == 0) {
        add_pose(nbv_ts);
      }

      // Add calibration view
      add_view(grid,
               problem,
               loss,
               cam_idx,
               cam_geoms[cam_idx],
               cam_params[cam_idx],
               cam_exts[cam_idx],
               poses[nbv_ts]);
      new_view_added = true;
    }

    // Check if new view has been added
    if (new_view_added == false) {
      continue;
    }

    // Solve with new view
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    // Filter last view and optimize again
    if (enable_nbv_filter) {
      _filter_view(nbv_ts);
      ceres::Solve(options, problem, &summary);
    }

    // Check information
    matx_t calib_covar;
    int retval = recover_calib_covar(calib_covar);
    if (retval != 0) {
      continue;
    }

    // Calculate entropy
    const real_t entropy = shannon_entropy(calib_covar);
    if (entropy < best_entropy) {
      best_entropy = entropy;
      best_nbv_ts = nbv_ts;
    }

    // Remove view
    remove_view(nbv_ts);
    bar.update();
  }
  printf("\n");

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
    const std::string proj_model = cam_param->proj_model;
    const std::string dist_model = cam_param->dist_model;
    const vecx_t proj_params = cam_param->proj_params();
    const vecx_t dist_params = cam_param->dist_params();

    calib_camera_t calib{calib_target};
    calib.add_camera(0,
                     cam_res,
                     proj_model,
                     dist_model,
                     proj_params,
                     dist_params);
    calib.add_camera_extrinsics(0);
    calib.add_camera_data(0, grids);
    calib.verbose = false;
    calib.enable_nbv = enable_intrinsics_nbv;
    calib.enable_outlier_filter = enable_intrinsics_outlier_filter;
    calib.outlier_threshold = intrinsics_outlier_threshold;
    calib.info_gain_threshold = intrinsics_info_gain_threshold;
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
  camchain_t camchain{cam_data, cam_geoms, cam_params};
  for (int j = 1; j < nb_cameras(); j++) {
    mat4_t T_C0Cj;
    if (camchain.find(0, j, T_C0Cj) != 0) {
      FATAL("Failed to initialze T_C0C%d", j);
    }
    add_camera_extrinsics(j, T_C0Cj);
  }

  // Initialize
  if (enable_nbv) {
    _solve_batch(enable_extrinsics_outlier_filter);
    remove_all_views();
  }
}

int calib_camera_t::_filter_view(const timestamp_t ts) {
  int removed = 0;

  // Calculate reprojection threshold on a per-camera basis
  std::map<int, vec2_t> cam_thresholds;
  std::map<int, vec2s_t> cam_residuals = get_residuals();
  for (const auto cam_idx : get_camera_indices()) {
    const vec2s_t cam_r = cam_residuals[cam_idx];
    const vec2_t cam_stddev = stddev(cam_r);
    cam_thresholds[cam_idx] = outlier_threshold * cam_stddev;
  }

  // Filter one view
  for (auto &[cam_idx, cam_views] : calib_views) {
    UNUSED(cam_idx);
    if (cam_views.count(ts) == 0) {
      continue;
    }

    if (cam_views[ts]) {
      removed += cam_views[ts]->filter_view(cam_thresholds[cam_idx]);
    }
  }

  return removed;
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
  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      removed += view->filter_view(cam_thresholds[cam_idx]);
    }
  }

  return removed;
}

int calib_camera_t::_eval_nbv(const timestamp_t nbv_ts) {
  // Ceres options and summary
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 50;
  ceres::Solver::Summary summary;

  // Keep track of initial values
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

  // Solve with new view
  if (nb_views() > min_nbv_views) {
    ceres::Solve(options, problem, &summary);
  }

  // Filter last view and optimize again
  int removed = 0;
  const auto first_camera = get_camera_indices()[0];
  if (enable_nbv_filter && nb_views() > min_nbv_views) {
    if (filter_views_init == false) {
      removed = _filter_all_views();
      filter_views_init = true;
    } else {
      removed = _filter_view(nbv_ts);
    }

    // Solve again
    ceres::Solve(options, problem, &summary);
  }

  // Estimate calibration covariance
  matx_t calib_covar;
  int retval = recover_calib_covar(calib_covar);
  if (retval != 0) {
    return -1;
  }
  if (fltcmp(covar_det_k, -1.0) == 0) {
    covar_det_k = calib_covar.determinant();
    return 0;
  }

  // Calculate information gain
  const real_t covar_det_kp1 = calib_covar.determinant();
  info_gain = 0.5 * (log(covar_det_k / covar_det_kp1) / log(2.0));
  if (info_gain < info_gain_threshold) {
    // Restore values before view was added
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

    // Remove view
    remove_view(nbv_ts);
    return -2;
  }

  // Update
  covar_det_k = covar_det_kp1;
  removed_outliers += removed;

  return 0;
}

void calib_camera_t::_print_stats(const real_t progress) {
  // if (validation_data.size()) {
  //   calib_camera_t valid{calib_target};
  //   for (const auto cam_idx : get_camera_indices()) {
  //     const auto cam_param = cam_params[cam_idx];
  //     const int *cam_res = cam_param->resolution;
  //     const std::string &proj_model = cam_param->proj_model;
  //     const std::string &dist_model = cam_param->dist_model;
  //     const vecx_t &proj_params = cam_param->proj_params();
  //     const vecx_t &dist_params = cam_param->dist_params();
  //
  //     valid.add_camera(cam_idx,
  //                      cam_res,
  //                      proj_model,
  //                      dist_model,
  //                      proj_params,
  //                      dist_params,
  //                      true);
  //     valid.add_camera_extrinsics(cam_idx, cam_exts[cam_idx]->tf());
  //   }
  //   const auto valid_error = valid.validate(validation_data);
  // }

  const auto reproj_errors_all = get_all_reproj_errors();
  // clang-format off
  printf("[%.2f%%] ", progress);
  printf("nb_views: %d  ", nb_views());
  printf("reproj_error: %.2f ", rmse(reproj_errors_all));
  // printf("entropy: %.2f ", calib_entropy_k);
  // printf("validation_error: %.2f ", valid_error);
  printf("info_gain: %.2f ", info_gain);
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
  fflush(stdout);
  // clang-format on
}

void calib_camera_t::_solve_batch(const bool filter_outliers) {
  // Setup batch problem
  if (problem_init == false) {
    for (const auto ts : timestamps) {
      add_view(cam_data[ts]);
    }
    problem_init = true;
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.parameter_tolerance = 1e-20;
  options.inner_iteration_tolerance = 1e-5;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;
    std::cout << std::endl;
  }

  // Final outlier rejection
  if (filter_outliers) {
    removed_outliers += _filter_all_views();
    if (verbose) {
      printf("Removed %d outliers\n", removed_outliers);
      printf("Solving again!\n");
    }

    // Solve again - second pass
    ceres::Solve(options, problem, &summary);
    if (verbose) {
      std::cout << summary.BriefReport() << std::endl;
      std::cout << std::endl;
    }
  }
}

void calib_camera_t::_solve_nbv() {
  // Shuffle timestamps
  timestamps_t nbv_timestamps;
  for (const auto ts : timestamps) {
    for (const auto &[cam_idx, grid] : cam_data[ts]) {
      if (grid.detected) {
        nbv_timestamps.push_back(ts);
        break;
      }
    }
  }
  std::shuffle(nbv_timestamps.begin(), nbv_timestamps.end(), calib_rng);
  const real_t nb_timestamps = (real_t)nbv_timestamps.size();

  // NBV
  if (verbose) {
    printf("Solving Incremental NBV problem\n");
  }
  size_t ts_idx = 0;
  size_t stale_counter = 0;
  size_t stale_threshold = 40;

  for (const auto &ts : nbv_timestamps) {
    // Add view
    if (add_view(cam_data[ts]) == false) {
      continue;
    }

    // Evaluate NBV
    if (_eval_nbv(ts) != 0) {
      stale_counter++;
    } else {
      stale_counter = 0;
    }

    // Stop early?
    // if (stale_counter > stale_threshold) {
    //   break;
    // }

    // Print stats
    const real_t progress = ((real_t)ts_idx / nb_timestamps) * 100.0;
    if (verbose) {
      _print_stats(progress);
    }

    // Update
    ts_idx++;
    problem_init = true;
    const auto nbv_begin = nbv_timestamps.begin();
    const auto nbv_end = nbv_timestamps.end();
    const auto nbv_idx = std::remove(nbv_begin, nbv_end, ts);
    nbv_timestamps.erase(nbv_idx, nbv_end);
  }

  // Final outlier rejection, then batch solve
  if (enable_outlier_filter) {
    removed_outliers += _filter_all_views();
    if (verbose) {
      printf("Removed %d outliers\n", removed_outliers);
    }
  }

  // Final Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }
}

void calib_camera_t::_solve_nbv_brute_force() {
  // Setup
  if (verbose) {
    printf("Solving Incremental NBV problem\n");
  }
  real_t calib_entropy_k = 0.0;

  // Form NBV timestamps where aprilgrids are actually detected
  std::vector<timestamp_t> nbv_timestamps;
  for (const auto &ts : timestamps) {
    for (const auto &[cam_idx, grid] : cam_data[ts]) {
      if (grid.detected) {
        nbv_timestamps.push_back(ts);
        break;
      }
    }
  }
  std::shuffle(nbv_timestamps.begin(), nbv_timestamps.end(), calib_rng);

  // Add min-views
  {
    auto it = nbv_timestamps.begin();
    while (it != nbv_timestamps.end()) {
      // Timestamp
      auto nbv_ts = *it;

      // Add view
      if (add_view(cam_data[nbv_ts]) == false) {
        continue;
      }

      // Update
      it = nbv_timestamps.erase(it);
      if (nb_views() >= min_nbv_views) {
        break;
      }
    }

    // Filter all views
    removed_outliers = _filter_all_views();
    if (verbose) {
      printf("Removed %d outliers\n", removed_outliers);
    }

    // Calculate entropy
    matx_t calib_covar;
    int retval = recover_calib_covar(calib_covar);
    if (retval != 0) {
      FATAL("Failed to estimate calibration covariance!");
    }
    calib_entropy_k = shannon_entropy(calib_covar);
  }

  // Find next best view
  const real_t nb_views = nbv_timestamps.size();
  while (nbv_timestamps.size()) {
    // Find next best view
    real_t calib_entropy_kp1 = 0.0;
    timestamp_t nbv_ts = 0;
    find_nbv(nbv_timestamps, calib_entropy_kp1, nbv_ts);

    // Calculate information gain
    real_t info_gain = 0.5 * (calib_entropy_k - calib_entropy_kp1);
    if (info_gain < info_gain_threshold) {
      break;
    } else {
      calib_entropy_k = calib_entropy_kp1;
    }

    // Add view
    if (add_view(cam_data[nbv_ts]) == false) {
      continue;
    }

    // Print stats
    _print_stats((nbv_timestamps.size() / nb_views) * 100.0);

    // Update
    const auto nbv_begin = nbv_timestamps.begin();
    const auto nbv_end = nbv_timestamps.end();
    const auto nbv_idx = std::remove(nbv_begin, nbv_end, nbv_ts);
    nbv_timestamps.erase(nbv_idx, nbv_end);
    problem_init = true;
  }
}

void calib_camera_t::solve() {
  // Print Calibration settings
  if (verbose) {
    // clang-format off
    printf("Calibrating cameras:\n");
    printf("--------------------------------------------------\n");
    printf("calib_settings:\n");
    printf("  verbose: %d\n", verbose);
    printf("  enable_intrinsics_nbv: %d\n", enable_intrinsics_nbv);
    printf("  enable_intrinsics_outlier_filter: %d\n", enable_intrinsics_outlier_filter);
    printf("  intrinsics_info_gain_threshold: %f\n", intrinsics_info_gain_threshold);
    printf("  intrinsics_outlier_threshold: %f\n", intrinsics_outlier_threshold);
    printf("  intrinsics_min_nbv_views: %d\n", intrinsics_min_nbv_views);
    printf("  enable_extrinsics_outlier_filter: %d\n", enable_extrinsics_outlier_filter);
    printf("  enable_nbv: %d\n", enable_nbv);
    printf("  enable_nbv_filter: %d\n", enable_nbv_filter);
    printf("  enable_outlier_filter: %d\n", enable_outlier_filter);
    printf("  min_nbv_views: %d\n", min_nbv_views);
    printf("  outlier_threshold: %f\n", outlier_threshold);
    printf("  info_gain_threshold: %f\n", info_gain_threshold);
    printf("\n");
    // clang-format on
  }

  // Setup
  if (initialized == false) {
    _initialize_intrinsics();
    _initialize_extrinsics();

    if (verbose) {
      printf("Initial camera intrinsics\n");
      // Camera intrinsics
      for (const auto cam_idx : get_camera_indices()) {
        const auto cam_param = cam_params[cam_idx];
        const auto cam_res = cam_param->resolution;
        const auto proj_params = cam_param->proj_params();
        const auto dist_params = cam_param->dist_params();
        printf("cam%d:\n", cam_idx);
        printf("  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
        printf("  proj_model: %s\n", cam_param->proj_model.c_str());
        printf("  dist_model: %s\n", cam_param->dist_model.c_str());
        printf("  proj_params: %s\n", vec2str(proj_params).c_str());
        printf("  dist_params: %s\n", vec2str(dist_params).c_str());
        printf("\n");
      }

      // Camera extrinsics
      printf("Initial camera extrinsics\n");
      const mat4_t T_BC0 = get_camera_extrinsics(0);
      const mat4_t T_C0B = T_BC0.inverse();
      for (const auto cam_idx : get_camera_indices()) {
        if (cam_idx == 0) {
          continue;
        }
        const mat4_t T_BCi = cam_exts[cam_idx]->tf();
        const mat4_t T_C0Ci = T_C0B * T_BCi;
        printf("T_cam0_cam%d: [\n", cam_idx);
        printf("%s\n", mat2str(T_C0Ci, "  ").c_str());
        printf("]\n");
        printf("\n");
      }
    }

    initialized = true;
  }

  // Solve
  if (enable_nbv) {
    _solve_nbv();
    // _solve_nbv_brute_force();
  } else {
    _solve_batch(enable_outlier_filter);
  }

  // Show results
  if (verbose) {
    show_results();
  }
}

void calib_camera_t::show_results() {
  // Show results
  printf("Optimization results:\n");
  printf("---------------------\n");

  // Stats - Reprojection Errors
  const auto reproj_errors = get_reproj_errors();
  const auto reproj_errors_all = get_all_reproj_errors();
  // -- Print total reprojection error
  printf("Total reprojection error:\n");
  printf("  rmse:   %.4f # px\n", rmse(reproj_errors_all));
  printf("  mean:   %.4f # px\n", mean(reproj_errors_all));
  printf("  median: %.4f # px\n", median(reproj_errors_all));
  printf("  stddev: %.4f # px\n", stddev(reproj_errors_all));
  printf("\n");
  // -- Print camera reprojection error
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
  for (int cam_idx = 0; cam_idx < nb_cameras(); cam_idx++) {
    const int *cam_res = cam_params[cam_idx]->resolution;
    const char *proj_model = cam_params[cam_idx]->proj_model.c_str();
    const char *dist_model = cam_params[cam_idx]->dist_model.c_str();

    printf("cam%d:\n", cam_idx);
    printf("  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    printf("  proj_model: \"%s\"\n", proj_model);
    printf("  dist_model: \"%s\"\n", dist_model);
    print_vector("  proj_params", cam_params[cam_idx]->proj_params());
    print_vector("  dist_params", cam_params[cam_idx]->dist_params());
    printf("\n");
  }

  // Camera Extrinsics
  for (int cam_idx = 1; cam_idx < nb_cameras(); cam_idx++) {
    const auto key = "T_cam0_cam" + std::to_string(cam_idx);
    const mat4_t T_C0Ci = get_camera_extrinsics(cam_idx);
    print_matrix(key, T_C0Ci, "  ");
  }
}

int calib_camera_t::save_results(const std::string &save_path) {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Get reprojection errors
  const auto reproj_errors = get_reproj_errors();
  const auto reproj_errors_all = get_all_reproj_errors();

  // Calibration metrics
  fprintf(outfile, "calib_metrics:\n");
  fprintf(outfile, "  total_reproj_error:\n");
  fprintf(outfile, "    rmse:   %f # [px]\n", rmse(reproj_errors_all));
  fprintf(outfile, "    mean:   %f # [px]\n", mean(reproj_errors_all));
  fprintf(outfile, "    median: %f # [px]\n", median(reproj_errors_all));
  fprintf(outfile, "    stddev: %f # [px]\n", stddev(reproj_errors_all));
  fprintf(outfile, "\n");
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    fprintf(outfile, "  %s_reproj_error:\n", cam_str.c_str());
    fprintf(outfile, "    rmse:   %f # [px]\n", rmse(cam_errors));
    fprintf(outfile, "    mean:   %f # [px]\n", mean(cam_errors));
    fprintf(outfile, "    median: %f # [px]\n", median(cam_errors));
    fprintf(outfile, "    stddev: %f # [px]\n", stddev(cam_errors));
    fprintf(outfile, "\n");
  }

  // Calibration settings
  // clang-format off
  fprintf(outfile, "calib_settings:\n");
  fprintf(outfile, "  verbose: %d\n", verbose);
  fprintf(outfile, "  # -- Intrinsics initialization\n");
  fprintf(outfile, "  enable_intrinsics_nbv: %d\n", enable_intrinsics_nbv);
  fprintf(outfile, "  enable_intrinsics_outlier_filter: %d\n", enable_intrinsics_outlier_filter);
  fprintf(outfile, "  intrinsics_info_gain_threshold: %f\n", intrinsics_info_gain_threshold);
  fprintf(outfile, "  intrinsics_outlier_threshold: %f\n", intrinsics_outlier_threshold);
  fprintf(outfile, "  intrinsics_min_nbv_views: %d\n", intrinsics_min_nbv_views);
  fprintf(outfile, "  # -- Extrinsics initialization\n");
  fprintf(outfile, "  enable_extrinsics_outlier_filter: %d\n", enable_extrinsics_outlier_filter);
  fprintf(outfile, "  # -- Final stage settings\n");
  fprintf(outfile, "  enable_nbv: %d\n", enable_nbv);
  fprintf(outfile, "  enable_nbv_filter: %d\n", enable_nbv_filter);
  fprintf(outfile, "  enable_outlier_filter: %d\n", enable_outlier_filter);
  fprintf(outfile, "  min_nbv_views: %d\n", min_nbv_views);
  fprintf(outfile, "  outlier_threshold: %f\n", outlier_threshold);
  fprintf(outfile, "  info_gain_threshold: %f\n", info_gain_threshold);
  fprintf(outfile, "\n");
  // clang-format on

  // Calibration target
  fprintf(outfile, "calib_target:\n");
  fprintf(outfile, "  target_type: %s\n", calib_target.target_type.c_str());
  fprintf(outfile, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(outfile, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(outfile, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(outfile, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(outfile, "\n");

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = cam_params.at(cam_idx);
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const std::string proj_params = vec2str(cam->proj_params(), true, true);
    const std::string dist_params = vec2str(cam->dist_params(), true, true);

    fprintf(outfile, "cam%d:\n", cam_idx);
    fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cameras() >= 2) {
    for (int i = 1; i < nb_cameras(); i++) {
      const mat4_t T_BCi = get_camera_extrinsics(i);
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      const mat4_t T_CiC0 = T_C0Ci.inverse();
      fprintf(outfile, "T_cam0_cam%d:\n", i);
      fprintf(outfile, "  rows: 4\n");
      fprintf(outfile, "  cols: 4\n");
      fprintf(outfile, "  data: [\n");
      fprintf(outfile, "%s\n", mat2str(T_C0Ci, "    ", true).c_str());
      fprintf(outfile, "  ]\n");
      fprintf(outfile, "\n");
    }
  }

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
  for (const auto &[cam_idx, cam_views] : calib_views) {
    // Camera view
    for (const auto &[ts, cam_view] : cam_views) {
      // Data
      for (const auto &res_fn : cam_view->res_fns) {
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

real_t calib_camera_t::validate(const std::map<int, aprilgrids_t> &cam_data) {
  // Pre-check
  if (cam_params.size() == 0) {
    FATAL("cam_params.size() == 0");
  } else if (cam_exts.size() == 0) {
    FATAL("cam_exts.size() == 0");
  } else if (cam_exts.size() != cam_params.size()) {
    FATAL("cam_exts.size() != cam_params.size()");
  }

  // Lambda to average poses
  auto find_mean_pose = [](const mat4s_t &poses) {
    std::vector<real_t> rx;
    std::vector<real_t> ry;
    std::vector<real_t> rz;

    std::vector<real_t> qw;
    std::vector<real_t> qx;
    std::vector<real_t> qy;
    std::vector<real_t> qz;

    for (const auto &pose : poses) {
      const vec3_t r = tf_trans(pose);
      const quat_t q = tf_quat(pose);

      rx.push_back(r.x());
      ry.push_back(r.y());
      rz.push_back(r.z());

      qw.push_back(q.w());
      qx.push_back(q.x());
      qy.push_back(q.y());
      qz.push_back(q.z());
    }

    const real_t rx_hat = mean(rx);
    const real_t ry_hat = mean(ry);
    const real_t rz_hat = mean(rz);
    const vec3_t r{rx_hat, ry_hat, rz_hat};

    const real_t qw_hat = mean(qw);
    const real_t qx_hat = mean(qx);
    const real_t qy_hat = mean(qy);
    const real_t qz_hat = mean(qz);
    const quat_t q{qw_hat, qx_hat, qy_hat, qz_hat};

    return tf(q, r);
  };

  // Estimate relaitve poses T_C0F
  std::map<timestamp_t, mat4s_t> fiducial_poses;
  for (const auto cam_idx : get_camera_indices()) {
    const auto cam_geom = cam_geoms[cam_idx];
    const auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const auto T_C0Ci = cam_exts[cam_idx]->tf();

    for (const auto &grid : cam_data.at(cam_idx)) {
      // Make sure aprilgrid is detected
      if (grid.detected == false) {
        continue;
      }

      // Estimate relative pose
      mat4_t T_CiF;
      if (grid.estimate(cam_geom, cam_res, cam_param->param, T_CiF) != 0) {
        continue;
      }
      fiducial_poses[grid.timestamp].push_back(T_C0Ci * T_CiF);
    }
  }

  // Calculate reprojection error
  std::vector<real_t> all_residuals;
  std::map<int, std::vector<real_t>> cam_residuals;
  for (const auto cam_idx : get_camera_indices()) {
    const auto cam_geom = cam_geoms[cam_idx];
    const auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const auto T_CiC0 = cam_exts[cam_idx]->tf().inverse();

    for (const auto &grid : cam_data.at(cam_idx)) {
      // Make sure aprilgrid is detected
      if (grid.detected == false) {
        continue;
      }

      // Estimate relative pose
      const timestamp_t ts = grid.timestamp;
      const mat4_t T_C0F = find_mean_pose(fiducial_poses[ts]);
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
        cam_residuals[cam_idx].push_back((z - z_hat).norm());
        all_residuals.push_back((z - z_hat).norm());
      }
    }
  }

  // Print stats
  printf("Validation Statistics:\n");
  for (const auto &[cam_idx, cam_r] : cam_residuals) {
    const auto cam_param = cam_params[cam_idx];
    const auto cam_res = cam_param->resolution;
    const auto T_C0Ci = cam_exts[cam_idx]->tf();

    printf("cam%d:\n", cam_idx);
    printf("  rmse: %.4f # px\n", rmse(cam_r));
    printf("  mean: %.4f # px\n", mean(cam_r));
    printf("  median: %.4f # px\n", median(cam_r));
    printf("  stddev: %.4f # px\n", stddev(cam_r));
    printf("\n");
    printf("  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    printf("  proj_model: %s\n", cam_param->proj_model.c_str());
    printf("  dist_model: %s\n", cam_param->dist_model.c_str());
    printf("  proj_params: %s\n", vec2str(cam_param->proj_params()).c_str());
    printf("  dist_params: %s\n", vec2str(cam_param->dist_params()).c_str());
    printf("\n");
  }

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cameras() >= 2) {
    for (const auto &[cam_idx, ext] : cam_exts) {
      if (cam_idx == 0) {
        continue;
      }

      const mat4_t T_BCi = ext->tf();
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      const mat4_t T_CiC0 = T_C0Ci.inverse();
      printf("T_cam0_cam%d:\n", cam_idx);
      printf("  rows: 4\n");
      printf("  cols: 4\n");
      printf("  data: [\n");
      printf("%s\n", mat2str(T_C0Ci, "    ").c_str());
      printf("  ]\n");
      printf("\n");
    }
  }

  return rmse(all_residuals);
}

int calib_camera_solve(const std::string &config_path) {
  // Load configuration
  config_t config{config_path};
  // -- Parse calibration target settings
  calib_target_t target;
  if (target.load(config_path, "calib_target") != 0) {
    LOG_ERROR("Failed to parse calib_target in [%s]!", config_path.c_str());
    return -1;
  }
  // -- Parse calibration settings
  std::string data_path;
  parse(config, "settings.data_path", data_path);
  // -- Parse camera settings
  std::map<int, veci2_t> cam_res;
  std::map<int, std::string> cam_proj_models;
  std::map<int, std::string> cam_dist_models;
  std::map<int, std::string> cam_paths;
  for (int cam_idx = 0; cam_idx < 100; cam_idx++) {
    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    if (yaml_has_key(config, cam_str) == 0) {
      continue;
    }

    // Parse
    veci2_t resolution;
    std::string proj_model;
    std::string dist_model;
    parse(config, cam_str + ".resolution", resolution);
    parse(config, cam_str + ".proj_model", proj_model);
    parse(config, cam_str + ".dist_model", dist_model);
    cam_res[cam_idx] = resolution;
    cam_proj_models[cam_idx] = proj_model;
    cam_dist_models[cam_idx] = dist_model;
    cam_paths[cam_idx] = data_path + "/" + cam_str + "/data";
  }
  if (cam_res.size() == 0) {
    LOG_ERROR("Failed to parse any camera parameters...");
    return -1;
  }

  // Preprocess camera images
  LOG_INFO("Preprocessing camera data ...");
  const std::string grids_path = data_path + "/grids0";
  const auto cam_grids = calib_data_preprocess(target, cam_paths, grids_path);
  if (cam_grids.size() == 0) {
    LOG_ERROR("Failed to load calib data!");
    return -1;
  }

  // Setup calibrator
  LOG_INFO("Setting up camera calibrator ...");
  calib_camera_t calib{target};
  for (auto &[cam_idx, _] : cam_res) {
    LOG_INFO("Adding [cam%d] params and data", cam_idx);
    calib.add_camera_data(cam_idx, cam_grids.at(cam_idx));
    calib.add_camera(cam_idx,
                     cam_res[cam_idx].data(),
                     cam_proj_models[cam_idx],
                     cam_dist_models[cam_idx]);
  }

  // Solve and save results
  LOG_INFO("Solving ...");
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_stats("/tmp/calib-stats.csv");

  return 0;
}

} //  namespace yac
