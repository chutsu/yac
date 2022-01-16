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

// REPROJECTION ERROR //////////////////////////////////////////////////////////

reproj_error_t::reproj_error_t(const camera_geometry_t *cam_geom_,
                               const camera_params_t *cam_params_,
                               const pose_t *T_BCi_,
                               const pose_t *T_C0F_,
                               const int tag_id_,
                               const int corner_idx_,
                               const vec3_t &r_FFi_,
                               const vec2_t &z_,
                               const mat2_t &covar_)
    : cam_geom{cam_geom_},
      cam_params{cam_params_}, T_BCi{T_BCi_}, T_C0F{T_C0F_}, tag_id{tag_id_},
      corner_idx{corner_idx_}, r_FFi{r_FFi_}, z{z_}, covar{covar_},
      info{covar.inverse()}, sqrt_info{info.llt().matrixL().transpose()} {
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // camera-fiducial relative pose
  block_sizes->push_back(7); // camera-camera extrinsics
  block_sizes->push_back(8); // camera parameters
}

int reproj_error_t::get_residual(vec2_t &r) const {
  assert(T_BCi != nullptr);
  assert(T_C0F != nullptr);

  // Map parameters out
  const mat4_t T_C0Ci_ = T_BCi->tf();
  const mat4_t T_C0F_ = T_C0F->tf();

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0_ = T_C0Ci_.inverse();
  const vec3_t r_CiFi = tf_point(T_CiC0_ * T_C0F_, r_FFi);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  auto param = cam_params->param;
  vec2_t z_hat;
  if (cam_geom->project(res, param, r_CiFi, z_hat) != 0) {
    return -1;
  }
  // -- Residual
  r = z - z_hat;

  return 0;
}

int reproj_error_t::get_reproj_error(real_t &error) const {
  vec2_t r;
  if (get_residual(r) != 0) {
    return -1;
  }
  error = r.norm();
  return 0;
}

bool reproj_error_t::Evaluate(double const *const *params,
                              double *residuals,
                              double **jacobians) const {
  // Map parameters out
  const mat4_t T_C0F = tf(params[0]);
  const mat4_t T_C0Ci = tf(params[1]);
  Eigen::Map<const vecx_t> param(params[2], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0 = T_C0Ci.inverse();
  const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  vec2_t z_hat;
  bool valid = true;
  if (cam_geom->project(res, param, r_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residual
  Eigen::Map<vec2_t> r(residuals);
  r = sqrt_info * (z - z_hat);

  // Jacobians
  const matx_t Jh = cam_geom->project_jacobian(param, r_CiFi);
  const matx_t Jh_weighted = -1 * sqrt_info * Jh;

  if (jacobians) {
    // Jacobians w.r.t T_C0F
    if (jacobians[0]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0F = tf_rot(T_C0F);

        mat_t<2, 6, row_major_t> J_min;
        J_min.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
        J_min.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * r_FFi);

        J = J_min * lift_pose_jacobian(T_C0F);
      }
    }

    // Jacobians w.r.t T_C0Ci
    if (jacobians[1]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0Ci = C_CiC0.transpose();
        const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);

        // clang-format off
        mat_t<2, 6, row_major_t> J_min;
        J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
        J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
        // clang-format on

        J = J_min * lift_pose_jacobian(T_C0Ci);
      }
    }

    // Jacobians w.r.t cam params
    if (jacobians[2]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
      J.setZero();
      if (valid) {
        const matx_t J_cam = cam_geom->params_jacobian(param, r_CiFi);
        J.block(0, 0, 2, 8) = -1 * sqrt_info * J_cam;
      }
    }
  }

  return true;
}

// CALIB VIEW //////////////////////////////////////////////////////////////////

calib_view_t::calib_view_t(ceres::Problem *problem_,
                           camera_geometry_t *cam_geom_,
                           camera_params_t *cam_params_,
                           pose_t *T_BCi_,
                           pose_t *T_C0F_,
                           const aprilgrid_t &grid_,
                           const mat2_t &covar_)
    : problem{problem_}, grid{grid_}, cam_geom{cam_geom_},
      cam_params{cam_params_}, T_BCi{T_BCi_}, T_C0F{T_C0F_}, covar{covar_} {
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
    const vec3_t r_FFi = pts[i];
    auto res_fn = std::make_shared<reproj_error_t>(cam_geom,
                                                   cam_params,
                                                   T_BCi_,
                                                   T_C0F_,
                                                   tag_id,
                                                   corner_idx,
                                                   r_FFi,
                                                   z,
                                                   covar);
    auto res_id = problem->AddResidualBlock(res_fn.get(),
                                            // &loss,
                                            nullptr,
                                            T_C0F->data(),
                                            T_BCi->data(),
                                            cam_params->data());
    res_fns.push_back(res_fn);
    res_ids.push_back(res_id);
  }
}

std::vector<real_t> calib_view_t::get_reproj_errors() const {
  // Estimate relative pose T_C0F
  const auto intrinsics = cam_params->param;
  const auto res = cam_params->resolution;
  mat4_t T_CiF;
  if (grid.estimate(cam_geom, res, intrinsics, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  // Get AprilGrid measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t kps;
  vec3s_t pts;
  grid.get_measurements(tag_ids, corner_idxs, kps, pts);

  // Calculate reprojection errors
  std::vector<real_t> reproj_errors;

  for (size_t i = 0; i < tag_ids.size(); i++) {
    // const int tag_id = tag_ids[i];
    // const int corner_idx = corner_idxs[i];
    const vec2_t z = kps[i];
    const vec3_t r_FFi = pts[i];
    const vec3_t r_CiFi = tf_point(T_CiF, r_FFi);

    vec2_t z_hat;
    cam_geom->project(res, intrinsics, r_CiFi, z_hat);
    reproj_errors.push_back((z - z_hat).norm());
  }

  return reproj_errors;
}

int calib_view_t::filter_view(const real_t outlier_threshold) {
  // Calculate threshold
  const auto reproj_errors = get_reproj_errors();
  const auto error_stddev = stddev(reproj_errors);
  const auto threshold = outlier_threshold * error_stddev;

  // Filter
  int nb_inliers = 0;
  int nb_outliers = 0;
  auto res_fns_it = res_fns.begin();
  auto res_ids_it = res_ids.begin();

  while (res_fns_it != res_fns.end()) {
    auto &res = *res_fns_it;
    auto &res_id = *res_ids_it;

    real_t err = 0.0;
    if (res->get_reproj_error(err) == 0 && err > threshold) {
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

// CALIB VIEWS /////////////////////////////////////////////////////////////////

calib_views_t::calib_views_t(ceres::Problem *problem_,
                             camera_geometry_t *cam_geom_,
                             camera_params_t *cam_params_,
                             pose_t *cam_exts_)
    : problem{problem_}, cam_geom{cam_geom_},
      cam_params{cam_params_}, cam_exts{cam_exts_} {}

size_t calib_views_t::nb_views() const { return views.size(); }

void calib_views_t::add_view(const aprilgrid_t &grid, pose_t *rel_pose) {
  views.emplace_back(problem, cam_geom, cam_params, cam_exts, rel_pose, grid);
}

std::vector<real_t> calib_views_t::get_reproj_errors() const {
  std::vector<real_t> reproj_errors;
  for (auto &view : views) {
    auto r = view.get_reproj_errors();
    reproj_errors.insert(reproj_errors.end(), r.begin(), r.end());
  }
  return reproj_errors;
}

// CALIBRATOR //////////////////////////////////////////////////////////////////

void initialize_camera(const aprilgrids_t &grids,
                       camera_geometry_t *cam_geom,
                       camera_params_t *cam_params,
                       const bool verbose) {
  // Problem options
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;

  // Problem
  ceres::Problem problem{prob_options};
  PoseLocalParameterization pose_plus;

  // Camera
  const auto cam_res = cam_params->resolution;
  problem.AddParameterBlock(cam_params->data(), 8);

  // Extrinsics
  extrinsics_t cam_exts{};
  problem.AddParameterBlock(cam_exts.data(), 7);
  problem.SetParameterization(cam_exts.data(), &pose_plus);
  problem.SetParameterBlockConstant(cam_exts.data());

  // Poses
  std::map<timestamp_t, pose_t> poses;

  // Build problem
  calib_views_t calib_views{&problem, cam_geom, cam_params, &cam_exts};

  for (auto grid : grids) {
    const auto ts = grid.timestamp;
    if (grid.detected == false) {
      continue;
    }

    // Estimate relative pose
    mat4_t T_CiF_k;
    if (grid.estimate(cam_geom, cam_res, cam_params->param, T_CiF_k) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Add relative pose
    poses[ts] = pose_t{ts, T_CiF_k};
    problem.AddParameterBlock(poses[ts].data(), 7);
    problem.SetParameterization(poses[ts].data(), &pose_plus);

    // Add calibration view
    calib_views.add_view(grid, &poses[ts]);
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 30;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    // Show optimization report
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;

    // Show optimization results
    const auto reproj_errors = calib_views.get_reproj_errors();
    printf("Optimization results\n");
    printf("------------------------------------------------------------\n");
    printf("reproj_errors: [%f, %f, %f] [px] (rmse, mean, median)\n",
           rmse(reproj_errors),
           mean(reproj_errors),
           median(reproj_errors));
    printf("nb_views: %ld\n", calib_views.nb_views());
    printf("nb_corners: %ld\n", reproj_errors.size());
    printf("\n");
    printf("cam%d:\n", cam_params->cam_index);
    printf("  proj_model: %s\n", cam_params->proj_model.c_str());
    printf("  dist_model: %s\n", cam_params->dist_model.c_str());
    print_vector("  proj_params", cam_params->proj_params());
    print_vector("  dist_params", cam_params->dist_params());
  }
}

// CALIB CAMERA ////////////////////////////////////////////////////////////////

calib_camera_t::calib_camera_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
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

  if (problem) {
    delete problem;
  }
}

int calib_camera_t::nb_cams() const { return cam_params.size(); }

aprilgrids_t calib_camera_t::get_cam_data(const int cam_idx) const {
  aprilgrids_t grids;
  for (const auto &ts : timestamps) {
    grids.push_back(cam_data.at(ts).at(cam_idx));
  }
  return grids;
}

mat4_t calib_camera_t::get_camera_extrinsics(const int cam_idx) const {
  return cam_exts.at(cam_idx)->tf();
}

void calib_camera_t::add_calib_target(const calib_target_t &target_) {
  target = target_;
}

void calib_camera_t::add_camera_data(const int cam_idx,
                                     const aprilgrids_t &grids) {
  for (const auto &grid : grids) {
    const auto ts = grid.timestamp;
    timestamps.insert(ts);
    cam_data[ts][cam_idx] = grid;
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
    const double fx = pinhole_focal(cam_res[0], 90);
    const double fy = pinhole_focal(cam_res[0], 90);
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
  }

  // Add parameter to problem
  int params_size = proj_params.size() + dist_params.size();
  problem->AddParameterBlock(cam_params[cam_idx]->data(), params_size);
  if (fixed) {
    problem->SetParameterBlockConstant(cam_params[cam_idx]->data());
  }
}

void calib_camera_t::add_camera_extrinsics(const int cam_idx,
                                           const mat4_t &ext,
                                           const bool fixed) {
  cam_exts[cam_idx] = new extrinsics_t{ext};
  problem->AddParameterBlock(cam_exts[cam_idx]->data(), 7);
  problem->SetParameterization(cam_exts[cam_idx]->data(), &pose_plus);
  if (cam_idx == 0 || fixed) {
    problem->SetParameterBlockConstant(cam_exts[cam_idx]->data());
  }
}

void calib_camera_t::add_pose(const int cam_idx,
                              const aprilgrid_t &grid,
                              const bool fixed) {
  // Setup
  const auto ts = grid.timestamp;
  if (poses.count(ts) > 0) {
    FATAL("Implementation Error!");
    return;
  }

  // Estimate relative pose T_C0F
  const auto cam_geom = cam_geoms[cam_idx];
  const auto param = cam_params[cam_idx]->param;
  const auto res = cam_params[cam_idx]->resolution;
  mat4_t T_CiF;
  if (grid.estimate(cam_geom, res, param, T_CiF) != 0) {
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

void calib_camera_t::add_view(const aprilgrid_t &grid,
                              ceres::Problem *problem,
                              int cam_idx,
                              camera_geometry_t *cam_geom,
                              camera_params_t *cam_params,
                              extrinsics_t *cam_exts,
                              pose_t *rel_pose) {
  const timestamp_t ts = grid.timestamp;
  calib_views[cam_idx][ts] = std::make_unique<calib_view_t>(problem,
                                                            cam_geom,
                                                            cam_params,
                                                            cam_exts,
                                                            rel_pose,
                                                            grid);
}

void calib_camera_t::_initialize_intrinsics() {
  for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    printf("initializing cam%d intrinsics ...\n", cam_idx);
    const aprilgrids_t grids = get_cam_data(cam_idx);
    initialize_camera(grids, cam_geoms[cam_idx], cam_params[cam_idx]);
  }
}

void calib_camera_t::_initialize_extrinsics() {
  printf("initializing extrinsics ...\n");
  add_camera_extrinsics(0);
  if (nb_cams() == 1) {
    return;
  }

  camchain_t camchain{cam_data, cam_geoms, cam_params};
  for (int j = 1; j < nb_cams(); j++) {
    mat4_t T_C0Cj;
    if (camchain.find(0, j, T_C0Cj) != 0) {
      FATAL("Failed to initialze T_C0C%d", j);
    }

    add_camera_extrinsics(j, T_C0Cj);
  }
}

void calib_camera_t::_setup_problem() {
  for (const auto &ts : timestamps) {
    for (const auto &kv : cam_data[ts]) {
      const auto cam_idx = kv.first;
      auto grid = kv.second;

      // Check if AprilGrid was detected
      if (grid.detected == false) {
        continue;
      }

      // Add pose
      if (poses.count(ts) == 0) {
        add_pose(cam_idx, grid);
      }

      // Add calibration view
      add_view(grid,
               problem,
               cam_idx,
               cam_geoms[cam_idx],
               cam_params[cam_idx],
               cam_exts[cam_idx],
               poses[ts]);
    }
  }
}

void calib_camera_t::_filter_views() {
  int nb_outliers = 0;
  for (auto &[cam_idx, cam_views] : calib_views) {
    UNUSED(cam_idx);

    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      nb_outliers += view->filter_view(outlier_threshold);
    }
  }
}

int calib_camera_t::recover_calib_covar(matx_t &calib_covar) {
  // Setup covariance blocks to estimate
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  // -- Add camera parameter blocks
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    covar_blocks.push_back({param->param.data(), param->param.data()});
  }
  // -- Add camera extrinsics blocks
  for (auto &[cam_idx, param] : cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    UNUSED(cam_idx);
    covar_blocks.push_back({param->param.data(), param->param.data()});
  }

  // Estimate covariance
  ::ceres::Covariance::Options options;
  ::ceres::Covariance covar_est(options);
  if (covar_est.Compute(covar_blocks, problem) == false) {
    LOG_ERROR("Failed to estimate covariance!");
    LOG_ERROR("Maybe Hessian is not full rank?");
    return -1;
  }

  // Extract covariances sub-blocks
  std::map<int, Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> covar_cam_params;
  std::map<int, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covar_cam_exts;
  for (auto &[cam_idx, param] : cam_params) {
    UNUSED(cam_idx);
    auto param_ptr = param->param.data();
    auto covar_ptr = covar_cam_params[cam_idx].data();
    if (!covar_est.GetCovarianceBlock(param_ptr, param_ptr, covar_ptr)) {
      LOG_ERROR("Failed to estimate cam%d parameter covariance!", cam_idx);
      return -1;
    }
  }
  for (auto &[cam_idx, param] : cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    auto param_ptr = param->param.data();
    auto covar_ptr = covar_cam_exts[cam_idx].data();
    if (!covar_est.GetCovarianceBlockInTangentSpace(param_ptr,
                                                    param_ptr,
                                                    covar_ptr)) {
      LOG_ERROR("Failed to estimate cam%d extrinsics covariance!", cam_idx);
      return -1;
    }
  }

  // Form covariance matrix block
  const int calib_covar_size = (nb_cams() * 8) + ((nb_cams() - 1) * 6);
  calib_covar = zeros(calib_covar_size, calib_covar_size);
  for (auto &[cam_idx, covar] : covar_cam_params) {
    const auto idx = cam_idx * 8;
    calib_covar.block(idx, idx, 8, 8) = covar;
  }
  for (auto &[cam_idx, covar] : covar_cam_exts) {
    if (cam_idx == 0) {
      continue;
    }
    const auto idx = nb_cams() * 8 + ((cam_idx - 1) * 6);
    calib_covar.block(idx, idx, 6, 6) = covar;
  }

  // Check if calib_covar is full-rank?
  if (rank(calib_covar) != calib_covar.rows()) {
    LOG_ERROR("calib_covar is not full rank!");
    return -1;
  }

  return 0;
}

void calib_camera_t::solve() {
  // Setup
  _initialize_intrinsics();
  _initialize_extrinsics();
  _setup_problem();

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  matx_t calib_covar;
  recover_calib_covar(calib_covar);

  // Filter views
  if (enable_outlier_rejection) {
    _filter_views();

    // Solve again - second pass
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }

  // Show results
  show_results();
}

void calib_camera_t::solve_incremental() {
  // Setup
  _initialize_intrinsics();
  _initialize_extrinsics();
}

void calib_camera_t::show_results() {
  // Show results
  printf("Optimization results:\n");
  printf("---------------------\n");

  // Stats - Reprojection Errors
  std::vector<real_t> reproj_errors_all;
  std::map<int, std::vector<real_t>> reproj_errors_cams;
  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      const auto view_reproj_errors = view->get_reproj_errors();
      extend(reproj_errors_all, view_reproj_errors);
      extend(reproj_errors_cams[cam_idx], view_reproj_errors);
    }
  }
  // -- Print total reprojection error
  printf("Total reprojection error:\n");
  printf("  rmse:   %.4f # px\n", rmse(reproj_errors_all));
  printf("  mean:   %.4f # px\n", mean(reproj_errors_all));
  printf("  median: %.4f # px\n", median(reproj_errors_all));
  printf("  stddev: %.4f # px\n", stddev(reproj_errors_all));
  printf("\n");
  // -- Print camera reprojection error
  for (const auto &[cam_idx, cam_errors] : reproj_errors_cams) {
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

  // Camera Extrinsics
  for (int cam_idx = 1; cam_idx < nb_cams(); cam_idx++) {
    const auto key = "T_C0C" + std::to_string(cam_idx);
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
  std::vector<real_t> reproj_errors_all;
  std::map<int, std::vector<real_t>> reproj_errors_cams;
  for (auto &[cam_idx, cam_views] : calib_views) {
    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      const auto view_reproj_errors = view->get_reproj_errors();
      extend(reproj_errors_all, view_reproj_errors);
      extend(reproj_errors_cams[cam_idx], view_reproj_errors);
    }
  }

  // Calibration metrics
  fprintf(outfile, "calib_metrics:\n");
  fprintf(outfile, "  total_reproj_error:\n");
  fprintf(outfile, "    rmse:   %f # [px]\n", rmse(reproj_errors_all));
  fprintf(outfile, "    mean:   %f # [px]\n", mean(reproj_errors_all));
  fprintf(outfile, "    median: %f # [px]\n", median(reproj_errors_all));
  fprintf(outfile, "    stddev: %f # [px]\n", stddev(reproj_errors_all));
  fprintf(outfile, "\n");
  for (const auto &[cam_idx, errors] : reproj_errors_cams) {
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

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cams() >= 2) {
    for (int i = 1; i < nb_cams(); i++) {
      const mat4_t T_BCi = get_camera_extrinsics(i);
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

  return -1;
}

} //  namespace yac
