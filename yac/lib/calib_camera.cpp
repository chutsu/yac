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
                               const fiducial_corner_t *p_FFi_,
                               const vec2_t &z_,
                               const mat2_t &covar_)
    : cam_geom{cam_geom_}, cam_params{cam_params_}, T_BCi{T_BCi_},
      T_C0F{T_C0F_}, p_FFi{p_FFi_}, z{z_}, covar{covar_}, info{covar.inverse()},
      sqrt_info{info.llt().matrixU()} {
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // Camera-camera extrinsics
  block_sizes->push_back(7); // Camera-fiducial relative pose
  block_sizes->push_back(3); // Fiducial corner parameter
  block_sizes->push_back(8); // Camera parameters

  J0_min = zeros(2, 6);
  J1_min = zeros(2, 6);
  J2_min = zeros(2, 3);
  J3_min = zeros(2, 8);
}

int reproj_error_t::get_residual(vec2_t &z_hat, vec2_t &r) const {
  assert(T_BCi != nullptr);
  assert(T_C0F != nullptr);

  // Map parameters out
  const mat4_t T_C0Ci_ = T_BCi->tf();
  const mat4_t T_C0F_ = T_C0F->tf();
  const vec3_t p_FFi_ = p_FFi->param;

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0_ = T_C0Ci_.inverse();
  const vec3_t p_CiFi = tf_point(T_CiC0_ * T_C0F_, p_FFi_);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  auto param = cam_params->param;
  if (cam_geom->project(res, param, p_CiFi, z_hat) != 0) {
    return -1;
  }
  // -- Residual
  r = z - z_hat;

  return 0;
}

int reproj_error_t::get_residual(vec2_t &r) const {
  assert(T_BCi != nullptr);
  assert(T_C0F != nullptr);

  vec2_t z_hat = zeros(2, 1);
  if (get_residual(z_hat, r) != 0) {
    return -1;
  }

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
  const mat4_t T_C0Ci = tf(params[0]);
  const mat4_t T_C0F = tf(params[1]);
  Eigen::Map<const vecx_t> p_FFi(params[2], 3);
  Eigen::Map<const vecx_t> param(params[3], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0 = T_C0Ci.inverse();
  const vec3_t p_CiFi = tf_point(T_CiC0 * T_C0F, p_FFi);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  vec2_t z_hat;
  bool valid = true;
  if (cam_geom->project(res, param, p_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residual
  Eigen::Map<vec2_t> r(residuals);
  r = sqrt_info * (z - z_hat);

  // Jacobians
  const matx_t Jh = cam_geom->project_jacobian(param, p_CiFi);
  const matx_t Jh_weighted = -1 * sqrt_info * Jh;

  if (jacobians) {
    // Jacobians w.r.t T_C0Ci
    if (jacobians[0]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0Ci = C_CiC0.transpose();
        const vec3_t p_CiFi = tf_point(T_CiC0 * T_C0F, p_FFi);

        // clang-format off
        J0_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
        J0_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * p_CiFi);
        // clang-format on

        J = J0_min * lift_pose_jacobian(T_C0Ci);
      }
    }

    // Jacobians w.r.t T_C0F
    if (jacobians[1]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0F = tf_rot(T_C0F);
        J1_min.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
        J1_min.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * p_FFi);
        J = J1_min * lift_pose_jacobian(T_C0F);
      }
    }

    // Jacobians w.r.t fiducial corner
    if (jacobians[2]) {
      Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[2]);
      J.setZero();
      if (valid) {
        const mat4_t T_CiF = T_CiC0 * T_C0F;
        const mat3_t C_CiF = tf_rot(T_CiF);
        J2_min = Jh_weighted * C_CiF;
        J.block(0, 0, 2, 3) = J2_min;
      }
    }

    // Jacobians w.r.t cam params
    if (jacobians[3]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
      J.setZero();
      if (valid) {
        const matx_t J_cam = cam_geom->params_jacobian(param, p_CiFi);
        J3_min = -1 * sqrt_info * J_cam;
        J = J3_min;
      }
    }
  }

  return true;
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

// CALIBRATOR //////////////////////////////////////////////////////////////////

void initialize_camera(const calib_target_t &calib_target,
                       const aprilgrids_t &grids,
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
  std::unique_ptr<ceres::LossFunction> loss;
  ceres::Problem problem{prob_options};
  PoseLocalParameterization pose_plus;

  // Fiducial corners
  fiducial_corners_t corners{calib_target};
  const int nb_tags = calib_target.tag_rows * calib_target.tag_cols;
  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      auto corner = corners.get_corner(tag_id, corner_idx);
      problem.AddParameterBlock(corner->data(), 3);
      problem.SetParameterBlockConstant(corner->data());
    }
  }

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
  std::deque<calib_view_t> calib_views;
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
    calib_views.emplace_back(&problem,
                             loss.get(),
                             &corners,
                             cam_geom,
                             cam_params,
                             &cam_exts,
                             &poses[ts],
                             grid);
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 30;
  options.use_nonmonotonic_steps = true;
  // options.function_tolerance = 1e-10;
  // options.gradient_tolerance = 1e-10;
  // options.parameter_tolerance = 1e-10;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    // Get reprojection errors
    std::vector<real_t> reproj_errors;
    for (auto &view : calib_views) {
      auto r = view.get_reproj_errors();
      reproj_errors.insert(reproj_errors.end(), r.begin(), r.end());
    }

    // Show optimization report
    // std::cout << summary.BriefReport() << std::endl;
    std::cout << summary.FullReport() << std::endl;
    std::cout << std::endl;

    // Show optimization results
    printf("Optimization results\n");
    printf("------------------------------------------------------------\n");
    printf("reproj_errors: [%f, %f, %f] [px] (rmse, mean, median)\n",
           rmse(reproj_errors),
           mean(reproj_errors),
           median(reproj_errors));
    printf("nb_views: %ld\n", calib_views.size());
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

calib_camera_t::calib_camera_t(const calib_target_t &calib_target_)
    : calib_target{calib_target_} {
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

int calib_camera_t::nb_cams() const { return cam_params.size(); }

aprilgrids_t calib_camera_t::get_cam_data(const int cam_idx) const {
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
  if (cam_exts.count(cam_idx) == 0) {
    cam_exts[cam_idx] = new extrinsics_t{ext};
  }

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
  auto pose_ptr = poses[ts];
  problem->RemoveParameterBlock(pose_ptr->param.data());
  poses.erase(ts);
  delete pose_ptr;
}

void calib_camera_t::_initialize_intrinsics() {
  for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    LOG_INFO("Initializing cam%d intrinsics ...", cam_idx);
    const aprilgrids_t grids = get_cam_data(cam_idx);
    initialize_camera(calib_target,
                      grids,
                      cam_geoms[cam_idx],
                      cam_params[cam_idx]);
  }
}

void calib_camera_t::_initialize_extrinsics() {
  add_camera_extrinsics(0);
  if (nb_cams() == 1) {
    return;
  }

  LOG_INFO("Initializing extrinsics ...");
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
  size_t ts_idx = 0;

  progressbar bar(timestamps.size());
  if (enable_nbv) {
    printf("Solving full problem: ");
  }

  for (const auto &ts : timestamps) {
    bool new_view_added = false;
    for (const auto &[cam_idx, grid] : cam_data[ts]) {
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
               loss,
               cam_idx,
               cam_geoms[cam_idx],
               cam_params[cam_idx],
               cam_exts[cam_idx],
               poses[ts]);
      new_view_added = true;
    } // Iterate camera data
    ts_idx++;

    // Check if new view has been added
    if (new_view_added == false) {
      continue;
    }

    // Check number of views before performing NBV
    if (enable_nbv == false || calib_views[0].size() < min_nbv_views) {
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
      _filter_view(ts);
      ceres::Solve(options, problem, &summary);
    }

    // Check information
    matx_t calib_covar;
    int retval = recover_calib_covar(calib_covar);
    if (retval != 0) {
      continue;
    }

    // Calculate information gain
    const auto reproj_errors_all = get_all_reproj_errors();
    const real_t calib_info_kp1 = info_entropy(calib_covar);
    const real_t info_gain = 0.5 * (calib_info_kp1 - calib_info_k);
    // clang-format off
    // printf("[%.2f%%] ", ((real_t)ts_idx / (real_t)timestamps.size()) * 100.0);
    // printf("nb_views: %ld  ", calib_views[0].size());
    // printf("info_k: %.2f ", calib_info_k);
    // printf("reproj_error: %.2f ", rmse(reproj_errors_all));
    // printf("shannon_entropy: %.2f  ", shannon_entropy(calib_covar));
    // printf("\n");
    // clang-format on

    if (info_gain < info_gain_threshold) {
      remove_view(ts);
    } else {
      calib_info_k = calib_info_kp1;
    }

    bar.update();
  } // Iterate timestamps
}

int calib_camera_t::_filter_views() {
  int removed = 0;
  for (auto &[cam_idx, cam_views] : calib_views) {
    UNUSED(cam_idx);

    for (auto &[ts, view] : cam_views) {
      UNUSED(ts);
      removed += view->filter_view(outlier_threshold);
    }
  }

  return removed;
}

int calib_camera_t::_filter_view(const timestamp_t ts) {
  int removed = 0;
  for (auto &[cam_idx, cam_views] : calib_views) {
    UNUSED(cam_idx);
    if (cam_views.count(ts) == 0) {
      continue;
    }

    if (cam_views[ts]) {
      removed += cam_views[ts]->filter_view(outlier_threshold);
    }
  }

  return removed;
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

int calib_camera_t::find_nbv(const std::map<int, mat4s_t> &nbv_poses,
                             int &cam_idx,
                             int &nbv_idx) {
  // Pre-check
  if (nbv_poses.size() == 0) {
    FATAL("NBV poses empty!?");
  }

  // Find NBV
  const timestamp_t last_ts = *timestamps.rbegin(); // std::set is orderd
  const timestamp_t nbv_ts = last_ts + 1;
  int best_cam = 0;
  int best_idx = 0;
  real_t best_entropy = 0;

  for (const auto &[nbv_cam_idx, nbv_cam_poses] : nbv_poses) {
    for (size_t i = 0; i < nbv_cam_poses.size(); i++) {
      // Simulate current NBV
      const mat4_t T_FC0 = nbv_cam_poses.at(i);
      const mat4_t T_C0F = T_FC0.inverse();
      pose_t rel_pose{nbv_ts, T_C0F};

      for (const auto cam_idx : get_camera_indices()) {
        const auto &grid = nbv_target_grid(calib_target,
                                           cam_geoms[cam_idx],
                                           cam_params[cam_idx],
                                           nbv_ts,
                                           rel_pose.tf());
        add_view(grid,
                 problem,
                 loss,
                 cam_idx,
                 cam_geoms[cam_idx],
                 cam_params[cam_idx],
                 cam_exts[cam_idx],
                 &rel_pose);
      }

      // Evaluate NBV
      // -- Estimate calibration covariance
      matx_t calib_covar;
      if (recover_calib_covar(calib_covar) == 0) {
        remove_view(nbv_ts);
      }
      remove_view(nbv_ts);
      // -- Calculate entropy
      const real_t entropy = info_entropy(calib_covar);
      if (entropy > best_entropy) {
        best_cam = nbv_cam_idx;
        best_idx = i;
        best_entropy = entropy;
      }
    }
  }

  // Return
  cam_idx = best_cam;
  nbv_idx = best_idx;

  return 0;
}

void calib_camera_t::solve() {
  // Setup
  LOG_INFO("Solving camera calibration problem:");
  _initialize_intrinsics();
  _initialize_extrinsics();
  _setup_problem();

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 100;
  options.use_nonmonotonic_steps = true;
  // options.function_tolerance = 1e-20;
  // options.gradient_tolerance = 1e-20;
  // options.parameter_tolerance = 1e-20;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }

  // Filter views
  if (enable_outlier_rejection) {
    LOG_INFO("Removing outliers\n");
    _filter_views();

    // Solve again - second pass
    LOG_INFO("Solving again!\n");
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    if (verbose) {
      std::cout << summary.BriefReport() << std::endl;
      std::cout << std::endl;
    }
  }

  // Show results
  show_results();
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
  fprintf(outfile, "\n");

  // Calibration target
  fprintf(outfile, "calib_target:\n");
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
