#include "calib_camera.hpp"

namespace yac {

// REPROJECTION ERROR //////////////////////////////////////////////////////////

reproj_error_t::reproj_error_t(const camera_geometry_t *cam_geom_,
                               const int cam_idx_,
                               const int cam_res_[2],
                               const int tag_id_,
                               const int corner_idx_,
                               const vec3_t &r_FFi_,
                               const vec2_t &z_,
                               const mat2_t &covar_)
    : cam_geom{cam_geom_}, cam_idx{cam_idx_}, cam_res{cam_res_[0], cam_res_[1]},
      tag_id{tag_id_},
      corner_idx{corner_idx_}, r_FFi{r_FFi_}, z{z_}, covar{covar_},
      info{covar.inverse()}, sqrt_info{info.llt().matrixL().transpose()} {
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // camera-fiducial relative pose
  block_sizes->push_back(7); // camera-camera extrinsics
  block_sizes->push_back(8); // camera parameters
}

/** Evaluate */
bool reproj_error_t::Evaluate(double const *const *params,
                              double *residuals,
                              double **jacobians) const {
  // Map parameters out
  const mat4_t T_C0F = tf(params[0]);
  const mat4_t T_C0Ci = tf(params[1]);
  Eigen::Map<const vecx_t> cam_params(params[2], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0 = T_C0Ci.inverse();
  const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
  // -- Project point from camera frame to image plane
  vec2_t z_hat;
  bool valid = true;
  if (cam_geom->project(cam_res, cam_params, r_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residual
  Eigen::Map<vec2_t> r(residuals);
  r = sqrt_info * (z - z_hat);

  // Jacobians
  const matx_t Jh = cam_geom->project_jacobian(cam_params, r_CiFi);
  const matx_t Jh_weighted = -1 * sqrt_info * Jh;

  if (jacobians) {
    // Jacobians w.r.t T_C0F
    if (jacobians[0]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0F = tf_rot(T_C0F);
        J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
        J.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * r_FFi);
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
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
        J.block(0, 3, 2, 3) =
            -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
      }
    }

    // Jacobians w.r.t cam params
    if (jacobians[2]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
      J.setZero();
      if (valid) {
        const matx_t J_cam = cam_geom->params_jacobian(cam_params, r_CiFi);
        J.block(0, 0, 2, 8) = -1 * sqrt_info * J_cam;
      }
    }
  }

  return true;
}

// CALIB VIEW //////////////////////////////////////////////////////////////////

calib_view_t::calib_view_t(ceres::Problem &problem_,
                           aprilgrid_t &grid_,
                           pose_t &T_C0F_,
                           pose_t &T_BCi_,
                           camera_geometry_t *cam_geom_,
                           camera_params_t &cam_params_,
                           const mat2_t &covar_)
    : problem{problem_}, grid{grid_}, cam_geom{cam_geom_},
      cam_params{cam_params_}, T_C0F{T_C0F_}, T_BCi{T_BCi_}, covar{covar_} {
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
    auto cost_fn = std::make_shared<reproj_error_t>(cam_geom,
                                                    cam_params.cam_index,
                                                    cam_params.resolution,
                                                    tag_id,
                                                    corner_idx,
                                                    r_FFi,
                                                    z,
                                                    covar);
    auto res_id = problem.AddResidualBlock(cost_fn.get(),
                                           NULL,
                                           T_C0F.data(),
                                           T_BCi.data(),
                                           cam_params.data());
    cost_fns.push_back(cost_fn);
    res_ids.push_back(res_id);
  }
}

std::vector<double> calib_view_t::calculate_reproj_errors() const {
  // Estimate relative pose T_C0F
  const auto intrinsics = cam_params.param;
  const auto res = cam_params.resolution;
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
  std::vector<double> reproj_errors;

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

// CALIB VIEWS /////////////////////////////////////////////////////////////////

calib_views_t::calib_views_t(ceres::Problem &problem_,
                             pose_t &extrinsics_,
                             camera_geometry_t *cam_geom_,
                             camera_params_t &cam_params_)
    : problem{problem_}, extrinsics{extrinsics_}, cam_geom{cam_geom_},
      cam_params{cam_params_} {}

size_t calib_views_t::size() const { return views.size(); }

void calib_views_t::add_view(aprilgrid_t &grid, pose_t &rel_pose) {
  views.emplace_back(problem, grid, rel_pose, extrinsics, cam_geom, cam_params);
}

std::vector<double> calib_views_t::calculate_reproj_errors() const {
  std::vector<double> reproj_errors;
  for (auto &view : views) {
    auto r = view.calculate_reproj_errors();
    reproj_errors.insert(reproj_errors.end(), r.begin(), r.end());
  }
  return reproj_errors;
}

// CALIBRATOR //////////////////////////////////////////////////////////////////

void initialize_camera(const aprilgrids_t &grids,
                       camera_geometry_t *cam_geom,
                       camera_params_t &cam_params,
                       const bool verbose) {
  // Problem options
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;

  // Problem
  ceres::Problem problem{prob_options};
  PoseLocalParameterization pose_plus;

  // Camera
  const auto cam_res = cam_params.resolution;
  problem.AddParameterBlock(cam_params.data(), 8);

  // Extrinsics
  extrinsics_t cam_exts{};
  problem.AddParameterBlock(cam_exts.data(), 7);
  problem.SetParameterization(cam_exts.data(), &pose_plus);
  problem.SetParameterBlockConstant(cam_exts.data());

  // Poses
  std::map<timestamp_t, pose_t> poses;

  // Build problem
  calib_views_t calib_views{problem, cam_exts, cam_geom, cam_params};

  for (auto grid : grids) {
    const auto ts = grid.timestamp;
    if (grid.detected == false) {
      continue;
    }

    // Estimate relative pose
    mat4_t T_CiF_k;
    if (grid.estimate(cam_geom, cam_res, cam_params.param, T_CiF_k) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Add relative pose
    poses[ts] = pose_t{ts, T_CiF_k};
    problem.AddParameterBlock(poses[ts].data(), 7);
    problem.SetParameterization(poses[ts].data(), &pose_plus);

    // Add calibration view
    calib_views.add_view(grid, poses[ts]);
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
    const auto reproj_errors = calib_views.calculate_reproj_errors();
    printf("Optimization results\n");
    printf("------------------------------------------------------------\n");
    printf("reproj_errors: [%f, %f, %f] [px] (rmse, mean, median)\n",
           rmse(reproj_errors),
           mean(reproj_errors),
           median(reproj_errors));
    printf("nb_images: %ld\n", calib_views.size());
    printf("nb_corners: %ld\n", reproj_errors.size());
    printf("\n");
    printf("cam%d:\n", cam_params.cam_index);
    printf("  proj_model: %s\n", cam_params.proj_model.c_str());
    printf("  dist_model: %s\n", cam_params.dist_model.c_str());
    print_vector("  proj_params", cam_params.proj_params());
    print_vector("  dist_params", cam_params.dist_params());
  }
}

// CALIB CAMERA ////////////////////////////////////////////////////////////////

calib_camera_t::calib_camera_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
}

calib_camera_t::~calib_camera_t() { delete problem; }

int calib_camera_t::nb_cams() const { return cam_params.size(); }

aprilgrids_t calib_camera_t::get_cam_data(const int cam_idx) const {
  aprilgrids_t grids;
  for (const auto &ts : timestamps) {
    grids.push_back(cam_data.at(ts).at(cam_idx));
  }
  return grids;
}

mat4_t calib_camera_t::get_camera_extrinsics(const int cam_idx) const {
  return cam_exts.at(cam_idx).tf();
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
  camera_params_t params(cam_idx,
                         cam_res,
                         proj_model,
                         dist_model,
                         proj_params,
                         dist_params,
                         fixed);
  cam_params[cam_idx] = params;

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = &pinhole_radtan4;
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = &pinhole_equi4;
  }

  // Add parameter to problem
  int params_size = proj_params.size() + dist_params.size();
  problem->AddParameterBlock(cam_params[cam_idx].data(), params_size);
  if (fixed) {
    problem->SetParameterBlockConstant(cam_params[cam_idx].data());
  }
}

void calib_camera_t::add_camera_extrinsics(const int cam_idx,
                                           const mat4_t &ext,
                                           const bool fixed) {
  cam_exts[cam_idx] = extrinsics_t{ext};
  problem->AddParameterBlock(cam_exts[cam_idx].data(), 7);
  problem->SetParameterization(cam_exts[cam_idx].data(), &pose_plus);
  if (cam_idx == 0 || fixed) {
    problem->SetParameterBlockConstant(cam_exts[cam_idx].data());
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
  const auto param = cam_params[cam_idx].param;
  const auto res = cam_params[cam_idx].resolution;
  mat4_t T_CiF;
  if (grid.estimate(cam_geom, res, param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  // Add relative pose T_C0F
  const mat4_t T_C0Ci = get_camera_extrinsics(cam_idx);
  const mat4_t T_C0F = T_C0Ci * T_CiF;
  poses[ts] = pose_t{ts, T_C0F};
  problem->AddParameterBlock(poses[ts].data(), 7);
  problem->SetParameterization(poses[ts].data(), &pose_plus);
  if (fixed) {
    problem->SetParameterBlockConstant(poses[ts].data());
  }
}

void calib_camera_t::add_view(ceres::Problem &problem,
                              int cam_idx,
                              aprilgrid_t &grid,
                              pose_t &rel_pose,
                              extrinsics_t &extrinsics,
                              camera_geometry_t *cam_geom,
                              camera_params_t &cam_params) {
  const timestamp_t ts = grid.timestamp;
  calib_views[cam_idx][ts] = new calib_view_t{problem,
                                              grid,
                                              rel_pose,
                                              extrinsics,
                                              cam_geom,
                                              cam_params};
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
      add_view(*problem,
               cam_idx,
               grid,
               poses[ts],
               cam_exts[cam_idx],
               cam_geoms[cam_idx],
               cam_params[cam_idx]);
    }
  }
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
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;
}

} //  namespace yac
