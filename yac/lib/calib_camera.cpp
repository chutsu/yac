#include "calib_camera.hpp"

namespace yac {

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
  mat2_t covar = I(2);
  PoseLocalParameterization pose_plus;

  // Camera
  const auto cam_idx = cam_params.cam_index;
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

calib_camera_t::~calib_camera_t() {
  delete problem;
}

int calib_camera_t::nb_cams() const {
  return cam_params.size();
}

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
  mat2_t covar = I(2);

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
