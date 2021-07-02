#include "calib_camera.hpp"

namespace yac {

void initialize_camera(const aprilgrids_t &grids,
                       const camera_geometry_t *cam_geom,
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

  for (const auto &grid : grids) {
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

    // Add reprojection factors to problem
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_indicies[i];
      const vec2_t z = keypoints[i];
      const vec3_t r_FFi = object_points[i];

      auto cost_fn = new reproj_error_t{cam_geom, cam_idx, cam_res,
                                        tag_id, corner_idx,
                                        r_FFi, z, covar};
      auto res_id = problem.AddResidualBlock(cost_fn,
                                             NULL,
                                             poses[ts].data(),
                                             cam_exts.data(),
                                             cam_params.data());
    }
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 30;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }
}

/**************************** calib_camera_t **********************************/

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

void calib_camera_t::add_camera_data(const int cam_idx, const aprilgrids_t &grids) {
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
  camera_params_t params(cam_idx, cam_res,
                         proj_model, dist_model,
                         proj_params, dist_params,
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

pose_t &calib_camera_t::add_pose(const timestamp_t &ts,
                                 const mat4_t &T,
                                 const bool fixed) {
  poses[ts] = pose_t{ts, T};
  problem->AddParameterBlock(poses[ts].data(), 7);
  problem->SetParameterization(poses[ts].data(), &pose_plus);
  if (fixed) {
    problem->SetParameterBlockConstant(poses[ts].data());
  }
  return poses[ts];
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

  for (const auto &ts: timestamps) {
    for (const auto &kv: cam_data[ts]) {
      const auto cam_idx = kv.first;
      const auto &grid = kv.second;
      if (grid.detected == false) {
        continue;
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
      if (poses.count(ts) == 0) {
        add_pose(ts, T_C0F);
      }

      // Add reprojection factors to problem
      std::vector<int> tag_ids;
      std::vector<int> corner_idxs;
      vec2s_t kps;
      vec3s_t pts;
      grid.get_measurements(tag_ids, corner_idxs, kps, pts);

      for (size_t i = 0; i < tag_ids.size(); i++) {
        const int tag_id = tag_ids[i];
        const int corner_idx = corner_idxs[i];
        const vec2_t z = kps[i];
        const vec3_t r_FFi = pts[i];

        auto cost_fn = new reproj_error_t{cam_geom, cam_idx, res,
                                          tag_id, corner_idx,
                                          r_FFi, z, covar};
        auto res_id = problem->AddResidualBlock(cost_fn,
                                                NULL,
                                                poses[ts].data(),
                                                cam_exts[cam_idx].data(),
                                                cam_params[cam_idx].data());
      }
    }
  }
}

void calib_camera_t::solve() {
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
