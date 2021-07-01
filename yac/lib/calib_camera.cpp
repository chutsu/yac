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
  extrinsics_t cam_exts{0};
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
    poses[ts] = pose_t{0, ts, T_CiF_k};
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

} //  namespace yac
