#include "CalibCamera.hpp"

namespace yac {

CalibCamera::CalibCamera(const std::string &config_file)
    : CalibData{config_file} {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);

  // Add camera intrinsic and extrinsics to problem
  for (auto &[camera_index, camera_geometry] : camera_geometries_) {
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    problem_->AddParameterBlock(intrinsic, intrinsic_size);
    problem_->AddParameterBlock(extrinsic, 7);
    problem_->SetManifold(extrinsic, &pose_plus_);
  }

  // Monocular camera calibration - Fix camera0 extrinsic
  if (getNumCameras() == 1) {
    auto camera_geometry = getCameraGeometry(0);
    problem_->SetParameterBlockConstant(camera_geometry->getExtrinsicPtr());
  }
}

void CalibCamera::initializeCamera(const int camera_index) {
  // Setup Problem
  std::map<timestamp_t, vec7_t> relposes;
  std::vector<std::shared_ptr<ResidualBlock>> resblocks;
  auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

  // Add camera to problem
  auto camera_geometry = getCameraGeometry(camera_index);
  const int intrinsic_size = camera_geometry->getIntrinsic().size();
  double *intrinsic = camera_geometry->getIntrinsicPtr();
  double *extrinsic = camera_geometry->getExtrinsicPtr();
  init_problem->AddParameterBlock(intrinsic, intrinsic_size);
  init_problem->AddParameterBlock(extrinsic, 7);
  init_problem->SetManifold(extrinsic, &pose_plus_);
  init_problem->SetParameterBlockConstant(extrinsic);

  // Build problem
  for (const auto &[ts, calib_target] : getCameraData(camera_index)) {
    // Check if detected
    if (calib_target->detected() == false) {
      continue;
    }

    // Get calibration target measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    calib_target->getMeasurements(tag_ids,
                                  corner_indicies,
                                  keypoints,
                                  object_points);
    if (keypoints.size() < 10) {
      continue;
    }

    // Estimate relative pose T_CF
    mat4_t T_CF;
    SolvePnp pnp{camera_geometry};
    int status = pnp.estimate(keypoints, object_points, T_CF);
    if (status != 0) {
      continue;
    }

    // Add pose
    mat2_t covar = I(2);
    relposes[ts] = tf_vec(T_CF);
    init_problem->AddParameterBlock(relposes[ts].data(), 7);
    init_problem->SetManifold(relposes[ts].data(), &pose_plus_);

    // Add residual blocks
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int point_id = (tag_ids[i] * 4) + corner_indicies[i];
      addTargetPoint(point_id, object_points[i]);

      vec3_t &pt = getTargetPoint(point_id);
      init_problem->AddParameterBlock(pt.data(), 3);
      init_problem->SetParameterBlockConstant(pt.data());

      auto resblock = CameraResidual::create(camera_geometry,
                                             relposes[ts].data(),
                                             pt.data(),
                                             keypoints[i],
                                             covar);
      resblocks.push_back(resblock);
      init_problem->AddResidualBlock(resblock.get(),
                                     nullptr,
                                     resblock->getParamPtrs());
    }
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 30;
  options.num_threads = 1;
  options.initial_trust_region_radius = 10; // Default: 1e4
  options.min_trust_region_radius = 1e-50;  // Default: 1e-32
  options.function_tolerance = 1e-20;       // Default: 1e-6
  options.gradient_tolerance = 1e-20;       // Default: 1e-10
  options.parameter_tolerance = 1e-20;      // Default: 1e-8

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, init_problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl << std::endl;
}

void CalibCamera::addView(const std::map<int, CalibTargetPtr> &measurements) {
  // Check if AprilGrid was detected
  bool detected = false;
  timestamp_t ts = 0;
  for (const auto &[camera_index, calib_target] : measurements) {
    if (calib_target->detected()) {
      detected = true;
      ts = calib_target->getTimestamp();
      break;
    }
  }
  if (detected == false) {
    return;
  }

  // Add to calib data if it does not exist
  for (const auto &[camera_index, calib_target] : measurements) {
    if (hasCameraMeasurement(ts, camera_index) == false) {
      addCameraMeasurement(ts, camera_index, calib_target);
    }
  }

  // Add calibration view
  timestamps_.insert(ts);
  // calib_views_[ts] = new calib_view_t{ts,
  //                                    cam_grids,
  //                                    corners.get(),
  //                                    solver,
  //                                    &cam_geoms,
  //                                    &cam_params,
  //                                    &cam_exts,
  //                                    poses[ts].get(),
  //                                    loss_fn};
}

} // namespace yac
