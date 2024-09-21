#include "CalibCamera.hpp"
#include "CameraChain.hpp"
#include "Timeline.hpp"

namespace yac {

CalibCamera::CalibCamera(const std::string &config_file)
    : CalibData{config_file} {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);
}

void CalibCamera::initializeIntrinsics() {
  for (const auto &[camera_index, camera_measurements] : getAllCameraData()) {
    // Setup Problem
    std::map<timestamp_t, vec7_t> relposes;
    std::vector<std::shared_ptr<ReprojectionError>> resblocks;
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

        auto resblock = ReprojectionError::create(camera_geometry,
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

    std::vector<double> reproj_errors;
    for (const auto &resblock : resblocks) {
      double error = 0.0;
      if (resblock->getReprojError(&error)) {
        reproj_errors.push_back(error);
      }
    }
    printf("mean reproj error: %f\n", mean(reproj_errors));
    printf("rmse reproj error: %f\n", rmse(reproj_errors));
    printf("median reproj error: %f\n", median(reproj_errors));
  }
}

void CalibCamera::initializeExtrinsics() {
  // Setup camera chain
  CameraChain camchain(getAllCameraGeometries(), getAllCameraData());
  for (auto &[camera_index, camera_geometry] : getAllCameraGeometries()) {
    mat4_t T_CiCj;
    if (camchain.find(0, camera_index, T_CiCj) != 0) {
      FATAL("No observations between camera0 and camera%d\n", camera_index);
    }
    camera_geometry->setExtrinsic(T_CiCj);
  }

  // Setup Problem
  mat2_t covar = I(2);
  std::map<timestamp_t, vec7_t> relposes;
  std::vector<std::shared_ptr<ReprojectionError>> resblocks;
  auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

  // Build problem
  for (const auto &[camera_index, camera_measurements] : getAllCameraData()) {
    auto camera_geometry = getCameraGeometry(camera_index);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus_);
    if (camera_index == 0) {
      init_problem->SetParameterBlockConstant(extrinsic);
    }

    // Add camera measurements
    for (const auto &[ts, calib_target] : camera_measurements) {
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

      // Estimate relative pose T_C0F
      mat4_t T_C0F;
      if (relposes.count(ts) == 0) {
        // Solvepnp T_CiF
        mat4_t T_CiF;
        SolvePnp pnp{camera_geometry};
        int status = pnp.estimate(keypoints, object_points, T_CiF);
        if (status != 0) {
          continue;
        }

        // Form T_C0F
        const mat4_t T_C0Ci = camera_geometry->getTransform();
        T_C0F = T_C0Ci * T_CiF;

        // Add pose
        relposes[ts] = tf_vec(T_C0F);
        init_problem->AddParameterBlock(relposes[ts].data(), 7);
        init_problem->SetManifold(relposes[ts].data(), &pose_plus_);
      } else {
        T_C0F = tf(relposes[ts]);
      }

      // Add residual blocks
      for (size_t i = 0; i < tag_ids.size(); i++) {
        const int point_id = (tag_ids[i] * 4) + corner_indicies[i];
        addTargetPoint(point_id, object_points[i]);

        vec3_t &pt = getTargetPoint(point_id);
        init_problem->AddParameterBlock(pt.data(), 3);
        init_problem->SetParameterBlockConstant(pt.data());

        auto resblock = ReprojectionError::create(camera_geometry,
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

  std::vector<double> reproj_errors;
  for (const auto &resblock : resblocks) {
    double error = 0.0;
    if (resblock->getReprojError(&error)) {
      reproj_errors.push_back(error);
    }
  }

  printf("mean reproj error: %f\n", mean(reproj_errors));
  printf("rmse reproj error: %f\n", rmse(reproj_errors));
  printf("median reproj error: %f\n", median(reproj_errors));
}

void CalibCamera::addView(const std::map<int, CalibTargetPtr> &measurements) {
  // Check if AprilGrid was detected and get the best detected target
  int best_camera_index = 0;
  CalibTargetPtr best_target = nullptr;
  for (const auto &[camera_index, calib_target] : measurements) {
    // Check if calibration target is detected
    if (calib_target->detected() == false) {
      continue;
    }

    // Keep track of the best detected calibration target
    if (best_target == nullptr) {
      best_camera_index = camera_index;
      best_target = calib_target;
    } else if (calib_target->getNumDetected() > best_target->getNumDetected()) {
      best_camera_index = camera_index;
      best_target = calib_target;
    }
  }
  if (best_target == nullptr) {
    return;
  }

  // Get calibration target measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  best_target->getMeasurements(tag_ids,
                               corner_indicies,
                               keypoints,
                               object_points);
  if (keypoints.size() < 10) {
    return;
  }

  // Estimate relative pose T_CF
  mat4_t T_CiF;
  SolvePnp pnp{camera_geometries_[best_camera_index]};
  int status = pnp.estimate(keypoints, object_points, T_CiF);
  if (status != 0) {
    return;
  }

  // Add to calib data if it does not exist
  const timestamp_t ts = best_target->getTimestamp();
  for (const auto &[camera_index, calib_target] : measurements) {
    if (hasCameraMeasurement(ts, camera_index) == false) {
      addCameraMeasurement(ts, camera_index, calib_target);
    }
  }

  // Add calibration view
  const mat4_t T_C0Ci = camera_geometries_[best_camera_index]->getTransform();
  const mat4_t T_C0F = T_C0Ci * T_CiF;
  const vec7_t pose = tf_vec(T_C0F);
  timestamps_.insert(ts);
  calib_views_[ts] = std::make_shared<CalibView>(problem_,
                                                 ts,
                                                 measurements,
                                                 camera_geometries_,
                                                 target_points_,
                                                 pose);
}

void CalibCamera::solve() {
  // Initialize intrinsics and extrinsics
  initializeIntrinsics();
  initializeExtrinsics();

  // Add camera intrinsic and extrinsics to problem
  for (auto &[camera_index, camera_geometry] : camera_geometries_) {
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    problem_->AddParameterBlock(intrinsic, intrinsic_size);
    problem_->AddParameterBlock(extrinsic, 7);
    problem_->SetManifold(extrinsic, &pose_plus_);

    if (camera_index == 0) {
      problem_->SetParameterBlockConstant(extrinsic);
    }
  }

  // Form timeline
  Timeline timeline;
  for (const auto &[camera_index, measurements] : camera_data_) {
    for (const auto &[ts, calib_target] : measurements) {
      timeline.add(ts, camera_index, calib_target);
    }
  }

  // Loop through timeline
  for (const auto ts : timeline.timestamps) {
    std::map<int, CalibTargetPtr> viewset;
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        auto camera_index = target_event->camera_index;
        auto calib_target = target_event->calib_target;
        viewset[camera_index] = calib_target;
      }
    }
    addView(viewset);
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
  ceres::Solve(options, problem_.get(), &summary);
  std::cout << summary.FullReport() << std::endl << std::endl;

  printSummary(stdout);
}

} // namespace yac
