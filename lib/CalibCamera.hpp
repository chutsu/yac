#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "CameraResidual.hpp"

namespace yac {

/** Calibration view */
class CalibView {
private:
  std::shared_ptr<ceres::Problem> &problem_;
  PoseLocalParameterization pose_plus_;
  timestamp_t ts_ = 0;
  const std::map<int, CalibTargetPtr> view_data_;
  std::map<int, CameraGeometryPtr> &camera_geometries_;
  std::map<int, vec3_t> &target_points_;
  vec7_t pose_;

  std::map<int, std::vector<std::shared_ptr<CameraResidual>>> resblocks_;
  std::map<int, std::vector<ceres::ResidualBlockId>> resblock_ids_;

public:
  CalibView() = delete;
  CalibView(std::shared_ptr<ceres::Problem> &problem,
            const timestamp_t ts,
            const std::map<int, CalibTargetPtr> &view_data,
            std::map<int, CameraGeometryPtr> &camera_geometries,
            std::map<int, vec3_t> &target_points,
            const vec7_t &pose)
      : problem_{problem}, ts_{ts}, view_data_{view_data},
        camera_geometries_{camera_geometries},
        target_points_{target_points}, pose_{pose} {
    // Add pose
    problem_->AddParameterBlock(pose_.data(), 7);
    problem_->SetManifold(pose_.data(), &pose_plus_);

    // Add camera residuals
    const mat2_t covar = I(2);
    for (const auto &[camera_index, calib_target] : view_data) {
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

      // Add residual blocks
      for (size_t i = 0; i < tag_ids.size(); i++) {
        // Add target point
        const int point_id = (tag_ids[i] * 4) + corner_indicies[i];
        vec3_t &pt = target_points[point_id];
        problem_->AddParameterBlock(pt.data(), 3);
        problem_->SetParameterBlockConstant(pt.data());

        // Create residual
        auto camera_geometry = camera_geometries_[camera_index];
        auto pose_ptr = pose_.data();
        auto pt_ptr = pt.data();
        auto kp = keypoints[i];
        auto resblock = CameraResidual::create(camera_geometry,
                                               pose_ptr,
                                               pt_ptr,
                                               kp,
                                               covar);

        // Add residual block
        auto res_ptr = resblock.get();
        auto loss_ptr = nullptr;
        auto param_ptrs = resblock->getParamPtrs();
        auto res_id = problem_->AddResidualBlock(res_ptr, loss_ptr, param_ptrs);

        // Book keeping
        resblocks_[camera_index].push_back(resblock);
        resblock_ids_[camera_index].push_back(res_id);
      }
    }
  }
  virtual ~CalibView() = default;

  /** Get number of detections */
  int numDetections() const {
    int num_detections = 0;
    for (const auto &[camera_index, camera_residuals] : resblocks_) {
      num_detections += camera_residuals.size();
    }
    return num_detections;
  }

  /** Get number of detections by camera */
  int numDetections(const int camera_index) const {
    return resblocks_.at(camera_index).size();
  }

  /** Get camera indicies */
  std::vector<int> getCameraIndices() const {
    std::vector<int> camera_indicies;
    for (const auto &[camera_index, camera_residuals] : resblocks_) {
      camera_indicies.push_back(camera_index);
    }
    return camera_indicies;
  }
};

/** Camera Calibrator **/
class CalibCamera : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseLocalParameterization pose_plus_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, std::shared_ptr<CalibView>> calib_views_;

public:
  CalibCamera(const std::string &config_file);

  /** Initialize camera parameters */
  void initializeCamera(const int camera_index);

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetPtr> &measurements);

  /** Solve */
  void solve();
};

} // namespace yac
