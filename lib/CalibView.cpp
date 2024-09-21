#include "CalibView.hpp"

namespace yac {

CalibView::CalibView(std::shared_ptr<ceres::Problem> &problem,
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
      auto resblock = ReprojectionError::create(camera_geometry,
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

int CalibView::numDetections() const {
  int num_detections = 0;
  for (const auto &[camera_index, camera_residuals] : resblocks_) {
    num_detections += camera_residuals.size();
  }
  return num_detections;
}

int CalibView::numDetections(const int camera_index) const {
  return resblocks_.at(camera_index).size();
}

std::vector<int> CalibView::getCameraIndices() const {
  std::vector<int> camera_indicies;
  for (const auto &[camera_index, camera_residuals] : resblocks_) {
    camera_indicies.push_back(camera_index);
  }
  return camera_indicies;
}

} // namespace yac
