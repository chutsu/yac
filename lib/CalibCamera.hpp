#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "CameraResidual.hpp"

namespace yac {

/** Calibration view */
class CalibView {
private:
  timestamp_t ts_ = 0;
  const std::map<int, CalibTargetPtr> view_data_;
  std::map<int, CameraGeometryPtr> &camera_geoms_;
  std::map<int, vec3_t> &target_points_;
  vec7_t &target_pose_;
  vec7_t pose_;

public:
  CalibView() = delete;
  CalibView(const timestamp_t ts,
            const std::map<int, CalibTargetPtr> &view_data,
            std::map<int, CameraGeometryPtr> &camera_geoms,
            std::map<int, vec3_t> &target_points,
            vec7_t &target_pose);
  ~CalibView() = default;

  /** Get number of detections */
  int numDetections() const;

  /** Get number of detections by camera */
  int numDetections(const int camera_index) const;

  /** Get camera indicies */
  std::vector<int> getCameraIndices() const;

  /** Get residuals */
  vec2s_t getResiduals() const;

  /** Get residuals by camera index */
  vec2s_t getResiduals(const int camera_index) const;

  /** Get reprojection errors */
  std::vector<double> getReprojErrors() const;

  /** Get reprojection errors by camera */
  std::vector<double> getReprojErrors(const int camera_index) const;
};

/** Camera Calibrator **/
class CalibCamera : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseLocalParameterization pose_plus_;
  // std::unordered_map<calib_residual_t *, ceres::ResidualBlockId> res2id_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, CalibView> calib_views_;

public:
  CalibCamera(const std::string &config_file) : CalibData{config_file};

  /** Initialize camera parameters */
  void initializeCamera(const int camera_index);

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetPtr> &measurements);

  // void buildProblem() {
  //   for (auto events : getTimeline()) {
  //   }
  // }

  /** Solve */
  void solve();
};

} // namespace yac
