#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "CalibView.hpp"
#include "CameraResidual.hpp"

namespace yac {

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
