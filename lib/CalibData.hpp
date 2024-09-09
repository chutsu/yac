#pragma once
#include "Core.hpp"
#include "AprilGrid.hpp"
#include "CameraGeometry.hpp"
#include "Timeline.hpp"

namespace yac {

using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CameraData = std::map<timestamp_t, CalibTargetPtr>;

/** Calibration Data */
class CalibData {
private:
  // Settings
  std::string config_path_;
  std::string data_path_;

  // Calibration Target
  std::string target_type_;
  int tag_rows_;
  int tag_cols_;
  double tag_size_;
  double tag_spacing_;

  // Data
  std::map<int, CameraData> camera_data_;
  std::map<int, CameraGeometryPtr> camera_geoms_;
  Timeline timeline_ = Timeline();

  /** Load Camera Data */
  void loadCameraData(const int camera_index);

public:
  CalibData() = delete;
  CalibData(const std::string &config_path);
  virtual ~CalibData() = default;

  /** Add Camera */
  void addCamera(const int camera_index,
                 const std::string &camera_model,
                 const vec2i_t &resolution,
                 const vecx_t &intrinsic,
                 const vec7_t &extrinsic);

  /** Add Camera Measurement */
  void addCameraMeasurement(const timestamp_t ts,
                            const int camera_index,
                            const std::shared_ptr<CalibTarget> &calib_target);

  /** Get number of cameras */
  int getNumCameras() const;

  /** Get camera data */
  const CameraData &getCameraData(const int camera_index) const;

  /** Get camera geometry */
  const CameraGeometryPtr &getCameraGeometry(const int camera_index) const;

  /** Get timeline */
  const Timeline &getTimeline() const;
};

} // namespace yac
