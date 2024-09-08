#pragma once
#include "Core.hpp"
#include "AprilGrid.hpp"
#include "CameraGeometry.hpp"

namespace yac {

using CalibTargetSharedPtr = std::shared_ptr<CalibTarget>;
using CameraData = std::map<timestamp_t, CalibTargetSharedPtr>;

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
  std::map<int, std::shared_ptr<CameraGeometry>> camera_geoms_;

  /** Add Camera */
  void addCamera(const int camera_index,
                 const std::string &camera_model,
                 const vec2i_t &resolution,
                 const vecx_t &intrinsic,
                 const vec7_t &extrinsic);

  /** Load Camera Data */
  void loadCameraData(const int camera_index);

  /** Load Config */
  void loadConfig(const std::string &config_path);

public:
  CalibData() = delete;
  CalibData(const std::string &config_path);
  virtual ~CalibData() = default;
};

} // namespace yac
