#pragma once
#include <ceres/ceres.h>

#include "Core.hpp"
#include "AprilGrid.hpp"
#include "CameraGeometry.hpp"
#include "PoseLocalParameterization.hpp"

namespace yac {

using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CameraData = std::map<timestamp_t, CalibTargetPtr>;

/** Calibration Data */
class CalibData {
protected:
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
  std::map<int, CameraGeometryPtr> camera_geometries_;
  std::map<int, vec3_t> target_points_;

  /** Load Camera Data */
  void loadCameraData(const int camera_index);

  /** Check if we have a camera measurement already */
  bool hasCameraMeasurement(const timestamp_t ts, const int camera_index) const;

  /** Print settings */
  void printSettings(FILE *fp) const;

  /** Print calibration target */
  void printCalibTarget(FILE *fp) const;

  /** Print camera geometries */
  void printCameraGeometries(FILE *fp, const bool max_digits=false) const;

  /** Print target points */
  void printTargetPoints(FILE *fp) const;

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

  /** Add camera measurement */
  void addCameraMeasurement(const timestamp_t ts,
                            const int camera_index,
                            const std::shared_ptr<CalibTarget> &calib_target);

  /** Add calibration target point */
  void addTargetPoint(const int point_id, const vec3_t &point);

  /** Get number of cameras */
  int getNumCameras() const;

  /** Get all camera data */
  std::map<int, CameraData> &getAllCameraData();

  /** Get camera data */
  CameraData &getCameraData(const int camera_index);

  /** Get camera geometries */
  std::map<int, CameraGeometryPtr> &getAllCameraGeometries();

  /** Get camera geometry */
  CameraGeometryPtr &getCameraGeometry(const int camera_index);

  /** Get target point */
  vec3_t &getTargetPoint(const int point_id);

  /** Print summary */
  void printSummary(FILE *fp, const bool max_digits=false) const;

  /** Save results */
  void saveResults(const std::string &save_path) const;
};

} // namespace yac
