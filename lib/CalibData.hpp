#pragma once
#include <ceres/ceres.h>

#include "Core.hpp"
#include "AprilGrid.hpp"
#include "CameraGeometry.hpp"
#include "Timeline.hpp"

namespace yac {

using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CameraData = std::map<timestamp_t, CalibTargetPtr>;

// Pose local parameterization
class PoseLocalParameterization : public ceres::Manifold {
public:
  PoseLocalParameterization() = default;
  virtual ~PoseLocalParameterization() = default;

  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const {
    // Form transform
    mat4_t T = tf(x);

    // Update rotation
    vec3_t dalpha{delta[3], delta[4], delta[5]};
    quat_t q = tf_quat(T);
    quat_t dq = quat_delta(dalpha);
    q = q * dq;
    q.normalize();

    // Update translation
    vec3_t dr{delta[0], delta[1], delta[2]};
    vec3_t r = tf_trans(T);
    r = r + dr;

    // Copy results to `x_plus_delta`
    x_plus_delta[0] = r.x();
    x_plus_delta[1] = r.y();
    x_plus_delta[2] = r.z();

    x_plus_delta[3] = q.x();
    x_plus_delta[4] = q.y();
    x_plus_delta[5] = q.z();
    x_plus_delta[6] = q.w();

    return true;
  }

  virtual bool PlusJacobian(const double *x, double *jacobian) const {
    // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
    UNUSED(x);
    Jp.topRows<6>().setIdentity();
    Jp.bottomRows<1>().setZero();
    return true;
  }

  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_minus_delta) const {
    UNUSED(x);
    UNUSED(delta);
    UNUSED(x_minus_delta);
    return false;
  }

  virtual bool MinusJacobian(const double *x, double *jacobian) const {
    UNUSED(x);
    UNUSED(jacobian);
    return false;
  }

  virtual int AmbientSize() const { return 7; }

  virtual int TangentSize() const { return 6; }
};

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

  /** Get camera data */
  CameraData &getCameraData(const int camera_index);

  /** Get camera geometry */
  CameraGeometryPtr &getCameraGeometry(const int camera_index);

  /** Get target point */
  vec3_t &getTargetPoint(const int point_id);

  /** Get timeline */
  Timeline &getTimeline() const;
};

} // namespace yac
