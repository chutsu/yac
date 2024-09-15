#pragma once
#include <ceres/ceres.h>

#include "Core.hpp"
#include "CameraGeometry.hpp"

namespace yac {

/** SolvePnp */
class SolvePnp {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;

public:
  SolvePnp() = delete;
  SolvePnp(const std::shared_ptr<CameraGeometry> &camera_geometry);
  virtual ~SolvePnp() = default;

  /** Estimate relative pose */
  int estimate(const vec2s_t &keypoints,
               const vec3s_t &object_points,
               mat4_t &T_camera_object);
};

} // namespace yac
