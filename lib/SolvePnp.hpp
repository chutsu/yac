#pragma once
#include <ceres/ceres.h>

#include "Core.hpp"
#include "CameraGeometry.hpp"

namespace yac {

/** SolvePnp */
class SolvePnp {
private:
  std::shared_ptr<CameraGeomtry> camera_geometry_;

public:
  SolvePnp() = delete;
  SolvePnp(const std::shared_ptr<CameraGeomtry> &camera_geometry)
      : camera_geometry_{camera_geometry};
  virtual ~SolvePnp() = default;

  /** Estimate relative pose */
  int estimate(mat4_t &T_camera_object);
};

} // namespace yac
