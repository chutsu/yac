#pragma once

#include "Core.hpp"

namespace yac {

struct CalibTarget {
  timestamp_t ts = 0;

  virtual void get_measurements(std::vector<int> &corner_ids,
                                vec2s_t &keypoints,
                                vec3s_t &object_points) const = 0;

  virtual int estimate(const CameraModel *cam,
                       const int cam_res[2],
                       const vecx_t &cam_params,
                       mat4_t &T_camera_target) const = 0;
};

} // namespace yac
