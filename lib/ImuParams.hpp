#pragma once
#include "Core.hpp"

namespace yac {

struct ImuParams {
  double noise_acc;
  double noise_gyr;
  double noise_ba;
  double noise_bg;
  vec3_t g{0.0, 0.0, 9.81};
};

} // namespace yac
