#pragma once
#include "Core.hpp"

namespace yac {

struct ImuParams {
  vec3_t noise_acc;
  vec3_t noise_gyr;
  vec3_t noise_ba;
  vec3_t noise_bg;
  vec3_t g;
};

} // namespace yac
