#pragma once
#include "Core.hpp"

namespace yac {

/** Parameter Block **/
class ParamBlock {
public:
  enum Type {
    POSE,
    EXTRINSIC,
    POINT,
    VELOCITY,
    BIAS,
    TIME_DELAY,
    INTRINSIC8,
    SPEED_BIASES,
  };

  /** Get Parameter Size */
  static int getParamSize(const ParamBlock::Type type);

  /** Get Local Size */
  static int getLocalSize(const ParamBlock::Type type);

  /** Perturb parameter based on type */
  static void perturb(const ParamBlock::Type type,
                      const int i,
                      const double step,
                      double *ptr);
};

} // namespace yac
