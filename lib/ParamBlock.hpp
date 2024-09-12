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
  };

  static int getParamSize(const ParamBlock::Type type) {
    switch (type) {
      case POSE:
      case EXTRINSIC:
        return 7;
      case POINT:
      case VELOCITY:
      case BIAS:
        return 3;
      case TIME_DELAY:
        return 1;
      case INTRINSIC8:
        return 8;
      default:
        FATAL("Invalid Parameter BlockType!");
    }
  }

  static int getLocalSize(const ParamBlock::Type type) {
    switch (type) {
      case POSE:
      case EXTRINSIC:
        return 6;
      case POINT:
      case VELOCITY:
      case BIAS:
        return 3;
      case TIME_DELAY:
        return 1;
      case INTRINSIC8:
        return 8;
      default:
        FATAL("Invalid Parameter BlockType!");
    }
  }

  static void perturb(const ParamBlock::Type type,
                      const int i,
                      const double step,
                      double *ptr) {
    switch (type) {
      case POSE:
      case EXTRINSIC:
        switch (i) {
          case 0:
          case 1:
          case 2:
            // Update translation
            ptr[i] += step;
            break;
          case 3:
          case 4:
          case 5:
            // Update rotation
            vec3_t dalpha{0.0, 0.0, 0.0};
            dalpha(i) = step;

            quat_t dq = quat_delta(dalpha);
            quat_t q{ptr[6], ptr[3], ptr[4], ptr[5]};
            quat_t q_new = q * dq;

            ptr[6] = q_new.w();
            ptr[3] = q_new.x();
            ptr[4] = q_new.y();
            ptr[5] = q_new.z();
        }
        break;
      case POINT:
      case VELOCITY:
      case BIAS:
      case TIME_DELAY:
      case INTRINSIC8:
        ptr[i] += step;
        break;
      default:
        FATAL("Invalid Parameter BlockType!");
    }
  }
};

} // namespace yac
