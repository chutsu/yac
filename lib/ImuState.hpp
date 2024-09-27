#pragma once
#include "Core.hpp"

namespace yac {

class ImuState {
private:
  timestamp_t ts_;
  vecx_t pose_;
  vecx_t speed_biases_;

public:
  ImuState(const timestamp_t ts, const vecx_t &pose, const vec3_t &vel);

  /** Create shared_ptr */
  static std::shared_ptr<ImuState> create(const timestamp_t ts,
                                          const vecx_t &pose,
                                          const vec3_t &vel);

  /** Return timestamp */
  timestamp_t getTimestamp() const;

  /** Return pose */
  vecx_t getPose() const;

  /** Return speed and biases */
  vecx_t getSpeedBiases() const;

  /** Return pose pointer */
  double *getPosePtr();

  /** Return speed biases pointer */
  double *getSpeedBiasesPtr();
};

} // namespace yac
