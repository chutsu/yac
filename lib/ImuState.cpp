#include "ImuState.hpp"

namespace yac {

ImuState::ImuState(const timestamp_t ts, const vecx_t &pose, const vec3_t &vel)
    : ts_{ts}, pose_{pose} {
  speed_biases_.resize(9);
  speed_biases_.segment<3>(0) = vel;
  speed_biases_.segment<3>(3) = vec3_t{0.0, 0.0, 0.0};
  speed_biases_.segment<3>(6) = vec3_t{0.0, 0.0, 0.0};
}

std::shared_ptr<ImuState> ImuState::create(const timestamp_t ts,
                                           const vecx_t &pose,
                                           const vec3_t &vel) {
  return std::make_shared<ImuState>(ts, pose, vel);
}

timestamp_t ImuState::getTimestamp() const { return ts_; }

vecx_t ImuState::getPose() const { return pose_; }

vecx_t ImuState::getSpeedBiases() const { return speed_biases_; }

double *ImuState::getPosePtr() { return pose_.data(); }

double *ImuState::getSpeedBiasesPtr() { return speed_biases_.data(); }

} // namespace yac
