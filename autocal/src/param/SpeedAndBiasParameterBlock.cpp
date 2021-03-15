#include "autocal/param/SpeedAndBiasParameterBlock.hpp"

namespace autocal {

SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock()
    : base_t::ParameterBlockSized() {
  setFixed(false);
}

SpeedAndBiasParameterBlock::~SpeedAndBiasParameterBlock() {}

SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock(
    const SpeedAndBias &speedAndBias,
    uint64_t id,
    const autocal::Time &timestamp) {
  setEstimate(speedAndBias);
  setId(id);
  setTimestamp(timestamp);
  setFixed(false);
}

void SpeedAndBiasParameterBlock::setEstimate(const SpeedAndBias &speedAndBias) {
  for (int i = 0; i < base_t::Dimension; ++i)
    parameters_[i] = speedAndBias[i];
}

SpeedAndBias SpeedAndBiasParameterBlock::estimate() const {
  SpeedAndBias speedAndBias;
  for (int i = 0; i < base_t::Dimension; ++i)
    speedAndBias[i] = parameters_[i];
  return speedAndBias;
}

} // namespace autocal
