#include "autocal/param/TimeDelayParameterBlock.hpp"

namespace autocal {

TimeDelayParameterBlock::TimeDelayParameterBlock()
    : base_t::ParameterBlockSized() {
  setFixed(false);
}

TimeDelayParameterBlock::~TimeDelayParameterBlock() {}

TimeDelayParameterBlock::TimeDelayParameterBlock(const double &time_delay,
                                                 uint64_t id) {
  setEstimate(time_delay);
  setId(id);
  setFixed(false);
}

void TimeDelayParameterBlock::setEstimate(const double &time_delay) {
  parameters_[0] = time_delay;
}

double TimeDelayParameterBlock::estimate() const { return parameters_[0]; }

} // namespace autocal
