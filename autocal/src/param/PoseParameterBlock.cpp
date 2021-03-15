#include "autocal/param/PoseParameterBlock.hpp"

namespace autocal {

PoseParameterBlock::PoseParameterBlock() : base_t::ParameterBlockSized() {
  setFixed(false);
}

PoseParameterBlock::~PoseParameterBlock() {}

PoseParameterBlock::PoseParameterBlock(
    const autocal::Transformation &T_WS,
    uint64_t id,
    const autocal::Time &timestamp) {
  setEstimate(T_WS);
  setId(id);
  setTimestamp(timestamp);
  setFixed(false);
}

void PoseParameterBlock::setEstimate(
    const autocal::Transformation &T_WS) {
  const Eigen::Vector3d r = T_WS.r();
  const Eigen::Vector4d q = T_WS.q().coeffs();
  parameters_[0] = r[0];
  parameters_[1] = r[1];
  parameters_[2] = r[2];
  parameters_[3] = q[0];
  parameters_[4] = q[1];
  parameters_[5] = q[2];
  parameters_[6] = q[3];
}

autocal::Transformation PoseParameterBlock::estimate() const {
  return autocal::
      Transformation(Eigen::Vector3d(parameters_[0],
                                     parameters_[1],
                                     parameters_[2]),
                     Eigen::Quaterniond(parameters_[6],
                                        parameters_[3],
                                        parameters_[4],
                                        parameters_[5]));
}

} // namespace autocal
