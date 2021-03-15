#include "autocal/param/CameraParameterBlock.hpp"

namespace autocal {

CameraParameterBlock::CameraParameterBlock() : base_t::ParameterBlockSized() {
  setFixed(false);
}

CameraParameterBlock::~CameraParameterBlock() {}

CameraParameterBlock::CameraParameterBlock(const CameraParams &camera_params,
                                           uint64_t id,
                                           const autocal::Time &timestamp) {
  setEstimate(camera_params);
  setId(id);
  setTimestamp(timestamp);
  setFixed(false);
}

void CameraParameterBlock::setEstimate(const CameraParams &camera_params) {
  for (int i = 0; i < base_t::Dimension; ++i)
    parameters_[i] = camera_params[i];
}

CameraParams CameraParameterBlock::estimate() const {
  CameraParams camera_params;
  for (int i = 0; i < base_t::Dimension; ++i)
    camera_params[i] = parameters_[i];
  return camera_params;
}

} // namespace autocal
