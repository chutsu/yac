#include "CameraGeometry.hpp"

namespace yac {

CameraGeometry::CameraGeometry(const int camera_index,
                               const std::string &camera_model,
                               const vec2i_t &resolution,
                               const vecx_t &intrinsic,
                               const vecx_t &extrinsic)
    : camera_index_{camera_index}, resolution_{resolution},
      intrinsic_{intrinsic}, extrinsic_{extrinsic} {
  if (camera_model == "pinhole-radtan4") {
    camera_model_ = std::make_shared<PinholeRadtan4>();
  } else if (camera_model == "pinhole-equi4") {
    camera_model_ = std::make_shared<PinholeEqui4>();
  } else {
    FATAL("Unsupported camera model [%s]", camera_model.c_str());
  }
}

int CameraGeometry::getCameraIndex() const { return camera_index_; }

std::shared_ptr<CameraModel> CameraGeometry::getCameraModel() const { return camera_model_; }

vec2i_t CameraGeometry::getResolution() const { return resolution_; }

vecx_t CameraGeometry::getIntrinsic() const { return intrinsic_; }

vecx_t CameraGeometry::getExtrinsic() const { return extrinsic_; }

mat4_t CameraGeometry::getTransform() const { return tf(extrinsic_); }

} // namespace yac
