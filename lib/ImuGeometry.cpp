#include "ImuGeometry.hpp"

namespace yac {

ImuGeometry::ImuGeometry(const int imu_index,
                         const ImuParams &imu_params,
                         const vecx_t &extrinsic)
    : imu_index_{imu_index}, imu_params_{imu_params}, extrinsic_{extrinsic} {}

ImuParams ImuGeometry::getImuParams() const { return imu_params_; }

vecx_t ImuGeometry::getExtrinsic() const { return extrinsic_; }

double *ImuGeometry::getExtrinsicPtr() { return extrinsic_.data(); }

mat4_t ImuGeometry::getTransform() const { return tf(extrinsic_); }

void ImuGeometry::setExtrinsic(const mat4_t &extrinsic) {
  extrinsic_ = tf_vec(extrinsic);
}

} // namespace yac
