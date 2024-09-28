#pragma once

#include "ImuParams.hpp"

namespace yac {

/** IMU Geometry */
class ImuGeometry {
private:
  int imu_index_;
  ImuParams imu_params_;
  vecx_t extrinsic_;

public:
  ImuGeometry() = delete;
  ImuGeometry(const int imu_index,
              const ImuParams &imu_params,
              const vecx_t &extrinsic);
  virtual ~ImuGeometry() = default;

  /** Get Imu Params */
  ImuParams getImuParams() const;

  /** Get extrinsic **/
  vecx_t getExtrinsic() const;

  /** Get extrinsic **/
  double *getExtrinsicPtr();

  /** Get transform T_body_camera **/
  mat4_t getTransform() const;

  /** Set extrinsic */
  void setExtrinsic(const mat4_t &extrinsic);
};

} // namespace yac
