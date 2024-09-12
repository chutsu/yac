#pragma once
#include "CameraModel.hpp"

namespace yac {

/** Camera Geometry */
class CameraGeometry {
private:
  int camera_index_;
  std::shared_ptr<CameraModel> camera_model_;
  vec2i_t resolution_;
  vecx_t intrinsic_;
  vecx_t extrinsic_;

public:
  CameraGeometry() = delete;
  CameraGeometry(const int camera_index,
                 const std::string &camera_model,
                 const vec2i_t &resolution,
                 const vecx_t &intrinsic,
                 const vecx_t &extrinsic);
  virtual ~CameraGeometry() = default;

  /** Get camera index **/
  int getCameraIndex() const;

  /** Get camera model **/
  std::shared_ptr<CameraModel> getCameraModel() const;

  /** Get resoultion **/
  vec2i_t getResolution() const;

  /** Get intrinsic **/
  vecx_t getIntrinsic() const;

  /** Get extrinsic **/
  vecx_t getExtrinsic() const;

  /** Get intrinsic pointer **/
  double *getIntrinsicPtr();

  /** Get extrinsic **/
  double *getExtrinsicPtr();

  /** Get transform T_body_camera **/
  mat4_t getTransform() const;
};

} // namespace yac
