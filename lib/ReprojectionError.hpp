#pragma once
#include "ResidualBlock.hpp"
#include "CameraGeometry.hpp"

namespace yac {

/** Reprojection Error */
class ReprojectionError : public ResidualBlock {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;
  vec2_t z_;
  mat2_t covar_;
  mat2_t info_;
  mat2_t sqrt_info_;
  mutable bool valid_ = true;
  mutable vec2_t residuals_;

public:
  /** Constructor */
  ReprojectionError(const std::shared_ptr<CameraGeometry> &camera_geometry,
                    const std::vector<double *> &param_ptrs,
                    const std::vector<ParamBlock::Type> &param_types,
                    const vec2_t &z,
                    const mat2_t &covar);

  /** Create residual block */
  static std::shared_ptr<ReprojectionError>
  create(const std::shared_ptr<CameraGeometry> &camera,
         double *T_C0F,
         double *p_FFi,
         const vec2_t &z,
         const mat2_t &covar);

  /** Get residuals */
  bool getResiduals(vec2_t &r) const;

  /** Get reprojection error */
  bool getReprojError(double *error) const;

  /** Evaluate with minimial Jacobians */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const override;
};

} // namespace yac
