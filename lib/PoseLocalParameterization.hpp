#pragma once
#include <ceres/ceres.h>

#include "Core.hpp"

namespace yac {

// Pose local parameterization
class PoseLocalParameterization : public ceres::Manifold {
public:
  PoseLocalParameterization() = default;
  virtual ~PoseLocalParameterization() = default;

  /** Plus */
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /** Minus Jacobian */
  virtual bool PlusJacobian(const double *x, double *jacobian) const;

  /** Minus */
  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_minus_delta) const;

  /** Minus Jacobian */
  virtual bool MinusJacobian(const double *x, double *jacobian) const;

  /** Return Ambient size */
  virtual int AmbientSize() const;

  /** Return Tangent size */
  virtual int TangentSize() const;
};

} // namespace yac
