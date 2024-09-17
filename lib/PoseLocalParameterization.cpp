#include "PoseLocalParameterization.hpp"

namespace yac {

bool PoseLocalParameterization::Plus(const double *x,
                                     const double *delta,
                                     double *x_plus_delta) const {
  // Form transform
  mat4_t T = tf(x);

  // Update rotation
  vec3_t dalpha{delta[3], delta[4], delta[5]};
  quat_t q = tf_quat(T);
  quat_t dq = quat_delta(dalpha);
  q = q * dq;
  q.normalize();

  // Update translation
  vec3_t dr{delta[0], delta[1], delta[2]};
  vec3_t r = tf_trans(T);
  r = r + dr;

  // Copy results to `x_plus_delta`
  x_plus_delta[0] = r.x();
  x_plus_delta[1] = r.y();
  x_plus_delta[2] = r.z();

  x_plus_delta[3] = q.x();
  x_plus_delta[4] = q.y();
  x_plus_delta[5] = q.z();
  x_plus_delta[6] = q.w();

  return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x,
                                             double *jacobian) const {
  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
  UNUSED(x);
  Jp.topRows<6>().setIdentity();
  Jp.bottomRows<1>().setZero();
  return true;
}

bool PoseLocalParameterization::Minus(const double *x,
                                      const double *delta,
                                      double *x_minus_delta) const {
  UNUSED(x);
  UNUSED(delta);
  UNUSED(x_minus_delta);
  return false;
}

bool PoseLocalParameterization::MinusJacobian(const double *x,
                                              double *jacobian) const {
  UNUSED(x);
  UNUSED(jacobian);
  return false;
}

int PoseLocalParameterization::AmbientSize() const { return 7; }

int PoseLocalParameterization::TangentSize() const { return 6; }

} // namespace yac
