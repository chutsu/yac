#ifndef YAC_CERES_UTILS_HPP
#define YAC_CERES_UTILS_HPP

#include <ceres/ceres.h>

#include "core.hpp"

namespace yac {

// Plus matrix of a quaternion.
// i.e. q_AB * q_BC = plus(q_AB) * q_BC.coeffs().
Eigen::Matrix4d plus(const quat_t &q_AB);

// Oplus matrix of a quaternion,
// Right quaternion product matrix
// i.e. q_AB * q_BC = oplus(q_BC) * q_AB.coeffs().
mat4_t oplus(const quat_t &q_BC);

// Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
mat3_t rightJacobian(const vec3_t &PhiVec);

// Delta quaternion.
quat_t deltaQ(const vec3_t &dAlpha);

// Calculate lift jacobian
mat_t<6, 7, row_major_t> lift_pose_jacobian(const mat4_t pose);

// Evaluate residual block
bool evaluate_residual_block(const ceres::Problem &problem,
                             const ceres::ResidualBlockId res_id,
                             vec2_t &r);

// Pose local parameterization
class PoseLocalParameterization : public ceres::Manifold {
public:
  PoseLocalParameterization();
  virtual ~PoseLocalParameterization();
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;
  virtual bool PlusJacobian(const double *x, double *jacobian) const;
  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_plus_delta) const;
  virtual bool MinusJacobian(const double *x, double *jacobian) const;
  virtual int AmbientSize() const;
  virtual int TangentSize() const;
};

} //  namespace yac
#endif // YAC_CERES_UTILS_HPP
