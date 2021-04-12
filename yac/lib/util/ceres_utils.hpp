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

// Compute matrix to lift a quaternion.
matx_t lift_quaternion(const quat_t &q);

// Only computes the J_lift jacobian. J = J_min * J_lift.
void lift_pose_jacobian(const quat_t &q, mat_t<6, 7, row_major_t> &J_lift);

// Convert from pose minimial 2x6 jacobians to the full 2x7 jacobian
void lift_pose_jacobian(const mat_t<2, 6> &J_min, const quat_t &q, double *J_raw);

// Evaluate residual block
bool evaluate_residual_block(const ceres::Problem &problem,
                             const ceres::ResidualBlockId res_id,
                             vec2_t &r);

// Pose local parameterization
class PoseLocalParameterization : public ceres::LocalParameterization {
public:
  PoseLocalParameterization();
  virtual ~PoseLocalParameterization();
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};

} //  namespace yac
#endif // YAC_CERES_UTILS_HPP
