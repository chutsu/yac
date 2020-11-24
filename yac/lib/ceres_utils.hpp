#ifndef YAC_CERES_UTILS_HPP
#define YAC_CERES_UTILS_HPP

#include "core.hpp"

#include <ceres/ceres.h>

namespace yac {

// Oplus matrix of a quaternion,
// Right quaternion product matrix
// i.e. q_AB * q_BC = oplus(q_BC) * q_AB.coeffs().
mat4_t oplus(const quat_t & q_BC);

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
  PoseLocalParameterization() {}
  virtual ~PoseLocalParameterization() {}

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
    // Form transform
    mat4_t T = tf(x);

    // Update rotation
    vec3_t dalpha{delta[3], delta[4], delta[5]};
    quat_t q = tf_quat(T);
    quat_t dq = quat_delta(dalpha);
    q = dq * q;
    q.normalize();

    // Update translation
    vec3_t dr{delta[0], delta[1], delta[2]};
    vec3_t r = tf_trans(T);
    r = r + dr;

    // Copy results to `x_plus_delta`
    x_plus_delta[0] = r(0);
    x_plus_delta[1] = r(1);
    x_plus_delta[2] = r(2);

    x_plus_delta[3] = q.x();
    x_plus_delta[4] = q.y();
    x_plus_delta[5] = q.z();
    x_plus_delta[6] = q.w();

    return true;
  }

  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
    Jp.setZero();
    Jp.topLeftCorner<3, 3>().setIdentity();

    mat_t<4, 3> S = zeros(4, 3);
    S(0, 0) = 0.5;
    S(1, 1) = 0.5;
    S(2, 2) = 0.5;

    mat4_t T = tf(x);
    quat_t q = tf_quat(T);
    Jp.block<4, 3>(3, 3) = oplus(q) * S;

    return true;
  }

  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
