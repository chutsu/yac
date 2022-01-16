#include "ceres_utils.hpp"

namespace yac {

Eigen::Matrix4d plus(const quat_t &q_AB) {
  Eigen::Vector4d q = q_AB.coeffs();
  Eigen::Matrix4d Q;
  Q(0, 0) = q[3];
  Q(0, 1) = -q[2];
  Q(0, 2) = q[1];
  Q(0, 3) = q[0];
  Q(1, 0) = q[2];
  Q(1, 1) = q[3];
  Q(1, 2) = -q[0];
  Q(1, 3) = q[1];
  Q(2, 0) = -q[1];
  Q(2, 1) = q[0];
  Q(2, 2) = q[3];
  Q(2, 3) = q[2];
  Q(3, 0) = -q[0];
  Q(3, 1) = -q[1];
  Q(3, 2) = -q[2];
  Q(3, 3) = q[3];
  return Q;
}

mat4_t oplus(const quat_t &q_BC) {
  // clang-format off
  vec4_t q = q_BC.coeffs();
  mat4_t Q;
  Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
  Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
  Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  // clang-format on
  return Q;
}

// Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
mat3_t rightJacobian(const vec3_t &PhiVec) {
  const double Phi = PhiVec.norm();
  mat3_t retMat = mat3_t::Identity();
  const mat3_t Phi_x = skew(PhiVec);
  const mat3_t Phi_x2 = Phi_x * Phi_x;
  if (Phi < 1.0e-4) {
    retMat += -0.5 * Phi_x + 1.0 / 6.0 * Phi_x2;
  } else {
    const double Phi2 = Phi * Phi;
    const double Phi3 = Phi2 * Phi;
    retMat +=
        -(1.0 - cos(Phi)) / (Phi2)*Phi_x + (Phi - sin(Phi)) / Phi3 * Phi_x2;
  }
  return retMat;
}

quat_t deltaQ(const vec3_t &dAlpha) {
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * dAlpha.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * dAlpha.template tail<3>();
  dq[3] = cos(halfnorm);
  return quat_t(dq);
}

mat_t<6, 7, row_major_t> lift_pose_jacobian(const mat4_t pose) {
  Eigen::Matrix<double, 3, 4> Jq_pinv;
  Jq_pinv.bottomRightCorner<3, 1>().setZero();
  Jq_pinv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 2.0;

  const quat_t q = tf_quat(pose);
  Eigen::Matrix4d Qplus = oplus(q.inverse());

  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  J_lift.bottomRightCorner<3, 4>() = Jq_pinv * Qplus;

  return J_lift;
}

bool evaluate_residual_block(const ceres::Problem &problem,
                             const ceres::ResidualBlockId res_id,
                             vec2_t &r) {
  // Get residual block and allocate residual vector
  const auto cost_fn = problem.GetCostFunctionForResidualBlock(res_id);
  r = zeros(cost_fn->num_residuals(), 1);

  // Get residual parameter blocks
  std::vector<double *> param_blocks;
  problem.GetParameterBlocksForResidualBlock(res_id, &param_blocks);

  // Evaluate residual block
  return cost_fn->Evaluate(param_blocks.data(), r.data(), nullptr);
}

PoseLocalParameterization::PoseLocalParameterization() {}

PoseLocalParameterization::~PoseLocalParameterization() {}

bool PoseLocalParameterization::Plus(const double *x,
                                     const double *delta,
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

bool PoseLocalParameterization::ComputeJacobian(const double *x,
                                                double *jacobian) const {
  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
  UNUSED(x);

  // Jp.topRows<6>().setIdentity();
  // Jp.bottomRows<1>().setZero();

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

} // namespace yac
