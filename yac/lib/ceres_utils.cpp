#include "ceres_utils.hpp"

namespace yac {

Eigen::Matrix4d plus(const quat_t & q_AB) {
  Eigen::Vector4d q = q_AB.coeffs();
  Eigen::Matrix4d Q;
  Q(0,0) =  q[3]; Q(0,1) = -q[2]; Q(0,2) =  q[1]; Q(0,3) =  q[0];
  Q(1,0) =  q[2]; Q(1,1) =  q[3]; Q(1,2) = -q[0]; Q(1,3) =  q[1];
  Q(2,0) = -q[1]; Q(2,1) =  q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  return Q;
}

mat4_t oplus(const quat_t & q_BC) {
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
mat3_t rightJacobian(const vec3_t & PhiVec) {
  const double Phi = PhiVec.norm();
  mat3_t retMat = mat3_t::Identity();
  const mat3_t Phi_x = skew(PhiVec);
  const mat3_t Phi_x2 = Phi_x*Phi_x;
  if (Phi < 1.0e-4) {
    retMat += -0.5*Phi_x + 1.0/6.0*Phi_x2;
  } else {
    const double Phi2 = Phi*Phi;
    const double Phi3 = Phi2*Phi;
    retMat += -(1.0-cos(Phi))/(Phi2)*Phi_x + (Phi-sin(Phi))/Phi3*Phi_x2;
  }
  return retMat;
}

quat_t deltaQ(const vec3_t& dAlpha) {
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * dAlpha.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * dAlpha.template tail<3>();
  dq[3] = cos(halfnorm);
  return quat_t(dq);
}

matx_t lift_quaternion(const quat_t &q) {
  // clang-format off
  mat_t<3, 4, Eigen::RowMajor> Jq_pinv;
  Jq_pinv << 2.0, 0.0, 0.0, 0.0,
             0.0, 2.0, 0.0, 0.0,
             0.0, 0.0, 2.0, 0.0;
  // clang-format on

  const quat_t q_inv(q.w(), -q.x(), -q.y(), -q.z());
  return Jq_pinv * oplus(q_inv);
}

void lift_pose_jacobian(const quat_t &q, mat_t<6, 7, row_major_t> &J_lift) {
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  J_lift.bottomRightCorner<3, 4>() = lift_quaternion(q);
}

void lift_pose_jacobian(const mat_t<2, 6> &J_min, const quat_t &q, double *J_raw) {
  mat_t<6, 7, row_major_t> J_lift;
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  J_lift.bottomRightCorner<3, 4>() = lift_quaternion(q);

  Eigen::Map<mat_t<2, 7, row_major_t>> J(J_raw);
  J = J_min * J_lift;
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

} // namespace yac
