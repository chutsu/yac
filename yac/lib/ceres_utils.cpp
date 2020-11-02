#include "ceres_utils.hpp"

namespace yac {

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

matx_t lift_quaternion(const quat_t &q) {
  // clang-format off
  mat_t<3, 4, Eigen::RowMajor> Jq_pinv;
  Jq_pinv << 2.0, 0.0, 0.0, 0.0,
             0.0, 2.0, 0.0, 0.0,
             0.0, 0.0, 2.0, 0.0;
  // clang-format on

  const Eigen::Quaterniond q_inv(q.w(), -q.x(), -q.y(), -q.z());
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
