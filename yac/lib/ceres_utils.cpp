#include "ceres_utils.hpp"

namespace yac {

void pose_lift_jacobian(const quat_t &q, mat_t<6, 7, row_major_t> &J_lift) {
  J_lift.setZero();
  J_lift.topLeftCorner<3, 4>() = lift_quaternion(q);
  J_lift.bottomRightCorner<3, 3>().setIdentity();
}

void pose_lift_jacobian(const mat_t<2, 6> &J_min, const quat_t &q, double *J_raw) {
  mat_t<6, 7, row_major_t> J_lift;
  J_lift.setZero();
  J_lift.topLeftCorner<3, 4>() = lift_quaternion(q);
  J_lift.bottomRightCorner<3, 3>().setIdentity();

  Eigen::Map<mat_t<2, 7, row_major_t>> J(J_raw);
  J = J_min * J_lift;
}

} // namespace yac
