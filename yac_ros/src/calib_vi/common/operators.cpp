#include "operators.hpp"

namespace yac {
namespace kinematics {

/// \brief Plus matrix of a quaternion, i.e. q_AB*q_BC =
/// plus(q_AB)*q_BC.coeffs().
/// Left Quaternion product matrix Q(L)
/// @param[in] q_AB A Quaternion.
Eigen::Matrix4d plus(const Eigen::Quaterniond &q_AB) {
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

/// \brief Oplus matrix of a quaternion, i.e. q_AB*q_BC =
/// oplus(q_BC)*q_AB.coeffs().
/// Right Quaternion product matrix Q(L)
/// @param[in] q_BC A Quaternion.
Eigen::Matrix4d oplus(const Eigen::Quaterniond &q_BC) {
  Eigen::Vector4d q = q_BC.coeffs();
  Eigen::Matrix4d Q;
  Q(0, 0) = q[3];
  Q(0, 1) = q[2];
  Q(0, 2) = -q[1];
  Q(0, 3) = q[0];

  Q(1, 0) = -q[2];
  Q(1, 1) = q[3];
  Q(1, 2) = q[0];
  Q(1, 3) = q[1];

  Q(2, 0) = q[1];
  Q(2, 1) = -q[0];
  Q(2, 2) = q[3];
  Q(2, 3) = q[2];

  Q(3, 0) = -q[0];
  Q(3, 1) = -q[1];
  Q(3, 2) = -q[2];
  Q(3, 3) = q[3];
  return Q;
}

} // namespace kinematics
} // namespace yac
