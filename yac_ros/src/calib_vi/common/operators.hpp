#ifndef YAC_COMMON_OPERATORS_HPP
#define YAC_COMMON_OPERATORS_HPP

#include <stdint.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace yac {
namespace kinematics {

/// \brief Cross matrix of a vector [x,y,z].
///        Adopted from Schweizer-Messer by Paul Furgale.
/// \tparam Scalar_T The scalar type, auto-deducible (typically double).
/// @param[in] x First vector element.
/// @param[in] y Second vector element.
/// @param[in] z Third vector element.
template <typename Scalar_T>
Eigen::Matrix<Scalar_T, 3, 3> crossMx(Scalar_T x, Scalar_T y, Scalar_T z) {
  Eigen::Matrix<Scalar_T, 3, 3> C;
  C(0, 0) = 0.0;
  C(0, 1) = -z;
  C(0, 2) = y;
  C(1, 0) = z;
  C(1, 1) = 0.0;
  C(1, 2) = -x;
  C(2, 0) = -y;
  C(2, 1) = x;
  C(2, 2) = 0.0;
  return C;
}

/// \brief Cross matrix of a vector v.
///        Adopted from Schweizer-Messer by Paul Furgale.
/// \tparam Derived_T The vector type, auto-deducible.
/// @param[in] v The vector.
template <typename Derived_T>
Eigen::Matrix<typename Eigen::internal::traits<Derived_T>::Scalar, 3, 3>
crossMx(Eigen::MatrixBase<Derived_T> const &v) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_T>, 3);
  assert((v.cols() == 3 && v.rows() == 1) || (v.rows() == 3 && v.cols() == 1));
  return crossMx(v(0, 0), v(1, 0), v(2, 0));
}

/// \brief Plus matrix of a quaternion, i.e. q_AB*q_BC =
/// plus(q_AB)*q_BC.coeffs().
/// @param[in] q_AB A Quaternion.
Eigen::Matrix4d plus(const Eigen::Quaterniond &q_AB);

/// \brief Oplus matrix of a quaternion, i.e. q_AB*q_BC =
/// oplus(q_BC)*q_AB.coeffs().
/// @param[in] q_BC A Quaternion.
Eigen::Matrix4d oplus(const Eigen::Quaterniond &q_BC);

} // namespace kinematics
} // namespace yac
#endif /* YAC_COMMON_OPERATORS_HPP */
