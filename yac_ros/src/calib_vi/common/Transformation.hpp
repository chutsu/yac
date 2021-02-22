#ifndef YAC_COMMON_TRANSFORMATION_HPP
#define YAC_COMMON_TRANSFORMATION_HPP

#include <stdint.h>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "operators.hpp"

namespace yac {

/**
 * Implements sin(x)/x for all x in R
 *
 * @param x The argument of the sinc function.
 * @return The result.
 */
double sinc(double x);

/**
 * Implements the exponential map for quaternions.
 * @param dAlpha a axis*angle (minimal) input in tangent space.
 * @return The corresponding Quaternion.
 */
Eigen::Quaterniond deltaQ(const Eigen::Vector3d &dAlpha);

/**
 * Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
 */
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &PhiVec);

/**
 * Homogeneous transformations
 *
 * This relates a frame A and B: T_AB; it consists of translation r_AB
 * (represented in frame A) and Quaternion q_AB (as an Eigen Quaternion).
 * See also the RSS'13 / IJRR'14 paper or the Thesis. Follows some of the
 * functionality of the SchweizerMesser library by Paul Furgale, but uses Eigen
 * quaternions underneath.
 *
 * @warning This means the convention is different to SchweizerMesser
 * and the RSS'13 / IJRR'14 paper / the Thesis.
 */
class Transformation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transformation();
  Transformation(const Transformation &other);
  Transformation(Transformation &&other);
  Transformation(const Eigen::Vector3d &r_AB, const Eigen::Quaterniond &q_AB);
  explicit Transformation(const Eigen::Matrix4d &T_AB);
  ~Transformation();

  /// Parameter setting, all 7.
  /// @tparam Derived_coeffs Deducible matrix type.
  /// @param coeffs The parameters as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen
  /// internal convention).
  template <typename Derived_coeffs>
  bool setCoeffs(const Eigen::MatrixBase<Derived_coeffs> &coeffs);

  /// Parameter setting, all 7.
  /// @tparam Derived_coeffs Deducible matrix type.
  /// @param parameters The parameters as [r_AB,q_AB], q_AB as [x,y,z,w]
  /// (Eigen internal convention).
  template <typename Derived_coeffs>
  bool setParameters(const Eigen::MatrixBase<Derived_coeffs> &parameters) {
    return setCoeffs(parameters);
  }

  ///< The underlying homogeneous transformation matrix.
  Eigen::Matrix4d T() const;

  ///< Returns the rotation matrix (cached).
  const Eigen::Matrix3d &C() const;

  ///< Returns the translation vector r_AB (represented in frame A).
  const Eigen::Map<Eigen::Vector3d> &r() const;

  ///< Returns the Quaternion q_AB (as an Eigen Quaternion).
  const Eigen::Map<Eigen::Quaterniond> &q() const;

  /// Get the upper 3x4 part of the homogeneous transformation matrix
  /// T_AB.
  Eigen::Matrix<double, 3, 4> T3x4() const;

  /// The coefficients (parameters) as [r_AB,q_AB], q_AB as [x,y,z,w]
  /// (Eigen internal convention).
  const Eigen::Matrix<double, 7, 1> &coeffs() const { return parameters_; }

  /// The parameters (coefficients) as [r_AB,q_AB], q_AB as [x,y,z,w]
  /// (Eigen internal convention).
  const Eigen::Matrix<double, 7, 1> &parameters() const { return parameters_; }

  /// Get the parameters --- support for ceres.
  /// @warning USE WITH CARE!
  const double *parameterPtr() const { return &parameters_[0]; }

  /// Set this to a random transformation.
  void setRandom();

  /// Set this to a random transformation with bounded rotation and
  /// translation.
  /// @param translationMaxMeters Maximum translation [m].
  /// @param rotationMaxRadians Maximum rotation [rad].
  void setRandom(double translationMaxMeters, double rotationMaxRadians);

  /// Set from a homogeneous transformation matrix.
  /// @param T_AB The homogeneous transformation matrix.
  void set(const Eigen::Matrix4d &T_AB);

  /// Set from a translation and quaternion.
  /// @param r_AB The translation r_AB (represented in frame A).
  /// @param q_AB The Quaternion q_AB (as an Eigen Quaternion).
  void set(const Eigen::Vector3d &r_AB, const Eigen::Quaternion<double> &q_AB);

  /// Set this transformation to identity
  void setIdentity();

  /// Get an identity transformation
  static Transformation Identity();

  /// Returns a copy of the transformation inverted.
  Transformation inverse() const;

  // operator* (group operator)
  /// Multiplication with another transformation object.
  /// @param rhs The right-hand side transformation for this to be
  /// multiplied with.
  Transformation operator*(const Transformation &rhs) const;

  /// Transform a direction as v_A = C_AB*v_B (with rhs = hp_B)..
  /// @warning This only applies the rotation!
  /// @param rhs The right-hand side direction for this to be multiplied
  /// with.
  Eigen::Vector3d operator*(const Eigen::Vector3d &rhs) const;

  /// Transform a homogenous point as hp_B = T_AB*hp_B (with rhs = hp_B).
  /// @param rhs The right-hand side direction for this to be multiplied
  /// with.
  Eigen::Vector4d operator*(const Eigen::Vector4d &rhs) const;

  /// Assignment -- copy. Takes care of proper caching.
  /// @param rhs The rhs for this to be assigned to.
  Transformation &operator=(const Transformation &rhs);

  /// Apply a small update with delta being 6x1.
  /// @tparam Derived_delta Deducible matrix type.
  /// @param delta The 6x1 minimal update.
  /// @return True on success.
  template <typename Derived_delta>
  bool oplus(const Eigen::MatrixBase<Derived_delta> &delta);

  /**
   * Apply a small update with delta being 6x1 -- the Jacobian is a 7 by 6
   * matrix.
   *
   * @param delta The 6x1 minimal update.
   * @param jacobian The output Jacobian.
   * @return True on success.
   */
  template <typename Derived_delta, typename Derived_jacobian>
  bool oplus(const Eigen::MatrixBase<Derived_delta> &delta,
             const Eigen::MatrixBase<Derived_jacobian> &jacobian);

  /// Get the Jacobian of the oplus operation (a 7 by 6 matrix).
  /// @param jacobian The output Jacobian.
  /// @return True on success.
  template <typename Derived_jacobian>
  bool oplusJacobian(const Eigen::MatrixBase<Derived_jacobian> &jacobian) const;

  /// Gets the jacobian dx/dChi,
  ///        i.e. lift the minimal Jacobian to a full one (as needed by ceres).
  /// @param jacobian The output lift Jacobian (6 by 7 matrix).
  /// @return True on success.
  template <typename Derived_jacobian>
  bool liftJacobian(const Eigen::MatrixBase<Derived_jacobian> &jacobian) const;

protected:
  /// Update the caching of the rotation matrix.
  void updateC();
  Eigen::Matrix<double, 7, 1> parameters_; ///< Concatenated parameters [r;q].
  Eigen::Map<Eigen::Vector3d> r_;          ///< Translation {_A}r_{B}.
  Eigen::Map<Eigen::Quaterniond> q_;       ///< Quaternion q_{AB}.
  Eigen::Matrix3d C_;                      ///< The cached DCM C_{AB}.
};

template <typename Derived_coeffs>
bool Transformation::setCoeffs(
    const Eigen::MatrixBase<Derived_coeffs> &coeffs) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_coeffs, 7);
  parameters_ = coeffs;
  updateC();
  return true;
}

template <typename Derived_delta>
bool Transformation::oplus(const Eigen::MatrixBase<Derived_delta> &delta) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, 6);
  // Translation
  r_ += delta.template head<3>();

  // Rotation
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * delta.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * delta.template tail<3>();
  dq[3] = cos(halfnorm);
  q_ = (Eigen::Quaterniond(dq) * q_);
  q_.normalize();

  updateC();
  return true;
}

template <typename Derived_delta, typename Derived_jacobian>
bool Transformation::oplus(
    const Eigen::MatrixBase<Derived_delta> &delta,
    const Eigen::MatrixBase<Derived_jacobian> &jacobian) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, 6);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, 6);
  if (!oplus(delta)) {
    return false;
  }
  return oplusJacobian(jacobian);
}

template <typename Derived_jacobian>
bool Transformation::oplusJacobian(
    const Eigen::MatrixBase<Derived_jacobian> &jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, 6);
  Eigen::Matrix<double, 4, 3> S = Eigen::Matrix<double, 4, 3>::Zero();
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).setZero();
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian)
      .template topLeftCorner<3, 3>()
      .setIdentity();
  S(0, 0) = 0.5;
  S(1, 1) = 0.5;
  S(2, 2) = 0.5;
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian)
      .template bottomRightCorner<4, 3>() = kinematics::oplus(q_) * S;
  return true;
}

template <typename Derived_jacobian>
bool Transformation::liftJacobian(
    const Eigen::MatrixBase<Derived_jacobian> &jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, 7);
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).setZero();
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian) .template topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian)
      .template bottomRightCorner<3, 4>() =
      2 *
      kinematics::oplus(q_.inverse()).template topLeftCorner<3, 4>();
  return true;
}

} // namespace yac
#endif /* YAC_COMMON_TRANSFORMATION_HPP */
