#include "Transformation.hpp"

namespace yac {

double sinc(double x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

Eigen::Quaterniond deltaQ(const Eigen::Vector3d &dAlpha) {
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * dAlpha.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * dAlpha.template tail<3>();
  dq[3] = cos(halfnorm);
  return Eigen::Quaterniond(dq);
}

// Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &PhiVec) {
  const double Phi = PhiVec.norm();
  Eigen::Matrix3d retMat = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Phi_x = kinematics::crossMx(PhiVec);
  const Eigen::Matrix3d Phi_x2 = Phi_x * Phi_x;
  if (Phi < 1.0e-4) {
    retMat += -0.5 * Phi_x + 1.0 / 6.0 * Phi_x2;
  } else {
    const double Phi2 = Phi * Phi;
    const double Phi3 = Phi2 * Phi;
    retMat +=
        -(1.0 - cos(Phi)) / (Phi2) *Phi_x + (Phi - sin(Phi)) / Phi3 * Phi_x2;
  }
  return retMat;
}

Transformation::Transformation(const Transformation &other)
    : parameters_(other.parameters_), r_(&parameters_[0]), q_(&parameters_[3]),
      C_(other.C_) {}

Transformation::Transformation(Transformation &&other)
    : parameters_(std::move(other.parameters_)), r_(&parameters_[0]),
      q_(&parameters_[3]), C_(std::move(other.C_)) {}

Transformation::Transformation()
    : r_(&parameters_[0]), q_(&parameters_[3]),
      C_(Eigen::Matrix3d::Identity()) {
  r_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  q_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

Transformation::Transformation(const Eigen::Vector3d &r_AB,
                                      const Eigen::Quaterniond &q_AB)
    : r_(&parameters_[0]), q_(&parameters_[3]) {
  r_ = r_AB;
  q_ = q_AB.normalized();
  updateC();
}

Transformation::Transformation(const Eigen::Matrix4d &T_AB)
    : r_(&parameters_[0]), q_(&parameters_[3]), C_(T_AB.topLeftCorner<3, 3>()) {
  r_ = (T_AB.topRightCorner<3, 1>());
  q_ = (T_AB.topLeftCorner<3, 3>());
  assert(fabs(T_AB(3, 0)) < 1.0e-12);
  assert(fabs(T_AB(3, 1)) < 1.0e-12);
  assert(fabs(T_AB(3, 2)) < 1.0e-12);
  assert(fabs(T_AB(3, 3) - 1.0) < 1.0e-12);
}

Transformation::~Transformation() {}

// template <typename Derived_coeffs>
// bool
// Transformation::setCoeffs(const Eigen::MatrixBase<Derived_coeffs> &coeffs) {
//   EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_coeffs, 7);
//   parameters_ = coeffs;
//   updateC();
//   return true;
// }

// The underlying transformation
Eigen::Matrix4d Transformation::T() const {
  Eigen::Matrix4d T_ret;
  T_ret.topLeftCorner<3, 3>() = C_;
  T_ret.topRightCorner<3, 1>() = r_;
  T_ret.bottomLeftCorner<1, 3>().setZero();
  T_ret(3, 3) = 1.0;
  return T_ret;
}

// return the rotation matrix
const Eigen::Matrix3d &Transformation::C() const { return C_; }

// return the translation vector
const Eigen::Map<Eigen::Vector3d> &Transformation::r() const {
  return r_;
}

const Eigen::Map<Eigen::Quaterniond> &Transformation::q() const {
  return q_;
}

Eigen::Matrix<double, 3, 4> Transformation::T3x4() const {
  Eigen::Matrix<double, 3, 4> T3x4_ret;
  T3x4_ret.topLeftCorner<3, 3>() = C_;
  T3x4_ret.topRightCorner<3, 1>() = r_;
  return T3x4_ret;
}

// Return a copy of the transformation inverted.
Transformation Transformation::inverse() const {
  return Transformation(-(C_.transpose() * r_), q_.inverse());
}

// Set this to a random transformation.
void Transformation::setRandom() { setRandom(1.0, M_PI); }
// Set this to a random transformation with bounded rotation and translation.
void Transformation::setRandom(double translationMaxMeters,
                                      double rotationMaxRadians) {
  // Create a random unit-length axis.
  Eigen::Vector3d axis = rotationMaxRadians * Eigen::Vector3d::Random();
  // Create a random rotation angle in radians.
  Eigen::Vector3d r = translationMaxMeters * Eigen::Vector3d::Random();
  r_ = r;
  q_ = Eigen::AngleAxisd(axis.norm(), axis.normalized());
  updateC();
}

// Setters
void Transformation::set(const Eigen::Matrix4d &T_AB) {
  r_ = (T_AB.topRightCorner<3, 1>());
  q_ = (T_AB.topLeftCorner<3, 3>());
  updateC();
}
void Transformation::set(const Eigen::Vector3d &r_AB,
                                const Eigen::Quaternion<double> &q_AB) {
  r_ = r_AB;
  q_ = q_AB.normalized();
  updateC();
}
// Set this transformation to identity
void Transformation::setIdentity() {
  q_.setIdentity();
  r_.setZero();
  C_.setIdentity();
}

Transformation Transformation::Identity() { return Transformation(); }

// operator*
Transformation Transformation::
operator*(const Transformation &rhs) const {
  return Transformation(C_ * rhs.r_ + r_, q_ * rhs.q_);
}
Eigen::Vector3d Transformation::
operator*(const Eigen::Vector3d &rhs) const {
  return C_ * rhs;
}
Eigen::Vector4d Transformation::
operator*(const Eigen::Vector4d &rhs) const {
  const double s = rhs[3];
  Eigen::Vector4d retVec;
  retVec.head<3>() = C_ * rhs.head<3>() + r_ * s;
  retVec[3] = s;
  return retVec;
}

Transformation &Transformation::operator=(const Transformation &rhs) {
  parameters_ = rhs.parameters_;
  C_ = rhs.C_;
  r_ = Eigen::Map<Eigen::Vector3d>(&parameters_[0]);
  q_ = Eigen::Map<Eigen::Quaterniond>(&parameters_[3]);
  return *this;
}

void Transformation::updateC() { C_ = q_.toRotationMatrix(); }

} // namespace yac
