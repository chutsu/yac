#ifndef AUTOCAL_CERES_POSELOCALPARAMETERIZATION_HPP
#define AUTOCAL_CERES_POSELOCALPARAMETERIZATION_HPP

#include <ceres/ceres.h>
#include "autocal/common/Common.hpp"
#include "autocal/common/Transformation.hpp"
#include "./LocalParamizationAdditionalInterfaces.hpp"

namespace autocal {

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
class PoseLocalParameterization : public ::ceres::LocalParameterization,
                                  public LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization() {}

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta.
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const;

  // provide these as static for easy use elsewhere:

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  static bool plus(const double *x, const double *delta, double *x_plus_delta);

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double *x, double *jacobian);

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  static bool minus(const double *x, const double *x_plus_delta, double *delta);

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double *x, double *jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const { return 7; }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const { return 6; }

  // added convenient check
  bool VerifyJacobianNumDiff(const double *x,
                             double *jacobian,
                             double *jacobianNumDiff);
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb the translation though.
class PoseLocalParameterization3d
    : public ::ceres::LocalParameterization,
      public LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization3d() {}

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double *x, double *jacobian);

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double *x, double *jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const { return 7; }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const { return 3; }
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb the translation and yaw though.
class PoseLocalParameterization4d
    : public ::ceres::LocalParameterization,
      public LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization4d() {}

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double *x, double *jacobian);

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double *x, double *jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const { return 7; }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const { return 4; }
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb roll and pitch, i.e. dalpha = [dalpha1,
///        dalpha2, 0]^T.
class PoseLocalParameterization2d
    : public ::ceres::LocalParameterization,
      public LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization2d() {}

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double *x, double *jacobian);

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double *x, double *jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const { return 7; }
  /// \brief The parameter block local dimension.
  virtual int LocalSize() const { return 2; }
};

} // namespace autocal
#endif /* AUTOCAL_CERES_POSELOCALPARAMETERIZATION_HPP */
