#ifndef AUTOCAL_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP
#define AUTOCAL_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP

#include <ceres/ceres.h>

namespace autocal {

/// \brief Provides some additional interfaces to ceres' LocalParamization
///        than are needed in the generic marginalisation
///        autocal::ceres::MarginalizationError.
class LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~LocalParamizationAdditionalInterfaces() {}

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const = 0;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const = 0;

  /// \brief Verifies the correctness of an inplementation by means of numeric
  /// Jacobians.
  /// @param[in] x_raw Linearisation point of the variable.
  /// @param[in] purturbation_magnitude Magnitude of the delta used for numeric
  /// Jacobians.
  /// \return True on success.
  virtual bool verify(const double *x_raw,
                      double purturbation_magnitude = 1.0e-6) const;
};

} // namespace autocal
#endif /* AUTOCAL_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP */
