#ifndef YAC_PARAMETER_BLOCK_HPP
#define YAC_PARAMETER_BLOCK_HPP

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <ceres/ceres.h>

namespace yac {

/// @brief Base class providing the interface for parameter blocks.
class ParameterBlock {
public:
  /// @brief Default constructor, assumes not fixed and no local
  /// parameterisation.
  ParameterBlock() : id_(0), fixed_(false), localParameterizationPtr_(0) {}

  /// \brief Trivial destructor.
  virtual ~ParameterBlock() {}

  /// @name Setters
  /// @{

  /// @brief Set parameter block ID
  /// @param[in] id A unique ID.
  void setId(uint64_t id) { id_ = id; }

  /// @brief Directly set values of this parameter block.
  /// @param[in] parameters Pointer to parameters to be copied into this.
  virtual void setParameters(const double *parameters) = 0;

  /// @brief Whether or not this should be optimised at all.
  /// @param[in] fixed True if not to be optimised.
  void setFixed(bool fixed) { fixed_ = fixed; }

  /// @}

  /// @name Getters
  /// @{

  /// @brief Get parameter values.
  virtual double *parameters() = 0;

  /// @brief Get parameter values.
  virtual const double *parameters() const = 0;

  /// @brief Get parameter block ID.
  uint64_t id() const { return id_; }
  /// @brief Get the dimension of the parameter block.
  virtual size_t dimension() const = 0;

  /// @brief The dimension of the internal parameterisation (minimal
  /// representation).
  virtual size_t minimalDimension() const = 0;

  /// @brief Whether or not this is optimised at all.
  bool fixed() const { return fixed_; }

  /// @}
  // minimal internal parameterization
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double *x0,
                    const double *Delta_Chi,
                    double *x0_plus_Delta) const = 0;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double *x0, double *jacobian) const = 0;

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  /// \return True on success.
  virtual void minus(const double *x0,
                     const double *x0_plus_Delta,
                     double *Delta_Chi) const = 0;

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double *x0, double *jacobian) const = 0;

  /// @name Local parameterization
  /// @{
  /**
   * @brief Set which local parameterisation object to use.
   * @param localParameterizationPtr The local parameterisation object to use.
   */
  virtual void setLocalParameterizationPtr(
      const ::ceres::LocalParameterization *localParameterizationPtr) {
    localParameterizationPtr_ = localParameterizationPtr;
  }

  /**
   * @brief The local parameterisation object to use.
   */
  virtual const ::ceres::LocalParameterization *
  localParameterizationPtr() const {
    return localParameterizationPtr_;
  }
  /// @}
  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const = 0;

protected:
  /// @brief ID of the parameter block
  uint64_t id_;
  /// @brief Whether or not this should be optimised at all
  /// (ceres::problem::setFixed)
  bool fixed_;
  /// @brief The local parameterisation object to use.
  const ::ceres::LocalParameterization *localParameterizationPtr_;
};

} // namespace yac
#endif // YAC_PARAMETER_BLOCK_HPP
