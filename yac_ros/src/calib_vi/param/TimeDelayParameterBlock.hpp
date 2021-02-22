#ifndef YAC_TIME_DELAY_PARAMETER_BLOCK_HPP
#define YAC_TIME_DELAY_PARAMETER_BLOCK_HPP

#include <Eigen/Core>
#include "ParameterBlockSized.hpp"

namespace yac {

class TimeDelayParameterBlock : public ParameterBlockSized<1, 1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The estimate type
  typedef double estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<1, 1, estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  TimeDelayParameterBlock()
      : base_t::ParameterBlockSized() {
    setFixed(false);
  }

  /// \brief Constructor with estimate and time.
  /// @param[in] time_delay Time delay in seconds
  /// @param[in] id The (unique) ID of this block.
  /// initialised.
  TimeDelayParameterBlock(const double &time_delay, uint64_t id) {
    setEstimate(time_delay);
    setId(id);
    setFixed(false);
  }

  /// \brief Trivial destructor.
  virtual ~TimeDelayParameterBlock() {}

  /// @name Setters
  /// @{

  /// @brief Set estimate of this parameter block.
  /// @param[in] time_delay Time delay in seconds
  virtual void setEstimate(const double &time_delay) {
    parameters_[0] = time_delay;
  }

  /// @}

  /// @name Getters
  /// @{

  /// @brief Get estimate
  /// \return The estimate.
  virtual double estimate() const { return parameters_[0]; }

  /// @}

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double *x0,
                    const double *Delta_Chi,
                    double *x0_plus_Delta) const {
    Eigen::Map<const Eigen::Matrix<double, 1, 1>> x0_(x0);
    Eigen::Map<const Eigen::Matrix<double, 1, 1>> Delta_Chi_(Delta_Chi);
    Eigen::Map<Eigen::Matrix<double, 1, 1>> x0_plus_Delta_(x0_plus_Delta);
    x0_plus_Delta_ = x0_ + Delta_Chi_;
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double * /* unused: x0 */,
                            double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> identity(jacobian);
    identity.setIdentity();
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  /// \return True on success.
  virtual void minus(const double *x0,
                     const double *x0_plus_Delta,
                     double *Delta_Chi) const {
    Eigen::Map<const Eigen::Matrix<double, 1, 1>> x0_(x0);
    Eigen::Map<Eigen::Matrix<double, 1, 1>> Delta_Chi_(Delta_Chi);
    Eigen::Map<const Eigen::Matrix<double, 1, 1>> x0_plus_Delta_(x0_plus_Delta);
    Delta_Chi_ = x0_plus_Delta_ - x0_;
  }

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double * /*unused: *x0 */,
                            double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> identity(jacobian);
    identity.setIdentity();
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "TimeDelayParameterBlock"; }
};

} // namespace yac
#endif /* YAC_TIME_DELAY_PARAMETER_BLOCK_HPP */
