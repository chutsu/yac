#ifndef AUTOCAL_CERES_POSEPARAMETERBLOCK_HPP
#define AUTOCAL_CERES_POSEPARAMETERBLOCK_HPP

#include <Eigen/Core>

#include "../common/Time.hpp"
#include "../common/Transformation.hpp"
#include "./ParameterBlockSized.hpp"
#include "./PoseLocalParameterization.hpp"

namespace autocal {

/// \brief Wraps the parameter block for a pose estimate
class PoseParameterBlock
    : public ParameterBlockSized<7, 6, autocal::Transformation> {
public:
  /// \brief The estimate type (autocal::Transformation ).
  typedef autocal::Transformation estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<7, 6, estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  PoseParameterBlock();

  /// \brief Constructor with estimate and time.
  /// @param[in] T_WS The pose estimate as T_WS.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  PoseParameterBlock(const autocal::Transformation &T_WS,
                     uint64_t id,
                     const autocal::Time &timestamp = autocal::Time(0));

  /// \brief Trivial destructor.
  virtual ~PoseParameterBlock();

  // setters
  /// @brief Set estimate of this parameter block.
  /// @param[in] T_WS The estimate to set this to.
  virtual void setEstimate(const autocal::Transformation &T_WS);

  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const autocal::Time &timestamp) { timestamp_ = timestamp; }

  // getters
  /// @brief Get estimate.
  /// \return The estimate.
  virtual autocal::Transformation estimate() const;

  /// \brief Get the time.
  /// \return The timestamp of this state.
  autocal::Time timestamp() const { return timestamp_; }

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
    PoseLocalParameterization::plus(x0, Delta_Chi, x0_plus_Delta);
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double *x0, double *jacobian) const {
    PoseLocalParameterization::plusJacobian(x0, jacobian);
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
    PoseLocalParameterization::minus(x0, x0_plus_Delta, Delta_Chi);
  }

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double *x0, double *jacobian) const {
    PoseLocalParameterization::liftJacobian(x0, jacobian);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "PoseParameterBlock"; }

private:
  autocal::Time timestamp_; ///< Time of this state.
};

/// \brief Wraps the parameter block for a pose estimate
class ExtrinsicsParameterBlock : public PoseParameterBlock {
public:
  /// \brief Default constructor (assumes not fixed).
  ExtrinsicsParameterBlock() : PoseParameterBlock() {}

  /// \brief Constructor with estimate and time.
  /// @param[in] T_WS The pose estimate as T_WS.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  ExtrinsicsParameterBlock(const autocal::Transformation &T_WS,
                           uint64_t id,
                           const autocal::Time &timestamp = autocal::Time(0))
    : PoseParameterBlock(T_WS, id, timestamp) {}

  /// \brief Trivial destructor.
  virtual ~ExtrinsicsParameterBlock() {}

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "ExtrinsicsParameterBlock"; }
};

} // namespace autocal
#endif /* AUTOCAL_CERES_POSEPARAMETERBLOCK_HPP */
