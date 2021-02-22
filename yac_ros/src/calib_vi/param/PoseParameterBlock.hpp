#ifndef YAC_CERES_POSEPARAMETERBLOCK_HPP
#define YAC_CERES_POSEPARAMETERBLOCK_HPP

#include <Eigen/Core>

#include "../common/Time.hpp"
#include "../common/Transformation.hpp"
#include "ParameterBlockSized.hpp"
#include "PoseLocalParameterization.hpp"

namespace yac {

/// \brief Wraps the parameter block for a pose estimate
class PoseParameterBlock
    : public ParameterBlockSized<7, 6, yac::Transformation> {
public:
  /// \brief The estimate type (yac::Transformation ).
  typedef yac::Transformation estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<7, 6, estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  PoseParameterBlock() : base_t::ParameterBlockSized() {
    setFixed(false);
  }

  /// \brief Constructor with estimate and time.
  /// @param[in] T_WS The pose estimate as T_WS.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  PoseParameterBlock(const yac::Transformation &T_WS,
                    uint64_t id,
                    const yac::Time &timestamp = yac::Time(0)) {
    setEstimate(T_WS);
    setId(id);
    setTimestamp(timestamp);
    setFixed(false);
  }

  /// \brief Trivial destructor.
  virtual ~PoseParameterBlock() {}

  // setters
  /// @brief Set estimate of this parameter block.
  /// @param[in] T_WS The estimate to set this to.
  virtual void setEstimate(const yac::Transformation &T_WS) {
    const Eigen::Vector3d r = T_WS.r();
    const Eigen::Vector4d q = T_WS.q().coeffs();
    parameters_[0] = r[0];
    parameters_[1] = r[1];
    parameters_[2] = r[2];
    parameters_[3] = q[0];
    parameters_[4] = q[1];
    parameters_[5] = q[2];
    parameters_[6] = q[3];
  }

  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const yac::Time &timestamp) { timestamp_ = timestamp; }

  // getters
  /// @brief Get estimate.
  /// \return The estimate.
  virtual yac::Transformation estimate() const {
    return yac::
        Transformation(Eigen::Vector3d(parameters_[0],
                                      parameters_[1],
                                      parameters_[2]),
                      Eigen::Quaterniond(parameters_[6],
                                          parameters_[3],
                                          parameters_[4],
                                          parameters_[5]));
  }

  /// \brief Get the time.
  /// \return The timestamp of this state.
  yac::Time timestamp() const { return timestamp_; }

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
    OKVISPoseLocalParameterization::plus(x0, Delta_Chi, x0_plus_Delta);
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double *x0, double *jacobian) const {
    OKVISPoseLocalParameterization::plusJacobian(x0, jacobian);
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
    OKVISPoseLocalParameterization::minus(x0, x0_plus_Delta, Delta_Chi);
  }

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double *x0, double *jacobian) const {
    OKVISPoseLocalParameterization::liftJacobian(x0, jacobian);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "PoseParameterBlock"; }

private:
  yac::Time timestamp_; ///< Time of this state.
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
  ExtrinsicsParameterBlock(const yac::Transformation &T_WS,
                           uint64_t id,
                           const yac::Time &timestamp = yac::Time(0))
    : PoseParameterBlock(T_WS, id, timestamp) {}

  /// \brief Trivial destructor.
  virtual ~ExtrinsicsParameterBlock() {}

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "ExtrinsicsParameterBlock"; }
};

} // namespace yac
#endif /* YAC_CERES_POSEPARAMETERBLOCK_HPP */
