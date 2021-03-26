#ifndef YAC_CAMERA_PARAMETER_BLOCK_HPP
#define YAC_CAMERA_PARAMETER_BLOCK_HPP

#include <Eigen/Core>
#include "../common/Time.hpp"
#include "ParameterBlockSized.hpp"

namespace yac {

typedef Eigen::Matrix<double, 8, 1> CameraParams;

class CameraParameterBlock : public ParameterBlockSized<8, 8, CameraParams> {
public:
  /// \brief The base class type.
  typedef ParameterBlockSized<8, 8, CameraParams> base_t;

  /// \brief The estimate type (8D vector).
  typedef CameraParams estimate_t;

  /// \brief Default constructor (assumes not fixed).
  CameraParameterBlock() : base_t::ParameterBlockSized() {
    setFixed(false);
  }

  /// \brief Constructor with estimate and time.
  /// @param[in] params The camera parameter estimates.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  CameraParameterBlock(const CameraParams &camera_params,
                       uint64_t id,
                       const yac::Time &timestamp=Time{0}) {
    setEstimate(camera_params);
    setId(id);
    setTimestamp(timestamp);
    setFixed(false);
  }

  /// \brief Trivial destructor.
  virtual ~CameraParameterBlock() {}

  // setters
  /// @brief Set estimate of this parameter block.
  /// @param[in] params The estimate to set this to.
  virtual void setEstimate(const CameraParams &params) {
    for (int i = 0; i < base_t::Dimension; ++i)
      parameters_[i] = params[i];
  }

  /// \brief Set the time.
  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const yac::Time &timestamp) { timestamp_ = timestamp; }

  // getters
  /// @brief Get estimate.
  /// \return The estimate.
  virtual CameraParams estimate() const {
    CameraParams camera_params;
    for (int i = 0; i < base_t::Dimension; ++i)
      camera_params[i] = parameters_[i];
    return camera_params;
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
    Eigen::Map<const Eigen::Matrix<double, 8, 1>> x0_(x0);
    Eigen::Map<const Eigen::Matrix<double, 8, 1>> Delta_Chi_(Delta_Chi);
    Eigen::Map<Eigen::Matrix<double, 8, 1>> x0_plus_Delta_(x0_plus_Delta);
    x0_plus_Delta_ = x0_ + Delta_Chi_;
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  //  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double * /*unused: x*/,
                            double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> identity(jacobian);
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
    Eigen::Map<const Eigen::Matrix<double, 8, 1>> x0_(x0);
    Eigen::Map<Eigen::Matrix<double, 8, 1>> Delta_Chi_(Delta_Chi);
    Eigen::Map<const Eigen::Matrix<double, 8, 1>> x0_plus_Delta_(x0_plus_Delta);
    Delta_Chi_ = x0_plus_Delta_ - x0_;
  }

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  //  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double * /*unused: x*/,
                            double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> identity(jacobian);
    identity.setIdentity();
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "CameraParameterBlock"; }

private:
  yac::Time timestamp_; ///< Time of this state.
};

} // namespace yac
#endif /* YAC_CAMERA_PARAMETER_BLOCK_HPP */
