#ifndef YAC_TIME_DELAY_ERROR_HPP
#define YAC_TIME_DELAY_ERROR_HPP

#include <vector>
#include <Eigen/Core>

#include <ceres/ceres.h>

#include "core.hpp"
// #include "yac/util/assert_macros.hpp"
#include "ErrorInterface.hpp"

namespace yac {

class TimeDelayError
    : public ::ceres::SizedCostFunction<1, 1>,
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<1, 1> base_t;

  /// \brief Number of residuals
  static const int kNumResiduals = 1;

  /// \brief The information matrix type
  typedef double information_t;

  /// \brief The covariance matrix type (same as information).
  typedef double covariance_t;

  /// \brief Default constructor.
  TimeDelayError() {
    // residuals_debug = (double *) malloc(sizeof(double) * 2);
    // error_debug = (double *) malloc(sizeof(double) * 2);
    // J.push_back(zeros(1, 1));
  }

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  TimeDelayError(const double &measurement,
                 const double &information) {
    // residuals_debug = (double *) malloc(sizeof(double) * 1);
    // error_debug = (double *) malloc(sizeof(double) * 1);
    // J.push_back(zeros(1, 1));

    setMeasurement(measurement);
    setInformation(information);
  }

  /// \brief Trivial destructor.
  virtual ~TimeDelayError() {
    // free(residuals_debug);
    // free(error_debug);
  }

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const double &measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const double &information) {
    information_ = information;
    covariance_ = 1.0 / information;
    squareRootInformation_ = sqrt(information);
  }

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  const double &measurement() const { return measurement_; }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t &information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const covariance_t &covariance() const { return covariance_; }

  bool Evaluate(double const *const *parameters,
                double *residuals,
                double **jacobians) const {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
  }

  bool EvaluateWithMinimalJacobians(
      double const *const *parameters,
      double *residuals,
      double **jacobians,
      double **jacobiansMinimal) const {
    // Compute error
    const double estimate = parameters[0][0];
    double error = measurement_ - estimate;

    // weigh it
    residuals[0] = squareRootInformation_ * error;
    // r = weighted_error;

    // assign:
    // residuals[0] = weighted_error[0];
    // residuals_debug[0] = residuals[0];
    // error_debug[0] = error;

    // compute Jacobian - this is rather trivial in this case...
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = -squareRootInformation_ * Eigen::Matrix<double, 1, 1>::Identity();
        // J[0] = J0;
      }
    }
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J0min(
            jacobiansMinimal[0]);
        J0min = -squareRootInformation_ * Eigen::Matrix<double, 1, 1>::Identity();
        // J[0] = J0min;
      }
    }

    return true;
  }

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return parameter_block_sizes().size(); }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Residual block type as string
  virtual std::string typeInfo() const { return "TimeDelayError"; }

protected:
  // The measurement
  double measurement_; ///< Measurement.

  // Weighting related
  information_t information_; ///< The information matrix.
  information_t squareRootInformation_; ///< The square root information matrix.
  covariance_t covariance_;   ///< The covariance matrix.
};

} // namespace yac
#endif /* YAC_TIME_DELAY_ERROR_HPP */
