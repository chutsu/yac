#ifndef YAC_FIDUCIALERROR_HPP
#define YAC_FIDUCIALERROR_HPP

#include <vector>
#include <Eigen/Core>

#include <ceres/ceres.h>

// #include "yac/util/assert_macros.hpp"
#include "core.hpp"
#include "ErrorInterface.hpp"

namespace yac {

class FiducialError
    : public ::ceres::SizedCostFunction<2, 2>,
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<2, 2> base_t;

  /// \brief Number of residuals (2)
  static const int kNumResiduals = 2;

  /// \brief The information matrix type (2x2).
  typedef Eigen::Matrix<double, 2, 2> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 2, 2> covariance_t;

  /// \brief Default constructor.
  FiducialError() {
    // residuals_debug = (double *) malloc(sizeof(double) * 2);
    // error_debug = (double *) malloc(sizeof(double) * 2);
    // J.push_back(zeros(2, 2));
  }

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  FiducialError(const vec2_t &measurement,
                const information_t &information) {
    // residuals_debug = (double *) malloc(sizeof(double) * 2);
    // error_debug = (double *) malloc(sizeof(double) * 2);
    // J.push_back(zeros(2, 2));

    setMeasurement(measurement);
    setInformation(information);
  }

  /// \brief Trivial destructor.
  virtual ~FiducialError() {
    // free(residuals_debug);
    // free(error_debug);
  }

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const vec2_t &measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t &information) {
    information_ = information;
    covariance_ = information.inverse();

    // perform the Cholesky decomposition on order to obtain the correct error
    // weighting
    Eigen::LLT<information_t> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();
  }

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  const vec2_t &measurement() const { return measurement_; }

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
    const double roll = parameters[0][0];
    const double pitch = parameters[0][1];
    const vec2_t estimate{roll, pitch};
    vec2_t error = measurement_ - estimate;

    // weigh it
    Eigen::Map<Eigen::Matrix<double, 2, 1>> weighted_error(residuals);
    weighted_error = squareRootInformation_ * error;
    // r = weighted_error;

    // assign:
    residuals[0] = weighted_error[0];
    residuals[1] = weighted_error[1];
    // residuals_debug[0] = weighted_error[0];
    // residuals_debug[1] = weighted_error[1];
    // error_debug[0] = error(0);
    // error_debug[1] = error(1);

    // compute Jacobian - this is rather trivial in this case...
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = -squareRootInformation_ * Eigen::Matrix<double, 2, 2>::Identity();
        // J[0] = J0;
      }
    }
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J0min(
            jacobiansMinimal[0]);
        J0min = -squareRootInformation_ * Eigen::Matrix<double, 2, 2>::Identity();
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
  virtual std::string typeInfo() const { return "FiducialError"; }

protected:
  // The measurement
  vec2_t measurement_; ///< Measurement.

  // Weighting related
  information_t information_; ///< The information matrix.
  information_t squareRootInformation_; ///< The square root information matrix.
  covariance_t covariance_;   ///< The covariance matrix.
};

} // namespace yac
#endif /* YAC_FIDUCIALERROR_HPP */
