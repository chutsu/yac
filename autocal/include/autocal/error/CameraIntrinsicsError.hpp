#ifndef AUTOCAL_CERES_CAMERA_INTRINSICS_ERROR_HPP
#define AUTOCAL_CERES_CAMERA_INTRINSICS_ERROR_HPP

#include <vector>
#include <Eigen/Core>

#include <ceres/ceres.h>

#include "../common/Common.hpp"
#include "autocal/error/ErrorInterface.hpp"

namespace autocal {

class CameraIntrinsicsError
    : public ::ceres::SizedCostFunction<8, 8>,
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<8, 8> base_t;

  /// \brief Number of residuals
  static const int kNumResiduals = 8;

  /// \brief The information matrix type
  typedef Eigen::Matrix<double, 8, 8> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 8, 8> covariance_t;

  /// \brief Default constructor.
  CameraIntrinsicsError() {
    // residuals_debug = (double *) malloc(sizeof(double) * 2);
    // error_debug = (double *) malloc(sizeof(double) * 2);
    // J.push_back(zeros(8, 8));
  }

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  CameraIntrinsicsError(const vecx_t &measurement,
                        const information_t &information) {
    // residuals_debug = (double *) malloc(sizeof(double) * 8);
    // error_debug = (double *) malloc(sizeof(double) * 8);
    // J.push_back(zeros(8, 8));

    setMeasurement(measurement);
    setInformation(information);
  }

  /// \brief Trivial destructor.
  virtual ~CameraIntrinsicsError() {
    // free(residuals_debug);
    // free(error_debug);
  }

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const vecx_t &measurement) {
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
  const vecx_t &measurement() const { return measurement_; }

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
    const vec_t<8> estimate{parameters[0]};
    vecx_t error = measurement_ - estimate;

    // weigh it
    Eigen::Map<Eigen::Matrix<double, 8, 1>> weighted_error(residuals);
    weighted_error = squareRootInformation_ * error;
    // r = weighted_error;

    // assign:
    for (int i = 0; i < 8; i++) {
      residuals[i] = weighted_error[i];
      // residuals_debug[i] = weighted_error[i];
      // error_debug[i] = error(i);
    }

    // compute Jacobian - this is rather trivial in this case...
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = -squareRootInformation_ * Eigen::Matrix<double, 8, 8>::Identity();
        // J[0] = J0;
      }
    }
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> J0min(
            jacobiansMinimal[0]);
        J0min = -squareRootInformation_ * Eigen::Matrix<double, 8, 8>::Identity();
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
  virtual std::string typeInfo() const { return "CameraIntrinsicsError"; }

protected:
  // The measurement
  vecx_t measurement_; ///< Measurement.

  // Weighting related
  information_t information_; ///< The information matrix.
  information_t squareRootInformation_; ///< The square root information matrix.
  covariance_t covariance_;   ///< The covariance matrix.
};

} // namespace autocal
#endif /* AUTOCAL_CERES_CAMERA_INTRINSICS_ERROR_HPP */
