#include "SpeedAndBiasError.hpp"

namespace yac {

SpeedAndBiasError::SpeedAndBiasError(const yac::SpeedAndBias &measurement,
                                     const information_t &information) {
  setMeasurement(measurement);
  setInformation(information);
}

SpeedAndBiasError::SpeedAndBiasError(const yac::SpeedAndBiases &measurement,
                                     double speedVariance,
                                     double gyrBiasVariance,
                                     double accBiasVariance) {
  setMeasurement(measurement);

  information_t information;
  information.setZero();
  information.topLeftCorner<3, 3>() =
      Eigen::Matrix3d::Identity() * 1.0 / speedVariance;
  information.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * 1.0 / gyrBiasVariance;
  information.bottomRightCorner<3, 3>() =
      Eigen::Matrix3d::Identity() * 1.0 / accBiasVariance;

  setInformation(information);
}

void SpeedAndBiasError::setInformation(const information_t &information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error
  // weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
}

bool SpeedAndBiasError::Evaluate(double const *const *parameters,
                                 double *residuals,
                                 double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

bool SpeedAndBiasError::EvaluateWithMinimalJacobians(
    double const *const *parameters,
    double *residuals,
    double **jacobians,
    double **jacobiansMinimal) const {

  // compute error
  Eigen::Map<const yac::SpeedAndBias> estimate(parameters[0]);
  yac::SpeedAndBias error = measurement_ - estimate;

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 9, 1>> weighted_error(residuals);
  weighted_error = squareRootInformation_ * error;

  // compute Jacobian - this is rather trivial in this case...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> J0(jacobians[0]);
      J0 = -squareRootInformation_ * Eigen::Matrix<double, 9, 9>::Identity();
    }
  }
  if (jacobiansMinimal != NULL) {
    if (jacobiansMinimal[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> J0min(
          jacobiansMinimal[0]);
      J0min = -squareRootInformation_ * Eigen::Matrix<double, 9, 9>::Identity();
    }
  }

  return true;
}

} // namespace yac
