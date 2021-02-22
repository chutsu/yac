#ifndef YAC_SPEEDANDBIASERROR_HPP
#define YAC_SPEEDANDBIASERROR_HPP

#include <vector>
#include <Eigen/Core>

#include <ceres/ceres.h>

// #include "yac/util/assert_macros.hpp"
#include "../common/Variables.hpp"
#include "ErrorInterface.hpp"

namespace yac {

class SpeedAndBiasError
    : public ::ceres::SizedCostFunction<9 /* number of residuals */,
                                        9 /* size of first parameter */>,
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<9, 9> base_t;

  /// \brief Number of residuals (9)
  static const int kNumResiduals = 9;

  /// \brief The information matrix type (9x9).
  typedef Eigen::Matrix<double, 9, 9> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 9, 9> covariance_t;

  /// \brief Default constructor.
  SpeedAndBiasError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  SpeedAndBiasError(const yac::SpeedAndBias &measurement,
                    const information_t &information);

  /// \brief Construct with measurement and variance.
  /// @param[in] measurement The measurement.
  /// @param[in] speedVariance The variance of the speed measurement, i.e.
  /// information_ has variance in its diagonal.
  /// @param[in] gyrBiasVariance The variance of the gyro bias measurement, i.e.
  /// information_ has variance in its diagonal.
  /// @param[in] accBiasVariance The variance of the accelerometer bias
  /// measurement, i.e. information_ has variance in its diagonal.
  SpeedAndBiasError(const yac::SpeedAndBiases &measurement,
                    double speedVariance,
                    double gyrBiasVariance,
                    double accBiasVariance);

  /// \brief Trivial destructor.
  virtual ~SpeedAndBiasError() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const yac::SpeedAndBias &measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t &information);

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  const yac::SpeedAndBias &measurement() const { return measurement_; }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t &information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const covariance_t &covariance() const { return covariance_; }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the
   * Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to
   * jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                            double *residuals,
                                            double **jacobians,
                                            double **jacobiansMinimal) const;

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
  virtual std::string typeInfo() const { return "SpeedAndBiasError"; }

protected:
  // the measurement
  yac::SpeedAndBias measurement_; ///< The (9D) measurement.

  // weighting related
  information_t information_; ///< The 9x9 information matrix.
  information_t
      squareRootInformation_; ///< The 9x9 square root information matrix.
  covariance_t covariance_;   ///< The 9x9 covariance matrix.
};

} // namespace yac
#endif /* YAC_SPEEDANDBIASERROR_HPP */
