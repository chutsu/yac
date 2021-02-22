#ifndef YAC_ERROR_POSE_ERROR_HPP
#define YAC_ERROR_POSE_ERROR_HPP

#include <vector>
#include <ceres/ceres.h>

// #include "yac/util/assert_macros.hpp"
#include "../common/Transformation.hpp"
#include "ErrorInterface.hpp"

namespace yac {

/// \brief Absolute error of a pose.
class PoseError
    : public ::ceres::SizedCostFunction<6 /* number of residuals */,
                                        7 /* size of first parameter */>,
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<6, 7> base_t;

  /// \brief The number of residuals (6).
  static const int kNumResiduals = 6;

  /// \brief The information matrix type (6x6).
  typedef Eigen::Matrix<double, 6, 6> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 6, 6> covariance_t;

  /// \brief Default constructor.
  PoseError() {}

  /// \brief Construct with measurement and information matrix.
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  PoseError(const yac::Transformation &measurement,
            const Eigen::Matrix<double, 6, 6> &information);

  /// \brief Construct with measurement and variance.
  /// @param[in] measurement The measurement.
  /// @param[in] translationVariance The translation variance.
  /// @param[in] rotationVariance The rotation variance.
  PoseError(const yac::Transformation &measurement,
            double translationVariance,
            double rotationVariance);

  /// \brief Trivial destructor.
  virtual ~PoseError();

  /// \name Setters
  /// \{

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const yac::Transformation &measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t &information);

  /// \}
  /// \name Getters
  /// \{

  /// \brief Get the measurement.
  /// \return The measurement vector.
  const yac::Transformation &measurement() const {
    return measurement_;
  }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t &information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const information_t &covariance() const { return covariance_; }

  /// \}

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
  size_t parameterBlocks() const {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "PoseError"; }

protected:
  // the measurement
  yac::Transformation measurement_; ///< The pose measurement.

  // weighting related
  information_t information_; ///< The 6x6 information matrix.
  information_t
      squareRootInformation_; ///< The 6x6 square root information matrix.
  covariance_t covariance_;   ///< The 6x6 covariance matrix.
};

} // namespace yac
#endif /* YAC_ERROR_POSE_ERROR_HPP */
