#ifndef AUTOCAL_CERES_IMU_ERROR_HPP
#define AUTOCAL_CERES_IMU_ERROR_HPP

#include <mutex>

#include <ceres/ceres.h>

#include "../common/Common.hpp"
#include "../common/Time.hpp"
#include "../common/Measurements.hpp"
#include "../common/Variables.hpp"
#include "../common/Parameters.hpp"
#include "../common/Transformation.hpp"
#include "../param/PoseLocalParameterization.hpp"
#include "ErrorInterface.hpp"

namespace autocal {

#ifndef EST_TIMEDELAY
#define EST_TIMEDELAY 1
#endif

/**
 * Implements a nonlinear IMU factor.
 */
class ImuError
#if EST_TIMEDELAY == 1
    : public ::ceres::SizedCostFunction<
          15 /* number of residuals */,
          7 /* size of first parameter (PoseParameterBlock k) */,
          9 /* size of second parameter (SpeedAndBiasParameterBlock k) */,
          7 /* size of third parameter (PoseParameterBlock k+1) */,
          9 /* size of fourth parameter (SpeedAndBiasParameterBlock k+1) */,
          1 /* size of fifth parameter (TimeDelayParameeterBlock) */>,
#else
    : public ::ceres::SizedCostFunction<
          15 /* number of residuals */,
          7 /* size of first parameter (PoseParameterBlock k) */,
          9 /* size of second parameter (SpeedAndBiasParameterBlock k) */,
          7 /* size of third parameter (PoseParameterBlock k+1) */,
          9 /* size of fourth parameter (SpeedAndBiasParameterBlock k+1) */>,
#endif
      public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // For debugging
  bool debug_jacobians_ = false;

  size_t state_id = 0;
  autocal::Time imu_t0{0};
  autocal::Time imu_t1{0};
  size_t pose0_id = 0;
  size_t pose1_id = 0;
  size_t state0_id = 0;
  size_t state1_id = 0;
  size_t timedelay_id = 0;

  mutable autocal::Transformation T_WS_0_last_;
  mutable autocal::Transformation T_WS_1_last_;
  mutable autocal::SpeedAndBias speedAndBiases_0_last_;
  mutable autocal::SpeedAndBias speedAndBiases_1_last_;
  mutable double time_delay_last_ = 0.0;

  ///< The base in ceres we derive from
#if EST_TIMEDELAY == 1
  typedef ::ceres::SizedCostFunction<15, 7, 9, 7, 9, 1> base_t;
#else
  typedef ::ceres::SizedCostFunction<15, 7, 9, 7, 9> base_t;
#endif

  ///< The number of residuals
  static const int kNumResiduals = 15;

  ///< The type of the covariance.
  typedef Eigen::Matrix<double, 15, 15> covariance_t;

  ///< The type of the information (same matrix dimension as covariance).
  typedef covariance_t information_t;

  ///< The type of hte overall Jacobian.
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;

  ///< The type of the Jacobian w.r.t. poses --
  /// \warning This is w.r.t. minimal tangential space coordinates...
  typedef Eigen::Matrix<double, 15, 7> jacobian0_t;

  ///< The type of Jacobian w.r.t. Speed and biases
  typedef Eigen::Matrix<double, 15, 9> jacobian1_t;

  ImuError();
  ImuError(const autocal::ImuMeasurementDeque &imuMeasurements,
           const autocal::ImuParameters &imuParameters,
           const autocal::Time &t_0,
           const autocal::Time &t_1);
  virtual ~ImuError();

  /**
   * Propagates pose, speeds and biases with given IMU measurements.
   * @remark This can be used externally to perform propagation
   *
   * @param[in] imuMeasurements All the IMU measurements.
   * @param[in] imuParams The parameters to be used.
   * @param[inout] T_WS Start pose.
   * @param[inout] speedAndBiases Start speed and biases.
   * @param[in] t_start Start time.
   * @param[in] t_end End time.
   * @param[out] covariance Covariance for GIVEN start states.
   * @param[out] jacobian Jacobian w.r.t. start states.
   * @return Number of integration steps.
   */
  static int propagation(const autocal::ImuMeasurementDeque &imuMeasurements,
                         const autocal::ImuParameters &imuParams,
                         autocal::Transformation &T_WS,
                         autocal::SpeedAndBias &speedAndBiases,
                         const autocal::Time &t_start,
                         const autocal::Time &t_end,
                         covariance_t *covariance = 0,
                         jacobian_t *jacobian = 0);

  /**
   * Propagates pose, speeds and biases with given IMU measurements.
   *
   * @warning This is not actually const, since the re-propagation must somehow
   * be stored...
   *
   * @param T_WS Start pose.
   * @param speedAndBiases Start speed and biases.
   * @return Number of integration steps.
   */
  int redoPreintegration(const autocal::Transformation &T_WS,
                         const autocal::SpeedAndBias &speedAndBiases,
                         autocal::Time time_start,
                         autocal::Time time_end) const;

  // setters
  /**
   * (Re)set the parameters.
   * @param[in] imuParameters The parameters to be used.
   */
  void setImuParameters(const autocal::ImuParameters &imuParameters) {
    imuParameters_ = imuParameters;
  }

  /**
   * (Re)set the measurements
   * @param imuMeasurements All the IMU measurements.
   */
  void setImuMeasurements(const autocal::ImuMeasurementDeque &imuMeasurements) {
    imuMeasurements_ = imuMeasurements;
  }

  /**
   * (Re)set the start time.
   * @param t_0 Start time.
   */
  void setT0(const autocal::Time &t_0) { t0_ = t_0; }

  /**
   * (Re)set the start time.
   * @param t_1 End time.
   */
  void setT1(const autocal::Time &t_1) { t1_ = t_1; }

  // getters

  /**
   * Get the IMU Parameters.
   * @return the IMU parameters.
   */
  const autocal::ImuParameters &imuParameters() const { return imuParameters_; }

  /**
   * Get the IMU measurements.
   */
  const autocal::ImuMeasurementDeque &imuMeasurements() const {
    return imuMeasurements_;
  }

  /**
   * Get the start time.
   */
  autocal::Time t0() const { return t0_; }

  /**
   * Get the end time.
   */
  autocal::Time t1() const { return t1_; }

  // error term and Jacobian implementation
  /**
   * This evaluates the error term and additionally computes the
   * Jacobians.
   *
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   *
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  /**
   * This evaluates the error term and additionally computes
   * the Jacobians in the minimal internal representation.
   *
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to
   * jacobians).
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const;

  // sizes
  /**
   * Residual dimension.
   */
  size_t residualDim() const { return kNumResiduals; }

  /**
   * Number of parameter blocks.
   */
  virtual size_t parameterBlocks() const {
    return base_t::parameter_block_sizes().size();
  }

  /**
   * Dimension of an individual parameter block.
   *
   * @param parameterBlockId ID of the parameter block of interest.
   * @return Dimension of parameber block
   */
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /**
   * Return parameter block type as string
   */
  virtual std::string typeInfo() const { return "ImuError"; }

  // Parameters
  ///< The IMU parameters.
  autocal::ImuParameters imuParameters_;

  // Measurements
  ///< The IMU measurements used. Must be spanning t0_ - t1_.
  autocal::ImuMeasurementDeque imuMeasurements_;

  // Times
  autocal::Time t0_; ///< The start time (i.e. time of the first set of states).
  autocal::Time t1_; ///< The end time (i.e. time of the sedond set of states).

protected:
  // Preintegration stuff. the mutable is a TERRIBLE HACK, but what can I do.
  ///< Protect access of intermediate results.
  mutable std::mutex preintegrationMutex_;
  // increments (initialise with identity)
  mutable Eigen::Quaterniond Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  mutable Eigen::Matrix3d C_integral_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d C_doubleintegral_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Vector3d acc_integral_ = Eigen::Vector3d::Zero();
  mutable Eigen::Vector3d acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // Cross matrix accumulatrion
  mutable Eigen::Matrix3d cross_ = Eigen::Matrix3d::Zero();

  // Sub-Jacobians
  mutable Eigen::Matrix3d dalpha_db_g_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d dv_db_g_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d dp_db_g_ = Eigen::Matrix3d::Zero();

  ///< The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double, 15, 15> P_delta_ =
      Eigen::Matrix<double, 15, 15>::Zero();

  ///< Reference biases that are updated when called redoPreintegration.
  mutable SpeedAndBiases speedAndBiases_ref_ = SpeedAndBiases::Zero();

  ///< Keeps track of whether redoPreintegration() needs to be called.
  mutable bool redo_ = true;

  ///< Counts the number of preintegrations for statistics.
  mutable int redoCounter_ = 0;

  // information matrix and its square root
  ///< Information matrix
  mutable information_t information_;
  ///< Square root information matrix
  mutable information_t squareRootInformation_;
};

} // namespace autocal
#endif // AUTOCAL_CERES_IMU_ERROR_HPP
