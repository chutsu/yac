#ifndef AUTOCAL_CERES_REPROJECTION_ERROR_HPP
#define AUTOCAL_CERES_REPROJECTION_ERROR_HPP

#include <vector>
#include <memory>
#include <ceres/ceres.h>

#include "../common/Common.hpp"
#include "../common/Time.hpp"
#include "../common/Transformation.hpp"
#include "../param/PoseLocalParameterization.hpp"
#include "ErrorInterface.hpp"
#include "ReprojectionErrorBase.hpp"

namespace autocal {

/// \brief The 2D keypoint reprojection error.
/// \tparam GEOMETRY_TYPE The camera gemetry type.
template <class GEOMETRY_TYPE>
class ReprojectionError : public ReprojectionError2dBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // For debugging
  Time timestamp;
  size_t pose_id;
  size_t point_id;

  /// \brief Make the camera geometry type accessible.
  typedef GEOMETRY_TYPE camera_geometry_t;

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<2, 7, 4, 7> base_t;

  /// \brief Number of residuals (2)
  static const int kNumResiduals = 2;

  /// \brief The keypoint type (measurement type).
  typedef Eigen::Vector2d keypoint_t;

  /// \brief Default constructor.
  ReprojectionError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] cameraGeometry The underlying camera geometry.
  /// @param[in] cameraId The id of the camera in the
  /// autocal::cameras::NCameraSystem.
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  ReprojectionError(std::shared_ptr<const camera_geometry_t> cameraGeometry,
                    uint64_t cameraId,
                    const measurement_t &measurement,
                    const covariance_t &information);

  /// \brief Trivial destructor.
  virtual ~ReprojectionError() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  virtual void setMeasurement(const measurement_t &measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the underlying camera model.
  /// @param[in] cameraGeometry The camera geometry.
  void
  setCameraGeometry(std::shared_ptr<const camera_geometry_t> cameraGeometry) {
    cameraGeometry_ = cameraGeometry;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  virtual void setInformation(const covariance_t &information);

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  virtual const measurement_t &measurement() const { return measurement_; }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  virtual const covariance_t &information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t &covariance() const { return covariance_; }

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
  virtual std::string typeInfo() const { return "ReprojectionError"; }

protected:
  // the measurement
  measurement_t measurement_; ///< The (2D) measurement.

  /// \brief The camera model:
  std::shared_ptr<const camera_geometry_t> cameraGeometry_;

  // weighting related
  covariance_t information_; ///< The 2x2 information matrix.
  covariance_t
      squareRootInformation_; ///< The 2x2 square root information matrix.
  covariance_t covariance_;   ///< The 2x2 covariance matrix.
};

// Default constructor.
template <class GEOMETRY_T>
ReprojectionError<GEOMETRY_T>::ReprojectionError()
    : cameraGeometry_(new camera_geometry_t) {}

// Construct with measurement and information matrix.
template <class GEOMETRY_T>
ReprojectionError<GEOMETRY_T>::ReprojectionError(
    std::shared_ptr<const camera_geometry_t> cameraGeometry,
    uint64_t cameraId,
    const measurement_t &measurement,
    const covariance_t &information) {
  setCameraId(cameraId);
  setMeasurement(measurement);
  setInformation(information);
  setCameraGeometry(cameraGeometry);
}

// Set the information.
template <class GEOMETRY_T>
void ReprojectionError<GEOMETRY_T>::setInformation(
    const covariance_t &information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error
  // weighting
  Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
}

// This evaluates the error term and additionally computes the Jacobians.
template <class GEOMETRY_T>
bool ReprojectionError<GEOMETRY_T>::Evaluate(double const *const *parameters,
                                             double *residuals,
                                             double **jacobians) const {

  return EvaluateWithMinimalJacobians(parameters,
                                      residuals,
                                      jacobians,
                                      NULL); // debug test only
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template <class GEOMETRY_T>
bool ReprojectionError<GEOMETRY_T>::EvaluateWithMinimalJacobians(
    double const *const *parameters,
    double *residuals,
    double **jacobians,
    double **jacobiansMinimal) const {

  // We avoid the use of autocal::Transformation here due to
  // quaternion normalization and so forth. This only matters in order to be
  // able to check Jacobians with numeric differentiation chained, first w.r.t.
  // q and then d_alpha.

  // pose: world to sensor transformation
  Eigen::Map<const Eigen::Vector3d> t_WS_W(&parameters[0][0]);
  const Eigen::Quaterniond q_WS(parameters[0][6],
                                parameters[0][3],
                                parameters[0][4],
                                parameters[0][5]);

  // The point in world coordinates
  Eigen::Map<const Eigen::Vector4d> hp_W(&parameters[1][0]);
  // std::cout << hp_W.transpose() << std::endl;

  // The sensor to camera transformation
  Eigen::Map<const Eigen::Vector3d> t_SC_S(&parameters[2][0]);
  const Eigen::Quaterniond q_SC(parameters[2][6],
                                parameters[2][3],
                                parameters[2][4],
                                parameters[2][5]);

  // Transform the point into the camera:
  Eigen::Matrix3d C_SC = q_SC.toRotationMatrix();
  Eigen::Matrix3d C_CS = C_SC.transpose();
  Eigen::Matrix4d T_CS = Eigen::Matrix4d::Identity();
  T_CS.topLeftCorner<3, 3>() = C_CS;
  T_CS.topRightCorner<3, 1>() = -C_CS * t_SC_S;
  Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();
  Eigen::Matrix3d C_SW = C_WS.transpose();
  Eigen::Matrix4d T_SW = Eigen::Matrix4d::Identity();
  T_SW.topLeftCorner<3, 3>() = C_SW;
  T_SW.topRightCorner<3, 1>() = -C_SW * t_WS_W;
  Eigen::Vector4d hp_S = T_SW * hp_W;
  Eigen::Vector4d hp_C = T_CS * hp_S;

  // Calculate the reprojection error
  measurement_t kp;
  Eigen::Matrix<double, 2, 4> Jh;
  Eigen::Matrix<double, 2, 4> Jh_weighted;
  if (jacobians != NULL) {
    cameraGeometry_->projectHomogeneous(hp_C, &kp, &Jh);
    Jh_weighted = squareRootInformation_ * Jh;
  } else {
    cameraGeometry_->projectHomogeneous(hp_C, &kp);
  }
  measurement_t error = measurement_ - kp;
  // std::cout << "t_WS: " << t_WS_W.transpose() << std::endl;
  // std::cout << "q_WS: " << q_WS.w() << " " << q_WS.x() << " " << q_WS.y() << " " << q_WS.z() << std::endl;
  // std::cout << "T_WS: " << proto::tf(q_WS, t_WS_W) << std::endl;
  // // std::cout << "q_SC: " << q_SC.w() << " " << q_SC.x() << " " << q_SC.y() << " " << q_SC.z() << std::endl;
  // std::cout << "T_SC: " << proto::tf(q_SC, t_SC_S) << std::endl;
  // std::cout << "hp_W: " << hp_W.transpose() << std::endl;
  // std::cout << "hp_C: " << hp_C.transpose() << std::endl;
  // std::cout << "measurement: " << measurement_.transpose() << std::endl;
  // std::cout << "kp: " << kp.transpose() << std::endl;
  // std::cout << "error: " << error.transpose() << std::endl;
  // std::cout << std::endl;
  // exit(0);

  // weight:
  measurement_t weighted_error = squareRootInformation_ * error;

  // assign:
  residuals[0] = weighted_error[0];
  residuals[1] = weighted_error[1];

  // Check validity:
  bool valid = true;
  if (fabs(hp_C[3]) > 1.0e-8) {
    Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
    if (p_C[2] < 0.2) { // 20 cm - not very generic... but reasonable
      // std::cout<<"INVALID POINT"<<std::endl;
      valid = false;
    }
  }

  // Calculate jacobians
  // This is pretty close to Paul Furgale's thesis. eq. 3.100 on page 40
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Vector3d p = hp_W.head<3>() - t_WS_W * hp_W[3];
      Eigen::Matrix<double, 4, 6> J;
      J.setZero();
      J.topLeftCorner<3, 3>() = C_SW * hp_W[3];
      J.topRightCorner<3, 3>() = -C_SW * kinematics::crossMx(p);

      // compute the minimal version
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal = Jh_weighted * T_CS * J;
      if (!valid) {
        J0_minimal.setZero();
      }

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J0(jacobians[0]);
      J0 = J0_minimal * J_lift;

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[0] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
              J0_minimal_mapped(jacobiansMinimal[0]);
          J0_minimal_mapped = J0_minimal;

          // proto::print_matrix("-> J0", J0);
          // proto::print_matrix("-> min J0", J0_minimal);
        }
      }
    }
    if (jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J1(
          jacobians[1]); // map the raw pointer to an Eigen matrix for
                         // convenience
      Eigen::Matrix4d T_CW = (T_CS * T_SW);
      J1 = -Jh_weighted * T_CW;
      if (!valid)
        J1.setZero();

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[1] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>
              J1_minimal_mapped(jacobiansMinimal[1]);
          Eigen::Matrix<double, 4, 3> S;
          S.setZero();
          S.topLeftCorner<3, 3>().setIdentity();
          J1_minimal_mapped =
              J1 * S; // this is for Euclidean-style perturbation only.

          // proto::print_matrix("-> J1", J1);
          // proto::print_matrix("-> min J0", J1_minimal_mapped);
        }
      }
    }
    if (jacobians[2] != NULL) {
      Eigen::Vector3d p = hp_S.head<3>() - t_SC_S * hp_S[3];
      Eigen::Matrix<double, 4, 6> J;
      J.setZero();
      J.topLeftCorner<3, 3>() = C_CS * hp_S[3];
      J.topRightCorner<3, 3>() = -C_CS * kinematics::crossMx(p);

      // compute the minimal version
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J2_minimal;
      J2_minimal = Jh_weighted * J;
      if (!valid)
        J2_minimal.setZero();

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J2(jacobians[2]);
      J2 = J2_minimal * J_lift;

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[2] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
              J2_minimal_mapped(jacobiansMinimal[2]);
          J2_minimal_mapped = J2_minimal;

          // proto::print_matrix("-> J2", J2);
          // proto::print_matrix("-> min J2", J2_minimal);
        }
      }
    }
  }

  return true;
}

} // namespace autocal
#endif /* AUTOCAL_CERES_REPROJECTION_ERROR_HPP */
