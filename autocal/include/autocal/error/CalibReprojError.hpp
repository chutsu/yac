#ifndef AUTOCAL_CERES_CALIB_REPROJ_ERROR_HPP
#define AUTOCAL_CERES_CALIB_REPROJ_ERROR_HPP

#include <vector>
#include <memory>
#include <ceres/ceres.h>

#include "core.hpp"
#include "../common/Common.hpp"
#include "../common/Time.hpp"
#include "../common/Transformation.hpp"
#include "../cv/CameraBase.hpp"
#include "../param/PoseLocalParameterization.hpp"
#include "ErrorInterface.hpp"
#include "ReprojectionErrorBase.hpp"

namespace autocal {

template <class GEOMETRY_TYPE>
class CalibReprojError : public CalibReprojErrorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // For debugging
  Time timestamp_;
  size_t pose_id_ = 0;
  size_t tag_id_ = 0;
  size_t corner_id_ = 0;

  ///< Fiducial pose
  mat4_t T_WF_;

  ///< Point in fiducial
  vec3_t p_F_;

  ///< Make the camera geometry type accessible.
  typedef GEOMETRY_TYPE camera_geometry_t;
  int image_width_ = 0;
  int image_height_ = 0;

  ///< The base class type.
  // typedef ::ceres::SizedCostFunction<2, 7, 2, 7, 8> base_t;
  typedef ::ceres::SizedCostFunction<2, 7, 2, 7, 7, 8> base_t;

  ///< Number of residuals (2)
  static const int kNumResiduals = 2;

  ///< The keypoint type (measurement type).
  typedef vec2_t keypoint_t;

  ///< Projected point
  mutable measurement_t kp_projected_;

  CalibReprojError() : cameraGeometry_(new camera_geometry_t) { }

  CalibReprojError(const uint64_t cameraId,
                   const int tag_id,
                   const int corner_id,
                   const vec3_t &p_F,
                   const mat4_t &T_WF,
                   const measurement_t &measurement,
                   const covariance_t &information) {
    setCameraId(cameraId);
    tag_id_ = tag_id;
    corner_id_ = corner_id;
    p_F_ = p_F;
    T_WF_ = T_WF;

    setMeasurement(measurement);
    setInformation(information);
  }

  virtual ~CalibReprojError() { }

  /// Set the measurement.
  /// @param[in] measurement The measurement.
  virtual void setMeasurement(const measurement_t &measurement) {
    measurement_ = measurement;
  }

  /// Set information matrix
  /// @param[in] information The information (weight) matrix.
  virtual void setInformation(const covariance_t &information) {
    information_ = information;
    covariance_ = information.inverse();
    // perform the Cholesky decomposition on order to obtain the correct error
    // weighting
    Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();
  }

  /// Get measurement
  /// @return The measurement vector
  virtual const measurement_t &measurement() const { return measurement_; }
  measurement_t measurement() { return measurement_; }

  /// Get information matrix
  /// @return The information (weight) matrix
  virtual const covariance_t &information() const { return information_; }
  virtual covariance_t information() { return information_; }

  /// Get covariance matrix
  /// @return The inverse information (covariance) matrix
  virtual const covariance_t &covariance() const { return covariance_; }
  virtual covariance_t covariance() { return covariance_; }

  /**
   * This evaluates the error term and additionally computes the
   * Jacobians.
   *
   * The parameter order is:
   *
   * - T_WS: sensor pose
   * - T_WF: fiducial pose
   * - T_SC: sensor-camera extrinsic pose
   * - KD: camera intrinsics + distortion params
   *
   * @param[in] parameters Pointer to the parameters (see ceres)
   * @param[in] residuals Pointer to the residual vector (see ceres)
   * @param[in] jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
    return EvaluateWithMinimalJacobians(parameters,
                                        residuals,
                                        jacobians,
                                        NULL); // debug test only
  }

  /**
   * This evaluates the error term and additionally computes the Jacobians in
   * the minimal internal representation.
   *
   * The parameter order is:
   *
   * - T_WS: sensor pose
   * - T_WF: fiducial pose
   * - T_SC: sensor-camera extrinsic pose
   * - KD: camera intrinsics + distortion params
   *
   * @param[in] parameters Pointer to the parameters (see ceres)
   * @param[in] residuals Pointer to the residual vector (see ceres)
   * @param[in] jacobians Pointer to the Jacobians (see ceres)
   * @param[in] jacobiansMinimal Pointer to the minimal Jacobians (equivalent to
   * jacobians).
   *
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const {
    // T_WS
    Eigen::Map<const Eigen::Vector3d> r_WS(&parameters[0][0]);
    const Eigen::Quaterniond q_WS(parameters[0][6],
                                  parameters[0][3],
                                  parameters[0][4],
                                  parameters[0][5]);
    Eigen::Matrix4d T_WS;
    T_WS.setIdentity();
    T_WS.block(0, 0, 3, 3) = q_WS.toRotationMatrix();
    T_WS.block(0, 3, 3, 1) = r_WS;

    // T_WF
    const auto q_WF = tf_quat(T_WF_);
    const double yaw = quat2euler(q_WF)(2);
    const double pitch = parameters[1][1];
    const double roll = parameters[1][0];
    const vec3_t rpy{roll, pitch, yaw};

    const vec3_t r_WF(T_WF_.block(0, 3, 3, 1));
    const mat3_t C_WF = euler321(rpy);
    const mat4_t T_WF = tf(C_WF, r_WF);

    // T_BS
    Eigen::Map<const Eigen::Vector3d> r_BS(&parameters[2][0]);
    const Eigen::Quaterniond q_BS(parameters[2][6],
                                  parameters[2][3],
                                  parameters[2][4],
                                  parameters[2][5]);
    Eigen::Matrix4d T_BS;
    T_BS.setIdentity();
    T_BS.block(0, 0, 3, 3) = q_BS.toRotationMatrix();
    T_BS.block(0, 3, 3, 1) = r_BS;

    // T_BCi
    Eigen::Map<const Eigen::Vector3d> r_BCi(&parameters[3][0]);
    const Eigen::Quaterniond q_BCi(parameters[3][6],
                                   parameters[3][3],
                                   parameters[3][4],
                                   parameters[3][5]);
    Eigen::Matrix4d T_BCi;
    T_BCi.setIdentity();
    T_BCi.block(0, 0, 3, 3) = q_BCi.toRotationMatrix();
    T_BCi.block(0, 3, 3, 1) = r_BCi;

    // camera intrinsics + distortion
    Eigen::VectorXd cam_params = zeros(8, 1);
    cam_params <<
        // Intrinsics
        parameters[4][0], // fx
        parameters[4][1], // fy
        parameters[4][2], // cx
        parameters[4][3], // cy
        // Distortion
        parameters[4][4], // k1
        parameters[4][5], // k2
        parameters[4][6], // p1
        parameters[4][7]; // p2

    // // Form transform from camera to sensor
    // const Eigen::Matrix3d C_SC = q_SC.toRotationMatrix();
    // const Eigen::Matrix3d C_CS = C_SC.transpose();
    // Eigen::Matrix4d T_CS = Eigen::Matrix4d::Identity();
    // T_CS.topLeftCorner<3, 3>() = C_CS;
    // T_CS.topRightCorner<3, 1>() = -C_CS * t_SC_S;
    // // auto T_SC = T_CS.inverse();

    // Form transform from fiducial to world
    // const Eigen::Matrix3d C_WF = q_WF.toRotationMatrix();
    // const Eigen::Matrix3d C_FW = C_WF.transpose();
    // Eigen::Matrix4d T_FW = Eigen::Matrix4d::Identity();
    // T_FW.topLeftCorner<3, 3>() = C_FW;
    // T_FW.topRightCorner<3, 1>() = -C_FW * t_WF_W;

    // // Form transform from sensor to world
    // const Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();
    // const Eigen::Matrix3d C_SW = C_WS.transpose();
    // Eigen::Matrix4d T_SW = Eigen::Matrix4d::Identity();
    // T_SW.topLeftCorner<3, 3>() = C_SW;
    // T_SW.topRightCorner<3, 1>() = -C_SW * t_WS_W;
    // // auto T_WS = T_SW.inverse();

    // Misc
    // const Eigen::Matrix4d T_WF = T_FW.inverse();
    const Eigen::Matrix4d T_CS = T_BCi.inverse() * T_BS;
    const Eigen::Matrix4d T_SW = T_WS.inverse();
    const Eigen::Matrix4d T_CF = T_CS * T_SW * T_WF;

    // Homogenous point in camera and world frame
    const Eigen::Vector4d hp_C = T_CF * p_F_.homogeneous();
    const Eigen::Vector4d hp_S = T_CS.inverse() * hp_C;
    // const Eigen::Vector4d hp_W = T_WF * p_F_.homogeneous();

    // Calculate the reprojection error
    // measurement_t kp;
    Eigen::Matrix<double, 2, 3> Jh;
    Eigen::Matrix2Xd J_cam_params;
    Eigen::Matrix<double, 2, 3> Jh_weighted;

    // Form camera geometry
    // const double fx = cam_params(0);
    // const double fy = cam_params(1);
    // const double cx = cam_params(2);
    // const double cy = cam_params(3);
    //
    // const double k1 = cam_params(4);
    // const double k2 = cam_params(5);
    // const double p1 = cam_params(6);
    // const double p2 = cam_params(7);

    assert(image_width_ > 0);
    assert(image_height_ > 0);
    camera_geometry_t camera_geometry;
    camera_geometry.imageWidth_ = image_width_;
    camera_geometry.imageHeight_ = image_height_;

    CameraBase::ProjectionStatus status;
    if (jacobians != NULL) {
      status = camera_geometry.projectWithExternalParameters(hp_C.head(3),
                                                    cam_params,
                                                    &kp_projected_,
                                                    &Jh,
                                                    &J_cam_params);
      Jh_weighted = squareRootInformation_ * Jh;
    } else {
      status = camera_geometry.projectWithExternalParameters(hp_C.head(3),
                                                    cam_params,
                                                    &kp_projected_,
                                                    nullptr);
    }
    measurement_t error = measurement_ - kp_projected_;

    // weight:
    measurement_t weighted_error = squareRootInformation_ * error;
    // r = weighted_error;

    // assign:
    residuals[0] = weighted_error[0];
    residuals[1] = weighted_error[1];
    // residuals_debug[0] = weighted_error[0];
    // residuals_debug[1] = weighted_error[1];
    // error_debug[0] = error(0);
    // error_debug[1] = error(1);

    // check validity:
    bool valid = true;
    if (status != CameraBase::ProjectionStatus::Successful) {
      valid = false;
    } else if (fabs(hp_C[3]) > 1.0e-8) {
      Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
      if (p_C[2] < 0.0) { // 20 cm - not very generic... but reasonable
      // if (p_C[2] < 0.2) { // 20 cm - not very generic... but reasonable
        // std::cout<<"INVALID POINT"<<std::endl;
        valid = false;
      }
    }

    // switch (status) {
    //   case CameraBase::ProjectionStatus::Invalid: printf("point inavlid!\n"); break;
    //   case CameraBase::ProjectionStatus::OutsideImage:
    //     printf("point outside image!\n");
    //     print_vector("p_F", p_F_);
    //     print_vector("camera res", vec2_t{camera_geometry.imageWidth(), camera_geometry.imageHeight()});
    //     print_vector("projected", kp_projected_);
    //     break;
    //   case CameraBase::ProjectionStatus::Masked: printf("point masked!\n"); break;
    //   case CameraBase::ProjectionStatus::Behind: printf("point behind!\n"); break;
    //   case CameraBase::ProjectionStatus::Successful: break;
    // }

    if (valid == false) {
      // print_matrix("T_WF", T_WF);
      // print_matrix("T_SC", T_CS.inverse());
      // print_vector("cam_params", cam_params);
      // print_vector("measurement", measurement_);
      // print_vector("projected", kp_projected_);
      // print_vector("error", error);

      if (jacobians) {
        if (jacobians[0]) {
          Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J0(jacobians[0]);
          J0.setZero();
        }
        if (jacobians[1]) {
          Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J1(jacobians[1]);
          J1.setZero();
        }

        if (jacobians[2]) {
          Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J2(jacobians[2]);
          J2.setZero();
        }

        if (jacobians[3]) {
          Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J3(jacobians[3]);
          J3.setZero();
        }

        if (jacobians[4]) {
          Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> J4(jacobians[4]);
          J4.setZero();
        }
      }

      if (jacobiansMinimal) {
        if (jacobiansMinimal[0]) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0_min(jacobiansMinimal[0]);
          J0_min.setZero();
        }

        if (jacobiansMinimal[1]) {
          Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J1_min(jacobiansMinimal[1]);
          J1_min.setZero();
        }

        if (jacobiansMinimal[2]) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J2_min(jacobiansMinimal[2]);
          J2_min.setZero();
        }

        if (jacobiansMinimal[3]) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J3_min(jacobiansMinimal[3]);
          J3_min.setZero();
        }

        if (jacobiansMinimal[4]) {
          Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> J4_min(jacobiansMinimal[4]);
          J4_min.setZero();
        }
      }

      return true;
    }

    // calculate jacobians, if required
    // This is pretty close to Paul Furgale's thesis. eq. 3.100 on page 40
    if (jacobians != NULL) {
      Eigen::Matrix4d T_SC = T_CS.inverse();
      Eigen::Matrix<double, 3, 3> C_CS = tf_rot(T_CS);
      Eigen::Matrix<double, 3, 3> C_SC = tf_rot(T_SC);
      Eigen::Matrix<double, 3, 3> C_WS = tf_rot(T_WS);
      Eigen::Matrix<double, 3, 3> C_SW = C_WS.transpose();

      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. sensor pose
        // clang-format off
        Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
        Eigen::Matrix<double, 3, 3> dp_W__dr_WS = I(3);
        Eigen::Matrix<double, 3, 3> dp_W__dtheta = -kinematics::crossMx(C_WS * hp_S.head(3));
        Eigen::Matrix<double, 2, 3> dh_dr_WS = Jh_weighted * dp_C__dp_W * dp_W__dr_WS;
        Eigen::Matrix<double, 2, 3> dh_dtheta = Jh_weighted * dp_C__dp_W * dp_W__dtheta;
        // clang-format on

        // Compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
        J0_minimal.block(0, 0, 2, 3) = dh_dr_WS;
        J0_minimal.block(0, 3, 2, 3) = dh_dtheta;
        // J[0] = J0_minimal;
        if (!valid) {
          J0_minimal.setZero();
        }

        // Pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // Hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = J0_minimal * J_lift;

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[0] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
                J0_minimal_mapped(jacobiansMinimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      if (jacobians[1] != NULL) {
        // Form jacobians
        const double cphi = cos(roll);
        const double sphi = sin(roll);
        const double ctheta = cos(pitch);
        const double stheta = sin(pitch);
        const double cpsi = cos(yaw);
        const double spsi = sin(yaw);

        const double x = p_F_(0);
        const double y = p_F_(1);
        const double z = p_F_(2);

        const vec3_t J_x{
          y * (sphi * spsi + stheta * cphi * cpsi) + z * (-sphi * stheta * cpsi + spsi * cphi),
          y * (-sphi * cpsi + spsi * stheta * cphi) + z * (-sphi * spsi * stheta - cphi * cpsi),
          y * cphi * ctheta - z * sphi * ctheta
        };
        const vec3_t J_y{
          -x * stheta * cpsi + y * sphi * cpsi * ctheta + z * cphi * cpsi * ctheta,
          -x * spsi * stheta + y * sphi * spsi * ctheta + z * spsi * cphi * ctheta,
          -x * ctheta - y * sphi * stheta - z * stheta * cphi
        };
        const vec3_t J_z{
          -x * spsi * ctheta + y * (-sphi * spsi * stheta - cphi * cpsi) + z * (sphi * cpsi - spsi * stheta * cphi),
          x * cpsi * ctheta + y * (sphi * stheta * cpsi - spsi * cphi) + z * (sphi * spsi + stheta * cphi * cpsi),
          0
        };

        // Fill Jacobian
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J1(jacobians[1]);
        J1.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CS * C_SW * J_x;
        J1.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CS * C_SW * J_y;

        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
                J1_minimal_mapped(jacobiansMinimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }

      if (jacobians[2] != NULL) {
        // Jacobian w.r.t. T_BS
        const mat4_t T_CiB = T_BCi.inverse();
        const mat3_t C_CiB = tf_rot(T_CiB);
        const mat3_t C_BS = tf_rot(T_BS);
        const vec3_t r_SFi = tf_point(T_SW * T_WF, p_F_);

        // Compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_min;
        J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
        J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);
        if (!valid) {
          J_min.setZero();
        }

        // Pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        // Hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J(jacobians[2]);
        J = J_min * J_lift;

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[2] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
                J_min_mapped(jacobiansMinimal[2]);
            J_min_mapped = J_min;
          }
        }
      }

      if (jacobians[3] != NULL) {
        // Jacobian w.r.t. T_BCi
        // clang-format off
        Eigen::Matrix<double, 3, 3> dp_C__dp_S = C_SC.transpose();
        Eigen::Matrix<double, 3, 3> dp_S__dr_SC = I(3);
        Eigen::Matrix<double, 3, 3> dp_S__dtheta = -kinematics::crossMx(C_SC * hp_C.head(3));
        Eigen::Matrix<double, 2, 3> dh__dr_SC = Jh_weighted * dp_C__dp_S * dp_S__dr_SC;
        Eigen::Matrix<double, 2, 3> dh__dtheta = Jh_weighted * dp_C__dp_S * dp_S__dtheta;
        // clang-format on

        // Compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_min;
        J_min.block(0, 0, 2, 3) = dh__dr_SC;
        J_min.block(0, 3, 2, 3) = dh__dtheta;
        if (!valid) {
          J_min.setZero();
        }

        // Pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[3], J_lift.data());

        // Hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J(jacobians[3]);
        J = J_min * J_lift;

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
                J_min_mapped(jacobiansMinimal[3]);
            J_min_mapped = J_min;
          }
        }
      }

      if (jacobians[4] != NULL) {
        // Jacobian w.r.t. camera intrinsics + distortion params
        Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> J_min(jacobians[4]);
        J_min = -1 * squareRootInformation_ * J_cam_params;
        // J[4] = J_min;

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[4] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>>
                J_min_mapped(jacobiansMinimal[4]);
            J_min_mapped = J_min;
          }
        }
      }
    }

    return true;
  }

  ///< Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  ///< Number of parameter blocks.
  size_t parameterBlocks() const { return parameter_block_sizes().size(); }

  /**
   * Dimension of an individual parameter block.
   *
   * @param[in] parameterBlockId ID of the parameter block of interest.
   * @return The dimension.
   */
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /**
   * Residual block type as string
   */
  virtual std::string typeInfo() const { return "CalibReprojError"; }

protected:
  //< Measurement
  measurement_t measurement_;

  ///< Camera model:
  std::shared_ptr<const camera_geometry_t> cameraGeometry_;

  // Weighting related
  covariance_t information_;           ///< 2x2 information matrix
  covariance_t squareRootInformation_; ///< 2x2 square root information matrix
  covariance_t covariance_;            ///< 2x2 covariance matrix
};

} // namespace autocal
#endif /* AUTOCAL_CERES_CALIB_REPROJ_ERROR_HPP */
