#ifndef YAC_CALIB_REPROJ_ERROR_HPP
#define YAC_CALIB_REPROJ_ERROR_HPP

#include <vector>
#include <memory>
#include <ceres/ceres.h>

#include "yac.hpp"
#include "../common/Time.hpp"
#include "../common/operators.hpp"
#include "../common/Transformation.hpp"
#include "../common/Measurements.hpp"
#include "../cv/CameraBase.hpp"
#include "../param/PoseLocalParameterization.hpp"
#include "ErrorInterface.hpp"
#include "ReprojectionErrorBase.hpp"

namespace yac {

template <class GEOMETRY_TYPE>
class CalibReprojError : public CalibReprojErrorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

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
  typedef ::ceres::SizedCostFunction<2, 7, 2, 7, 7, 8> base_t;
  // typedef ::ceres::SizedCostFunction<2, 7, 2, 7, 7, 8, 1> base_t;

  ///< Number of residuals (2)
  static const int kNumResiduals = 2;

  ///< The keypoint type (measurement type).
  typedef vec2_t keypoint_t;

  ///< Projected point
  mutable measurement_t kp_projected_;

  ///< Pixel velocity
  vec2_t z_i_{0.0, 0.0};
  vec2_t z_j_{0.0, 0.0};
  vec2_t v_ij_{0.0, 0.0};  // pixel velocity: (z_j - z_i) / dt

  CalibReprojError() : cameraGeometry_(new camera_geometry_t) {}

  CalibReprojError(const uint64_t cameraId,
                   const int tag_id,
                   const int corner_id,
                   const vec3_t &p_F,
                   const mat4_t &T_WF,
                   const vec2_t &z_i,
                   const vec2_t &z_j,
                   const timestamp_t &ts_i,
                   const timestamp_t &ts_j,
                   const covariance_t &information) {
    setCameraId(cameraId);
    tag_id_ = tag_id;
    corner_id_ = corner_id;
    p_F_ = p_F;
    T_WF_ = T_WF;

    z_i_ = z_i;
    z_j_ = z_j;
    v_ij_ = (z_j_ - z_i_) / ns2sec(ts_j - ts_i);

    setMeasurement(z_i_);
    setInformation(information);
  }

  virtual ~CalibReprojError() {}

  virtual void setMeasurement(const measurement_t &measurement) {
    measurement_ = measurement;
  }

  virtual void setInformation(const covariance_t &information) {
    information_ = information;
    covariance_ = information.inverse();
    Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();
  }

  virtual const measurement_t &measurement() const { return measurement_; }
  measurement_t measurement() { return measurement_; }

  virtual const covariance_t &information() const { return information_; }
  virtual covariance_t information() { return information_; }

  virtual const covariance_t &covariance() const { return covariance_; }
  virtual covariance_t covariance() { return covariance_; }

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
    return EvaluateWithMinimalJacobians(parameters,
                                        residuals,
                                        jacobians,
                                        NULL); // debug test only
  }

  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const {
    // Fiducial param
    const double roll = params[1][0];
    const double pitch = params[1][1];
    const double yaw = quat2euler(tf_quat(T_WF_))(2);
    const vec3_t rpy{roll, pitch, yaw};
    const mat3_t C_WF = euler321(rpy);
    const vec3_t r_WF = tf_trans(T_WF_);
    const mat4_t T_WF = tf(C_WF, r_WF);

    // Sensor pose, sensor-camera extrinsics, camera parameters
    const mat4_t T_WS = tf(params[0]);
    const mat4_t T_BS = tf(params[2]);
    const mat4_t T_BCi = tf(params[3]);
    Eigen::Map<const vecx_t> cam_params(params[4], 8);
    // const double td = params[5][0];

    // Transform fiducial point to camera frame
    const mat4_t T_CiB = T_BCi.inverse();
    const mat4_t T_SW = T_WS.inverse();
    const vec3_t r_CFi = tf_point(T_CiB * T_BS * T_SW * T_WF, p_F_);
    const vec4_t hp_C = r_CFi.homogeneous();

    // Calculate the reprojection error
    // measurement_t kp;
    Eigen::Matrix<double, 2, 3> Jh;
    Eigen::Matrix2Xd J_cam_params;
    Eigen::Matrix<double, 2, 3> Jh_weighted;

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
    // measurement_t error = (measurement_ + td * v_ij_) - kp_projected_;
    // printf("kp: %f %f\n", kp_projected_(0), kp_projected_(1));

    // weight:
    measurement_t weighted_error = squareRootInformation_ * error;
    residuals[0] = weighted_error[0];
    residuals[1] = weighted_error[1];

    // check validity:
    bool valid = true;
    if (status != CameraBase::ProjectionStatus::Successful) {
      valid = false;
    // } else if (fabs(hp_C[3]) > 1.0e-8) {
    //   Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
    //   if (p_C[2] < 0.0) {
    //     valid = false;
    //   }
      // printf("FALSE\n");
      // printf("error: %f %f\n", error(0), error(1));
    }

    // print_vector("err" , error);
    // printf("cam_idx: %d, valid: %d\n", cameraId_, valid);

    if (jacobians) {
      // Jacobians w.r.t T_WS
      if (jacobians[0]) {
        const mat3_t C_CiS = tf_rot(T_CiB * T_BS);
        const mat3_t C_WS = tf_rot(T_WS);
        const mat3_t C_SW = C_WS.transpose();
        const mat3_t C_CiW = C_CiS * C_SW;
        const mat4_t T_SW = T_WS.inverse();
        const vec3_t r_SFi = tf_point(T_SW * T_WF, p_F_);

        mat_t<2, 6, row_major_t> J_min;
        J_min.setZero();
        J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiW * I(3);
        J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiW * -skew(C_WS * r_SFi);
        if (valid == false) {
          J_min.setZero();
        }

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        OKVISPoseLocalParameterization::liftJacobian(params[0], J_lift.data());

        // Global jacobian
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J = J_min * J_lift;

        // Minimal jacobian
        if (jacobiansMinimal && jacobiansMinimal[0]) {
          Eigen::Map<mat_t<2, 6, row_major_t>> J_min_map(jacobiansMinimal[0]);
          J_min_map = J_min;
        }
      }

      // Jacobians w.r.t. T_WF
      if (jacobians[1]) {
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
        const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
        Eigen::Map<mat_t<2, 2, row_major_t>> J(jacobians[1]);
        J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
        J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
        if (valid == false) {
          J.setZero();
        }

        if (jacobiansMinimal && jacobiansMinimal[1]) {
          Eigen::Map<mat_t<2, 2, row_major_t>> J_min(jacobiansMinimal[1]);
          J_min = J;
        }
      }


      // Jacobians w.r.t T_BS
      if (jacobians[2]) {
        const mat3_t C_CiB = tf_rot(T_CiB);
        const mat3_t C_BS = tf_rot(T_BS);
        const vec3_t r_SFi = tf_point(T_SW * T_WF, p_F_);

        mat_t<2, 6, row_major_t> J_min;
        J_min.setZero();
        J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
        J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);
        if (valid == false) {
          J_min.setZero();
        }

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        OKVISPoseLocalParameterization::liftJacobian(params[2], J_lift.data());

        // Global jacobian
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
        J = J_min * J_lift;

        // Minimal jacobian
        if (jacobiansMinimal && jacobiansMinimal[2]) {
          Eigen::Map<mat_t<2, 6, row_major_t>> J_min_map(jacobiansMinimal[2]);
          J_min_map = J_min;
        }
      }

      // Jacobians w.r.t T_BCi
      if (jacobians[3]) {
        const mat3_t C_CiB = tf_rot(T_CiB);
        const mat3_t C_BCi = C_CiB.transpose();
        const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, p_F_);

        mat_t<2, 6, row_major_t> J_min;
        J_min.setZero();
        J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiB * I(3);
        J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiB * -skew(C_BCi * r_CiFi);
        if (valid == false) {
          J_min.setZero();
        }

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        OKVISPoseLocalParameterization::liftJacobian(params[3], J_lift.data());

        // Global jacobian
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[3]);
        J = J_min * J_lift;

        // Minimal jacobian
        if (jacobiansMinimal && jacobiansMinimal[3]) {
          Eigen::Map<mat_t<2, 6, row_major_t>> J_min_map(jacobiansMinimal[3]);
          J_min_map = J_min;
        }
      }

      // Jacobians w.r.t. camera parameters
      if (jacobians[4]) {
        const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};

        Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> J(jacobians[4]);
        J = -1 * squareRootInformation_ * J_cam_params;
        if (valid == false) {
          J.setZero();
        }

        if (jacobiansMinimal && jacobiansMinimal[4]) {
          Eigen::Map<mat_t<2, 8, row_major_t>> J_min_map(jacobiansMinimal[4]);
          J_min_map = J;
        }
      }

      // // Jacobians w.r.t. time delay
      // if (jacobians[5]) {
      //   Eigen::Map<vec2_t> J(jacobians[5]);
      //   J = squareRootInformation_ * v_ij_;
      //   if (valid == false) {
      //     J.setZero();
      //   }
      //
      //   if (jacobiansMinimal && jacobiansMinimal[5]) {
      //     Eigen::Map<vec2_t> J_min(jacobiansMinimal[5]);
      //     J_min = J;
      //   }
      // }
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

} // namespace yac
#endif /* YAC_CALIB_REPROJ_ERROR_HPP */
