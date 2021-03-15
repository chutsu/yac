#ifndef AUTOCAL_INITIALIZER_HPP
#define AUTOCAL_INITIALIZER_HPP

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <array>
#include <map>
#include <thread>
#include <future>
#include <functional>

#include <ceres/ceres.h>

#include "core.hpp"

// #include "autocal/common/Core.hpp"
#include "autocal/common/AprilGrid.hpp"
#include "autocal/common/CalibData.hpp"
#include "autocal/common/Measurements.hpp"
#include "autocal/common/Variables.hpp"
#include "autocal/common/Parameters.hpp"
#include "autocal/common/Timeline.hpp"
#include "autocal/common/Transformation.hpp"
#include "autocal/common/Calibrator.hpp"
#include "autocal/common/CeresIterationCallback.hpp"

#include "autocal/cv/CameraGeometry.hpp"
#include "autocal/cv/PinholeCamera.hpp"
#include "autocal/cv/RadialTangentialDistortion.hpp"

#include "autocal/param/CameraParameterBlock.hpp"
#include "autocal/param/PoseParameterBlock.hpp"
#include "autocal/param/SpeedAndBiasParameterBlock.hpp"
#include "autocal/param/TimeDelayParameterBlock.hpp"
#include "autocal/param/FiducialParameterBlock.hpp"
#include "autocal/error/ImuError.hpp"
#include "autocal/error/PoseError.hpp"
#include "autocal/error/FiducialError.hpp"
#include "autocal/error/CalibReprojError.hpp"
#include "autocal/error/ReprojectionError.hpp"

#include "autocal/Map.hpp"

namespace autocal {

template <class GEOMETRY_TYPE>
class ReprojErrorFiducialPose : public ReprojErrorFiducialPoseBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  Time timestamp_;
  size_t pose_id_ = 0;
  size_t tag_id_ = 0;
  size_t corner_id_ = 0;

  mat4_t T_WF_;
  vec3_t p_F_;

  typedef GEOMETRY_TYPE camera_geometry_t;
  typedef ::ceres::SizedCostFunction<2, 7, 7, 7, 7, 8> base_t;
  static const int kNumResiduals = 2;
  mutable vec2_t kp_projected_;

  ReprojErrorFiducialPose()
		: cameraGeometry_(new camera_geometry_t) {}

  ReprojErrorFiducialPose(const uint64_t cameraId,
                   const int tag_id,
                   const int corner_id,
                   const vec3_t &p_F,
                   const mat4_t &T_WF,
                   const vec2_t &measurement,
                   const mat2_t &information) {
    setCameraId(cameraId);
    tag_id_ = tag_id;
    corner_id_ = corner_id;
    p_F_ = p_F;
    T_WF_ = T_WF;

    setMeasurement(measurement);
    setInformation(information);
  }

  virtual ~ReprojErrorFiducialPose() {}

  virtual void setMeasurement(const vec2_t &measurement) {
    measurement_ = measurement;
  }

  virtual void setInformation(const mat2_t &information) {
    information_ = information;
    covariance_ = information.inverse();
    // perform the Cholesky decomposition on order to obtain the correct error
    // weighting
    Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();
  }

  virtual const vec2_t &measurement() const { return measurement_; }
  vec2_t measurement() { return measurement_; }
  virtual const mat2_t &information() const { return information_; }
  virtual mat2_t information() { return information_; }
  virtual const mat2_t &covariance() const { return covariance_; }
  virtual mat2_t covariance() { return covariance_; }

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
    return EvaluateWithMinimalJacobians(parameters,
                                        residuals,
                                        jacobians,
                                        NULL); // debug test only
  }

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
    const mat4_t T_WS= tf(q_WS, r_WS);

		// T_WF
    Eigen::Map<const Eigen::Vector3d> t_WF_W(&parameters[1][0]);
    const Eigen::Quaterniond q_WF(parameters[1][6],
                                  parameters[1][3],
                                  parameters[1][4],
                                  parameters[1][5]);
    const mat3_t C_WF = q_WF.toRotationMatrix();
    const mat4_t T_WF = tf(q_WF, t_WF_W);

    // T_BS
    Eigen::Map<const Eigen::Vector3d> r_BS(&parameters[2][0]);
    const Eigen::Quaterniond q_BS(parameters[2][6],
                                  parameters[2][3],
                                  parameters[2][4],
                                  parameters[2][5]);
    const mat4_t T_BS = tf(q_BS, r_BS);

    // T_BCi
    Eigen::Map<const Eigen::Vector3d> r_BCi(&parameters[2][0]);
    const Eigen::Quaterniond q_BCi(parameters[2][6],
                                  parameters[2][3],
                                  parameters[2][4],
                                  parameters[2][5]);
    const mat4_t T_BCi = tf(q_BCi, r_BCi);


    // camera intrinsics + distortion
    Eigen::VectorXd cam_params = zeros(8, 1);
    cam_params <<
        // Intrinsics
        parameters[3][0], // fx
        parameters[3][1], // fy
        parameters[3][2], // cx
        parameters[3][3], // cy
        // Distortion
        parameters[3][4], // k1
        parameters[3][5], // k2
        parameters[3][6], // p1
        parameters[3][7]; // p2

		const Eigen::Matrix4d T_SW = T_WS.inverse();
		const Eigen::Matrix4d T_SCi = T_BS.inverse() * T_BCi;
		const Eigen::Matrix4d T_CiS = T_SCi.inverse();
    const Eigen::Matrix4d T_CiF = T_CiS * T_SW * T_WF;
    const Eigen::Vector4d hp_C = T_CiF * p_F_.homogeneous();
    const Eigen::Vector4d hp_S = T_CiS.inverse() * hp_C;

    // Calculate the reprojection error
    Eigen::Matrix<double, 2, 3> Jh;
    Eigen::Matrix2Xd J_cam_params;
    Eigen::Matrix<double, 2, 3> Jh_weighted;
    camera_geometry_t camera_geometry;

    // CameraBase::ProjectionStatus status;
    if (jacobians != NULL) {
      camera_geometry.projectWithExternalParameters(hp_C.head(3),
                                                    cam_params,
                                                    &kp_projected_,
                                                    &Jh,
                                                    &J_cam_params);
      Jh_weighted = squareRootInformation_ * Jh;
    } else {
      camera_geometry.projectWithExternalParameters(hp_C.head(3),
                                                    cam_params,
                                                    &kp_projected_,
                                                    nullptr);
    }
    const vec2_t error = measurement_ - kp_projected_;
    const vec2_t weighted_error = squareRootInformation_ * error;
    residuals[0] = weighted_error[0];
    residuals[1] = weighted_error[1];

    // check validity:
    bool valid = true;
    if (fabs(hp_C[3]) > 1.0e-8) {
      Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
      if (p_C[2] < 0.0) { // 20 cm - not very generic... but reasonable
      // if (p_C[2] < 0.2) { // 20 cm - not very generic... but reasonable
        // std::cout<<"INVALID POINT"<<std::endl;
        valid = false;
      }
    }

    // calculate jacobians, if required
    // This is pretty close to Paul Furgale's thesis. eq. 3.100 on page 40
    if (jacobians != NULL) {
      Eigen::Matrix4d T_SCi = T_CiS.inverse();
      // Eigen::Matrix<double, 3, 3> C_CiS = tf_rot(T_CiS);
      Eigen::Matrix<double, 3, 3> C_SCi = tf_rot(T_SCi);
      Eigen::Matrix<double, 3, 3> C_WS = tf_rot(T_WS);
      // Eigen::Matrix<double, 3, 3> C_SW = C_WS.transpose();

      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. T_WS
        // clang-format off
        Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SCi.transpose() * C_WS.transpose();
        Eigen::Matrix<double, 3, 3> dp_W__dr_WS = I(3);
        Eigen::Matrix<double, 3, 3> dp_W__dtheta = -autocal::kinematics::crossMx(C_WS * hp_S.head(3));
        Eigen::Matrix<double, 2, 3> dh_dr_WS = Jh_weighted * dp_C__dp_W * dp_W__dr_WS;
        Eigen::Matrix<double, 2, 3> dh_dtheta = Jh_weighted * dp_C__dp_W * dp_W__dtheta;
        // clang-format on

        // Compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
        J0_minimal.block(0, 0, 2, 3) = dh_dr_WS;
        J0_minimal.block(0, 3, 2, 3) = dh_dtheta;
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
        // Jacobian w.r.t. T_WF
        Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SCi.transpose() * C_WS.transpose();
        Eigen::Matrix<double, 3, 3> dp_W__dr_WF = I(3);
        Eigen::Matrix<double, 3, 3> dp_W__dtheta = -autocal::kinematics::crossMx(C_WF * p_F_);
        Eigen::Matrix<double, 2, 3> dh__dr_WF = -1 * Jh_weighted * dp_C__dp_W * dp_W__dr_WF;
        Eigen::Matrix<double, 2, 3> dh__dtheta = -1 * Jh_weighted * dp_C__dp_W * dp_W__dtheta;
        // Eigen::Matrix<double, 2, 3> dh__dr_WF = Jh_weighted * dp_C__dp_W *
        // dp_W__dr_WF; Eigen::Matrix<double, 2, 3> dh__dtheta = Jh_weighted *
        // dp_C__dp_W * dp_W__dtheta;

        // Compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J1_minimal;
        J1_minimal.block(0, 0, 2, 3) = dh__dr_WF;
        J1_minimal.block(0, 3, 2, 3) = dh__dtheta;
        if (!valid) {
          J1_minimal.setZero();
        }

        // Pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[1], J_lift.data());

        // Hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = J1_minimal * J_lift;

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>
                J1_minimal_mapped(jacobiansMinimal[1]);
            J1_minimal_mapped = J1_minimal;
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
        Eigen::Matrix<double, 3, 3> dp_C__dp_S = C_SCi.transpose();
        Eigen::Matrix<double, 3, 3> dp_S__dr_SC = I(3);
        Eigen::Matrix<double, 3, 3> dp_S__dtheta = -kinematics::crossMx(C_SCi * hp_C.head(3));
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

  size_t residualDim() const { return kNumResiduals; }

  size_t parameterBlocks() const {
		return parameter_block_sizes().size();
	}

  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  virtual std::string typeInfo() const { return "ReprojErrorFiducialPose"; }

protected:
  vec2_t measurement_;
  std::shared_ptr<const camera_geometry_t> cameraGeometry_;
  mat2_t information_;
  mat2_t squareRootInformation_;
  mat2_t covariance_;
};

template <typename CameraGeometry>
class VisualInertialInitializer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Counters
  size_t new_id_ = 0;
  size_t frame_index_ = 0;

  // Flags
  bool prior_set_ = false;
  bool added_target_params_ = false;
  bool added_cam_params_ = false;
  bool added_imu_params_ = false;
  bool added_imu_exts_ = false;
  bool added_imu_timedelay_= false;
  bool added_fiducial_pose_ = false;
  bool added_cam_exts_ = false;

  std::map<int, aprilgrid_t> prev_grids_;
  std::map<int, aprilgrid_t> grids_;
  ImuMeasurementDeque imu_data_;
  bool grids_init_ = false;
  bool sensor_init_ = false;
  bool fiducial_init_ = false;

  // AprilGrid
  aprilgrid_detector_t *detector_;
  calib_target_t target_params_;

  // Parameters
  ImuParameters imu_params_;
  std::map<int, PinholeRadtan> cameras_;
  std::shared_ptr<Map> problem_;

  // Parameter ids
  Time t0_{0};
  uint64_t pose0_id_ = 0;
  uint64_t sb0_id_ = 0;
  uint64_t td_id_ = 0;
  uint64_t fiducial_pose_id_ = 0;
  uint64_t imu_exts_id_ = 0;
  std::map<uint64_t, uint64_t> cam_param_ids_;
  std::map<uint64_t, uint64_t> cam_exts_ids_;

  // Parameter blocks
  std::shared_ptr<TimeDelayParameterBlock> time_delay_block_;
  mat4_t T_WF_init_;
  std::shared_ptr<FiducialParameterBlock> T_WF_block_;
  // std::shared_ptr<PoseParameterBlock> T_WF_block_;
  std::shared_ptr<PoseParameterBlock> T_BS_block_;
  std::map<int, std::shared_ptr<CameraParameterBlock>> cam_blocks_;
  std::map<int, std::shared_ptr<PoseParameterBlock>> T_BC_blocks_;
  std::vector<std::shared_ptr<PoseParameterBlock>> T_WS_blocks_;
  std::vector<std::shared_ptr<SpeedAndBiasParameterBlock>> sb_blocks_;

  // Residual blocks
  std::deque<std::pair<::ceres::ResidualBlockId, std::shared_ptr<CalibReprojError<PinholeRadtan>>>> vision_errors_;

	VisualInertialInitializer();
	virtual ~VisualInertialInitializer();

  void configure(const std::string &config_file);
  int addCalibTarget(const calib_target_t &target_params);
  int addImu(const ImuParameters &params);
  uint64_t addImuTimeDelay(const double time_delay);
  uint64_t addFiducialPose(const mat4_t &T_WF);
	uint64_t addImuExtrinsics(const mat4_t &T_BS);
  uint64_t addCamera(const size_t cam_id, const CameraGeometry &cam);
  uint64_t addCameraExtrinsics(const size_t cam_id, const mat4_t &T_SC);
  uint64_t addSensorPoseParameter(const Time &ts, const mat4_t &T_WS);
  uint64_t addSpeedBiasParameter(const Time &ts, const autocal::SpeedAndBias &sb);

  mat4_t getImuExtrinsicsEstimate();
  vecx_t getCameraParameterEstimate(const size_t cam_index);
  mat4_t getCameraExtrinsicsEstimate(const size_t cam_index);
  mat4_t getFiducialPoseEstimate();
  double getTimeDelayEstimate();
  mat4_t getSensorPoseEstimate(const size_t pose_id);
  vec_t<9> getSpeedBiasEstimate(const size_t sb_id);

  size_t nb_cams();

  void propagate(const Time &ts,
                 const ImuMeasurementDeque &imu_data,
                 Transformation &T_WS,
                 SpeedAndBias &sb);
  bool addSensorPosePrior(const Time timestamp,
                          const mat4_t &T_WS,
                          const vec3_t &v_WS);
  void addImuError(const ImuMeasurementDeque & imu_data,
                   const Time &t0,
                   const Time &t1,
                   const size_t pose0_id,
                   const size_t pose1_id,
                   const size_t sb0_id,
                   const size_t sb1_id,
                   const size_t timedelay_id);
  void addReprojErrors(const std::map<int, aprilgrid_t> &cam_grids,
                            const size_t pose_id);
  void addMeasurement(const int cam_idx, const aprilgrid_t &grid);
  void addMeasurement(const timestamp_t &ts, const std::vector<cv::Mat> &cam_images);
  void addMeasurement(const Time &ts, const vec3_t &w_m, const vec3_t &a_m);
  bool addState(const Time &ts,
                const ImuMeasurementDeque & imuMeasurements,
                const std::map<int, aprilgrid_t> &cam_grids,
                const mat4_t &T_WS,
                const vec3_t &v_WS);
  void optimize(size_t numIter, size_t numThreads=1, bool verbose=false);
  std::vector<double> getReprojectionErrors(const int cam_idx);
  double rmseReprojectionError(const int cam_idx);
  double meanReprojectionError(const int cam_idx);
};

template <typename T>
VisualInertialInitializer<T>::VisualInertialInitializer()
    : problem_(new Map()) {}

template <typename T>
VisualInertialInitializer<T>::~VisualInertialInitializer() {
  delete detector_;
}

template <typename T>
void VisualInertialInitializer<T>::configure(const std::string &config_file) {
  // Load config
  config_t config{config_file};
  // -- Calibration data path
  std::string data_path;
  parse(config, "settings.data_path", data_path);
  // -- Create results file
  auto results_path = data_path + "/calib_results.yaml";
  std::ofstream results(results_path);
  if (results.is_open() == false) {
    FATAL("Failed to create file at [%s]!", results_path.c_str());
  }
  // -- Load calibration target settings
  load_calib_target_params(config, target_params_);
  added_target_params_ = true;
  // -- Load optimization settings
  OptSettings opt;
  load_optimization_settings(config, opt);
  // -- Load camera params.
  int nb_cams = config_nb_cameras(config_file);
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    CameraParameters params;
    load_camera_params(config, cam_idx, params);

    const int img_w = params.resolution(0);
    const int img_h = params.resolution(1);

    const auto fx = params.intrinsics(0);
    const auto fy = params.intrinsics(1);
    const auto cx = params.intrinsics(2);
    const auto cy = params.intrinsics(3);

    const auto k1 = params.intrinsics(4);
    const auto k2 = params.intrinsics(5);
    const auto p1 = params.intrinsics(6);
    const auto p2 = params.intrinsics(7);

    T camera{img_w, img_h, fx, fy, cx, cy, {k1, k2, p1, p2}};
    cameras_[cam_idx] = camera;
  }
  // -- Load imu params.
  autocal::ImuParameters imu_params;
  int nb_imus = config_nb_imus(config_file);
  if (nb_imus == 1) {
    load_imu_params(config, 0, imu_params);
  } else if (nb_imus > 1) {
    FATAL("At current the calibrator only supports 1 imu!");
  }
  // -- Load extrinsic params
  std::vector<mat4_t> camera_extrinsics;
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    mat4_t T_BCi;
    load_camera_extrinsics(config, cam_idx, T_BCi);
    camera_extrinsics.push_back(T_BCi);
  }

  // Setup Estimator
  // -- Add IMU
  addImu(imu_params);
  addImuTimeDelay(0.0);
  // -- Add Camera
  addCamera(0, cameras_[0]);
  addCamera(1, cameras_[1]);
  // -- Add sensor camera extrinsics
  addCameraExtrinsics(0, camera_extrinsics[0]);
  addCameraExtrinsics(1, camera_extrinsics[1]);
}

template <typename T>
int VisualInertialInitializer<T>::addCalibTarget(const calib_target_t &target_params) {
  target_params_ = target_params;
  detector_ = new aprilgrid_detector_t(target_params_.tag_rows,
                                       target_params_.tag_cols,
                                       target_params_.tag_size,
                                       target_params_.tag_spacing);
  added_target_params_ = true;
  return 0;
}

template <typename T>
int VisualInertialInitializer<T>::addImu(const ImuParameters &params) {
  imu_params_ = params;
  added_imu_params_ = true;
  return 0;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addImuTimeDelay(const double time_delay) {
  uint64_t id = new_id_++;
  std::shared_ptr<TimeDelayParameterBlock> block(new TimeDelayParameterBlock(time_delay, id));
  problem_->addParameterBlock(block, Map::Trivial);
  problem_->setParameterBlockConstant(block);

  td_id_ = id;
  time_delay_block_ = block;
  added_imu_timedelay_ = true;
  return id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addFiducialPose(const mat4_t &T_WF) {
  uint64_t id = new_id_++;

  T_WF_init_ = T_WF;
	Transformation T_WF_{T_WF};

  std::shared_ptr<FiducialParameterBlock> block(new FiducialParameterBlock(T_WF_, id));
  problem_->addParameterBlock(block, Map::Trivial);

  // std::shared_ptr<PoseParameterBlock> block(new PoseParameterBlock(T_WF_, id));
  // problem_->addParameterBlock(block, Map::Pose6d);

  // Add pose error as a prior on fiducial pose
  // Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
  // information(0, 0) = 1.0e2;
  // information(1, 1) = 1.0e2;
  // information(2, 2) = 1.0e2;
  // information(3, 3) = 1.0e2;
  // information(4, 4) = 1.0e2;
  // information(5, 5) = 1.0e2;
  // information(0, 0) = 1.0e8;
  // information(1, 1) = 1.0e8;
  // information(2, 2) = 1.0e8;
  // information(3, 3) = 1.0e8;
  // information(4, 4) = 1.0e8;
  // information(5, 5) = 1.0e8;
  // std::shared_ptr<PoseError > pose(new PoseError(T_WF, information));
  // problem_->addResidualBlock(pose, NULL, block);

  // Fix fiducial pose
  // problem_->setParameterBlockConstant(id);

  // const auto q_WF = tf_quat(T_WF_init_);
  // const vec2_t m = quat2euler(q_WF).head(2);
  // std::shared_ptr<FiducialError > error(new FiducialError(m, 1.0e2 * I(2)));
  // error->param_ids.push_back(id);
  // problem_->addResidualBlock(error, NULL, block);

  fiducial_pose_id_ = id;
  T_WF_block_ = block;
  added_fiducial_pose_ = true;
  return id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addCamera(const size_t cam_idx, const T &cam) {
  const uint64_t id = new_id_++;

  vecx_t params;
  cam.getIntrinsics(params);
  cameras_[cam_idx] = cam;
  std::shared_ptr<CameraParameterBlock> block(new CameraParameterBlock(params, id));
  problem_->addParameterBlock(block, Map::Trivial);
  problem_->setParameterBlockConstant(block);

  cam_param_ids_[cam_idx] = id;
  cam_blocks_[cam_idx] = block;
  added_cam_params_ = true;

  return id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addCameraExtrinsics(const size_t cam_id, const mat4_t &T_BC) {
  uint64_t id = new_id_++;

  Transformation T_BC_{T_BC};
  std::shared_ptr<PoseParameterBlock> block(new PoseParameterBlock(T_BC_, id));
  problem_->addParameterBlock(block, Map::Pose6d);
	problem_->setParameterBlockConstant(block);

  cam_exts_ids_[cam_id] = id;
  T_BC_blocks_[cam_id] = block;

  added_cam_exts_ = true;
  return id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addImuExtrinsics(const mat4_t &T_BS) {
  uint64_t id = new_id_++;

  Transformation T_BS_{T_BS};
  std::shared_ptr<PoseParameterBlock> block(new PoseParameterBlock(T_BS_, id));
  problem_->addParameterBlock(block, Map::Pose6d);
	problem_->setParameterBlockConstant(block);

  imu_exts_id_ = id;
  T_BS_block_ = block;

  added_imu_exts_ = true;
  return id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addSensorPoseParameter(const Time &ts, const mat4_t &T_WS) {
  const uint64_t pose_id = new_id_++;
  const Transformation T_WS_{T_WS};
  const auto block = std::make_shared<PoseParameterBlock>(T_WS_, pose_id, ts);
  T_WS_blocks_.push_back(block);
  problem_->addParameterBlock(block, Map::Pose6d);
  return pose_id;
}

template <typename T>
uint64_t VisualInertialInitializer<T>::addSpeedBiasParameter(const Time &ts, const autocal::SpeedAndBias &sb) {
  const uint64_t id = new_id_++;
  const auto block = std::make_shared<SpeedAndBiasParameterBlock>(sb, id, ts);
  sb_blocks_.push_back(block);
  problem_->addParameterBlock(block, Map::Trivial);
  return id;
}

template <typename T>
mat4_t VisualInertialInitializer<T>::getImuExtrinsicsEstimate() {
  auto id = imu_exts_id_;
  auto param_block = problem_->parameterBlockPtr(id);
  auto param = static_cast<PoseParameterBlock *>(param_block.get());
  return param->estimate().T();
}

template <typename T>
vecx_t VisualInertialInitializer<T>::getCameraParameterEstimate(const size_t cam_index) {
  auto id = cam_param_ids_.at(cam_index);
  auto param_block = problem_->parameterBlockPtr(id);
  auto param = static_cast<CameraParameterBlock *>( param_block.get());
  return param->estimate();
}

template <typename T>
mat4_t VisualInertialInitializer<T>::getCameraExtrinsicsEstimate(const size_t cam_index) {
  auto id = cam_exts_ids_.at(cam_index);
  auto param_block = problem_->parameterBlockPtr(id);
  auto param = static_cast<PoseParameterBlock *>(param_block.get());
  return param->estimate().T();
}

template <typename T>
mat4_t VisualInertialInitializer<T>::getFiducialPoseEstimate() {
  vec2_t xy = T_WF_block_->estimate();
  const auto r_WF = tf_trans(T_WF_init_);
  const auto euler_WF = quat2euler(tf_quat(T_WF_init_));

  const double roll = xy(0);
  const double pitch = xy(1);
  const double yaw = euler_WF(2);
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);

  return tf(C_WF, r_WF);;

	// return T_WF_block_->estimate().T();
}

template <typename T>
double VisualInertialInitializer<T>::getTimeDelayEstimate() {
  auto data = problem_->parameterBlockPtr(td_id_)->parameters();
  return data[0];
}

template <typename T>
mat4_t VisualInertialInitializer<T>::getSensorPoseEstimate(const size_t pose_id) {
  const auto T_WS_block = problem_->parameterBlockPtr(pose_id);
  const auto T_WS_pose = static_cast<PoseParameterBlock *>(T_WS_block.get());
  return T_WS_pose->estimate().T();
}

template <typename T>
vec_t<9> VisualInertialInitializer<T>::getSpeedBiasEstimate(const size_t sb_id) {
  const auto sb_block = problem_->parameterBlockPtr(sb_id);
  const auto sb_param = static_cast<SpeedAndBiasParameterBlock *>(sb_block.get());
  return sb_param->estimate();
}

template <typename T>
size_t VisualInertialInitializer<T>::nb_cams() {
  return cam_param_ids_.size();
}

template <typename T>
void VisualInertialInitializer<T>::propagate(const Time &ts,
                                  const ImuMeasurementDeque &imu_data,
                                  Transformation &T_WS,
                                  SpeedAndBias &sb) {
  // Propagate pose and speedAndBias
  const auto t0 = t0_;
  const auto t1 = ts;
  int nb_meas = ImuError::propagation(imu_data, imu_params_, T_WS, sb, t0, t1);

  // Make sure used more than 1 imu measurement to propagate the sensor pose
  AUTOCAL_ASSERT_TRUE_DBG(Exception, nb_meas > 1, "propagation failed");
  if (nb_meas < 1){
    LOG_ERROR("nb_meas [%d]", nb_meas);
    FATAL("Failed to propagate imu measurements!");
  }
}

template <typename T>
bool VisualInertialInitializer<T>::addSensorPosePrior(const Time ts,
                                           	 	 	 	 	 	const mat4_t &T_WS,
                                           	 	 	 	 	 	const vec3_t &v_WS) {
  // Set time t0 (later for ImuError term)
  t0_ = ts;

  // Add a pose parameter block
  const uint64_t pose_id = addSensorPoseParameter(ts, T_WS);
  pose0_id_ = pose_id;

  // Add speed and bias parameter block
  autocal::SpeedAndBias sb;
  sb.setZero();
  sb.segment(0, 3) = v_WS;
  // sb.segment(6, 3) = imu_params_.a0;
  const uint64_t sb_id = addSpeedBiasParameter(ts, sb);
  sb0_id_ = sb_id;

  // // Add a pose residual block
  // Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Identity();
  // information(0, 0) = 1.0e8;
  // information(1, 1) = 1.0e8;
  // information(2, 2) = 1.0e8;
  // information(3, 3) = 1.0e8;
  // information(4, 4) = 1.0e8;
  // information(5, 5) = 1.0e8;
	// const Transformation T_WS_{T_WS};
  // const auto pose_error = std::make_shared<PoseError>(T_WS_, information);
  // const auto pose_param = problem_->parameterBlockPtr(pose0_id_);
  // problem_->addResidualBlock(pose_error, NULL, pose_param);

  // Mark prior as set
  prior_set_ = true;

  return true;
}

template <typename T>
void VisualInertialInitializer<T>::addImuError(const ImuMeasurementDeque & imu_data,
                                    const Time &t0,
                                    const Time &t1,
                                    const size_t pose0_id,
                                    const size_t pose1_id,
                                    const size_t sb0_id,
                                    const size_t sb1_id,
                                    const size_t timedelay_id) {
  // Add IMU error term
  // clang-format off
  auto imu_error = std::make_shared<ImuError>(imu_data, imu_params_, t0, t1);
#if EST_TIMEDELAY
  problem_->addResidualBlock(
    imu_error,
    nullptr,
    problem_->parameterBlockPtr(pose0_id),
    problem_->parameterBlockPtr(sb0_id),
    problem_->parameterBlockPtr(pose1_id),
    problem_->parameterBlockPtr(sb1_id),
    problem_->parameterBlockPtr(timedelay_id)
  );
#else
  problem_->addResidualBlock(
    imu_error,
    nullptr,
    problem_->parameterBlockPtr(pose0_id),
    problem_->parameterBlockPtr(sb0_id),
    problem_->parameterBlockPtr(pose1_id),
    problem_->parameterBlockPtr(sb1_id)
  );
#endif
  // clang-format on
}

template <typename T>
void VisualInertialInitializer<T>::addReprojErrors(
    const std::map<int, aprilgrid_t> &cam_grids,
    const size_t pose_id) {
  for (size_t cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    // Loop over every tag seen
    const auto &grid = cam_grids.at(cam_idx);

    // Add reprojection errors for batch problem
    for (const auto tag_id : grid.ids) {
      vec3s_t object_points;
      aprilgrid_object_points(grid, tag_id, object_points);

      vec2s_t keypoints;
      aprilgrid_get(grid, tag_id, keypoints);

      for (size_t j = 0; j < 4; j++) {
        // Create and add IntrinsicsExtrinsicsError block
        const int corner_id = j;
        auto error = std::make_shared<CalibReprojError<T>>(
            cam_idx,
            tag_id,
            corner_id,
            object_points[j],
            T_WF_init_,
            keypoints[j],
            I(2)
        );
        error->image_width_ = cameras_[cam_idx].imageWidth();
        error->image_height_ = cameras_[cam_idx].imageHeight();
        error->timestamp_ = Time{ns2sec(grid.timestamp)};
        error->pose_id_ = pose_id;
        error->tag_id_ = tag_id;
        error->corner_id_ = corner_id;

        ::ceres::ResidualBlockId res_id = problem_->addResidualBlock(
          error,
          NULL,
          problem_->parameterBlockPtr(pose_id),
          problem_->parameterBlockPtr(fiducial_pose_id_),
          problem_->parameterBlockPtr(cam_exts_ids_[cam_idx]),
          problem_->parameterBlockPtr(cam_param_ids_[cam_idx])
        );
        vision_errors_.push_back({res_id, error});
      }
    }
  }
}

template <typename T>
void VisualInertialInitializer<T>::addMeasurement(const int cam_idx, const aprilgrid_t &grid) {
  grids_[cam_idx] = grid;
}

template <typename T>
void VisualInertialInitializer<T>::addMeasurement(const timestamp_t &ts,
                                       	 	 	 	 	  const std::vector<cv::Mat> &cam_images) {
  assert(added_target_params_ == true);

  for (int i = 0; i < (int) nb_cams(); i++) {
    aprilgrid_t grid(ts,
                     target_params_.tag_rows,
                     target_params_.tag_cols,
                     target_params_.tag_size,
                     target_params_.tag_spacing);
    const vecx_t cam_params = getCameraParameterEstimate(i);
    const mat3_t K = pinhole_K(cam_params.head(4));
    const vecx_t D = cam_params.tail(4);
    aprilgrid_detect(grid, *detector_, cam_images[i], K, D);
    grid.timestamp = ts;
    grids_[i] = grid;
  }

  if (grids_[0].detected == false || grids_[1].detected == false) {
    grids_.clear();
  }
}

template <typename T>
void VisualInertialInitializer<T>::addMeasurement(const Time &ts,
                                       const vec3_t &w_m,
                                       const vec3_t &a_m) {
  imu_data_.emplace_back(ts, ImuSensorReadings{w_m, a_m}, 0);

  if (grids_.size() == nb_cams()) {
    // ------------------------- Initialize grids ------------------------------
    if (grids_init_ == false && grids_[0].detected && grids_[1].detected) {
      prev_grids_ = grids_;
      grids_init_ = true;
    }

    // Make sure first_grids are initialized
    if (grids_init_ == false) {
      LOG_WARN("grids not initialized yet!");
      grids_.clear();
      return;
    }

    // ------------------- Initialize T_WS and T_WF --------------------------
    if (sensor_init_ == false && imu_data_.size() > 1 && grids_[0].detected) {
      // Initialize initial IMU attitude
      vec3s_t gyro;
      vec3s_t accel;
      for (size_t i = 0; i < imu_data_.size(); i++) {
        gyro.push_back(imu_data_.at(i).measurement.gyroscopes);
        accel.push_back(imu_data_.at(i).measurement.accelerometers);
      }
      mat3_t C_WS;
      imu_init_attitude(gyro, accel, C_WS, 1);

      // Construct T_WS
      mat4_t T_WS = tf(C_WS, zeros(3, 1));

      // Construct T_WF
      const mat4_t T_C0F = grids_[0].T_CF;
      const mat4_t T_BS = getImuExtrinsicsEstimate();
      const mat4_t T_BC0 = getCameraExtrinsicsEstimate(0);
			const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
      mat4_t T_WF = T_WS * T_SC0 * T_C0F;

      // // Set T_WF as origin and set T_WS accordingly
      // const vec3_t offset = -1 * tf_trans(T_WF);
      // T_WF = tf(tf_rot(T_WF), zeros(3, 1));
      // T_WS = tf(tf_rot(T_WS), offset);

      const mat4_t T_C0S = T_SC0.inverse();
      const mat4_t T_WS_prev = T_WF * prev_grids_[0].T_CF.inverse() * T_C0S;
      const vec3_t v_WS = tf_trans(T_WS) - tf_trans(T_WS_prev);

      addFiducialPose(T_WF);
      addSensorPosePrior(ts, T_WS, v_WS);

      sensor_init_ = true;
      fiducial_init_ = true;
      prev_grids_ = grids_;
      grids_.clear();
      return;
    }

    // Make sure imu data is streaming first
    if (sensor_init_ == false || fiducial_init_ == false) {
      LOG_WARN("T_WS and T_WF not initialized yet!");
      grids_.clear();
      return;
    }

    // -------------------------- Add new state - -----------------------------
    auto grid_ts = Time(ns2sec(grids_[0].timestamp));
    if (imu_data_.back().timeStamp.toNSec() < grid_ts.toNSec()) {
      return;
    }
    if (grids_[0].detected == false && grids_[1].detected == false) {
      return;
    }

    const mat4_t T_WF = getFiducialPoseEstimate();
    mat4_t T_WS = I(4);
    mat4_t T_WS_prev = I(4);
    if (grids_[0].detected) {
      const mat4_t T_FC0 = grids_[0].T_CF.inverse();
      const mat4_t T_BS = getImuExtrinsicsEstimate();
      const mat4_t T_BC0 = getCameraExtrinsicsEstimate(0);
			const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
      const mat4_t T_C0S = T_SC0.inverse();
      T_WS = T_WF * T_FC0 * T_C0S;
      T_WS_prev = T_WF * prev_grids_[0].T_CF.inverse() * T_C0S;
    } else if (grids_[1].detected) {
      const mat4_t T_FC1 = grids_[1].T_CF.inverse();
      const mat4_t T_BS = getImuExtrinsicsEstimate();
      const mat4_t T_BC1 = getCameraExtrinsicsEstimate(1);
			const mat4_t T_SC1 = T_BS.inverse() * T_BC1;
      const mat4_t T_C1S = T_SC1.inverse();
      T_WS = T_WF * T_FC1 * T_C1S;
      T_WS_prev = T_WF * prev_grids_[1].T_CF.inverse() * T_C1S;
    }

    // Add state
    const vec3_t v_WS = tf_trans(T_WS) - tf_trans(T_WS_prev);
    addState(grid_ts, imu_data_, grids_, T_WS, v_WS);

    // Drop oldest IMU measurements and clear aprilgrids
    trim_imu_data(imu_data_, grid_ts.toNSec());
    prev_grids_ = grids_;
    grids_.clear();
  }
}

template <typename T>
bool VisualInertialInitializer<T>::addState(const Time &ts,
                                  	 	 	 	  const ImuMeasurementDeque &imu_data,
                                  	 	 	 	  const std::map<int, aprilgrid_t> &cam_grids,
                                  	 	 	 	  const mat4_t &T_WS_,
                                  	 	 	 	  const vec3_t &v_WS_) {
  assert(time_delay_block_ != nullptr);
  assert(T_WF_block_ != nullptr);
  assert(cam_blocks_.size() == cam_grids.size());
  assert(T_SC_blocks_.size() == cam_grids.size());
  assert(T_WS_blocks_.size() > 0);
  assert(sb_blocks_.size() > 0);
  assert(added_cam_params_);
  assert(added_imu_params_);
  assert(added_imu_exts_);
  assert(added_imu_timedelay_);
  assert(added_fiducial_pose_);
  assert(added_cam_exts_);

  // Propagate pose and speedAndBias
  auto T_WS = Transformation(getSensorPoseEstimate(pose0_id_));
  auto sb = getSpeedBiasEstimate(sb0_id_);
  const auto t0 = t0_;
  const auto t1 = ts;

  // Add updated sensor pose
	// Note: instead of using the propagated sensor pose `T_WS`, we are using the
	// user provided `T_WS_`, this is because in the scenario where we are using AprilGrid,
	// as a fiducial target, we actually have a better estimation of the sensor
	// pose, as supposed to the imu propagated sensor pose.
  uint64_t pose1_id;
	pose1_id = addSensorPoseParameter(t1, T_WS_);

  // Add updated speed and bias
  uint64_t sb1_id;
	const vec3_t bg = sb.segment(3, 3);
	const vec3_t ba = sb.segment(6, 3);
	sb = sb_init(v_WS_, bg, ba);
	sb1_id = addSpeedBiasParameter(t1, sb);

  // Add error terms
  addImuError(imu_data, t0, t1, pose0_id_, pose1_id, sb0_id_, sb1_id, td_id_);
  addReprojErrors(cam_grids, pose1_id);

  // Update t0, pose0 and sb0
  t0_ = t1;
  pose0_id_ = pose1_id;
  sb0_id_ = sb1_id;

  return true;
}

template <typename T>
void VisualInertialInitializer<T>::optimize(size_t numIter,
																						size_t numThreads,
																						bool verbose) {
  // assemble options
  problem_->options.minimizer_progress_to_stdout = verbose;
  problem_->options.num_threads = numThreads;
  problem_->options.max_num_iterations = numIter;
  // problem_->options.check_gradients = true;

  // call solver
  problem_->solve();
  // std::cout << problem_->summary.FullReport() << std::endl;
  std::cout << problem_->summary.BriefReport() << std::endl;

  // Show result metrics
  std::vector<double> cam_threshold;
  for (size_t i = 0; i < nb_cams(); i++) {
    auto errs = getReprojectionErrors(i);
    printf("cam%ld reprojection error: ", i);
    printf("[rmse: %f", sample_rmse(errs));
    printf(" mean: %f", sample_mean(errs));
    printf(" stddev: %f]", sample_stddev(errs));
    printf("\n");
    cam_threshold.push_back(sample_stddev(errs) * 4.0);
  }

  // Remove outliers
  auto eval = [](CalibReprojError<T> *residual_block, vecx_t &r) {
    if (residual_block == nullptr) {
      return false;
    }

    // Setup residual vector, parameter vector
    r = zeros(residual_block->residualDim(), 1);
    std::vector<double *> params_raw;
    for (const auto &param : residual_block->params) {
      params_raw.push_back(param->parameters());
    }

    // Store orignal information and set information in residual block to
    // identity.  This is so that the reprojection error is not weighed by the
    // square root information
    const matx_t info = residual_block->information();
    residual_block->setInformation(I(2));

    // Evaluate marginalization error
    bool status = residual_block->EvaluateWithMinimalJacobians(
      params_raw.data(),
      r.data(),
      nullptr,
      nullptr
    );

    // Restore original information
    residual_block->setInformation(info);

    return status;
  };

  // Remove residual block if reprojection error is beyond threshold
  LOG_INFO("Removing outlier residual blocks...");
  printf("cam0 threshold: %f\n", cam_threshold[0]);
  printf("cam1 threshold: %f\n", cam_threshold[1]);
  int nb_residual_blocks = vision_errors_.size();
  int nb_outliers = 0;

  #pragma omp parallel for
  for (size_t i = 0; i < vision_errors_.size(); i++) {
    const auto &kv = vision_errors_[i];
    const auto res_block = kv.second;
    vecx_t r;
    eval(res_block.get(), r);

    const int cam_idx = res_block->cameraId();
    const double error = r.norm();
    if (error > cam_threshold[cam_idx]) {
      problem_->removeResidualBlock(kv.first);
      nb_outliers++;
    }
  }
  printf("\n");
  LOG_INFO("Removed %d outliers out of %d", nb_outliers, nb_residual_blocks);

  // Call solver
  LOG_INFO("Optimizing again...");
  problem_->solve();
  std::cout << problem_->summary.FullReport() << std::endl;
  // std::cout << problem_->summary.BriefReport() << std::endl;

  // Show result metrics
  for (size_t i = 0; i < nb_cams(); i++) {
    auto errs = getReprojectionErrors(i);
    printf("cam%ld reprojection error: ", i);
    printf("[rmse: %f", sample_rmse(errs));
    printf(" mean: %f", sample_mean(errs));
    printf(" stddev: %f]", sample_stddev(errs));
    printf("\n");
  }
}

template <typename T>
std::vector<double> VisualInertialInitializer<T>::getReprojectionErrors(const int cam_idx) {
  auto eval = [](CalibReprojError<T> *residual_block, vecx_t &r) {
    if (residual_block == nullptr) {
      return false;
    }

    // Setup residual vector, parameter vector
    r = zeros(residual_block->residualDim(), 1);
    std::vector<double *> params_raw;
    for (const auto &param : residual_block->params) {
      params_raw.push_back(param->parameters());
    }

    // Store orignal information and set information in residual block to
    // identity.  This is so that the reprojection error is not weighed by the
    // square root information
    const matx_t info = residual_block->information();
    residual_block->setInformation(I(2));

    // Evaluate marginalization error
    bool status = residual_block->EvaluateWithMinimalJacobians(
      params_raw.data(),
      r.data(),
      nullptr,
      nullptr
    );

    // Restore original information
    residual_block->setInformation(info);

    return status;
  };

  // Calculate RMSE reprojection error
  std::vector<double> reproj_errors;
  for (const auto &kv : vision_errors_) {
    const auto res_block = kv.second;

    if ((int) res_block->cameraId() == cam_idx) {
      // Evaluate residual block
      vecx_t r;
      eval(res_block.get(), r);
      reproj_errors.push_back(r.norm());
    }
  }

  return reproj_errors;
}

template <typename T>
double VisualInertialInitializer<T>::rmseReprojectionError(const int cam_idx) {
  std::vector<double> reproj_errors = getReprojectionErrors(cam_idx);
  return sample_rmse(reproj_errors);
}

template <typename T>
double VisualInertialInitializer<T>::meanReprojectionError(const int cam_idx) {
  std::vector<double> reproj_errors = getReprojectionErrors(cam_idx);
  return sample_mean(reproj_errors);
}

}  // namespace autocal
#endif /* AUTOCAL_INITIALIZER_HPP */
