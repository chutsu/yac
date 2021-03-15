#ifndef AUTOCAL_CERES_ESTIMATOR_HPP
#define AUTOCAL_CERES_ESTIMATOR_HPP

#include <stdint.h>
#include <iostream>
#include <atomic>
#include <memory>
#include <mutex>
#include <array>
#include <map>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"

#include "common/Common.hpp"
// #include "common/AprilGrid.hpp"
// #include "common/CalibData.hpp"
#include "common/Transformation.hpp"
#include "common/Measurements.hpp"
#include "common/Variables.hpp"
#include "common/Parameters.hpp"

#include "cv/CameraGeometry.hpp"
#include "cv/PinholeCamera.hpp"
#include "cv/RadialTangentialDistortion.hpp"

#include "common/Calibrator.hpp"
#include "common/CeresIterationCallback.hpp"

#include "param/CameraParameterBlock.hpp"
#include "param/PoseParameterBlock.hpp"
#include "param/SpeedAndBiasParameterBlock.hpp"
#include "param/TimeDelayParameterBlock.hpp"
#include "param/FiducialParameterBlock.hpp"

#include "error/ImuError.hpp"
#include "error/PoseError.hpp"
#include "error/FiducialError.hpp"
#include "error/CameraIntrinsicsError.hpp"
#include "error/TimeDelayError.hpp"
#include "error/SpeedAndBiasError.hpp"
#include "error/CalibReprojError.hpp"
#include "error/MarginalizationError.hpp"
#include "error/ReprojectionError.hpp"

#include "Map.hpp"

namespace autocal {

typedef std::shared_ptr<ImuError> ImuErrorPtr;
typedef std::shared_ptr<SpeedAndBiasError> SpeedAndBiasErrorPtr;
typedef std::shared_ptr<CalibReprojError<PinholeRadtan>> CalibReprojErrorPtr;
typedef std::shared_ptr<FiducialError> FiducialErrorPtr;
typedef std::shared_ptr<TimeDelayError> TimeDelayErrorPtr;
typedef std::shared_ptr<PoseError> PoseErrorPtr;
typedef std::shared_ptr<CameraIntrinsicsError> CameraIntrinsicsErrorPtr;
typedef std::shared_ptr<PoseError> ExtrinsicsErrorPtr;

void form_hessian(const std::vector<ImuErrorPtr> &imu_errors,
                  const std::vector<SpeedAndBiasErrorPtr> &sb_errors,
                  const std::vector<CalibReprojErrorPtr> &reproj_errors,
                  const MarginalizationError *marg_err,
                  const PoseError *pose_prior,
                  const FiducialError *fiducial_prior,
                  const TimeDelayError *timedelay_prior,
                  const std::map<int, CameraIntrinsicsErrorPtr> &cam_priors,
                  const std::map<int, ExtrinsicsErrorPtr> &ext_priors,
                  matx_t &H,
                  vecx_t *b=nullptr);

struct ResidualCollection {
  std::vector<std::shared_ptr<ImuError>> imu_errors;
  std::vector<std::shared_ptr<SpeedAndBiasError>> sb_errors;
  std::vector<std::shared_ptr<CalibReprojError<PinholeRadtan>>> reproj_errors;
  std::vector<::ceres::ResidualBlockId> batch_reproj_error_ids;
};

class Estimator {
 public:
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Counters
  uint64_t new_id_ = 0;
  size_t frame_index_ = 0;
  profiler_t profiler_;

  // Flags
  std::string mode_ = "realtime";
  bool prior_set_ = false;
  bool prior_marged_ = false;
  bool added_target_params_ = false;
  bool added_cam_params_ = false;
  bool added_cam_exts_ = false;
  bool added_imu_exts_ = false;
  bool added_imu_params_ = false;
  bool added_imu_timedelay_= false;
  bool added_fiducial_pose_ = false;
	bool initialized_ = false;
	bool estimating_ = false;

  // std::vector<CameraParameters> cam_params_;
  std::map<int, PinholeRadtan> cameras_;
  std::map<int, aprilgrid_t> prev_grids_;
  std::map<int, aprilgrid_t> grids_;
  ImuParameters imu_params_;
  ImuMeasurementDeque imu_data_;
  bool grids_init_ = false;
  bool sensor_init_ = false;
  bool fiducial_init_ = false;
  mat4_t T_WS_init_ = I(4);
  mat4_t T_WF_init_ = I(4);

  // AprilGrid
  aprilgrid_detector_t *detector_;
  calib_target_t target_params_;

  // Parameter ids
  Time t0_{0};
  uint64_t pose0_id_ = 0;
  uint64_t sb0_id_ = 0;
  uint64_t td_id_ = 0;
  uint64_t fiducial_pose_id_ = 0;
  std::vector<uint64_t> sensor_pose_param_ids_;
  std::vector<uint64_t> speed_bias_param_ids_;
  uint64_t imu_exts_id_;
  std::map<int, uint64_t> cam_param_ids_;
  std::map<int, uint64_t> cam_exts_ids_;

  // Parameter blocks
  std::shared_ptr<TimeDelayParameterBlock> time_delay_block_;
  std::shared_ptr<FiducialParameterBlock> T_WF_block_;
  std::shared_ptr<PoseParameterBlock> T_BS_block_;
  std::map<int, std::shared_ptr<CameraParameterBlock>> cam_blocks_;
  std::map<int, std::shared_ptr<PoseParameterBlock>> T_BC_blocks_;
  std::map<uint64_t, std::shared_ptr<PoseParameterBlock>> T_WS_blocks_;
  std::map<uint64_t, std::shared_ptr<SpeedAndBiasParameterBlock>> sb_blocks_;

  // Priors
  bool marged_fiducial_prior_ = false;
  bool marged_timedelay_prior_ = false;
  bool marged_cam_priors_ = false;
  bool marged_ext_priors_ = false;
  ::ceres::ResidualBlockId T_WF_prior_id_ = 0;
  ::ceres::ResidualBlockId timedelay_prior_id_ = 0;
  std::map<int, ::ceres::ResidualBlockId> cam_prior_ids_;
  std::map<int, ::ceres::ResidualBlockId> ext_prior_ids_;
  std::shared_ptr<PoseError> pose_prior_;
  std::shared_ptr<FiducialError> T_WF_prior_;
  std::shared_ptr<TimeDelayError> timedelay_prior_;
  std::map<int, std::shared_ptr<CameraIntrinsicsError>> cam_priors_;
  std::map<int, std::shared_ptr<PoseError>> ext_priors_;

  // Residual blocks
  ResidualCollection residual_collection_;

  // Sliding window residual blocks
  std::deque<std::pair<::ceres::ResidualBlockId, std::shared_ptr<ImuError>>> imu_errors_;
  std::deque<std::pair<::ceres::ResidualBlockId, std::shared_ptr<SpeedAndBiasError>>> sb_errors_;
  std::deque<std::pair<::ceres::ResidualBlockId, std::shared_ptr<CalibReprojError<PinholeRadtan>>>> reproj_errors_;

  // Optimization
  OptSettings opt_config_;
  matx_t calib_covar_;
  std::shared_ptr<Map> problem_;
  std::shared_ptr<Map> batch_problem_;
  std::shared_ptr< ::ceres::LossFunction> cauchyLossFunctionPtr_;
  std::shared_ptr< ::ceres::LossFunction> huberLossFunctionPtr_;
  ::ceres::ResidualBlockId marg_id_;
  std::shared_ptr<MarginalizationError> marg_error_;
  std::unique_ptr<CeresIterationCallback> ceres_cb_;

  // Estimate outputs
  bool est_files_ready_ = false;
  std::string save_dir_ = "/tmp/calib";
  FILE *cam0_file_ = nullptr;
  FILE *cam1_file_ = nullptr;
  FILE *T_WF_file_ = nullptr;
  FILE *T_SC0_file_ = nullptr;
  FILE *T_SC1_file_ = nullptr;
  FILE *td_file_ = nullptr;
  FILE *T_WS_file_ = nullptr;
  FILE *sb_file_ = nullptr;
  FILE *grids_file_ = nullptr;
  FILE *opt_file_ = nullptr;

  // Costs
  FILE *costs_file = nullptr;
  FILE *imu_residuals_file = nullptr;
  FILE *vision_residuals_file = nullptr;
  FILE *reproj_errors_file = nullptr;

  // States
  struct States {
    bool isPrior = false;
    uint64_t id = 0;
    Time timestamp;

    // Prior
    ::ceres::ResidualBlockId prior_id = nullptr;
    uint64_t prior_param_id = 0;

    ::ceres::ResidualBlockId sb_prior_id = nullptr;
    uint64_t sb_prior_param_id = 0;

    // IMU error
    ::ceres::ResidualBlockId imu_error_id = nullptr;
    std::vector<uint64_t> imu_error_param_ids;

    // Speed and bias error
    ::ceres::ResidualBlockId sb_error_id = nullptr;
    std::vector<uint64_t> sb_error_param_ids;

    // Vision error
    std::vector<::ceres::ResidualBlockId> reproj_error_ids;
    std::vector<std::vector<uint64_t>> reproj_error_param_ids;

    States() {}
    States(uint64_t id, Time timestamp) : id(id), timestamp(timestamp) {}

    void print() const {
      printf("\n");
      printf("state[%ld]:\n", id);
      printf("  is_prior: %d\n", isPrior);
      printf("  timestamp: %ld\n", timestamp.toNSec());
      printf("  prior_id: %p\n", prior_id);
      printf("  sb_prior_id: %p\n", sb_prior_id);
      printf("  imu_error_id: %p\n", imu_error_id);
      printf("  sb_error_id: %p\n", sb_error_id);
      printf("  nb_reproj_errors: %ld\n", reproj_error_ids.size());
      printf("\n");
    }
  };
  std::map<uint64_t, States> statesMap_;

  Estimator();
  virtual ~Estimator();

  // void setupOutputs();
  // void configure(const std::string &config_file);
  void configure(const yac::calib_data_t &calib_data);
  int addCalibTarget(const calib_target_t &calib_target);
  int addImu(const ImuParameters &params);
  uint64_t addImuTimeDelay(const double time_delay, const bool fix=false);
  uint64_t addFiducialPose(const mat4_t &T_WF);
  uint64_t addCamera(const size_t cam_idx,
                     const PinholeRadtan &camera,
                     const bool fixed=true);
  uint64_t addImuExtrinsics(const mat4_t &T_BS);
  uint64_t addCameraExtrinsics(const size_t cam_id, const mat4_t &T_SC, const bool fix=true);
  uint64_t addSensorPoseParameter(const Time &ts, const mat4_t &T_WS);
  uint64_t addSpeedBiasParameter(const Time &ts, const autocal::SpeedAndBias &sb);

  mat4_t getImuExtrinsicsEstimate();
  vecx_t getCameraParameterEstimate(const size_t cam_idx);
  mat4_t getCameraExtrinsicsEstimate(const size_t cam_idx);
  mat4_t getFiducialPoseEstimate();
  double getTimeDelayEstimate();
  mat4_t getSensorPoseEstimate(const size_t pose_id);
  vec_t<9> getSpeedBiasEstimate(const size_t sb_id);

  size_t nb_cams();
  size_t nb_frames();

  void propagate(const Time &ts, const ImuMeasurementDeque &imu_data, Transformation &T_WS, SpeedAndBias &sb);
  bool addSensorPosePrior(const Time timestamp,
                          const mat4_t &T_WS,
                          const vec3_t &v_WS);
  ::ceres::ResidualBlockId addImuError(const ImuMeasurementDeque & imu_data,
                                       const Time &t0,
                                       const Time &t1,
                                       const size_t pose0_id,
                                       const size_t pose1_id,
                                       const size_t sb0_id,
                                       const size_t sb1_id,
                                       const size_t timedelay_id);
  void addReprojErrors(const std::map<int, aprilgrid_t> &cam_grids,
                       const size_t pose_id,
                       std::vector<::ceres::ResidualBlockId> &reproj_error_ids,
                       std::vector<std::vector<uint64_t>> &reproj_error_param_ids);
  ::ceres::ResidualBlockId addSpeedBiasError(const SpeedAndBias &sb, uint64_t sb1_id);

  void addMeasurement(const int cam_idx, const aprilgrid_t &grid);
  // void addMeasurement(const timestamp_t &ts, const std::vector<cv::Mat> &cam_images);
  int estimateRelPose(const int cam_idx, const aprilgrid_t &grid, mat4_t &T_CiF);
  void addMeasurement(const Time &ts, const vec3_t &w_m, const vec3_t &a_m, const bool verbose=false);
  bool addState(const Time &ts,
                const ImuMeasurementDeque & imuMeasurements,
                const std::map<int, aprilgrid_t> &cam_grids,
                const mat4_t *T_WS_=nullptr,
                const vec3_t *v_WS_=nullptr);
  bool applyMarginalizationStrategy(size_t window_size);
  void optimize(size_t numIter, size_t numThreads = 1, bool verbose=false);
  void optimizeBatch(size_t numIter, size_t numThreads=1, bool verbose=false);
  void removeOutliers();
  bool setOptimizationTimeLimit(double timeLimit, int minIterations);
  // int recoverCalibCovariance(matx_t &calib_covar);
  PinholeRadtan generateCamera(const int cam_idx);

  // void evalResiduals(std::vector<double> &imu_residuals, std::vector<double> &reproj_residuals);
  void reprojectionErrors(const int cam_idx, std::vector<double> &errs);
  double rmseReprojectionError(const int cam_idx);
  double meanReprojectionError(const int cam_idx);
  double varianceReprojectionError(const int cam_idx);
  // void meanBiases(vec3_t &gyr_mu, vec3_t &acc_mu, vec3_t &gyr_stddev, vec3_t &acc_stddev);
  //
  // void saveEstimates();
  // void saveDetected();
  // void saveOptSummary();
  // void saveCosts();
  // void saveResults(const std::string &results_path);
};

}  // namespace autocal
#endif /* AUTOCAL_CERES_ESTIMATOR_HPP */
