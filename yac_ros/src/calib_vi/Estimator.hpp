#ifndef YAC_ESTIMATOR_HPP
#define YAC_ESTIMATOR_HPP

#include <stdint.h>
#include <iostream>
#include <atomic>
#include <memory>
#include <mutex>
#include <array>
#include <map>

#include <ceres/ceres.h>

#include "core.hpp"
#include "aprilgrid.hpp"
#include "calib_nbv.hpp"

#include "common/Transformation.hpp"
#include "common/Measurements.hpp"
#include "common/Variables.hpp"
#include "common/Parameters.hpp"

#include "cv/CameraGeometry.hpp"
#include "cv/PinholeCamera.hpp"
#include "cv/RadialTangentialDistortion.hpp"

#include "param/CameraParameterBlock.hpp"
#include "param/PoseParameterBlock.hpp"
#include "param/SpeedAndBiasParameterBlock.hpp"
#include "param/TimeDelayParameterBlock.hpp"
#include "param/FiducialParameterBlock.hpp"

#include "error/CalibReprojError.hpp"
#include "error/CameraIntrinsicsError.hpp"
#include "error/FiducialError.hpp"
#include "error/ImuError.hpp"
#include "error/MarginalizationError.hpp"
#include "error/PoseError.hpp"
#include "error/TimeDelayError.hpp"
#include "error/SpeedAndBiasError.hpp"

#include "Map.hpp"
#include "Calibrator.hpp"
#include "CeresIterationCallback.hpp"

namespace yac {

class Estimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ImuError2> ImuError2Ptr;
  typedef std::shared_ptr<SpeedAndBiasError> SpeedAndBiasErrorPtr;
  typedef std::shared_ptr<CalibReprojError<PinholeRadtan>> CalibReprojErrorPtr;
  typedef std::shared_ptr<FiducialError> FiducialErrorPtr;
  typedef std::shared_ptr<TimeDelayError> TimeDelayErrorPtr;
  typedef std::shared_ptr<PoseError> PoseErrorPtr;
  typedef std::shared_ptr<CameraIntrinsicsError> CameraIntrinsicsErrorPtr;
  typedef std::shared_ptr<PoseError> ExtrinsicsErrorPtr;

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
  bool added_imu_params_ = false;
  bool added_imu_timedelay_= false;
  bool added_fiducial_pose_ = false;
  bool added_imu_exts_ = false;
  bool added_cam_exts_ = false;
	bool initialized_ = false;
	bool estimating_ = false;

  // std::vector<CameraParameters> cam_params_;
  std::map<int, PinholeRadtan> cameras_;
  std::map<int, aprilgrid_t> grids_;
  std::map<int, aprilgrid_t> prev_grids_;
  mat4_t prev_T_C0F_ = I(4);
  ImuParameters imu_params_;
  ImuMeasurementDeque imu_data_;
  mat4_t T_WF_init_ = I(4);

  // AprilGrid
  aprilgrid_detector_t *detector_;
  calib_target_t target_params_;

  // Parameter ids
  uint64_t td_id_ = 0;
  uint64_t fiducial_pose_id_ = 0;
  uint64_t imu_exts_id_;
  std::vector<uint64_t> sensor_pose_param_ids_;
  std::vector<uint64_t> speed_bias_param_ids_;
  std::map<int, uint64_t> camera_param_ids_;
  std::map<int, uint64_t> camera_exts_ids_;

  // Parameter blocks
  std::shared_ptr<TimeDelayParameterBlock> time_delay_block_;
  std::shared_ptr<FiducialParameterBlock> T_WF_block_;
  std::shared_ptr<PoseParameterBlock> T_BS_block_;
  std::map<int, std::shared_ptr<CameraParameterBlock>> cam_blocks_;
  std::map<int, std::shared_ptr<PoseParameterBlock>> T_BC_blocks_;
  std::map<uint64_t, std::shared_ptr<PoseParameterBlock>> T_WS_blocks_;
  std::map<uint64_t, std::shared_ptr<SpeedAndBiasParameterBlock>> sb_blocks_;

  // Residual blocks
  struct ResidualCollection {
    std::vector<std::shared_ptr<ImuError2>> imu_errors;
    std::vector<std::shared_ptr<SpeedAndBiasError>> sb_errors;
    std::vector<std::shared_ptr<CalibReprojError<PinholeRadtan>>> reproj_errors;
    std::vector<::ceres::ResidualBlockId> batch_reproj_error_ids;
  };
  ResidualCollection residual_collection_;

  // Sliding window residual blocks
  std::deque<std::pair<::ceres::ResidualBlockId, std::shared_ptr<ImuError2>>> imu_errors_;
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

  struct sliding_window_t {
    size_t size_ = 0;

    std::deque<::ceres::ResidualBlockId> imu_error_ids;
    std::deque<std::vector<uint64_t>> imu_error_param_ids;

    std::deque<::ceres::ResidualBlockId> sb_error_ids;
    std::deque<uint64_t> sb_error_param_ids;

    std::deque<::ceres::ResidualBlockId> reproj_error_ids;
    std::deque<std::vector<uint64_t>> reproj_error_param_ids;

    size_t size() {
      return size_;
    }

    void push(const ::ceres::ResidualBlockId &imu_error_id_,
              const std::vector<uint64_t> &imu_error_param_ids_,
              const ::ceres::ResidualBlockId &sb_error_id_,
              const uint64_t &sb_error_param_,
              const std::vector<::ceres::ResidualBlockId> &reproj_error_ids_,
              const std::vector<std::vector<uint64_t>> &reproj_error_param_ids_) {
      imu_error_ids.push_back(imu_error_id_);
      imu_error_param_ids.push_back(imu_error_param_ids_);

      sb_error_ids.push_back(sb_error_id_);
      sb_error_param_ids.push_back(sb_error_param_);

      for (size_t i = 0; i < reproj_error_ids_.size(); i++) {
        reproj_error_ids.push_back(reproj_error_ids_[i]);
        reproj_error_param_ids.push_back(reproj_error_param_ids_[i]);
      }

      size_++;
    }

    void pop(::ceres::ResidualBlockId &imu_error_id_,
             std::vector<uint64_t> &imu_error_param_ids_,
             ::ceres::ResidualBlockId &sb_error_id_,
             uint64_t &sb_error_param_id_,
             std::vector<::ceres::ResidualBlockId> &reproj_error_ids_,
             std::vector<std::vector<uint64_t>> &reproj_error_param_ids_) {
      // Pop imu error id and params
      imu_error_id_ = imu_error_ids.front();
      imu_error_param_ids_ = imu_error_param_ids.front();
      imu_error_ids.pop_front();
      imu_error_param_ids.pop_front();
      const auto pose_id = imu_error_param_ids_[0];

      // Pop sb error id and params
      sb_error_id_ = sb_error_ids.front();
      sb_error_param_id_ = sb_error_param_ids.front();
      sb_error_ids.pop_front();
      sb_error_param_ids.pop_front();

      // Pop reproj error ids and params
      while (reproj_error_ids.size() != 0) {
        const auto error = reproj_error_ids.front();
        const auto params = reproj_error_param_ids.front();
        // printf("reproj_error_pose_i: %ld\n", params[0]);

        if (params[0] == pose_id) {
          reproj_error_ids_.push_back(error);
          reproj_error_param_ids_.push_back(params);
          reproj_error_ids.pop_front();
          reproj_error_param_ids.pop_front();
        } else {
          break;
        }
      }

      size_--;
    }

  };
  sliding_window_t sliding_window_;

  // struct state_t {
  //   Time timestamp;
  //   ::ceres::ResidualBlockId imu_error_id = nullptr;
  //   std::vector<uint64_t> imu_error_param_ids;
  //
  //   ::ceres::ResidualBlockId sb_error_id = nullptr;
  //   uint64_t sb_param_id;
  //
  //   std::vector<::ceres::ResidualBlockId> reproj_error_ids;
  //   std::vector<std::vector<uint64_t>> reproj_error_param_ids;
  // };
  // std::deque<state_t> sliding_window_;
  int max_window_size_ = 6;

  Estimator()
      : problem_(new Map()),
        batch_problem_(new Map()),
        cauchyLossFunctionPtr_(new ::ceres::CauchyLoss(1)),
        huberLossFunctionPtr_(new ::ceres::HuberLoss(1)),
        marg_id_(0) {}

  virtual ~Estimator() {
    delete detector_;
  }

  void configure(const calib_data_t &calib_data) {
    // Calibration target
    addCalibTarget(calib_data.target);

    // IMU
    // -- Parameters
    yac::ImuParameters imu_params;
    imu_params.a_max = calib_data.imu_params.a_max;
    imu_params.g_max = calib_data.imu_params.g_max;
    imu_params.sigma_g_c = calib_data.imu_params.sigma_g_c;
    imu_params.sigma_a_c = calib_data.imu_params.sigma_a_c;
    imu_params.sigma_gw_c = calib_data.imu_params.sigma_gw_c;
    imu_params.sigma_aw_c = calib_data.imu_params.sigma_aw_c;
    imu_params.sigma_bg = calib_data.imu_params.sigma_bg;
    imu_params.sigma_ba = calib_data.imu_params.sigma_ba;
    imu_params.g = calib_data.imu_params.g;
    imu_params.rate = calib_data.imu_params.rate;
    addImu(imu_params);
    // -- Time delay
    addImuTimeDelay(0.0);
    // -- Extrinsics
    addImuExtrinsics(I(4));

    // Cameras
    for (int i = 0; i < calib_data.nb_cams; i++) {
      // -- Camera parameters
      auto params = calib_data.cam_params.at(i);
      const int img_w = params.resolution[0];
      const int img_h = params.resolution[1];
      const auto fx = params.param(0);
      const auto fy = params.param(1);
      const auto cx = params.param(2);
      const auto cy = params.param(3);
      const auto k1 = params.param(4);
      const auto k2 = params.param(5);
      const auto p1 = params.param(6);
      const auto p2 = params.param(7);
      PinholeRadtan camera{img_w, img_h, fx, fy, cx, cy, {k1, k2, p1, p2}};
      cameras_[i] = camera;
      addCamera(i, cameras_[i]);

      // -- Camera extrinsics
      const mat4_t T_BCi = calib_data.cam_exts.at(i).tf();
      addCameraExtrinsics(i, T_BCi);
    }
  }

  int addCalibTarget(const calib_target_t &target_params) {
    target_params_ = target_params;
    added_target_params_ = true;
    detector_ = new aprilgrid_detector_t(target_params_.tag_rows,
                                         target_params_.tag_cols,
                                         target_params_.tag_size,
                                         target_params_.tag_spacing);
    return 0;
  }

  int addImu(const ImuParameters &params) {
    imu_params_ = params;
    added_imu_params_ = true;
    return 0;
  }

  uint64_t addImuTimeDelay(const double time_delay) {
    // Add Imu delay parameter
    uint64_t id = new_id_++;
    std::shared_ptr<TimeDelayParameterBlock> block(new TimeDelayParameterBlock(time_delay, id));

    // LOG_INFO("Adding time delay parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Trivial);

    // auto td_prior = std::make_shared<TimeDelayError>(0.0, 1.0 / pow(0.01, 2));
    // auto td_prior_id = problem_->addResidualBlock(td_prior,
    //                                               nullptr,
    //                                               problem_->parameterBlockPtr(id));
    // problem_->problem_->SetParameterBlockConstant(block->parameters());

    // Keep track of time delay parameter
    td_id_ = id;
    time_delay_block_ = block;
    added_imu_timedelay_ = true;
    return id;
  }

  uint64_t addFiducialPose(const mat4_t &T_WF) {
    uint64_t id = new_id_++;
    T_WF_init_ = T_WF;
    Transformation T_WF_{T_WF};
    std::shared_ptr<FiducialParameterBlock> block(new FiducialParameterBlock(T_WF_, id));

    // LOG_INFO("Adding fiducial pose parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Trivial);
    batch_problem_->addParameterBlock(block, Map::Trivial);

    fiducial_pose_id_ = id;
    T_WF_block_ = block;
    added_fiducial_pose_ = true;

    return id;
  }

  uint64_t addCamera(const size_t cam_idx, const PinholeRadtan &camera) {
    uint64_t id = new_id_++;
    vecx_t params;
    camera.getIntrinsics(params);
    cameras_[cam_idx] = camera;
    std::shared_ptr<CameraParameterBlock> block(new CameraParameterBlock(params, id));

    // LOG_INFO("Adding camera parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Trivial);
    problem_->setParameterBlockConstant(block);

    camera_param_ids_[cam_idx] = id;
    cam_blocks_[cam_idx] = block;
    added_cam_params_ = true;

    return id;
  }

  uint64_t addImuExtrinsics(const mat4_t &T_BS) {
    uint64_t id = new_id_++;
    Transformation T_BS_{T_BS};
    auto block = std::make_shared<PoseParameterBlock>(T_BS_, id);

    // LOG_INFO("Adding imu extrinsics parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Pose6d);
    // batch_problem_->addParameterBlock(block, Map::Pose6d);

    imu_exts_id_ = id;
    T_BS_block_ = block;
    added_cam_exts_ = true;

    return id;
  }

  uint64_t addCameraExtrinsics(const size_t cam_idx, const mat4_t &T_BC) {
    uint64_t id = new_id_++;
    Transformation T_BC_{T_BC};
    auto block = std::make_shared<PoseParameterBlock>(T_BC_, id);

    // LOG_INFO("Adding sensor camera extrinsics parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Pose6d);
    // if (cam_idx == 0) {
    //   problem_->setParameterBlockConstant(block);
    // }
    problem_->setParameterBlockConstant(block);

    camera_exts_ids_[cam_idx] = id;
    T_BC_blocks_[cam_idx] = block;
    added_cam_exts_ = true;

    return id;
  }

  uint64_t addSensorPoseParameter(const Time &ts, const mat4_t &T_WS) {
    uint64_t id = new_id_++;
    Transformation T_WS_{T_WS};
    auto block = std::make_shared<PoseParameterBlock>(T_WS_, id, ts);
    T_WS_blocks_[id] = block;
    sensor_pose_param_ids_.push_back(id);

    // LOG_INFO("Adding sensor pose parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Pose6d);

    return id;
  }

  uint64_t addSpeedBiasParameter(const Time &ts, const yac::SpeedAndBias &sb) {
    uint64_t id = new_id_++;
    const auto block = std::make_shared<SpeedAndBiasParameterBlock>(sb, id, ts);
    sb_blocks_[id] = block;
    speed_bias_param_ids_.push_back(id);

    // LOG_INFO("Adding sb parameter [%ld]", id);
    problem_->addParameterBlock(block, Map::Trivial);

    return id;
  }

  PinholeRadtan getCamera(const int cam_idx) {
    const auto img_w = cameras_[cam_idx].imageWidth();
    const auto img_h = cameras_[cam_idx].imageHeight();
    const auto params = getCameraParameterEstimate(cam_idx);
    const double fx = params(0);
    const double fy = params(1);
    const double cx = params(2);
    const double cy = params(3);
    const double k1 = params(4);
    const double k2 = params(5);
    const double k3 = params(6);
    const double k4 = params(7);
    PinholeRadtan camera(img_w, img_h, fx, fy, cx, cy, {k1, k2, k3, k4});
    return camera;
  }

  vecx_t getCameraParameterEstimate(const size_t cam_idx) {
    auto param_block = cam_blocks_[cam_idx];
    auto param = static_cast<CameraParameterBlock *>(param_block.get());
    return param->estimate();
  }

  mat4_t getCameraExtrinsicsEstimate(const size_t cam_idx) {
    auto param_block = T_BC_blocks_[cam_idx];
    auto param = static_cast<PoseParameterBlock *>(param_block.get());
    return param->estimate().T();
  }

  mat4_t getImuExtrinsicsEstimate() {
    auto param_block = T_BS_block_;
    auto param = static_cast<PoseParameterBlock *>(param_block.get());
    return param->estimate().T();
  }

  mat4_t getFiducialPoseEstimate() {
    vec2_t xy = T_WF_block_->estimate();
    const vec3_t r_WF = tf_trans(T_WF_init_);
    const vec3_t euler_WF = quat2euler(tf_quat(T_WF_init_));

    const double roll = xy(0);
    const double pitch = xy(1);
    const double yaw = euler_WF(2);
    const vec3_t rpy{roll, pitch, yaw};
    const mat3_t C_WF = euler321(rpy);
    return tf(C_WF, r_WF);
  }

  double getTimeDelayEstimate() {
    auto data = time_delay_block_->parameters();
    return data[0];
  }

  mat4_t getSensorPoseEstimate(const size_t pose_id) {
    if (pose_id == -1) {
      const auto T_WS_block = T_WS_blocks_[sensor_pose_param_ids_.back()];
      const auto T_WS_pose = static_cast<PoseParameterBlock *>(T_WS_block.get());
      return T_WS_pose->estimate().T();
    }
    const auto T_WS_block = T_WS_blocks_[pose_id];
    const auto T_WS_pose = static_cast<PoseParameterBlock *>(T_WS_block.get());
    return T_WS_pose->estimate().T();
  }

  vec_t<9> getSpeedBiasEstimate(const size_t sb_id) {
    const auto sb_block = sb_blocks_[sb_id];
    const auto sb_param = static_cast<SpeedAndBiasParameterBlock *>(sb_block.get());
    return sb_param->estimate();
  }

  size_t nb_cams() { return camera_param_ids_.size(); }

  ::ceres::ResidualBlockId addImuError(const ImuMeasurementDeque & imu_data,
                                       std::vector<uint64_t> &imu_error_param_ids) {
    const auto nb_poses = sensor_pose_param_ids_.size();
    const uint64_t pose0_id = sensor_pose_param_ids_[nb_poses - 2];
    const uint64_t pose1_id = sensor_pose_param_ids_[nb_poses - 1];
    const uint64_t sb0_id = speed_bias_param_ids_[nb_poses - 2];
    const uint64_t sb1_id = speed_bias_param_ids_[nb_poses - 1];
    const auto pose0 = T_WS_blocks_[pose0_id];
    const auto pose1 = T_WS_blocks_[pose1_id];
    const Time t0 = pose0->timestamp();
    const Time t1 = pose1->timestamp();

    imu_error_param_ids.push_back(pose0_id);
    imu_error_param_ids.push_back(sb0_id);
    imu_error_param_ids.push_back(pose1_id);
    imu_error_param_ids.push_back(sb1_id);

    // Add IMU error term
    // clang-format off
    auto imu_error = std::make_shared<ImuError2>(imu_data, imu_params_, t0, t1);
    imu_error->imu_t0 = t0;
    imu_error->imu_t1 = t1;
    imu_error->pose0_id = pose0_id;
    imu_error->pose1_id = pose1_id;
    imu_error->state0_id = sb0_id;
    imu_error->state1_id = sb1_id;
    imu_error->timedelay_id = td_id_;
    imu_error->param_ids.push_back(pose0_id);
    imu_error->param_ids.push_back(sb0_id);
    imu_error->param_ids.push_back(pose1_id);
    imu_error->param_ids.push_back(sb1_id);
    imu_error->params.push_back(problem_->parameterBlockPtr(pose0_id));
    imu_error->params.push_back(problem_->parameterBlockPtr(sb0_id));
    imu_error->params.push_back(problem_->parameterBlockPtr(pose1_id));
    imu_error->params.push_back(problem_->parameterBlockPtr(sb1_id));
    auto imu_error_id = problem_->addResidualBlock(
      imu_error,
      nullptr,
      problem_->parameterBlockPtr(pose0_id),
      problem_->parameterBlockPtr(sb0_id),
      problem_->parameterBlockPtr(pose1_id),
      problem_->parameterBlockPtr(sb1_id)
    );
    residual_collection_.imu_errors.push_back(imu_error);
    imu_errors_.push_back({imu_error_id, imu_error});
    // clang-format on

    return imu_error_id;
  }

  ::ceres::ResidualBlockId addSpeedBiasError(uint64_t &sb_param_id) {
    const auto nb_poses = sensor_pose_param_ids_.size();
    sb_param_id = speed_bias_param_ids_[nb_poses - 2];
    auto sb_block = sb_blocks_[sb_param_id];
    auto sb = sb_block->estimate();

    auto sb_error = std::make_shared<SpeedAndBiasError>(
      sb,
      1.0,
      imu_params_.sigma_bg * imu_params_.sigma_bg,
      imu_params_.sigma_ba * imu_params_.sigma_ba
    );
    sb_error->param_ids.push_back(sb_param_id);
    sb_error->params.push_back(problem_->parameterBlockPtr(sb_param_id));
    auto sb_error_id = problem_->addResidualBlock(
      sb_error, nullptr, problem_->parameterBlockPtr(sb_param_id));

    sb_errors_.push_back({sb_error_id, sb_error});
    return sb_error_id;
  }

  void addReprojErrors(std::map<int, aprilgrid_t> &grids,
                       std::vector<::ceres::ResidualBlockId> &reproj_error_ids,
                       std::vector<std::vector<uint64_t>> &reproj_error_param_ids) {
    // Pose id
    const auto nb_poses = sensor_pose_param_ids_.size();
    // const uint64_t pose_id = sensor_pose_param_ids_[nb_poses - 2];
    const uint64_t pose_id = sensor_pose_param_ids_[nb_poses - 1];

    // Get fiducial pose estimate
    const mat4_t T_WF = getFiducialPoseEstimate();

    // Add CalibReprojErrors for Realtime
    for (size_t cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
      const aprilgrid_t &grid_i = prev_grids_[cam_idx];
      const aprilgrid_t &grid_j = grids[cam_idx];
      const auto ts_i = grid_i.timestamp;
      const auto ts_j = grid_j.timestamp;

      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t grid_i_keypoints;
      vec2s_t grid_j_keypoints;
      vec3s_t object_points;
			grid_j.get_measurements(tag_ids, corner_indicies, grid_j_keypoints, object_points);
			grid_i_keypoints = grid_j_keypoints;

      // Random indicies
      std::set<int> indicies;
      int nb_tag_ids = tag_ids.size();
      for (size_t i = 0; i < std::min(nb_tag_ids, 10 * 4); i++) {
        auto idx = randi(0, nb_tag_ids);
        if (indicies.count(idx) == 0) {
          indicies.insert(idx);
        } else {
          i--;
        }
      }

      // std::vector<int> tag_ids;
      // std::vector<int> corner_indicies;
      // vec2s_t grid_i_keypoints;
      // vec2s_t grid_j_keypoints;
      // vec3s_t object_points;
      // aprilgrid_t::common_measurements(grid_i, grid_j,
      //                                  tag_ids, corner_indicies,
      //                                  grid_i_keypoints,
      //                                  grid_j_keypoints,
      //                                  object_points);
			// printf("nb_tag_ids: %ld\n", tag_ids.size());

      // Add reprojection errors for real time
      // for (size_t i = 0; i < tag_ids.size(); i++) {
      for (auto i : indicies) {
        auto tag_id = tag_ids[i];
        auto corner_idx = corner_indicies[i];
        vec2_t z_i = grid_i_keypoints[i];
        vec2_t z_j = grid_j_keypoints[i];
        vec3_t obj_pt = object_points[i];

        // Create and add vision block
        auto error = std::make_shared<CalibReprojError<PinholeRadtan>>(
          cam_idx,
          tag_id,
          corner_idx,
          obj_pt,
          T_WF,
          z_i,
          z_j,
          ts_i,
          ts_j,
          I(2)
        );
        error->image_width_ = cameras_[cam_idx].imageWidth();
        error->image_height_ = cameras_[cam_idx].imageHeight();
        error->timestamp_ = Time{ns2sec(grid_i.timestamp)};
        error->pose_id_ = pose_id;
        error->tag_id_ = tag_id;
        error->corner_id_ = corner_idx;
        error->param_ids.push_back(pose_id);
        error->param_ids.push_back(fiducial_pose_id_);
        error->param_ids.push_back(imu_exts_id_);
        error->param_ids.push_back(camera_exts_ids_[cam_idx]);
        error->param_ids.push_back(camera_param_ids_[cam_idx]);
        // error->param_ids.push_back(td_id_);
        error->params.push_back(problem_->parameterBlockPtr(pose_id));
        error->params.push_back(problem_->parameterBlockPtr(fiducial_pose_id_));
        error->params.push_back(problem_->parameterBlockPtr(imu_exts_id_));
        error->params.push_back(problem_->parameterBlockPtr(camera_exts_ids_[cam_idx]));
        error->params.push_back(problem_->parameterBlockPtr(camera_param_ids_[cam_idx]));
        // error->params.push_back(problem_->parameterBlockPtr(td_id_));

        auto reproj_error_id = problem_->addResidualBlock(
          error,
          NULL,
          problem_->parameterBlockPtr(pose_id),
          problem_->parameterBlockPtr(fiducial_pose_id_),
          problem_->parameterBlockPtr(imu_exts_id_),
          problem_->parameterBlockPtr(camera_exts_ids_[cam_idx]),
          problem_->parameterBlockPtr(camera_param_ids_[cam_idx])
          // problem_->parameterBlockPtr(td_id_)
        );

        // Return param ids for this individual CalibReprojError
        reproj_error_ids.push_back(reproj_error_id);
        std::vector<uint64_t> params = {
          pose_id,
          fiducial_pose_id_,
          imu_exts_id_,
          camera_exts_ids_[cam_idx],
          camera_param_ids_[cam_idx]
          // td_id_
        };
        reproj_error_param_ids.push_back(params);

        // Keep track of this CalibReprojError
        reproj_errors_.push_back({reproj_error_id, error});
      }
    }
  }

  int estimateRelPose(const int cam_idx,
                      const aprilgrid_t &grid,
                      mat4_t &T_CiF) {
    // assert(cam_params.count(0) == 1);

    // Estimate relative pose
    const auto cam_params = getCameraParameterEstimate(cam_idx);
    const int cam_res[2] = {(int) cameras_[cam_idx].imageWidth(),
                            (int) cameras_[cam_idx].imageHeight()};
    const std::string proj_model = "pinhole";
    const std::string dist_model = "radtan4";
    const vec4_t proj_params = cam_params.head(4);
    const vec4_t dist_params = cam_params.tail(4);

    int retval = 0;
    if (proj_model == "pinhole" && dist_model == "radtan4") {
      const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
      retval = grid.estimate(cam, T_CiF);
    } else if (proj_model == "pinhole" && dist_model == "equi4") {
      const pinhole_equi4_t cam{cam_res, proj_params, dist_params};
      retval = grid.estimate(cam, T_CiF);
    } else {
      FATAL("Unsupported projection-distorion type [%s-%s]!",
            proj_model.c_str(), dist_model.c_str());
    }

    return retval;
  }

  void addMeasurement(const int cam_idx, const aprilgrid_t &grid) {
    grids_[cam_idx] = grid;
  }

  void addMeasurement(const timestamp_t &ts,
                      const std::vector<cv::Mat> &cam_images) {
    assert(added_target_params_ == true);
    profiler_.start("detect aprilgrid");

    for (int i = 0; i < (int) nb_cams(); i++) {
      auto grid = detector_->detect(ts, cam_images[i], true);
      grid.timestamp = ts;
      grids_[i] = grid;
    }

    if (grids_[0].detected == false || grids_[1].detected == false) {
      grids_.clear();
    }

    profiler_.stop("detect aprilgrid");
  }

  void trim_imu_data(ImuMeasurementDeque &imu_data, const Time ts_crop) {
    // Makesure the trime timestamp is after the first imu measurement
    if (ts_crop.toNSec() < imu_data.front().timeStamp.toNSec()) {
      return;
    }

    // Trim IMU measurements
    ImuMeasurementDeque imu_data_cropped;
    for (const auto &meas : imu_data) {
      if (meas.timeStamp.toNSec() >= ts_crop.toNSec()) {
        imu_data_cropped.push_back(meas);
      }
    }

    // Set estimates
    if (imu_data_cropped.size()) {
      imu_data = imu_data_cropped;
    }
  }

  void update_prev_grids(std::map<int, aprilgrid_t> &grids) {
    // Keep track of old aprilgrids
    prev_grids_.clear();
    for (int i = 0; i < nb_cams(); i++) {
      prev_grids_[i] = grids.at(i);
    }
    grids.clear();
  }

  void initialize(const Time &ts,
                  std::map<int, aprilgrid_t> &grids,
                  ImuMeasurementDeque &imu_buf) {
    // Estimate relative pose - T_C0F
    assert(grids.size() > 0);
    assert(grids.at(0).detected);
    mat4_t T_C0F = I(4);
    if (estimateRelPose(0, grids.at(0), T_C0F) != 0) {
      FATAL("Failed to estimate relative pose!");
      return;
    }

    // Estimate initial IMU attitude
    vec3s_t accel;
    vec3s_t gyro;
    for (auto &meas : imu_buf) {
      accel.push_back(meas.measurement.accelerometers);
      gyro.push_back(meas.measurement.gyroscopes);
    }
    mat3_t C_WS;
    imu_init_attitude(gyro, accel, C_WS, 1);

    // Sensor pose - T_WS
    mat4_t T_WS = tf(C_WS, zeros(3, 1));

    // Fiducial pose - T_WF
    const mat4_t T_BC0 = getCameraExtrinsicsEstimate(0);
    const mat4_t T_BS = getImuExtrinsicsEstimate();
    const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
    mat4_t T_WF = T_WS * T_SC0 * T_C0F;

    // Set fiducial target as origin (with r_WF (0, 0, 0))
    // and calculate the sensor pose offset relative to target origin
    const vec3_t offset = -1.0 * tf_trans(T_WF);
    const mat3_t C_WF = tf_rot(T_WF);
    const vec3_t r_WF{0.0, 0.0, 0.0};
    T_WF = tf(C_WF, r_WF);
    T_WS = tf(C_WS, offset);

    // Finish up
    addSensorPoseParameter(ts, T_WS);
    addSpeedBiasParameter(ts, zeros(9, 1));
    addFiducialPose(T_WF);

    std::vector<::ceres::ResidualBlockId> reproj_error_ids;
    std::vector<std::vector<uint64_t>> reproj_error_param_ids;
    addReprojErrors(grids, reproj_error_ids, reproj_error_param_ids);

    LOG_INFO("Initialize:");
    print_matrix("T_WS", T_WS);
    print_matrix("T_WF", T_WF);

    initialized_ = true;
    trim_imu_data(imu_buf, ts);  // Remove imu measurements up to ts
    update_prev_grids(grids);
  }

  bool fiducial_detected(const std::map<int, aprilgrid_t> &grids) {
    assert(grids.size() > 0);

    bool detected = false;
    for (int i = 0; i < nb_cams(); i++) {
      if (grids.at(i).detected) {
        detected = true;
      }
    }

    return detected;
  }

  mat4_t estimateSensorPose(const std::map<int, aprilgrid_t> &grids) {
    if (fiducial_detected(grids) == false) {
      FATAL("No fiducials detected!");
    }

    for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
      // Skip if not detected
      if (grids.at(cam_idx).detected == false) {
        continue;
      }

      // Estimate relative pose
      mat4_t T_CiF = I(4);
      if (estimateRelPose(cam_idx, grids.at(cam_idx), T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      // Infer current pose T_WS using T_C0F, T_SC0 and T_WF
      const mat4_t T_FCi_k = T_CiF.inverse();
      const mat4_t T_BCi = getCameraExtrinsicsEstimate(cam_idx);
      const mat4_t T_BS = getImuExtrinsicsEstimate();
      const mat4_t T_CiS = T_BCi.inverse() * T_BS;
      const mat4_t T_WF = getFiducialPoseEstimate();
      const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;

      return T_WS_k;
    }

    // Should never reach here
    FATAL("Implementation Error!");
    return zeros(4, 4);
  }

  bool addState(const Time &ts,
                ImuMeasurementDeque & imu_data,
                std::map<int, aprilgrid_t> &grids) {
    // Estimate current sensor pose
    const mat4_t T_WS_k = estimateSensorPose(grids);
    const vec3_t r_WS_k = tf_trans(T_WS_k);

    // Infer velocity from two poses T_WS_k and T_WS_km1
    const uint64_t pose_i_id = sensor_pose_param_ids_.back();
    const auto T_WS_km1 = T_WS_blocks_[pose_i_id]->estimate().T();
    const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
    const vec3_t v_WS_k = r_WS_k - r_WS_km1;

    // Add sensor pose parameter
    addSensorPoseParameter(ts, T_WS_k);

    // Add speed bias parameter
    vec_t<9> sb_k;
    sb_k.setZero();
    sb_k << v_WS_k, zeros(3, 1), zeros(3, 1);
    addSpeedBiasParameter(ts, sb_k);

    // Add inertial factor
    std::vector<uint64_t> imu_error_param_ids;
    auto imu_error_id = addImuError(imu_data, imu_error_param_ids);

    // Add speed bias factor
    uint64_t sb_param_id;
    auto sb_error_id = addSpeedBiasError(sb_param_id);

    // Add vision factors
    std::vector<::ceres::ResidualBlockId> reproj_error_ids;
    std::vector<std::vector<uint64_t>> reproj_error_param_ids;
    addReprojErrors(grids, reproj_error_ids, reproj_error_param_ids);

    // Add state
    sliding_window_.push(imu_error_id, imu_error_param_ids,
                         sb_error_id, sb_param_id,
                         reproj_error_ids, reproj_error_param_ids);

    // Optimize sliding window
    if (sliding_window_.size() >= max_window_size_) {
      marginalize();
      optimize();
    }

    // Finish up
    trim_imu_data(imu_data, ts);
    update_prev_grids(grids);
  }

  void addMeasurement(const Time &imu_ts,
                      const vec3_t &imu_gyr,
                      const vec3_t &imu_acc,
                      const bool verbose=false) {
    imu_data_.emplace_back(imu_ts, ImuSensorReadings{imu_gyr, imu_acc}, 0);

    if (grids_.size() == nb_cams()) {
      // Initialize T_WS and T_WF
      if (initialized_ == false) {
        if (grids_[0].detected && imu_data_.size() > 2) {
          const auto ts = imu_data_.front().timeStamp;
          initialize(ts, grids_, imu_data_);
        }
        return;
      }

      // Add new state
      timestamp_t grid_ts = 0;
      for (int i = 0; i < nb_cams(); i++) {
        if (grids_.at(i).detected) {
          grid_ts = grids_.at(i).timestamp;
          break;
        }
      }
      if (fiducial_detected(grids_) && imu_ts.toNSec() >= grid_ts) {
        addState(imu_ts, imu_data_, grids_);
        // addState(Time(ns2sec(grid_ts)), imu_data_, grids_);
      }
    }
  }

  bool marginalize() {
    // Remove marginalizationError from the ceres::Problem and modify it.
    // -------------------------------------------------------------------------
    // This is because to modify the MarginalizationError inserted into a
    // ceres::Problem you need to remove it first. In our case the same
    // MarginalizationError should be reused with new parameters to be
    // marginalized.
    if (marg_error_ && marg_id_) {
      bool success = problem_->removeResidualBlock(marg_id_);
      marg_id_ = 0;
      if (!success) {
        FATAL("Failed to remove marginalization Error!");
        return false;
      }
    }

    // Create new marginalization error if non-exists
    if (marg_error_ == nullptr) {
      marg_error_.reset(new MarginalizationError(*problem_.get()));
    }

    // Form / Modify MarginalizationError
    std::vector<uint64_t> marg_params;
    std::vector<bool> keep_params;

    ::ceres::ResidualBlockId imu_error_id;
    std::vector<uint64_t> imu_error_param_ids;
    ::ceres::ResidualBlockId sb_error_id;
    uint64_t sb_error_param_id;
    std::vector<::ceres::ResidualBlockId> reproj_error_ids;
    std::vector<std::vector<uint64_t>> reproj_error_param_ids;
    sliding_window_.pop(imu_error_id,
                        imu_error_param_ids,
                        sb_error_id,
                        sb_error_param_id,
                        reproj_error_ids,
                        reproj_error_param_ids);

    // -- Add ImuError to MarginalizationError
    // if (state.imu_error_id) {
    if (imu_error_id) {
      auto pose0_param = imu_error_param_ids[0];
      auto sb0_param = imu_error_param_ids[1];

      // marg_error_->addResidualBlock(error_id);
      marg_error_->addResidualBlock(imu_error_id);
      marg_params.push_back(pose0_param);
      marg_params.push_back(sb0_param);
      keep_params.push_back(false);
      keep_params.push_back(false);
    }
    // -- Add SpeedBiasError to MarginalizationError
    if (sb_error_id) {
      marg_error_->addResidualBlock(sb_error_id);
      marg_params.push_back(sb_error_param_id);
      keep_params.push_back(false);
    }
    // -- Add CalibReprojError to MarginalizationError
    for (size_t i = 0; i < reproj_error_ids.size(); i++) {
      auto &error_id = reproj_error_ids[i];
      auto &error_param_ids = reproj_error_param_ids[i];
      marg_error_->addResidualBlock(error_id);
      marg_params.push_back(error_param_ids[0]);
      keep_params.push_back(false);
    }

    // Marginalize out terms
    if (marg_error_->marginalizeOut(marg_params, keep_params) == false) {
      FATAL("Failed to marginalize out params!");
    }
    marg_error_->updateErrorComputation();

    // Add marginalization term
    if (marg_error_->num_residuals() == 0) {
      marg_error_.reset();
    }
    if (marg_error_) {
      std::vector<std::shared_ptr<ParameterBlock> > param_blocks;
      marg_error_->getParameterBlockPtrs(param_blocks);
      marg_id_ = problem_->addResidualBlock(marg_error_, NULL, param_blocks);
      if (!marg_id_) {
        return false;
      }
    }

    return true;
  }

  bool setOptimizationTimeLimit(double timeLimit, int minIterations) {
    if (ceres_cb_ != nullptr) {
      if (timeLimit < 0.0) {
        // no time limit => set minimum iterations to maximum iterations
        ceres_cb_->setMinimumIterations(problem_->options.max_num_iterations);
        return true;
      }
      ceres_cb_->setTimeLimit(timeLimit);
      ceres_cb_->setMinimumIterations(minIterations);
      return true;

    } else if (timeLimit >= 0.0) {
      ceres_cb_ = std::unique_ptr<CeresIterationCallback>(
            new CeresIterationCallback(timeLimit,minIterations));
      problem_->options.callbacks.push_back(ceres_cb_.get());
      return true;

    }
    // no callback yet registered with ceres.
    // but given time limit is lower than 0, so no callback needed
    return true;
  }

  void optimize(const size_t numIter=2,
                const size_t numThreads=4,
                const bool verbose=false) {
    // printf("nb_residual_blocks: %d\n", problem_->problem_->NumResidualBlocks());
    // printf("nb_param_blocks: %d\n", problem_->problem_->NumParameterBlocks());

    // Call solver
    problem_->options.minimizer_progress_to_stdout = verbose;
    problem_->options.num_threads = numThreads;
    problem_->options.max_num_iterations = numIter;
    problem_->solve();

    // matx_t calib_covar;
    // recoverCalibCovariance(calib_covar);
    // printf("trace(calib_covar): %f\n", calib_covar.trace());

    // Update camera parameter estimation
    for (size_t i = 0; i < cameras_.size(); i++) {
      auto params = getCameraParameterEstimate(i);
      cameras_[i].setIntrinsics(params);
    }

    // Summary output
    if (verbose) {
      LOG(INFO) << problem_->summary.FullReport();
    }
    // printf("time_delay: %f\t", getTimeDelayEstimate());
    // std::cout << problem_->summary.BriefReport() << std::endl;
  }

  int recoverCalibCovariance(matx_t &calib_covar) {
    // Recover calibration covariance
    auto T_BS = T_BS_block_->parameters();
    auto time_delay = time_delay_block_->parameters();

    // Setup covariance blocks to estimate
    std::vector<std::pair<const double *, const double *>> covar_blocks;
    covar_blocks.push_back({T_BS, T_BS});
    covar_blocks.push_back({time_delay, time_delay});

    // Estimate covariance
    ::ceres::Covariance::Options options;
    ::ceres::Covariance covar_est(options);
    auto problem_ptr = problem_->problem_.get();
    if (covar_est.Compute(covar_blocks, problem_ptr) == false) {
      LOG_ERROR("Failed to estimate covariance!");
      LOG_ERROR("Maybe Hessian is not full rank?");
      return -1;
    }
    // printf("nb residual blocks: %d\n", problem_ptr->NumResidualBlocks());
    // printf("nb param blocks: %d\n", problem_ptr->NumParameterBlocks());

    // Extract covariances sub-blocks
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_BS_covar;
    Eigen::Matrix<double, 1, 1, Eigen::RowMajor> td_covar;
    covar_est.GetCovarianceBlockInTangentSpace(T_BS, T_BS, T_BS_covar.data());
    covar_est.GetCovarianceBlock(time_delay, time_delay, td_covar.data());

    // Form covariance matrix block
    calib_covar = zeros(7, 7);
    calib_covar.block(0, 0, 6, 6) = T_BS_covar;
    calib_covar.block(6, 6, 1, 1) = td_covar;

    // Check if calib_covar is full-rank?
    if (full_rank(calib_covar) == false) {
      LOG_ERROR("calib_covar is not full rank!");
      return -1;
    }

    return 0;
  }

  // void optimizeBatch(size_t numIter, size_t numThreads=1, bool verbose=false);
  // void removeOutliers();

  // void evalResiduals(std::vector<double> &imu_residuals, std::vector<double> &reproj_residuals);
  // void reprojectionErrors(const int cam_idx, std::vector<double> &errs);
  // double rmseReprojectionError(const int cam_idx);
  // double meanReprojectionError(const int cam_idx);
  // double varianceReprojectionError(const int cam_idx);
  // void meanBiases(vec3_t &gyr_mu, vec3_t &acc_mu, vec3_t &gyr_stddev, vec3_t &acc_stddev);

  // void saveEstimates();
  // void saveDetected();
  // void saveOptSummary();
  // void saveCosts();
  // void saveResults(const std::string &results_path);
};

static void create_timeline(const timestamps_t &imu_ts,
                      	 	  const vec3s_t &imu_acc,
                      	 	  const vec3s_t &imu_gyr,
                      	 	  const std::vector<aprilgrids_t> grid_data,
                      	 	  timeline_t &timeline) {
  // -- Add imu events
  for (size_t i = 0; i < imu_ts.size(); i++) {
    const timestamp_t ts = imu_ts[i];
    const vec3_t a_B = imu_acc[i];
    const vec3_t w_B = imu_gyr[i];
    const timeline_event_t event{ts, a_B, w_B};
    timeline.add(event);
  }

  // -- Add aprilgrids observed from all cameras
  for (size_t cam_idx = 0; cam_idx < grid_data.size(); cam_idx++) {
    const auto grids = grid_data[cam_idx];
    for (const auto &grid : grids) {
      if (grid.detected == false) {
        continue;
      }

      const auto ts = grid.timestamp;
      const timeline_event_t event{ts, (int) cam_idx, grid};
      timeline.add(event);
    }
  }
}

template <typename T>
static int eval_traj(const ctraj_t &traj,
              	 	   const calib_target_t &target,
              	 	   const timestamp_t &ts_start,
              	 	   const timestamp_t &ts_end,
              	 	   const yac::ImuParameters &imu_params,
              	 	   const camera_params_t &cam0_params,
							 		 	 const camera_params_t &cam1_params,
              	 	   const double cam_rate,
              	 	   const mat4_t T_WF,
              	 	   const mat4_t T_BC0,
              	 	   const mat4_t T_BC1,
              	 	   const mat4_t T_BS,
              	 	   matx_t &calib_info) {
  // Simulate camera frames
  aprilgrids_t grids0;
  aprilgrids_t grids1;
  mat4s_t T_WC_sim;
  simulate_cameras<T>(traj, target,
                    	cam0_params,
				 	 	 	 	 	 	  cam1_params,
											cam_rate,
                    	T_WF, T_BC0, T_BC1,
                    	ts_start, ts_end,
                    	grids0, grids1,
                    	T_WC_sim);

  // Simulate imu measurements
  imu_params_t imu_params_;
  imu_params_.rate = imu_params.rate;
  imu_params_.a_max = imu_params.a_max;
  imu_params_.g_max = imu_params.g_max;
  imu_params_.sigma_g_c = imu_params.sigma_g_c;
  imu_params_.sigma_a_c = imu_params.sigma_a_c;
  imu_params_.sigma_gw_c = imu_params.sigma_gw_c;
  imu_params_.sigma_aw_c = imu_params.sigma_aw_c;
  imu_params_.sigma_bg = imu_params.sigma_bg;
  imu_params_.sigma_ba = imu_params.sigma_ba;
  imu_params_.g = imu_params.g;

  timestamps_t imu_time;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  mat4s_t imu_poses;
  vec3s_t imu_vels;
  simulate_imu(traj, ts_start, ts_end,
               T_BC0, T_BS, imu_params_,
               imu_time, imu_accel, imu_gyro,
               imu_poses, imu_vels);

  // Create timeline
  timeline_t timeline;
  create_timeline(imu_time, imu_accel, imu_gyro, {grids0, grids1}, timeline);


	// Setup Camera
	PinholeRadtan cam0;
	{
		const int img_w = cam0_params.resolution[0];
		const int img_h = cam0_params.resolution[1];
		const auto fx = cam0_params.param(0);
		const auto fy = cam0_params.param(1);
		const auto cx = cam0_params.param(2);
		const auto cy = cam0_params.param(3);
		const auto k1 = cam0_params.param(4);
		const auto k2 = cam0_params.param(5);
		const auto p1 = cam0_params.param(6);
		const auto p2 = cam0_params.param(7);
		cam0 = PinholeRadtan{img_w, img_h, fx, fy, cx, cy, {k1, k2, p1, p2}};
	}
	PinholeRadtan cam1;
	{
		const int img_w = cam1_params.resolution[0];
		const int img_h = cam1_params.resolution[1];
		const auto fx = cam1_params.param(0);
		const auto fy = cam1_params.param(1);
		const auto cx = cam1_params.param(2);
		const auto cy = cam1_params.param(3);
		const auto k1 = cam1_params.param(4);
		const auto k2 = cam1_params.param(5);
		const auto p1 = cam1_params.param(6);
		const auto p2 = cam1_params.param(7);
		cam1 = PinholeRadtan{img_w, img_h, fx, fy, cx, cy, {k1, k2, p1, p2}};
	}

  // Setup Estimator
  Estimator est;
  est.addCalibTarget(target);
  est.addImu(imu_params);
  est.addImuTimeDelay(0.0);
	est.addImuExtrinsics(T_BS);
  est.addCamera(0, cam0);
  est.addCamera(1, cam1);
  est.addCameraExtrinsics(0, T_BC0);
  est.addCameraExtrinsics(1, T_BC1);

  for (const auto &ts : timeline.timestamps) {
    const auto result = timeline.data.equal_range(ts);
    // -- Loop through events at timestamp ts since there could be two events
    // at the same timestamp
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;
      // Camera event
      if (event.type == APRILGRID_EVENT) {
        est.addMeasurement(event.camera_index, event.grid);
      }

      // Imu event
      if (event.type == IMU_EVENT) {
        const auto ts = Time(ns2sec(event.ts));
        const vec3_t w_m = event.w_m;
        const vec3_t a_m = event.a_m;
        est.addMeasurement(ts, w_m, a_m);
      }
    }
  }
	est.optimize(5, 1, true);

	matx_t calib_covar;
  est.recoverCalibCovariance(calib_covar);
	calib_info = calib_covar.inverse();

  return 0;
}

template <typename T>
static void nbt_compute(const calib_target_t &target,
                 	 	 	  const ImuParameters &imu_params,
                 	 	 	  const camera_params_t &cam0,
                 	 	 	  const camera_params_t &cam1,
                 	 	 	  const double cam_rate,
                 	 	 	  const mat4_t &T_WF,
                 	 	 	  const mat4_t &T_BC0,
                 	 	 	  const mat4_t &T_BC1,
                 	 	 	  const mat4_t &T_BS,
                 	 	 	  ctrajs_t &trajs,
                 	 	 	  std::vector<matx_t> &calib_infos) {
  // Generate trajectories
  const mat4_t T_FO = calib_target_origin<T>(target, cam0);
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = sec2ts(2.0);
  calib_orbit_trajs<T>(target, cam0, cam1, T_BC0, T_BC1, T_WF, T_FO, ts_start, ts_end, trajs);

  // Evaluate trajectory
  calib_infos.resize(trajs.size());
  #pragma omp parallel for
  for (size_t i = 0; i < trajs.size(); i++) {
    int retval = eval_traj<T>(trajs[i], target, ts_start, ts_end,
                              imu_params, cam0, cam1, cam_rate,
                              T_WF, T_BC0, T_BC1, T_BS, calib_infos[i]);
    if (retval != 0) {
      FATAL("Failed to eval NBT");
    }
  }
}

}  // namespace yac
#endif /* YAC_ESTIMATOR_HPP */
