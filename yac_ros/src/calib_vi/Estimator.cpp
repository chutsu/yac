#include "Estimator.hpp"

namespace yac {

void form_hessian(const std::vector<ImuError2Ptr> &imu_errors,
                  const std::vector<SpeedAndBiasErrorPtr> &sb_errors,
                  const std::vector<CalibReprojErrorPtr> &reproj_errors,
                  const MarginalizationError *marg_err,
                  const PoseError *pose_prior,
                  const FiducialError *fiducial_prior,
                  const TimeDelayError *timedelay_prior,
                  const std::map<int, CameraIntrinsicsErrorPtr> &cam_priors,
                  const std::map<int, ExtrinsicsErrorPtr> &ext_priors,
                  matx_t &H,
                  vecx_t *b) {
  // ---------------------------------------------------------------------------
  // First pass: Determine what parameters we have
  std::set<ParameterBlock *> param_tracker;
  std::map<std::string, int> param_counter;  // param, total size

  // -- Lambda function to track parameters
  auto track_params_fn = [&](std::vector<std::shared_ptr<ParameterBlock>> &params) {
    for (const auto &param : params) {
      // Check if param is already tracked or fixed
      if (param_tracker.count(param.get()) > 0) {
        continue; // Skip this param
      }
      // Keep track of param blocks
      param_counter[param->typeInfo()] += param->minimalDimension();
      param_tracker.insert(param.get());
    }
  };

  // -- MarginalizationError params
  if (marg_err) {
    std::vector<std::shared_ptr<ParameterBlock>> marg_params;
    marg_err->getParameterBlockPtrs(marg_params);
    track_params_fn(marg_params);
  }
  // -- Fiducial, Time delay and camera parameter prior params
  if (pose_prior) {
    track_params_fn(pose_prior->params);
  }
  if (fiducial_prior) {
    track_params_fn(fiducial_prior->params);
  }
  if (timedelay_prior) {
    track_params_fn(timedelay_prior->params);
  }
  for (const auto &kv : cam_priors) track_params_fn(kv.second->params);
  for (const auto &kv : ext_priors) track_params_fn(kv.second->params);
  // -- ImuError2, SpeedAndBiasError, and CalibReprojError params
  for (const auto &err : imu_errors) track_params_fn(err->params);
  for (const auto &err : sb_errors) track_params_fn(err->params);
  for (const auto &err : reproj_errors) track_params_fn(err->params);

  // for (const auto &kv : param_counter) {
  //   printf("%s: %d\n", kv.first.c_str(), kv.second);
  // }
  // printf("nb imu errors: %ld\n", imu_errors.size());
  // printf("nb sb errors: %ld\n", sb_errors.size());
  // printf("nb reproj errors: %ld\n", reproj_errors.size());
  // printf("marg error: %d\n", (marg_err != nullptr));
  // printf("pose prior: %d\n", (pose_prior != nullptr));
  // printf("fiducial prior: %d\n", (fiducial_prior != nullptr));
  // printf("timedelay prior: %d\n", (timedelay_prior != nullptr));
  // printf("cam priors: %ld\n", cam_priors.size());
  // printf("ext priors: %ld\n", ext_priors.size());

  // ---------------------------------------------------------------------------
  // Second pass: Assign jacobian order for each parameter and evaluate factor
  std::map<std::string, int> param_cs;  // Map param type to col index
  std::vector<std::string> param_order = {
    "PoseParameterBlock",
    "SpeedAndBiasParameterBlock",
    "TimeDelayParameterBlock",
    "FiducialParameterBlock",
    "ExtrinsicsParameterBlock",
    "CameraParameterBlock"
  };

  // -- Check which param is not in defined param order
  // for (int i = 0; i < (int) param_order.size(); i++) {
  //   if (param_counter.find(param_order[i]) == param_counter.end()) {
  //     FATAL("Param [%s] not found!", param_order[i].c_str());
  //   }
  // }

  // -- Assign param start index
  for (int i = 0; i < (int) param_order.size(); i++) {
    auto param_i = param_order[i];
    param_cs[param_i] = 0;

    int j = i - 1;
    while (j > -1) {
      auto param_j = param_order[j];
      param_cs[param_order[i]] += param_counter[param_j];
      j--;
    }
  }

  // -- Assign param global index and evaluate residuals
  size_t residuals_size = 0;
  size_t params_size = 0;
  std::vector<bool> factor_ok;
  std::map<ParameterBlock *, size_t> param_index;
  std::vector<ErrorInterface *> errors;

  // -- Lambda fn to assign param order, track global residual and param size
  auto assign_order_fn = [&](ErrorInterface *error) {
    if (error == nullptr) {
      return;
    }
    errors.push_back(error);
    residuals_size += error->residualDim();

    // Assign parameter order in jacobian
    for (auto &param : error->params) {
      // Check if param is already tracked or fixed
      if (param_index.count(param.get()) > 0) {
        continue; // Skip this param
      }

      // Assign jacobian column index for parameter
      param_index.insert({param.get(), param_cs[param->typeInfo()]});
      param_cs[param->typeInfo()] += param->minimalDimension();
      params_size += param->minimalDimension();
    }
  };

  // -- Assign param order for MarginalizationError params
  if (marg_err) {
    assign_order_fn((ErrorInterface *) marg_err);
  }
  // -- Assign param order for prior params
  if (pose_prior) {
    assign_order_fn((ErrorInterface *) pose_prior);
  }
  if (fiducial_prior) {
    assign_order_fn((ErrorInterface *) fiducial_prior);
  }
  if (timedelay_prior) {
    assign_order_fn((ErrorInterface *) timedelay_prior);
  }
  for (const auto &kv : cam_priors) assign_order_fn(kv.second.get());
  for (const auto &kv : ext_priors) assign_order_fn(kv.second.get());
  // -- Assign param order for ImuError2, SpeedAndBiasError, and CalibReprojError params
  for (const auto &err : imu_errors) assign_order_fn(err.get());
  for (const auto &err : sb_errors) assign_order_fn(err.get());
  for (const auto &err : reproj_errors) assign_order_fn(err.get());

  // ---------------------------------------------------------------------------
  // Third pass: Form Hessian H
  auto evaluate_residual_block = [](
      ErrorInterface *residual_block,
      vecx_t &r,
      std::vector<mat_t<dynamic_t, dynamic_t, row_major_t>> &J,
      std::vector<mat_t<dynamic_t, dynamic_t, row_major_t>> &J_min) {
    if (residual_block == nullptr) {
      return false;
    }

    // LOG_INFO("Evaluating %s", residual_block->typeInfo().c_str());
    // Setup residual vector, parameter vector, and jacobian matrices
    r = zeros(residual_block->residualDim(), 1);
    std::vector<double *> params_raw;
    std::vector<double *> J_raw;
    std::vector<double *> J_raw_min;
    for (const auto &param : residual_block->params) {
      params_raw.push_back(param->parameters());
      J.push_back(zeros(r.size(), param->dimension()));
      J_min.push_back(zeros(r.size(), param->minimalDimension()));
      J_raw.push_back(J.back().data());
      J_raw_min.push_back(J_min.back().data());
    }

    // Evaluate marginalization error
    bool status = residual_block->EvaluateWithMinimalJacobians(
      params_raw.data(),
      r.data(),
      J_raw.data(),
      J_raw_min.data()
    );

    return status;
  };

  // Form Hessian and R.H.S of H dx = b
  H = zeros(params_size, params_size);
  if (b) {
    *b = zeros(params_size, 1);
  }

  for (size_t k = 0; k < errors.size(); k++) {
    const auto &error = errors.at(k);
    vecx_t r;
    std::vector<mat_t<dynamic_t, dynamic_t, row_major_t>> J;
    std::vector<mat_t<dynamic_t, dynamic_t, row_major_t>> J_min;
    if (evaluate_residual_block(error, r, J, J_min) == false) {
      LOG_WARN("Failed to evaluate [%s]!", error->typeInfo().c_str());
      continue;
    }

    auto params = error->params;
    for (size_t i = 0; i < error->param_ids.size(); i++) {
      const auto param_i = params[i].get();
      const int idx_i = param_index[param_i];
      const int size_i = param_i->minimalDimension();
      const matx_t &J_i = J_min[i];

      for (size_t j = i; j < error->param_ids.size(); j++) {
        const auto param_j = params[j].get();
        const int idx_j = param_index[param_j];
        const int size_j = param_j->minimalDimension();
        const matx_t &J_j = J_min[j];

        if (i == j) {  // Form diagonals of H
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
        } else {  // Form off-diagonals of H
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) =
            H.block(idx_i, idx_j, size_i, size_j).transpose();
        }

        // Form R.H.S. vector b
        if (b) {
          (*b).segment(idx_i, size_i) += -J_i.transpose() * r;
        }
      }
    }
  }
}

Estimator::Estimator()
    : problem_(new Map()),
      batch_problem_(new Map()),
      cauchyLossFunctionPtr_(new ::ceres::CauchyLoss(1)),
      huberLossFunctionPtr_(new ::ceres::HuberLoss(1)),
      marg_id_(0) {}

Estimator::~Estimator() {
  delete detector_;

  if (opt_config_.save_estimates) {
    fclose(cam0_file_);
    fclose(cam1_file_);
    fclose(T_WF_file_);
    fclose(T_SC0_file_);
    fclose(T_SC1_file_);
    fclose(td_file_);
    fclose(T_WS_file_);
    fclose(sb_file_);
    fclose(grids_file_);
    fclose(opt_file_);
  }
}

void Estimator::setupOutputs() {
  if (opt_config_.save_estimates) {
    if (system(("rm -rf " + save_dir_).c_str()) != 0) {
      FATAL("Failed to remove dir at [%s]", save_dir_.c_str());
    }
    if (system(("mkdir -p " + save_dir_).c_str()) != 0) {
      FATAL("Failed to create dir at [%s]", save_dir_.c_str());
    }
    if (system(("mkdir -p " + save_dir_ + "/calib_covars").c_str()) != 0) {
      FATAL("Failed to create dir");
    }
    if (system(("mkdir -p " + save_dir_ + "/calib_infos").c_str()) != 0) {
      FATAL("Failed to create dir");
    }
    if (system(("mkdir -p " + save_dir_ + "/nbt_covars").c_str()) != 0) {
      FATAL("Failed to create dir");
    }
    if (system(("mkdir -p " + save_dir_ + "/nbt_infos").c_str()) != 0) {
      FATAL("Failed to create dir");
    }
    cam0_file_ = fopen((save_dir_ + "/cam0.csv").c_str(), "w");
    cam1_file_ = fopen((save_dir_ + "/cam1.csv").c_str(), "w");
    T_WF_file_ = fopen((save_dir_ + "/T_WF.csv").c_str(), "w");
    T_SC0_file_ = fopen((save_dir_ + "/T_SC0.csv").c_str(), "w");
    T_SC1_file_ = fopen((save_dir_ + "/T_SC1.csv").c_str(), "w");
    td_file_ = fopen((save_dir_ + "/td.csv").c_str(), "w");
    T_WS_file_ = fopen((save_dir_ + "/T_WS.csv").c_str(), "w");
    sb_file_ = fopen((save_dir_ + "/sb.csv").c_str(), "w");
    grids_file_ = fopen((save_dir_ + "/grids.csv").c_str(), "w");
    opt_file_ = fopen((save_dir_ + "/opt.csv").c_str(), "w");

    est_files_ready_ = true;
  }

  if (opt_config_.save_costs) {
    if (system(("mkdir -p " + save_dir_).c_str()) != 0) {
      FATAL("Failed to create dir at [%s]", save_dir_.c_str());
    }
    costs_file = fopen((save_dir_ + "/costs.csv").c_str(), "w");
    imu_residuals_file = fopen((save_dir_ + "/imu_residuals.csv").c_str(), "w");
    vision_residuals_file = fopen((save_dir_ + "/vision_residuals.csv").c_str(), "w");
    reproj_errors_file = fopen((save_dir_ + "/reproj_errors.csv").c_str(), "w");
  }
}

void Estimator::configure(const std::string &config_file) {
  // Setup estimator
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
  detector_ = new aprilgrid_detector_t(target_params_.tag_rows,
                                       target_params_.tag_cols,
                                       target_params_.tag_size,
                                       target_params_.tag_spacing);
  added_target_params_ = true;
  // -- Load optimization settings
  load_optimization_settings(config, opt_config_);
  // -- Load camera params.
  int nb_cams = config_nb_cameras(config_file);
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    CameraParameters params;
    load_camera_params(config, cam_idx, params, true);

    const int img_w = params.resolution(0);
    const int img_h = params.resolution(1);

    const auto fx = params.intrinsics(0);
    const auto fy = params.intrinsics(1);
    const auto cx = params.intrinsics(2);
    const auto cy = params.intrinsics(3);

    const auto k1 = params.distortion(0);
    const auto k2 = params.distortion(1);
    const auto p1 = params.distortion(2);
    const auto p2 = params.distortion(3);

    PinholeRadtan camera{img_w, img_h, fx, fy, cx, cy, {k1, k2, p1, p2}};
    cameras_[cam_idx] = camera;
  }
  // -- Load imu params.
  yac::ImuParameters imu_params;
  int nb_imus = config_nb_imus(config_file);
  if (nb_imus == 1) {
    load_imu_params(config, 0, imu_params);
  } else if (nb_imus > 1) {
    FATAL("At current the calibrator only supports 1 imu!");
  }
  // -- Load extrinsic params
  std::vector<mat4_t> extrinsic_params;
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    mat4_t T_SC;
    load_extrinsic_params(config, 0, cam_idx, T_SC);
    extrinsic_params.push_back(T_SC);
  }

  // Setup Estimator
  setOptimizationTimeLimit(0.02, 1);
  // setOptimizationTimeLimit(5, 1);
  // -- Add IMU
  addImu(imu_params);
  addImuTimeDelay(0.0, true);
  // -- Add Camera
  addCamera(0, cameras_[0], true, opt_config_.fix_intrinsics);
  addCamera(1, cameras_[1], true, opt_config_.fix_intrinsics);
  // -- Add sensor camera extrinsics
  addSensorCameraExtrinsics(0, extrinsic_params[0]);
  addSensorCameraExtrinsics(1, extrinsic_params[1]);
}

int Estimator::addCalibTarget(const calib_target_t &target_params) {
  target_params_ = target_params;
  added_target_params_ = true;
  return 0;
}

int Estimator::addImu(const ImuParameters &params) {
  imu_params_ = params;
  added_imu_params_ = true;
  return 0;
}

uint64_t Estimator::addImuTimeDelay(const double time_delay, const bool add_prior) {
  // Add Imu delay parameter
  uint64_t id = new_id_++;
  std::shared_ptr<TimeDelayParameterBlock> block(new TimeDelayParameterBlock(time_delay, id));

  // LOG_INFO("Adding time delay parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Trivial);
  problem_->problem_->SetParameterBlockConstant(block->parameters());
  batch_problem_->addParameterBlock(block, Map::Trivial);
  batch_problem_->problem_->SetParameterBlockConstant(block->parameters());

  // Add Imu time delay prior
  if (add_prior) {
    timedelay_prior_ = std::make_shared<TimeDelayError>(time_delay, 1.0e2); // stddev 0.1s
    timedelay_prior_->param_ids.push_back(id);
    timedelay_prior_->params.push_back(problem_->parameterBlockPtr(id));
    timedelay_prior_id_ = problem_->addResidualBlock(timedelay_prior_, NULL, block);
    // batch_problem_->addResidualBlock(timedelay_prior_, NULL, block);
  }

  // Keep track of time delay parameter
  td_id_ = id;
  time_delay_block_ = block;
  added_imu_timedelay_ = true;
  return id;
}

uint64_t Estimator::addFiducialPose(const mat4_t &T_WF, const bool add_prior) {
  uint64_t id = new_id_++;

  T_WF_init_ = T_WF;
  Transformation T_WF_{T_WF};
  std::shared_ptr<FiducialParameterBlock> block(new FiducialParameterBlock(T_WF_, id));
  // std::shared_ptr<PoseParameterBlock> block(new PoseParameterBlock(T_WF_, id));

  // LOG_INFO("Adding fiducial pose parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Trivial);
  batch_problem_->addParameterBlock(block, Map::Trivial);

  // Fix fiducial pose
  // problem_->setParameterBlockConstant(id);

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

  // // Add fiducial pose prior
  // if (add_prior) {
  //   const quat_t q_WF = tf_quat(T_WF_init_);
  //   const vec2_t m = quat2euler(q_WF).head(2);
  //   T_WF_prior_ = std::make_shared<FiducialError>(m, 1.0e2 * I(2)); // stdev 0.1 rad ~ 5.72 deg
  //   T_WF_prior_->param_ids.push_back(id);
  //   T_WF_prior_->params.push_back(problem_->parameterBlockPtr(id));
  //   T_WF_prior_id_ = problem_->addResidualBlock(T_WF_prior_, NULL, block);
  //   // batch_problem_->addResidualBlock(T_WF_prior_, NULL, block);
  // }

  fiducial_pose_id_ = id;
  T_WF_block_ = block;
  added_fiducial_pose_ = true;

  return id;
}

uint64_t Estimator::addCamera(const size_t cam_idx,
                              const PinholeRadtan &camera,
                              const bool add_prior,
                              const bool fixed) {
  uint64_t id = new_id_++;

  vecx_t params;
  camera.getIntrinsics(params);
  cameras_[cam_idx] = camera;
  std::shared_ptr<CameraParameterBlock> block(new CameraParameterBlock(params, id));

  // LOG_INFO("Adding camera parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Trivial);
  batch_problem_->addParameterBlock(block, Map::Trivial);
  if (fixed) {
    problem_->setParameterBlockConstant(block);
    batch_problem_->setParameterBlockConstant(block);
  }

  // if (add_prior) {
  //   matx_t cam_prior_info = I(8);
  //   cam_prior_info(0, 0) = 1.0 / pow(10.0, 2);  // stddev 10 pixels
  //   cam_prior_info(1, 1) = 1.0 / pow(10.0, 2);
  //   cam_prior_info(2, 2) = 1.0 / pow(10.0, 2);
  //   cam_prior_info(3, 3) = 1.0 / pow(10.0, 2);
  //   cam_prior_info(4, 4) = 1.0 / pow(1.0, 2);   // stddev 1.0 (distortion is unit-less)
  //   cam_prior_info(5, 5) = 1.0 / pow(1.0, 2);
  //   cam_prior_info(6, 6) = 1.0 / pow(1.0, 2);
  //   cam_prior_info(7, 7) = 1.0 / pow(1.0, 2);
  //
  //   cam_priors_[cam_idx] = std::make_shared<CameraIntrinsicsError>(params, cam_prior_info);
  //   cam_priors_[cam_idx]->param_ids.push_back(id);
  //   cam_priors_[cam_idx]->params.push_back(problem_->parameterBlockPtr(id));
  //   cam_prior_ids_[cam_idx] =  problem_->addResidualBlock(cam_priors_[cam_idx], NULL, block);
  //   // batch_problem_->addResidualBlock(cam_priors_[cam_idx], NULL, block);
  // }

  camera_param_ids_[cam_idx] = id;
  cam_blocks_[cam_idx] = block;
  added_cam_params_ = true;

  return id;
}

uint64_t Estimator::addSensorCameraExtrinsics(const size_t cam_idx,
                                              const mat4_t &T_SC,
                                              const bool add_prior) {
  uint64_t id = new_id_++;
  Transformation T_SC_{T_SC};
  std::shared_ptr<PoseParameterBlock> block(new ExtrinsicsParameterBlock(T_SC_, id));

  // LOG_INFO("Adding sensor camera extrinsics parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Pose6d);
  batch_problem_->addParameterBlock(block, Map::Pose6d);

  sensor_camera_pose_ids_[cam_idx] = id;
  T_SC_blocks_[cam_idx] = block;
  added_imucam_pose_ = true;

  // Add pose error as a prior on sensor-camera pose
  if (add_prior) {
    Eigen::Matrix<double, 6, 6> info;
    info.setZero();
    info(0, 0) = 1.0 / pow(0.1, 2); // stdev 0.1 radians ~ 5.72 degrees
    info(1, 1) = 1.0 / pow(0.1, 2);
    info(2, 2) = 1.0 / pow(0.1, 2);
    info(3, 3) = 1.0 / pow(0.1, 2); // stdev 0.1m or 10cm
    info(4, 4) = 1.0 / pow(0.1, 2);
    info(5, 5) = 1.0 / pow(0.1, 2);
    ext_priors_[cam_idx] = std::make_shared<PoseError>(T_SC_, info);
    ext_priors_[cam_idx]->param_ids.push_back(id);
    ext_priors_[cam_idx]->params.push_back(problem_->parameterBlockPtr(id));
    ext_prior_ids_[cam_idx] = problem_->addResidualBlock(ext_priors_[cam_idx], NULL, block);
  }

  return id;
}

uint64_t Estimator::addSensorPoseParameter(const Time &ts, const mat4_t &T_WS) {
  uint64_t id = new_id_++;
  Transformation T_WS_{T_WS};
  const auto block = std::make_shared<PoseParameterBlock>(T_WS_, id, ts);
  T_WS_blocks_[id] = block;
  sensor_pose_param_ids_.push_back(id);

  // LOG_INFO("Adding sensor pose parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Pose6d);
  batch_problem_->addParameterBlock(block, Map::Pose6d);

  return id;
}

uint64_t Estimator::addSpeedBiasParameter(const Time &ts, const yac::SpeedAndBias &sb) {
  uint64_t id = new_id_++;
  const auto block = std::make_shared<SpeedAndBiasParameterBlock>(sb, id, ts);
  sb_blocks_[id] = block;
  speed_bias_param_ids_.push_back(id);

  // LOG_INFO("Adding sb parameter [%ld]", id);
  problem_->addParameterBlock(block, Map::Trivial);
  batch_problem_->addParameterBlock(block, Map::Trivial);

  return id;
}

vecx_t Estimator::getCameraParameterEstimate(const size_t cam_idx) {
  auto param_block = cam_blocks_[cam_idx];
  auto param = static_cast<CameraParameterBlock *>(param_block.get());
  return param->estimate();
}

mat4_t Estimator::getSensorCameraPoseEstimate(const size_t cam_idx) {
  auto param_block = T_SC_blocks_[cam_idx];
  auto param = static_cast<PoseParameterBlock *>(param_block.get());
  return param->estimate().T();
}

mat4_t Estimator::getFiducialPoseEstimate() {
  vec2_t xy = T_WF_block_->estimate();
  const vec3_t r_WF = tf_trans(T_WF_init_);
  const vec3_t euler_WF = quat2euler(tf_quat(T_WF_init_));

  const double roll = xy(0);
  const double pitch = xy(1);
  const double yaw = euler_WF(2);
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);

  return tf(C_WF, r_WF);;

  // return T_WF_block_->estimate().T();
}

double Estimator::getTimeDelayEstimate() {
  auto data = time_delay_block_->parameters();
  return data[0];
}

mat4_t Estimator::getSensorPoseEstimate(const size_t pose_id) {
  const auto T_WS_block = T_WS_blocks_[pose_id];
  const auto T_WS_pose = static_cast<PoseParameterBlock *>(T_WS_block.get());
  return T_WS_pose->estimate().T();
}

vec_t<9> Estimator::getSpeedBiasEstimate(const size_t sb_id) {
  const auto sb_block = sb_blocks_[sb_id];
  const auto sb_param = static_cast<SpeedAndBiasParameterBlock *>(sb_block.get());
  return sb_param->estimate();
}

size_t Estimator::nb_cams() {
  return camera_param_ids_.size();
}

size_t Estimator::nb_frames() {
  return statesMap_.size();
}

void Estimator::propagate(const Time &ts,
                          const ImuMeasurementDeque &imu_data,
                          Transformation &T_WS,
                          SpeedAndBias &sb) {
  assert(imu_data.back().timeStamp.toNSec() >= ts.toNSec());
  // Propagate pose and speedAndBias
  const auto t0 = t0_;
  const auto t1 = ts;
  int nb_used_imu_meas = ImuError2::propagation(
    imu_data,
    imu_params_,
    T_WS,  // T_WS is propagated to time step `ts`
    sb,     // sb is propagated to time step `ts`
    t0,
    t1
  );

  // Make sure we have used more than 1 imu measurement to propagate the sensor pose
  // AUTOCAL_ASSERT_TRUE_DBG(Exception, nb_used_imu_meas > 1, "propagation failed");
  if (nb_used_imu_meas < 1){
    LOG_ERROR("nb_used_imu_meas [%d]", nb_used_imu_meas);
    FATAL("Failed to propagate imu measurements!");
  }
}

bool Estimator::addSensorPosePrior(const Time ts,
                                   const mat4_t &T_WS,
                                   const vec3_t &v_WS) {
  // Set time t0 (later for ImuError2 term)
  t0_ = ts;

  // Add a pose parameter block
  Transformation T_WS_{T_WS};
  const uint64_t pose_id = addSensorPoseParameter(ts, T_WS);
  pose0_id_ = pose_id;

  // Add speed and bias parameter block
  yac::SpeedAndBias sb;
  sb.setZero();
  sb.segment<3>(0) = v_WS;
  sb.segment<3>(6) = imu_params_.a0;
  const uint64_t sb_id = addSpeedBiasParameter(ts, sb);
  sb0_id_ = sb_id;

  // // Add sensor pose prior
  // Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Identity();
  // information(0, 0) = 1.0e8;
  // information(1, 1) = 1.0e8;
  // information(2, 2) = 1.0e8;
  // information(3, 3) = 1.0e8;
  // information(4, 4) = 1.0e8;
  // information(5, 5) = 1.0e8;
  // pose_prior_= std::make_shared<PoseError>(T_WS_, information);
  // pose_prior_->param_ids.push_back(pose_id);
  // pose_prior_->params.push_back(problem_->parameterBlockPtr(pose_id));
  // const auto pose_param = problem_->parameterBlockPtr(pose0_id_);
  // const auto pose_prior_id = problem_->addResidualBlock(pose_prior_, NULL, pose_param);
  // batch_problem_->addResidualBlock(pose_prior_, NULL, pose_param);
  //
  // // Add speed and bias prior
  // const auto sb_prior_id = addSpeedBiasError(sb, sb_id);

  // Add state for book keeping
  States states(pose0_id_, ts);
  states.isPrior = true;
  // states.prior_id = pose_prior_id;
  // states.prior_param_id = pose0_id_;
  // states.sb_prior_id = sb_prior_id;
  // states.sb_prior_param_id = sb0_id_;
  statesMap_.insert({states.id, states});

  // Mark prior as set
  prior_set_ = true;

  return true;
}

::ceres::ResidualBlockId Estimator::addImuError(
    const ImuMeasurementDeque & imu_data,
    const Time &t0,
    const Time &t1,
    const size_t pose0_id,
    const size_t pose1_id,
    const size_t sb0_id,
    const size_t sb1_id,
    const size_t timedelay_id) {
  // Add IMU error term
  // clang-format off
  auto imu_error = std::make_shared<ImuError2>(imu_data, imu_params_, t0, t1);
  imu_error->imu_t0 = t0;
  imu_error->imu_t1 = t1;
  imu_error->pose0_id = pose0_id;
  imu_error->pose1_id = pose1_id;
  imu_error->state0_id = sb0_id;
  imu_error->state1_id = sb1_id;
  imu_error->timedelay_id = timedelay_id;
  imu_error->param_ids.push_back(pose0_id);
  imu_error->param_ids.push_back(sb0_id);
  imu_error->param_ids.push_back(pose1_id);
  imu_error->param_ids.push_back(sb1_id);
  imu_error->param_ids.push_back(timedelay_id);
  imu_error->params.push_back(problem_->parameterBlockPtr(pose0_id));
  imu_error->params.push_back(problem_->parameterBlockPtr(sb0_id));
  imu_error->params.push_back(problem_->parameterBlockPtr(pose1_id));
  imu_error->params.push_back(problem_->parameterBlockPtr(sb1_id));
  imu_error->params.push_back(problem_->parameterBlockPtr(timedelay_id));
  auto imu_error_id = problem_->addResidualBlock(
    imu_error,
    nullptr,
    problem_->parameterBlockPtr(pose0_id_),
    problem_->parameterBlockPtr(sb0_id),
    problem_->parameterBlockPtr(pose1_id),
    problem_->parameterBlockPtr(sb1_id),
    problem_->parameterBlockPtr(timedelay_id)
  );
  batch_problem_->addResidualBlock(
    imu_error,
    nullptr,
    batch_problem_->parameterBlockPtr(pose0_id_),
    batch_problem_->parameterBlockPtr(sb0_id),
    batch_problem_->parameterBlockPtr(pose1_id),
    batch_problem_->parameterBlockPtr(sb1_id),
    batch_problem_->parameterBlockPtr(timedelay_id)
  );
  residual_collection_.imu_errors.push_back(imu_error);
  // clang-format on

  imu_errors_.push_back({imu_error_id, imu_error});
  return imu_error_id;
}

::ceres::ResidualBlockId Estimator::addSpeedBiasError(const SpeedAndBias &sb,
                                                      uint64_t sb_id) {
  auto sb_error = std::make_shared<SpeedAndBiasError>(
    sb,
    1.0,
    imu_params_.sigma_bg * imu_params_.sigma_bg,
    imu_params_.sigma_ba * imu_params_.sigma_ba
  );
  sb_error->param_ids.push_back(sb_id);
  sb_error->params.push_back(problem_->parameterBlockPtr(sb_id));
  auto sb_error_id = problem_->addResidualBlock(
    sb_error, nullptr, problem_->parameterBlockPtr(sb_id));
  // batch_problem_->addResidualBlock(
  //   sb_error, nullptr, batch_problem_->parameterBlockPtr(sb_id));
  // residual_collection_.sb_errors.push_back(sb_error);

  sb_errors_.push_back({sb_error_id, sb_error});
  return sb_error_id;
}

void Estimator::addReprojErrors(
    std::map<int, aprilgrid_t> &cam_grids,
    const size_t pose_id,
    std::vector<::ceres::ResidualBlockId> &reproj_error_ids,
    std::vector<std::vector<uint64_t>> &reproj_error_param_ids) {
  // Add CalibReprojErrors for RT
  for (size_t cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    // Loop over every tag seen
    auto &grid = cam_grids.at(cam_idx);

    // Random sample grid observation
		std::vector<int> sample_tag_ids;
		std::vector<int> sample_corner_indicies;
		vec2s_t sample_keypoints;
		vec3s_t sample_object_points;
		grid.sample(opt_config_.grid_limit,
								sample_tag_ids,
								sample_corner_indicies,
								sample_keypoints,
								sample_object_points);

    // Add reprojection errors for real time
    for (size_t i = 0; i < sample_tag_ids.size(); i++) {
			auto tag_id = sample_tag_ids[i];
			auto corner_idx = sample_corner_indicies[i];
			vec2_t kp = sample_keypoints[i];
			vec3_t obj_pt = sample_object_points[i];

			// Create and add vision block
			auto error = std::make_shared<CalibReprojError<PinholeRadtan>>(
					cam_idx,
					tag_id,
					corner_idx,
					obj_pt,
					T_WF_init_,
					kp,
					I(2) * (1 / (opt_config_.sigma_vision * opt_config_.sigma_vision))
			);
			error->image_width_ = cameras_[cam_idx].imageWidth();
			error->image_height_ = cameras_[cam_idx].imageHeight();
			error->timestamp_ = Time{ns2sec(grid.timestamp)};
			error->pose_id_ = pose_id;
			error->tag_id_ = tag_id;
			error->corner_id_ = corner_idx;
			error->param_ids.push_back(pose_id);
			error->param_ids.push_back(fiducial_pose_id_);
			error->param_ids.push_back(sensor_camera_pose_ids_[cam_idx]);
			error->param_ids.push_back(camera_param_ids_[cam_idx]);
			error->params.push_back(problem_->parameterBlockPtr(pose_id));
			error->params.push_back(problem_->parameterBlockPtr(fiducial_pose_id_));
			error->params.push_back(problem_->parameterBlockPtr(sensor_camera_pose_ids_[cam_idx]));
			error->params.push_back(problem_->parameterBlockPtr(camera_param_ids_[cam_idx]));

			auto reproj_error_id = problem_->addResidualBlock(
				error,
				NULL,
				problem_->parameterBlockPtr(pose_id),
				problem_->parameterBlockPtr(fiducial_pose_id_),
				problem_->parameterBlockPtr(sensor_camera_pose_ids_[cam_idx]),
				problem_->parameterBlockPtr(camera_param_ids_[cam_idx])
			);

			// Return param ids for this individual CalibReprojError
			reproj_error_ids.push_back(reproj_error_id);
			std::vector<uint64_t> params = {
				pose_id,
				fiducial_pose_id_,
				sensor_camera_pose_ids_[cam_idx],
				camera_param_ids_[cam_idx],
			};
			reproj_error_param_ids.push_back(params);

			// Keep track of this CalibReprojError
			reproj_errors_.push_back({reproj_error_id, error});
    }
  }

  // Add CalibReprojErrors for batch problem
  for (size_t cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    // Loop over every tag seen
    const auto &grid = cam_grids.at(cam_idx);

		// Grid observation
		std::vector<int> tag_ids;
		std::vector<int> corner_indicies;
		vec2s_t keypoints;
		vec3s_t object_points;
		grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

		for (size_t i = 0; i < tag_ids.size(); i++) {
			auto tag_id = tag_ids[i];
			auto corner_idx = corner_indicies[i];
			vec2_t kp = keypoints[i];
			vec3_t obj_pt = object_points[i];

			// Create and add vision block
			auto error = std::make_shared<CalibReprojError<PinholeRadtan>>(
					cam_idx,
					tag_id,
					corner_idx,
					obj_pt,
					T_WF_init_,
					kp,
					I(2) * (1 / (opt_config_.sigma_vision * opt_config_.sigma_vision))
			);
			error->image_width_ = cameras_[cam_idx].imageWidth();
			error->image_height_ = cameras_[cam_idx].imageHeight();
			error->timestamp_ = Time{ns2sec(grid.timestamp)};
			error->pose_id_ = pose_id;
			error->tag_id_ = tag_id;
			error->corner_id_ = corner_idx;
			error->param_ids.push_back(pose_id);
			error->param_ids.push_back(fiducial_pose_id_);
			error->param_ids.push_back(sensor_camera_pose_ids_[cam_idx]);
			error->param_ids.push_back(camera_param_ids_[cam_idx]);
			error->params.push_back(problem_->parameterBlockPtr(pose_id));
			error->params.push_back(problem_->parameterBlockPtr(fiducial_pose_id_));
			error->params.push_back(problem_->parameterBlockPtr(sensor_camera_pose_ids_[cam_idx]));
			error->params.push_back(problem_->parameterBlockPtr(camera_param_ids_[cam_idx]));

			auto reproj_error_id = batch_problem_->addResidualBlock(
				error,
				NULL,
				problem_->parameterBlockPtr(pose_id),
				problem_->parameterBlockPtr(fiducial_pose_id_),
				problem_->parameterBlockPtr(sensor_camera_pose_ids_[cam_idx]),
				problem_->parameterBlockPtr(camera_param_ids_[cam_idx])
			);
			residual_collection_.reproj_errors.push_back(error);
			residual_collection_.batch_reproj_error_ids.push_back(reproj_error_id);
		}
  }
}

void Estimator::addMeasurement(const int cam_idx, const aprilgrid_t &grid) {
  grids_[cam_idx] = grid;
}

void Estimator::addMeasurement(const timestamp_t &ts,
                               const std::vector<cv::Mat> &cam_images) {
  assert(added_target_params_ == true);
  profiler_.start("detect aprilgrid");

  for (int i = 0; i < (int) nb_cams(); i++) {
    // aprilgrid_t grid(ts,
    //                  target_params_.tag_rows,
    //                  target_params_.tag_cols,
    //                  target_params_.tag_size,
    //                  target_params_.tag_spacing);
    // const vecx_t cam_params = getCameraParameterEstimate(i);
    // const mat3_t K = pinhole_K(cam_params.head(4));
    // const vecx_t D = cam_params.tail(4);
    // aprilgrid_detect(grid, *detector_, cam_images[i], K, D, true);
		auto grid = detector_->detect(ts, cam_images[i]);
    grid.timestamp = ts;
    grids_[i] = grid;
  }

  if (grids_[0].detected == false || grids_[1].detected == false) {
    grids_.clear();
  }

  profiler_.stop("detect aprilgrid");
}

int Estimator::estimateRelPose(const aprilgrid_t &grid, mat4_t &T_C0F) {
	// assert(cam_params.count(0) == 1);

	// Estimate relative pose
	const auto cam_params = getCameraParameterEstimate(0);
	const int cam_res[2] = {(int) cameras_[0].imageWidth(), (int) cameras_[0].imageHeight()};
	const std::string proj_model = "pinhole";
	const std::string dist_model = "radtan4";
	const vec4_t proj_params = cam_params.head(4);
	const vec4_t dist_params = cam_params.tail(4);

	// const int *cam_res = cam_params[0]->resolution;
	// const auto proj_model = cam_params[0]->proj_model;
	// const auto dist_model = cam_params[0]->dist_model;
	// const vec4_t proj_params = cam_params[0]->proj_params();
	// const vec4_t dist_params = cam_params[0]->dist_params();

	int retval = 0;
	if (proj_model == "pinhole" && dist_model == "radtan4") {
		const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
		retval = grid.estimate(cam, T_C0F);
	} else if (proj_model == "pinhole" && dist_model == "equi4") {
		const pinhole_equi4_t cam{cam_res, proj_params, dist_params};
		retval = grid.estimate(cam, T_C0F);
	} else {
		FATAL("Unsupported projection-distorion type [%s-%s]!",
					proj_model.c_str(), dist_model.c_str());
	}

	return retval;
}

void Estimator::addMeasurement(const Time &ts,
                               const vec3_t &w_m,
                               const vec3_t &a_m,
                               const bool verbose) {
  imu_data_.emplace_back(ts, ImuSensorReadings{w_m, a_m}, 0);

  // Setup output files
  if (opt_config_.save_estimates && est_files_ready_ == false) {
    setupOutputs();
  }

  // Handle measurements
  if (grids_.size() == nb_cams()) {
    // // -------------------------- Initialize T_WS ------------------------------
    // if (sensor_init_ == false && imu_data_.size() > 5) {
    //   // Extract gyro and accel data
    //   vec3s_t gyro;
    //   vec3s_t accel;
    //   for (size_t i = 0; i < imu_data_.size(); i++) {
    //     gyro.push_back(imu_data_.at(i).measurement.gyroscopes);
    //     accel.push_back(imu_data_.at(i).measurement.accelerometers);
    //   }
    //
    //   // Initialize initial IMU attitude
    //   mat3_t C_WS;
    //   imu_init_attitude({gyro.begin(), gyro.end()},
    //                     {accel.begin(), accel.end()},
    //                     C_WS,
    //                     5);
    //
    //   // Construct T_WS
    //   T_WS_init_ = tf(C_WS, zeros(3, 1));
    //   sensor_init_ = true;
    //
    //   // Add T_WS prior
    //   addSensorPosePrior(ts, T_WS_init_, zeros(3, 1));
    // }
    //
    // // Make sure imu data is streaming first
    // if (sensor_init_ == false) {
    //   // LOG_WARN("IMU not initialized yet!");
    //   grids_.clear();
    //   return;
    // }
    //
    // // -------------------------- Initialize T_WF ------------------------------
    // if (sensor_init_ && fiducial_init_ == false && grids_[0].detected) {
		// 	// Make sure we have a good detection
		// 	if (grids_[0].nb_detections < 5) {
		// 		grids_.clear();
		// 		return;
		// 	}
    //
    //   // Add T_WF
    //   const mat4_t T_C0F = grids_[0].T_CF;
    //   const mat4_t T_SC0 = getSensorCameraPoseEstimate(0);
    //   const mat4_t T_WF{T_WS_init_ * T_SC0 * T_C0F};
    //   addFiducialPose(T_WF, true);
    //   fiducial_init_ = true;
    //
    //   // Clear grids
    //   grids_.clear();
    //   return;
    // }
    //
    // // Make sure fiducial target is initialized
    // if (fiducial_init_ == false) {
    //   // LOG_WARN("Fiducial not initialized yet!");
    //   grids_.clear();
    //   return;
    // }

    // ------------------------- Initialize grids ------------------------------
    if (grids_init_ == false && grids_[0].detected && grids_[1].detected) {
      grids_init_ = true;
      prev_grids_ = grids_;
			estimateRelPose(grids_[0], prev_T_C0F_);
    }

    // Make sure first_grids are initialized
    if (grids_init_ == false) {
      LOG_WARN("grids not initialized yet!");
      grids_.clear();
      return;
    }

    // ------------------- Initialize T_WS and T_WF --------------------------
    if (sensor_init_ == false && imu_data_.size() > 5 && grids_[0].detected) {
      // Initialize initial IMU attitude
      vec3s_t gyro;
      vec3s_t accel;
      for (size_t i = 0; i < imu_data_.size(); i++) {
        gyro.push_back(imu_data_.at(i).measurement.gyroscopes);
        accel.push_back(imu_data_.at(i).measurement.accelerometers);
      }
      mat3_t C_WS;
      imu_init_attitude(gyro, accel, C_WS, 5);

      // Construct T_WS
      T_WS_init_ = tf(C_WS, zeros(3, 1));

      // Construct T_WF
      // const mat4_t T_C0F = grids_[0].T_CF;
      mat4_t T_C0F;
			estimateRelPose(grids_[0], T_C0F);
      const mat4_t T_SC0 = getSensorCameraPoseEstimate(0);
      mat4_t T_WF = T_WS_init_ * T_SC0 * T_C0F;

      // Calculate offset
      const vec3_t offset = -1 * tf_trans(T_WF);

      // Set T_WF as origin and set T_WS accordingly
      T_WF = tf(tf_rot(T_WF), zeros(3, 1));
      T_WS_init_ = tf(tf_rot(T_WS_init_), offset);

      const mat4_t T_C0S = T_SC0.inverse();
      const mat4_t T_WS_prev = T_WF * prev_T_C0F_.inverse() * T_C0S;
      const vec3_t v_WS = tf_trans(T_WS_init_) - tf_trans(T_WS_prev);

      addFiducialPose(T_WF);
      addSensorPosePrior(ts, T_WS_init_, v_WS);
      print_matrix("T_WF", T_WF);
      print_matrix("T_WS", T_WS_init_);

      sensor_init_ = true;
      fiducial_init_ = true;
      prev_grids_ = grids_;
			estimateRelPose(grids_[0], prev_T_C0F_);
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
    // Make sure we have enough imu timestamps
    auto grid_ts = Time(ns2sec(grids_[0].timestamp));
    if (imu_data_.back().timeStamp.toNSec() < grid_ts.toNSec()) {
      return;
    }
    if (grids_[0].detected == false || grids_[1].detected == false) {
      grids_.clear();
      return;
    }

    // Add states
    initialized_ = true;
    // const mat4_t T_FC0 = grids_[0].T_CF.inverse();
    // const mat4_t T_SC0 = getSensorCameraPoseEstimate(0);
    // const mat4_t T_C0S = T_SC0.inverse();
    // const mat4_t T_WF = getFiducialPoseEstimate();
    // const mat4_t T_WS = T_WF * T_FC0 * T_C0S;
    // const mat4_t T_FC0 = grids_[0].T_CF.inverse();

		mat4_t T_C0F;
		estimateRelPose(grids_[0], T_C0F);

    const mat4_t T_FC0 = T_C0F.inverse();
    const mat4_t T_SC0 = getSensorCameraPoseEstimate(0);
    const mat4_t T_C0S = T_SC0.inverse();
    const mat4_t T_WF = getFiducialPoseEstimate();
    const mat4_t T_WS = T_WF * T_FC0 * T_C0S;
    const mat4_t T_WS_prev = T_WF * prev_T_C0F_.inverse() * T_C0S;
    const vec3_t v_WS = tf_trans(T_WS) - tf_trans(T_WS_prev);
    addState(grid_ts, imu_data_, grids_, &T_WS, &v_WS);

    // Optimize sliding window
    if (statesMap_.size() > opt_config_.window_size && mode_ == "realtime") {
      profiler_.start("optimization");
      applyMarginalizationStrategy(opt_config_.window_size);
      optimize(opt_config_.nb_iters, opt_config_.nb_threads, opt_config_.verbose);

      estimating_ = true;
      profiler_.stop("optimization");
    }

    // Drop oldest IMU measurements and clear aprilgrids
    trim_imu_data(imu_data_, grid_ts.toNSec());
    prev_grids_ = grids_;
		estimateRelPose(grids_[0], prev_T_C0F_);
    grids_.clear();
  }
}

bool Estimator::addState(const Time &ts,
                         const ImuMeasurementDeque & imu_data,
                         std::map<int, aprilgrid_t> &cam_grids,
                         const mat4_t *T_WS_,
                         const vec3_t *v_WS_) {
  assert(imu_data.back().timeStamp.toNSec() >= ts.toNSec());
  assert(time_delay_block_ != nullptr);
  assert(T_WF_block_ != nullptr);
  assert(cam_blocks_.size() == cam_grids.size());
  assert(T_SC_blocks_.size() == cam_grids.size());
  assert(T_WS_blocks_.size() > 0);
  assert(sb_blocks_.size() > 0);
  assert(added_cam_params_);
  assert(added_imu_params_);
  assert(added_imu_timedelay_);
  assert(added_fiducial_pose_);
  assert(added_imucam_pose_);

  // Save detected
  if (opt_config_.save_estimates) {
    saveDetected();
  }

  // Propagate pose and speedAndBias
  auto T_WS = Transformation(getSensorPoseEstimate(pose0_id_));
  auto sb = getSpeedBiasEstimate(sb0_id_);
  const auto t0 = t0_;
  const auto t1 = ts;
  if (T_WS_ == nullptr && v_WS_ == nullptr) {
    propagate(ts, imu_data, T_WS, sb);
  }

  // Add updated sensor pose
  uint64_t pose1_id;
  if (T_WS_) {
    // Note: instead of using the propagated sensor pose `T_WS`, we are using the
    // user provided `T_WS_`, this is because in the scenario where we are using AprilGrid,
    // as a fiducial target, we actually have a better estimation of the sensor
    // pose, as supposed to the imu propagated sensor pose.
    pose1_id = addSensorPoseParameter(t1, *T_WS_);
    // print_matrix("propagated T_WS", T_WS.T());
    // print_matrix("aprilgrid  T_WS", *T_WS_);
    // printf("\n");
  } else {
    pose1_id = addSensorPoseParameter(t1, T_WS.T());
  }

  // Add updated speed and bias
  uint64_t sb1_id;
  if (v_WS_) {
    sb = sb_init(*v_WS_, zeros(3, 1), zeros(3, 1));
    sb1_id = addSpeedBiasParameter(t1, sb);
  } else {
    sb1_id = addSpeedBiasParameter(t1, sb);
  }

  // Add error terms
  // clang-format off
  auto imu_error_id = addImuError(imu_data, t0, t1,
                                  pose0_id_, pose1_id,
                                  sb0_id_, sb1_id,
                                  td_id_);
  auto sb_error_id = addSpeedBiasError(sb, sb1_id);

  std::vector<::ceres::ResidualBlockId> reproj_error_ids;
  std::vector<std::vector<uint64_t>> reproj_error_param_ids;
  addReprojErrors(cam_grids, pose1_id, reproj_error_ids, reproj_error_param_ids);
  // clang-format on

  // Add states for book keeping
  assert(statesMap_.find(pose1_id) == statesMap_.end());
  States states(pose1_id, ts);
  states.isPrior = false;
  states.imu_error_id = imu_error_id;
  states.imu_error_param_ids.push_back(pose0_id_);
  states.imu_error_param_ids.push_back(sb0_id_);
  states.imu_error_param_ids.push_back(pose1_id);
  states.imu_error_param_ids.push_back(sb1_id);
  states.imu_error_param_ids.push_back(td_id_);
  states.sb_error_id = sb_error_id;
  states.sb_error_param_ids.push_back(sb1_id);
  states.reproj_error_ids = reproj_error_ids;
  states.reproj_error_param_ids = reproj_error_param_ids;
  statesMap_.insert({states.id, states});

  // Make sure ImuError2 knows the state.id
  // imu_error->state_id = states.id;

  // Update t0, pose0 and sb0
  t0_ = t1;
  pose0_id_ = pose1_id;
  sb0_id_ = sb1_id;
  frame_index_++;

  return true;
}

bool Estimator::applyMarginalizationStrategy(size_t window_size) {
  // Check if we have anything to marginalize out
  if (statesMap_.size() <= window_size) {
    return false; // Nothing to marginalize out
  }

  // Iterate over states and obtain frame ids to be marginalized out.
  std::vector<uint64_t> states_marg;
  for (const auto &kv : statesMap_) {
    const auto &state = kv.second;
    states_marg.push_back(state.id);
    if (states_marg.size() == 1) break;
  }

  // Remove marginalizationError from the ceres::Problem and modify it.
  // -------------------------------------------------------------------------
  // This is because to modify the MarginalizationError inserted into a
  // ceres::Problem you need to remove it first. In our case the same
  // MarginalizationError should be reused with new parameters to be
  // marginalized.
  if (marg_error_ && marg_id_) {
    bool success = problem_->removeResidualBlock(marg_id_);
    // AUTOCAL_ASSERT_TRUE_DBG(Exception, success, "Failed to remove marginalization error");
    marg_id_ = 0;
    if (!success) return false;
  }

  // Create new marginalization error if non-exists
  if (marg_error_ == nullptr) {
    marg_error_.reset(new MarginalizationError(*problem_.get()));
  }

  // Form / Modify MarginalizationError
  std::vector<uint64_t> marg_params;
  std::vector<bool> keep_params;

  for (size_t k = 0; k < states_marg.size(); k++) {
    const auto &kv = statesMap_.find(states_marg[k]);
    const auto &state = kv->second;
    // LOG_INFO("Marginalizing state [%ld]", state.id);

    // // Add pose prior to MarginalizationError
    // if (state.prior_id) {
    //   LOG_INFO("Adding pose prior linked to pose [%ld]", state.prior_param_id);
    //   marg_error_->addResidualBlock(state.prior_id);
    //   marg_params.push_back(state.prior_param_id);  // T_WS_0
    //   keep_params.push_back(false);  // T_WS_0
    // }

    // Add speed bias prior to MarginalizationError
    if (state.sb_prior_id) {
      // LOG_INFO("Adding sb prior linked to sb [%ld]", state.sb_prior_param_id);
      marg_error_->addResidualBlock(state.sb_prior_id);
      marg_params.push_back(state.sb_prior_param_id);  // sb_0
      keep_params.push_back(false);  // sb_0
      sb_errors_.pop_front();
    }

    // Add imu error to MarginalizationError
    if (imu_errors_.size()) {
      auto &error_id = imu_errors_.front().first;
      auto &error = imu_errors_.front().second;
      if (error->param_ids[0] == state.id) {
        auto pose0_param = error->param_ids[0];
        auto sb0_param = error->param_ids[1];
        // auto pose1_param = error->param_ids[2];
        // LOG_INFO("Adding imu error linked to poses [%ld - %ld]", pose0_param, pose1_param);
        marg_error_->addResidualBlock(error_id);
        marg_params.push_back(pose0_param);  // T_WS_0
        marg_params.push_back(sb0_param);    // SB_0
        keep_params.push_back(false);  // T_WS_0
        keep_params.push_back(false);  // sb_0
        imu_errors_.pop_front();
      }
    }

    // Add SpeedAndBisaError to MarginalizationError
    if (state.isPrior == false &&  sb_errors_.size()) {
      auto &error_id = sb_errors_.front().first;
      auto &error = sb_errors_.front().second;

      if (error->param_ids[0] == (state.id + 1)) {
        // LOG_INFO("Adding sb error linked to sb [%ld]", error->param_ids[0]);
        marg_error_->addResidualBlock(error_id);
        marg_params.push_back(error->param_ids[0]);  // SB_0
        keep_params.push_back(false);  // sb_0
        sb_errors_.pop_front();
      }
    }

    // Add CalibReprojError to MarginalizationError
    while (reproj_errors_.size()) {
      auto &error_id = reproj_errors_.front().first;
      auto &error = reproj_errors_.front().second;
      if (error->param_ids[0] != state.id) {
        break;
      }

      // LOG_INFO("Adding reproj error linked to pose [%ld]", error->param_ids[0]);
      marg_error_->addResidualBlock(error_id);
      marg_params.push_back(error->param_ids[0]);
      keep_params.push_back(false);
      reproj_errors_.pop_front();
    }

    // Remember to remove the state from the states map
    statesMap_.erase(states_marg[k]);
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
    marg_id_ = problem_->addResidualBlock(
      marg_error_,
      NULL,
      param_blocks
    );
    // AUTOCAL_ASSERT_TRUE_DBG(
    //   Exception,
    //   marg_id_,
    //   "could not add marginalization error"
    // );
    if (!marg_id_) {
      return false;
    }
  }

  return true;
}

void Estimator::optimize(size_t numIter, size_t numThreads, bool verbose) {
  // Switch off priors
  // // -- Switch off T_WF prior
  // matx_t T_WF_info;
  // if (T_WF_prior_) {
  //   T_WF_info = T_WF_prior_->information();
  //   matx_t zero_info = zeros(T_WF_info.rows(), T_WF_info.cols());
  //   T_WF_prior_->setInformation(zero_info);
  // }
  // // -- Switch off time delay prior
  // double timedelay_info;
  // if (timedelay_prior_) {
  //   timedelay_info = timedelay_prior_->information();
  //   timedelay_prior_->setInformation(0);
  // }
  // // -- Switch off camera prior
  // std::map<int, matx_t> cam_infos;
  // for (auto &kv : cam_priors_) {
  //   auto cam_idx = kv.first;
  //   auto prior = kv.second;
  //
  //   cam_infos[cam_idx] = prior->information();
  //   matx_t zero_info = zeros(cam_infos[cam_idx].rows(), cam_infos[cam_idx].cols());
  //   prior->setInformation(zero_info);
  // }

  // Call solver
  problem_->options.minimizer_progress_to_stdout = verbose;
  problem_->options.num_threads = numThreads;
  problem_->options.max_num_iterations = numIter;
  // problem_->options.linear_solver_type = ::ceres::DENSE_QR;
  // problem_->options.logging_type = ::ceres::SILENT;
  problem_->solve();

  // Update camera parameter estimation
  for (size_t i = 0; i < cameras_.size(); i++) {
    auto params = getCameraParameterEstimate(i);
    cameras_[i].setIntrinsics(params);
  }

  // Reactivate priors
  // // -- Reactivate T_WF prior
  // if (T_WF_prior_) {
  //   T_WF_prior_->setInformation(T_WF_info);
  // }
  // // -- Reactivate time delay prior
  // if (timedelay_prior_) {
  //   timedelay_prior_->setInformation(timedelay_info);
  // }
  // // -- Reactivate camera intrinsic priors
  // if (cam_priors_.size()) {
  //   for (auto &kv : cam_priors_) {
  //     auto cam_idx = kv.first;
  //     auto prior = kv.second;
  //     prior->setInformation(cam_infos[cam_idx]);
  //   }
  // }

  // Summary output
  if (verbose) {
    LOG(INFO) << problem_->summary.FullReport();
  }
  std::cout << problem_->summary.BriefReport() << std::endl;
  // std::cout << "nb param blocks: " << problem_->summary.num_parameter_blocks << std::endl;
  // std::cout << "nb residual blocks: " << problem_->summary.num_residual_blocks << std::endl;

  // Save estimates
  if (opt_config_.save_estimates) {
    saveEstimates();
    saveOptSummary();
  }

  // Save costs
  if (opt_config_.save_costs) {
    saveCosts();
  }
}

void Estimator::optimizeBatch(size_t numIter, size_t numThreads, bool verbose) {
  // Call solver
  batch_problem_->options.minimizer_progress_to_stdout = verbose;
  batch_problem_->options.num_threads = numThreads;
  batch_problem_->options.max_num_iterations = numIter;
  // batch_problem_->options.logging_type = ::ceres::SILENT;
  batch_problem_->solve();
  std::cout << batch_problem_->summary.FullReport() << std::endl;

  printf("Reprojection Error:\n");
  printf("cam0: [%f, %f, %f]  # rmse, mean, stddev\n",
         rmseReprojectionError(0),
         meanReprojectionError(0),
         sqrt(varianceReprojectionError(0)));
  printf("cam1: [%f, %f, %f]  # rmse, mean, stddev\n",
         rmseReprojectionError(1),
         meanReprojectionError(1),
         sqrt(varianceReprojectionError(1)));
  printf("\n");

  // removeOutliers();
  //
  // printf("Solve again\n");
  // batch_problem_->solve();
  // std::cout << batch_problem_->summary.FullReport() << std::endl;
  //
  // const auto cam0 = getCameraParameterEstimate(0);
  // const auto cam1 = getCameraParameterEstimate(1);
  // const auto T_SC0 = getSensorCameraPoseEstimate(0);
  // const auto T_SC1 = getSensorCameraPoseEstimate(1);
  // const auto T_C0C1 = T_SC0.inverse() * T_SC1;
  // print_vector("cam0", cam0);
  // print_vector("cam1", cam1);
  // print_matrix("T_SC0", T_SC0);
  // print_matrix("T_SC1", T_SC1);
  // print_matrix("T_C0C1", T_C0C1);
  // printf("time_delay: %f\n", getTimeDelayEstimate());
  //
  // // Update camera parameter estimation
  // for (size_t i = 0; i < cameras_.size(); i++) {
  //   vecx_t params = getCameraParameterEstimate(i);
  //   cameras_[i].setIntrinsics(params);
  // }
}

void Estimator::removeOutliers() {
  const double cam0_stddev = sqrt(varianceReprojectionError(0));
  const double cam1_stddev = sqrt(varianceReprojectionError(1));
  const double cam0_threshold = cam0_stddev * 2.0;
  const double cam1_threshold = cam1_stddev * 2.0;
  printf("cam0_threshold: %f\n", cam0_threshold);
  printf("cam1_threshold: %f\n", cam1_threshold);

  auto eval = [](CalibReprojError<PinholeRadtan> *residual_block, vecx_t &r) {
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

  int nb_removed = 0;
  auto problem = batch_problem_->problem_;

  for (size_t i = 0; i < residual_collection_.reproj_errors.size(); i++) {
    const auto res_id = residual_collection_.batch_reproj_error_ids[i];
    const auto res_block = residual_collection_.reproj_errors[i];
    vecx_t r;

    if (problem->GetCostFunctionForResidualBlock(res_id) == NULL) {
      continue;
    }

    switch (res_block->cameraId()) {
      case 0:
        eval(res_block.get(), r);
        if (r.norm() > cam0_threshold) {
          batch_problem_->problem_->RemoveResidualBlock(res_id);
          // batch_problem_->removeResidualBlock(res_id);
          nb_removed++;
        }
        break;
      case 1:
        eval(res_block.get(), r);
        if (r.norm() > cam1_threshold) {
          batch_problem_->problem_->RemoveResidualBlock(res_id);
          // batch_problem_->removeResidualBlock(res_id);
          nb_removed++;
        }
        break;
    }
  }

  printf("nb_removed: %d\n", nb_removed);
}

// Set a time limit for the optimization process.
bool Estimator::setOptimizationTimeLimit(double timeLimit, int minIterations) {
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

int Estimator::recoverCalibCovariance(matx_t &calib_covar) {
  // Recover calibration covariance
  auto T_SC0 = T_SC_blocks_[0]->parameters();
  auto T_SC1 = T_SC_blocks_[1]->parameters();
  auto cam0 = cam_blocks_[0]->parameters();
  auto cam1 = cam_blocks_[1]->parameters();

  // Setup covariance blocks to estimate
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  // -- Row 1
  covar_blocks.push_back({T_SC0, T_SC0});
  covar_blocks.push_back({T_SC0, T_SC1});
  covar_blocks.push_back({T_SC0, cam0});
  covar_blocks.push_back({T_SC0, cam1});
  // -- Row 2
  covar_blocks.push_back({T_SC1, T_SC1});
  covar_blocks.push_back({T_SC1, cam0});
  covar_blocks.push_back({T_SC1, cam1});
  // -- Row 3
  covar_blocks.push_back({cam0, cam0});
  covar_blocks.push_back({cam0, cam1});
  // -- Row 4
  covar_blocks.push_back({cam1, cam1});

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
  // -- Row 1
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_SC0__T_SC0_covar;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_SC0__T_SC1_covar;
  Eigen::Matrix<double, 6, 8, Eigen::RowMajor> T_SC0__cam0_covar;
  Eigen::Matrix<double, 6, 8, Eigen::RowMajor> T_SC0__cam1_covar;
  covar_est.GetCovarianceBlockInTangentSpace(T_SC0, T_SC0, T_SC0__T_SC0_covar.data());
  covar_est.GetCovarianceBlockInTangentSpace(T_SC0, T_SC1, T_SC0__T_SC1_covar.data());
  covar_est.GetCovarianceBlockInTangentSpace(T_SC0, cam0, T_SC0__cam0_covar.data());
  covar_est.GetCovarianceBlockInTangentSpace(T_SC0, cam1, T_SC0__cam1_covar.data());
  // -- Row 2
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_SC1__T_SC1_covar;
  Eigen::Matrix<double, 6, 8, Eigen::RowMajor> T_SC1__cam0_covar;
  Eigen::Matrix<double, 6, 8, Eigen::RowMajor> T_SC1__cam1_covar;
  covar_est.GetCovarianceBlockInTangentSpace(T_SC1, T_SC1, T_SC1__T_SC1_covar.data());
  covar_est.GetCovarianceBlockInTangentSpace(T_SC1, cam0, T_SC1__cam0_covar.data());
  covar_est.GetCovarianceBlockInTangentSpace(T_SC1, cam1, T_SC1__cam1_covar.data());
  // -- Row 3
  Eigen::Matrix<double, 8, 8, Eigen::RowMajor> cam0__cam0_covar;
  Eigen::Matrix<double, 8, 8, Eigen::RowMajor> cam0__cam1_covar;
  covar_est.GetCovarianceBlock(cam0, cam0, cam0__cam0_covar.data());
  covar_est.GetCovarianceBlock(cam0, cam1, cam0__cam1_covar.data());
  // -- Row 4
  Eigen::Matrix<double, 8, 8, Eigen::RowMajor> cam1__cam1_covar;
  covar_est.GetCovarianceBlock(cam1, cam1, cam1__cam1_covar.data());

  // Form covariance matrix block
  calib_covar = zeros(28, 28);
  // -- Row 1
  calib_covar.block(0, 0, 6, 6) = T_SC0__T_SC0_covar;
  calib_covar.block(0, 6, 6, 6) = T_SC0__T_SC1_covar;
  calib_covar.block(0, 12, 6, 8) = T_SC0__cam0_covar;
  calib_covar.block(0, 20, 6, 8) = T_SC0__cam1_covar;
  // -- Row 2
  calib_covar.block(6, 0, 6, 6) = T_SC0__T_SC1_covar.transpose();
  calib_covar.block(6, 6, 6, 6) = T_SC1__T_SC1_covar;
  calib_covar.block(6, 12, 6, 8) = T_SC1__cam0_covar;
  calib_covar.block(6, 20, 6, 8) = T_SC1__cam1_covar;
  // -- Row 3
  calib_covar.block(12, 0, 8, 6) = T_SC0__cam0_covar.transpose();
  calib_covar.block(12, 6, 8, 6) = T_SC1__cam0_covar.transpose();
  calib_covar.block(12, 12, 8, 8) = cam0__cam0_covar;
  calib_covar.block(12, 20, 8, 8) = cam0__cam1_covar;
  // -- Row 4
  calib_covar.block(20, 0, 8, 6) = T_SC0__cam1_covar.transpose();
  calib_covar.block(20, 6, 8, 6) = T_SC1__cam1_covar.transpose();
  calib_covar.block(20, 12, 8, 8) = cam0__cam1_covar.transpose();
  calib_covar.block(20, 20, 8, 8) = cam1__cam1_covar;

  // Check if calib_covar is full-rank?
  if (rank(calib_covar) != calib_covar.rows()) {
    LOG_ERROR("calib_covar is not full rank!");
    return -1;
  }

  return 0;
}

PinholeRadtan Estimator::generateCamera(const int cam_idx) {
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

void Estimator::evalResiduals(std::vector<double> &imu_residuals,
                              std::vector<double> &reproj_residuals) {
  auto eval = [](ErrorInterface *residual_block,
                 std::vector<double> &residuals) {
    if (residual_block == nullptr) {
      return;
    }

    // Setup residual vector, parameter vector
    vecx_t r = zeros(residual_block->residualDim(), 1);
    std::vector<double *> params_raw;
    for (const auto &param : residual_block->params) {
      params_raw.push_back(param->parameters());
    }

    // Evaluate marginalization error
    residual_block->EvaluateWithMinimalJacobians(
      params_raw.data(),
      r.data(),
      nullptr,
      nullptr
    );

    // Push to residuals std::vector
    for (long i = 0; i < r.size(); i++) {
      residuals.push_back(r(i));
    }
  };

  // Evaluate residual blocks
  for (const auto &error : residual_collection_.imu_errors) {
    eval(error.get(), imu_residuals);
  }
  for (const auto &error : residual_collection_.reproj_errors) {
    eval(error.get(), reproj_residuals);
  }
}

void Estimator::reprojectionErrors(const int cam_idx, std::vector<double> &errs) {
  auto eval = [](CalibReprojError<PinholeRadtan> *residual_block, vecx_t &r) {
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

  // Evaluate residual blocks
  for (const auto &res_block : residual_collection_.reproj_errors) {
    if ((int) res_block->cameraId() == cam_idx) {
      vecx_t r;
      eval(res_block.get(), r);
      errs.push_back(r.norm());
    }
  }
}

double Estimator::rmseReprojectionError(const int cam_idx) {
  std::vector<double> errs;
  reprojectionErrors(cam_idx, errs);
  return rmse(errs);
}

double Estimator::meanReprojectionError(const int cam_idx) {
  std::vector<double> errs;
  reprojectionErrors(cam_idx, errs);
  return mean(errs);
}

double Estimator::varianceReprojectionError(const int cam_idx) {
  std::vector<double> errs;
  reprojectionErrors(cam_idx, errs);
  return var(errs);
}

void Estimator::meanBiases(vec3_t &gyr_mu,
                           vec3_t &acc_mu,
                           vec3_t &gyr_stddev,
                           vec3_t &acc_stddev) {
  std::vector<double> vx;
  std::vector<double> vy;
  std::vector<double> vz;

  std::vector<double> bg_x;
  std::vector<double> bg_y;
  std::vector<double> bg_z;

  std::vector<double> ba_x;
  std::vector<double> ba_y;
  std::vector<double> ba_z;

  double N = speed_bias_param_ids_.size();
  for (const auto &param_id : speed_bias_param_ids_) {
    const auto sb_param = sb_blocks_[param_id];
    const vecx_t sb = sb_param->estimate();

    const vec3_t v = sb.head(3);
    vx.push_back(v(0));
    vy.push_back(v(1));
    vz.push_back(v(2));

    const vec3_t bg = sb.segment<3>(3);
    bg_x.push_back(bg(0));
    bg_y.push_back(bg(1));
    bg_z.push_back(bg(2));

    const vec3_t ba = sb.segment<3>(6);
    ba_x.push_back(ba(0));
    ba_y.push_back(ba(1));
    ba_z.push_back(ba(2));
  }

  auto sum = [](const std::vector<double> &vals) {
    double retval = 0;
    for (auto &x : vals) {
      retval += x;
    }
    return retval;
  };

  auto stddev = [](const double mu, const std::vector<double> &vals) {
    double sse = 0.0;
    for (auto &x : vals) {
      sse += pow(x - mu, 2);
    }
    double N = vals.size();
    double variance = sse / (N - 1);
    return sqrt(variance);
  };

  gyr_mu(0) = sum(bg_x) / N;
  gyr_mu(1) = sum(bg_y) / N;
  gyr_mu(2) = sum(bg_z) / N;

  acc_mu(0) = sum(ba_x) / N;
  acc_mu(1) = sum(ba_y) / N;
  acc_mu(2) = sum(ba_z) / N;

  gyr_stddev(0) = stddev(gyr_mu(0), bg_x);
  gyr_stddev(1) = stddev(gyr_mu(1), bg_y);
  gyr_stddev(2) = stddev(gyr_mu(2), bg_z);

  acc_stddev(0) = stddev(acc_mu(0), ba_x);
  acc_stddev(1) = stddev(acc_mu(1), ba_y);
  acc_stddev(2) = stddev(acc_mu(2), ba_z);
}

void Estimator::saveEstimates() {
  // Save camera params estimate
  {
    auto csv_file = cam0_file_;
    auto cam_idx = 0;
    vecx_t data = getCameraParameterEstimate(cam_idx);
    fprintf(csv_file, "%zu,", frame_index_);
    for (long i = 0; i < data.size(); i++) {
      fprintf(csv_file, "%f", data[i]);
      if ((i + 1) != data.size()) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }
  {
    auto csv_file = cam1_file_;
    auto cam_idx = 1;
    vecx_t data = getCameraParameterEstimate(cam_idx);
    fprintf(csv_file, "%zu,", frame_index_);
    for (long i = 0; i < data.size(); i++) {
      fprintf(csv_file, "%f", data[i]);
      if ((i + 1) != data.size()) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }

  // Save sensor-camera extrinsics estimate
  {
    const auto csv_file = T_SC0_file_;
    const auto cam_idx = 0;
    const auto pose = getSensorCameraPoseEstimate(cam_idx);
    const auto pos = tf_trans(pose);
    const auto rot = tf_quat(pose);

    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f,%f,%f,%f", rot.w(), rot.x(), rot.y(), rot.z());
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", pos(0), pos(1), pos(2));
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }
  {
    const auto csv_file = T_SC1_file_;
    const auto cam_idx = 1;
    const auto pose = getSensorCameraPoseEstimate(cam_idx);
    const auto pos = tf_trans(pose);
    const auto rot = tf_quat(pose);

    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f,%f,%f,%f", rot.w(), rot.x(), rot.y(), rot.z());
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", pos(0), pos(1), pos(2));
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }

  // Save fiducial pose estimate
  {
    const auto csv_file = T_WF_file_;
    const auto pose = getFiducialPoseEstimate();
    const auto pos = tf_trans(pose);
    const auto rot = tf_quat(pose);

    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f,%f,%f,%f", rot.w(), rot.x(), rot.y(), rot.z());
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", pos(0), pos(1), pos(2));
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }

  // Save time delay estimate
  {
    const auto csv_file = td_file_;
    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f", getTimeDelayEstimate());
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }

  // Save sensor pose estimate
  {
    const auto csv_file = T_WS_file_;
    const auto block = T_WS_blocks_.rbegin()->second;
    const auto pose = block->estimate().T();
    const auto pos = tf_trans(pose);
    const auto rot = tf_quat(pose);
    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f,%f,%f,%f", rot.w(), rot.x(), rot.y(), rot.z());
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", pos(0), pos(1), pos(2));
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }

  // Save speed bias estimate
  {
    const auto csv_file = sb_file_;
    const auto block = sb_blocks_.rbegin()->second;
    const auto sb = block->estimate();
    fprintf(csv_file, "%zu,", frame_index_);
    fprintf(csv_file, "%f,%f,%f", sb[0], sb[1], sb[2]);
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", sb[3], sb[4], sb[5]);
    fprintf(csv_file, ",");
    fprintf(csv_file, "%f,%f,%f", sb[6], sb[7], sb[8]);
    fprintf(csv_file, "\n");
    fflush(csv_file);
  }
}

void Estimator::saveDetected() {
  fprintf(grids_file_, "%zu,", frame_index_);
  fprintf(grids_file_, "%d,", grids_[0].nb_detections);
  fprintf(grids_file_, "%d", grids_[1].nb_detections);
  fprintf(grids_file_, "\n");
  fflush(grids_file_);
}

void Estimator::saveOptSummary() {
  fprintf(opt_file_, "%zu,", frame_index_);
  fprintf(opt_file_, "%f,", problem_->summary.initial_cost);
  fprintf(opt_file_, "%f,", problem_->summary.final_cost);
  fprintf(opt_file_, "%f,", problem_->summary.total_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.preprocessor_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.minimizer_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.postprocessor_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.linear_solver_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.residual_evaluation_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.jacobian_evaluation_time_in_seconds);
  fprintf(opt_file_, "%f,", problem_->summary.inner_iteration_time_in_seconds);
  fprintf(opt_file_, "%d,", problem_->summary.num_threads_given);
  fprintf(opt_file_, "%d", problem_->summary.num_threads_used);
  fprintf(opt_file_, "\n");
  fflush(opt_file_);
}

void Estimator::saveCosts() {
  // Evaluate residuals
  std::vector<double> imu_residuals;
  std::vector<double> reproj_residuals;
  evalResiduals(imu_residuals, reproj_residuals);

  // Calculate cost: cost = 1/2 * r' * r
  const Eigen::Map<Eigen::VectorXd> imu_r{imu_residuals.data(), (long) imu_residuals.size()};
  const Eigen::Map<Eigen::VectorXd> reproj_r{reproj_residuals.data(), (long) reproj_residuals.size()};
  const auto imu_cost = (0.5 * (imu_r.transpose() * imu_r))(0);
  const auto reproj_cost = (0.5 * (reproj_r.transpose() * reproj_r))(0);

  // Save costs to file
  fprintf(costs_file, "%zu,", frame_index_);
  fprintf(costs_file, "%f,", imu_cost);
  fprintf(costs_file, "%f,", reproj_cost);
  fprintf(costs_file, "\n");
  fflush(costs_file);

  // Lambda function to analuze residuals
  auto analyze_residuals = [](const std::vector<double> &residuals,
                              const size_t r_size,
                              double *r_low,
                              double *r_high,
                              double *r_mean) {
    size_t idx = 0;
    if (residuals.size() == 0) {
      return;
    }

    // Cost of each error term at timestep k
    size_t cost_idx = 0;
    vecx_t costs{residuals.size() / r_size};
    while (idx <= (residuals.size() - r_size)) {
      vecx_t r{r_size};
      for (size_t i = 0; i < r_size; i++) {
        r(i) = residuals.at(idx);
        idx++;
      }

      costs(cost_idx) = 0.5 * r.transpose() * r;
      cost_idx++;
    }
    *r_low = costs.minCoeff();
    *r_high = costs.maxCoeff();
    *r_mean = costs.mean();
  };

  // IMU residuals file
  {
    const size_t r_size = 15;
    double r_low = 0.0;
    double r_high = 0.0;
    double r_mean = 0.0;
    analyze_residuals(imu_residuals, r_size, &r_low, &r_high, &r_mean);
    fprintf(imu_residuals_file, "%zu,", frame_index_);
    fprintf(imu_residuals_file, "%f,", r_low);
    fprintf(imu_residuals_file, "%f,", r_high);
    fprintf(imu_residuals_file, "%f", r_mean);
    fprintf(imu_residuals_file, "\n");
    fflush(imu_residuals_file);
  }

  // Vision residuals file
  {
    const size_t r_size = 2;
    double r_low = 0.0;
    double r_high = 0.0;
    double r_mean = 0.0;
    analyze_residuals(reproj_residuals, r_size, &r_low, &r_high, &r_mean);
    fprintf(vision_residuals_file, "%zu,", frame_index_);
    fprintf(vision_residuals_file, "%f,", r_low);
    fprintf(vision_residuals_file, "%f,", r_high);
    fprintf(vision_residuals_file, "%f", r_mean);
    fprintf(vision_residuals_file, "\n");
    fflush(vision_residuals_file);
  }

  // // Reprojection errors file
  // {
  //   const size_t r_size = 2;
  //   double r_low = 0.0;
  //   double r_high = 0.0;
  //   double r_mean = 0.0;
  //   analyze_residuals(reproj_errors1, r_size, &r_low, &r_high, &r_mean);
  //   fprintf(reproj_errors_file, "%zu,", frame_index);
  //   fprintf(reproj_errors_file, "%f,", r_low);
  //   fprintf(reproj_errors_file, "%f,", r_high);
  //   fprintf(reproj_errors_file, "%f", r_mean);
  //   fprintf(reproj_errors_file, "\n");
  //   fflush(reproj_errors_file);
  // }
}

void Estimator::saveResults(const std::string &results_path) {
  LOG_INFO("**Results saved to [%s]**", results_path.c_str());
  const std::string tab = "  ";

  // Open file to save
  std::ofstream outfile(results_path);
  if (outfile.is_open() == false) {
    FATAL("Failed to create file at [%s]!", results_path.c_str());
  }

  // Save calibration results
  outfile << "calib_results:" << std::endl;
  // -- Camera reprojection errors
  for (size_t i = 0; i < nb_cams(); i++) {
    auto cam_name = "cam" + std::to_string(i);
    auto field = cam_name + "_" + "rmse_reproj_error: ";
    auto rmse = rmseReprojectionError(i);
    outfile << tab << field << rmse << std::endl;
  }
  for (size_t i = 0; i < nb_cams(); i++) {
    auto cam_name = "cam" + std::to_string(i);
    auto field = cam_name + "_" + "mean_reproj_error: ";
    auto mean_err = meanReprojectionError(i);
    outfile << tab << field << mean_err << std::endl;
  }
  outfile << std::endl;
  // -- Mean biases and stddev
  vec3_t gyro_mu, gyro_stddev;
  vec3_t accel_mu, accel_stddev;
  meanBiases(gyro_mu, accel_mu, gyro_stddev, accel_stddev);
  outfile << tab << "gyro_bias_mean: " << vec2str(gyro_mu) << std::endl;
  outfile << tab << "gyro_bias_stddev: " << vec2str(gyro_stddev) << std::endl;
  outfile << tab << "accel_bias_mean: " << vec2str(accel_mu) << std::endl;
  outfile << tab << "accel_bias_stddev: " << vec2str(accel_stddev) << std::endl;
  outfile << std::endl;
  // -- Number of residual blocks
  const auto nb_param_blocks = std::to_string(batch_problem_->problem_->NumParameterBlocks());
  const auto nb_residual_blocks = std::to_string(batch_problem_->problem_->NumResidualBlocks());
  const auto nb_imu_errs = std::to_string(residual_collection_.imu_errors.size());
  const auto nb_reproj_errs = std::to_string(residual_collection_.reproj_errors.size());
  outfile << tab << "nb_param_blocks: " << nb_param_blocks << std::endl;
  outfile << tab << "nb_residual_blocks: " << nb_residual_blocks << std::endl;
  outfile << tab << "nb_imu_errors: " << nb_imu_errs << std::endl;
  outfile << tab << "nb_reproj_errors: " << nb_reproj_errs << std::endl;
  outfile << std::endl;
  outfile << std::endl;

  // Save camera parameters
  // clang-format off
  for (size_t i = 0; i < nb_cams(); i++) {
    const auto cam_params = getCameraParameterEstimate(i);
    const vec2_t resolution{cameras_[i].imageWidth(), cameras_[i].imageHeight()};
    const vec4_t intrinsics = cam_params.head(4);
    const vec4_t distortion = cam_params.tail(4);

    outfile << "cam" + std::to_string(i) + ":" << std::endl;
    outfile << tab << "resolution: " << vec2str(resolution) << std::endl;
    outfile << tab << "proj_model: " << "\"pinhole\"" << std::endl;
    outfile << tab << "dist_model: " << "\"radtan4\"" << std::endl;
    outfile << tab << "proj_params: " << vec2str(intrinsics) << std::endl;
    outfile << tab << "dist_params: " << vec2str(distortion) << std::endl;
    outfile << std::endl;
  }
  // clang-format on

  // Save imu parameters
  outfile << "imu0:" << std::endl;
  outfile << tab << "a_max: " << imu_params_.a_max << std::endl;
  outfile << tab << "g_max: " << imu_params_.g_max << std::endl;
  outfile << tab << "sigma_g_c: " << imu_params_.sigma_g_c << std::endl;
  outfile << tab << "sigma_a_c: " << imu_params_.sigma_a_c << std::endl;
  outfile << tab << "sigma_gw_c: " << imu_params_.sigma_gw_c << std::endl;
  outfile << tab << "sigma_aw_c: " << imu_params_.sigma_aw_c << std::endl;
  outfile << tab << "sigma_bg: " << imu_params_.sigma_bg << std::endl;
  outfile << tab << "sigma_ba: " << imu_params_.sigma_ba << std::endl;
  outfile << tab << "tau: " << imu_params_.tau << std::endl;
  outfile << tab << "g: " << imu_params_.g << std::endl;
  outfile << tab << "a0: " << vec2str(imu_params_.a0) << std::endl;
  outfile << tab << "update_rate: " << imu_params_.rate << std::endl;
  outfile << tab << "time_delay: " << time_delay_block_.get()->estimate()
           << std::endl;
  outfile << std::endl;

  // Save sensor-camera extrinsics
  for (size_t cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
    const std::string imu_str = "imu0";
    const std::string cam_str = "cam" + std::to_string(cam_idx);

    // T_SCi
    {
      const std::string tf_str = "T_" + imu_str + "_" + cam_str;
      const mat4_t extrinsic = T_SC_blocks_.at(cam_idx).get()->estimate().T();
      outfile << tf_str << ":" << std::endl;
      outfile << tab << "rows: 4" << std::endl;
      outfile << tab << "cols: 4" << std::endl;
      outfile << tab << "data: [" << std::endl;
      outfile << mat2str(extrinsic, tab + tab) << std::endl;
      outfile << tab << "]" << std::endl;
      outfile << std::endl;
    }

    // T_CiS
    {
      const std::string tf_str = "T_" + cam_str + "_" + imu_str;
      const mat4_t extrinsic = T_SC_blocks_.at(cam_idx).get()->estimate().T();
      const mat4_t extrinsic_inv = extrinsic.inverse();
      outfile << tf_str << ":" << std::endl;
      outfile << tab << "rows: 4" << std::endl;
      outfile << tab << "cols: 4" << std::endl;
      outfile << tab << "data: [" << std::endl;
      outfile << mat2str(extrinsic_inv, tab + tab) << std::endl;
      outfile << tab << "]" << std::endl;
      outfile << std::endl;
    }
  }

  // Save camera-camera extrinsics
  {
    const mat4_t T_SC0 = T_SC_blocks_.at(0).get()->estimate().T();
    const mat4_t T_SC1 = T_SC_blocks_.at(1).get()->estimate().T();
    const mat4_t T_C0C1 = T_SC0.inverse() * T_SC1;

    outfile << "T_cam0_cam1:" << std::endl;
    outfile << tab << "rows: 4" << std::endl;
    outfile << tab << "cols: 4" << std::endl;
    outfile << tab << "data: [" << std::endl;
    outfile << mat2str(T_C0C1, tab + tab) << std::endl;
    outfile << tab << "]" << std::endl;
    outfile << std::endl;

    outfile << "T_cam1_cam0:" << std::endl;
    outfile << tab << "rows: 4" << std::endl;
    outfile << tab << "cols: 4" << std::endl;
    outfile << tab << "data: [" << std::endl;
    outfile << mat2str(T_C0C1.inverse(), tab + tab) << std::endl;
    outfile << tab << "]" << std::endl;
    outfile << std::endl;
  }
}

}  // namespace yac
