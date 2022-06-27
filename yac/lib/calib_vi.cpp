#include "calib_vi.hpp"

namespace yac {

// VISUAL INERTIAL CALIBRATION VIEW ////////////////////////////////////////////

calib_vi_view_t::calib_vi_view_t(const timestamp_t ts_,
                                 const CamIdx2Grids &grids_,
                                 const mat4_t &T_WS_,
                                 const vec_t<9> &sb_,
                                 CamIdx2Geometry &cam_geoms_,
                                 CamIdx2Parameters &cam_params_,
                                 CamIdx2Extrinsics &cam_exts_,
                                 std::shared_ptr<extrinsics_t> imu_exts_,
                                 std::shared_ptr<fiducial_t> fiducial_,
                                 std::shared_ptr<ceres::Problem> problem_,
                                 std::shared_ptr<calib_loss_t> vision_loss_,
                                 std::shared_ptr<calib_loss_t> imu_loss_,
                                 PoseLocalParameterization *pose_plus)
    : ts{ts_}, grids{grids_}, pose{ts_, T_WS_}, sb{ts_, sb_},
      cam_geoms{cam_geoms_}, cam_params{cam_params_}, cam_exts{cam_exts_},
      imu_exts{imu_exts_}, fiducial{fiducial_}, problem{problem_},
      vision_loss{vision_loss_}, imu_loss{imu_loss_} {
  // Add pose to problem
  problem->AddParameterBlock(pose.param.data(), 7);
  problem->SetParameterization(pose.param.data(), pose_plus);

  // Add speed and biases to problem
  problem->AddParameterBlock(sb.param.data(), 9);

  // Add fiducial errors
  const mat2_t covar = I(2);

  for (const auto &[cam_idx, grid] : grids) {
    // Get AprilGrid measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t pts;
    grid.get_measurements(tag_ids, corner_idxs, kps, pts);

    // Add residuals to problem
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_idxs[i];
      const vec2_t z = kps[i];
      const vec3_t r_FFi = pts[i];

      // Form residual
      auto res =
          std::make_shared<fiducial_residual_t>(ts,
                                                cam_geoms[cam_idx].get(),
                                                cam_params[cam_idx].get(),
                                                cam_exts[cam_idx].get(),
                                                imu_exts.get(),
                                                fiducial.get(),
                                                &pose,
                                                tag_id,
                                                corner_idx,
                                                r_FFi,
                                                z,
                                                covar);
      fiducial_residuals[cam_idx].emplace_back(res);

      // Add to problem
      auto res_id =
          problem->AddResidualBlock(fiducial_residuals[cam_idx].back().get(),
                                    vision_loss.get(),
                                    fiducial->param.data(),
                                    pose.param.data(),
                                    imu_exts->param.data(),
                                    cam_exts[cam_idx]->param.data(),
                                    cam_params[cam_idx]->param.data());
      fiducial_residual_ids[cam_idx].push_back(res_id);
    }
  }
}

calib_vi_view_t::~calib_vi_view_t() {
  // Remove all residual blocks in ceres::Problem associated with this pose
  if (problem->HasParameterBlock(pose.param.data())) {
    problem->RemoveParameterBlock(pose.param.data());
  }
  if (problem->HasParameterBlock(sb.param.data())) {
    problem->RemoveParameterBlock(sb.param.data());
  }
}

std::vector<int> calib_vi_view_t::get_camera_indices() const {
  std::vector<int> cam_idxs;
  for (const auto &[cam_idx, params] : cam_params) {
    UNUSED(params);
    cam_idxs.push_back(cam_idx);
  }

  return cam_idxs;
}

std::vector<real_t>
calib_vi_view_t::get_reproj_errors(const int cam_idx) const {
  std::vector<real_t> cam_errors;

  if (fiducial_residuals.count(cam_idx) == 0) {
    return cam_errors;
  }

  for (const auto &res_fn : fiducial_residuals.at(cam_idx)) {
    real_t e;
    if (res_fn->get_reproj_error(e) == 0) {
      cam_errors.push_back(e);
    }
  }

  return cam_errors;
}

std::map<int, std::vector<real_t>> calib_vi_view_t::get_reproj_errors() const {
  std::map<int, std::vector<real_t>> cam_errors;

  for (const auto &[cam_idx, res_fns] : fiducial_residuals) {
    for (const auto &res : res_fns) {
      real_t e;
      if (res->get_reproj_error(e) == 0) {
        cam_errors[cam_idx].push_back(e);
      }
    }
  }

  return cam_errors;
}

int calib_vi_view_t::filter_view(const real_t outlier_threshold) {
  // Get reprojection errors
  const auto reproj_errors = get_reproj_errors();
  std::vector<real_t> reproj_errors_all;
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    extend(reproj_errors_all, cam_errors);
  }

  // Calculate threshold
  const auto error_stddev = stddev(reproj_errors_all);
  const auto threshold = outlier_threshold * error_stddev;

  int nb_inliers = 0;
  int nb_outliers = 0;
  for (const auto cam_idx : get_camera_indices()) {
    auto &res_fns = fiducial_residuals[cam_idx];
    auto &res_ids = fiducial_residual_ids[cam_idx];
    auto res_it = res_fns.begin();
    auto id_it = res_ids.begin();

    while (res_it != res_fns.end()) {
      auto &fiducial_residual = *res_it;
      auto &fiducial_id = *id_it;

      vec2_t r;
      if (fiducial_residual->get_residual(r) != 0) {
        ++res_it;
        ++id_it;
        continue;
      }

      if (r.x() > threshold || r.y() > threshold) {
        problem->RemoveResidualBlock(fiducial_id);
        res_it = res_fns.erase(res_it);
        id_it = res_ids.erase(id_it);
        nb_outliers++;
      } else {
        ++res_it;
        ++id_it;
        nb_inliers++;
      }
    }
  }

  return nb_outliers;
}

void calib_vi_view_t::form_imu_residual(const imu_params_t &imu_params,
                                        const imu_data_t imu_buf,
                                        pose_t *pose_j,
                                        sb_params_t *sb_j) {
  assert(imu_params.rate > 0);
  assert(fltcmp(imu_params.sigma_a_c, 0.0) != 0);
  assert(fltcmp(imu_params.sigma_g_c, 0.0) != 0);
  assert(fltcmp(imu_params.sigma_aw_c, 0.0) != 0);
  assert(fltcmp(imu_params.sigma_gw_c, 0.0) != 0);

  // Pre-check IMU measurements
  // const auto ts_i = pose.ts;
  // const auto ts_j = pose_j->ts;
  // if (imu_buf.timestamps.front() > ts_i) {
  //   LOG_ERROR("imu_buf.timestamps.front() > ts_i");
  //   LOG_ERROR("imu_buf.timestamps.front(): %ld", imu_buf.timestamps.front());
  //   LOG_ERROR("ts_i:                       %ld", ts_i);
  //   FATAL("imu_data.timestamps.front() > ts_i!");
  // }
  // if (imu_buf.timestamps.back() < ts_j) {
  //   LOG_ERROR("imu_buf.timestamps.back() < ts_j");
  //   LOG_ERROR("imu_buf.timestamps.back(): %ld", imu_buf.timestamps.back());
  //   LOG_ERROR("ts_j:                      %ld", ts_j);
  //   FATAL("imu_buf.timestamps.back() < ts_j");
  // }

  // Form IMU residual
  imu_residual = std::make_shared<imu_residual_t>(imu_params,
                                                  imu_buf,
                                                  &pose,
                                                  &sb,
                                                  pose_j,
                                                  sb_j);
  imu_residual_id = problem->AddResidualBlock(imu_residual.get(),
                                              imu_loss.get(),
                                              pose.param.data(),
                                              sb.param.data(),
                                              pose_j->param.data(),
                                              sb_j->param.data());
}

ceres::ResidualBlockId
calib_vi_view_t::marginalize(marg_residual_t *marg_residual) {
  // Mark pose T_WS and speed and biases sb to be marginalized
  pose.marginalize = true;
  sb.marginalize = true;

  // Transfer residual ownership to marginalization residual
  marg_residual->add(imu_residual);
  for (auto &[cam_idx, residuals] : fiducial_residuals) {
    for (auto &residual : residuals) {
      marg_residual->add(residual);
    }
  }
  const auto res_id = marg_residual->marginalize(problem.get());

  // Clear residuals
  fiducial_residual_ids.clear();
  fiducial_residuals.clear();

  return res_id;
}

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

calib_vi_t::calib_vi_t(const calib_target_t &calib_target_)
    : calib_target{calib_target_} {
  // Ceres-Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = std::make_shared<ceres::Problem>(prob_options);

  // Create loss functions
  if (enable_vision_loss_fn) {
    vision_loss = _create_loss_fn(vision_loss_fn_type, vision_loss_fn_param);
  }
  if (enable_imu_loss_fn) {
    imu_loss = _create_loss_fn(imu_loss_fn_type, imu_loss_fn_param);
  }

  // AprilGrid detector
  detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);
}

calib_vi_t::calib_vi_t(const std::string &config_path) {
  // Ceres-Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = std::make_shared<ceres::Problem>(prob_options);

  // Load configuration
  config_t config{config_path};
  // -- Parse settings
  // clang-format off
  parse(config, "settings.verbose", verbose, true);
  parse(config, "settings.max_num_threads", max_num_threads, true);
  parse(config, "settings.max_iter", max_iter, true);
  parse(config, "settings.enable_outlier_rejection", enable_outlier_rejection, true);
  parse(config, "settings.enable_marginalization", enable_marginalization, true);
  parse(config, "settings.enable_vision_loss_fn", enable_vision_loss_fn, true);
  parse(config, "settings.vision_loss_fn_type", vision_loss_fn_type, true);
  parse(config, "settings.vision_loss_fn_param", vision_loss_fn_param, true);
  parse(config, "settings.enable_imu_loss_fn", enable_imu_loss_fn, true);
  parse(config, "settings.imu_loss_fn_type", imu_loss_fn_type, true);
  parse(config, "settings.imu_loss_fn_param", imu_loss_fn_param, true);
  parse(config, "settings.outlier_threshold", outlier_threshold, true);
  parse(config, "settings.window_size", window_size, true);
  // clang-format on
  // -- Parse calibration target
  if (calib_target.load(config_path, "calib_target") != 0) {
    FATAL("Failed to parse calib_target in [%s]!", config_path.c_str());
  }
  // -- Parse imu settings
  parse(config, "imu0.rate", imu_params.rate);
  parse(config, "imu0.sigma_g_c", imu_params.sigma_g_c);
  parse(config, "imu0.sigma_a_c", imu_params.sigma_a_c);
  parse(config, "imu0.sigma_gw_c", imu_params.sigma_gw_c);
  parse(config, "imu0.sigma_aw_c", imu_params.sigma_aw_c);
  parse(config, "imu0.sigma_bg", imu_params.sigma_bg, true);
  parse(config, "imu0.sigma_ba", imu_params.sigma_ba, true);
  parse(config, "imu0.g", imu_params.g);
  add_imu(imu_params, I(4), 0.0, false, false);

  // -- Parse camera settings
  for (int cam_idx = 0; cam_idx < 100; cam_idx++) {
    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    if (yaml_has_key(config, cam_str) == 0) {
      continue;
    }

    // Add camera intrinsics + extrinsics
    veci2_t cam_res;
    std::string proj_model;
    std::string dist_model;
    vecx_t proj_params;
    vecx_t dist_params;
    mat4_t T_C0Ci = I(4);
    parse(config, cam_str + ".resolution", cam_res);
    parse(config, cam_str + ".proj_model", proj_model);
    parse(config, cam_str + ".dist_model", dist_model);
    parse(config, cam_str + ".proj_params", proj_params);
    parse(config, cam_str + ".dist_params", dist_params);
    if (cam_idx != 0) {
      parse(config, "T_cam0_" + cam_str, T_C0Ci);
    }
    add_camera(cam_idx,
               cam_res.data(),
               proj_model,
               dist_model,
               proj_params,
               dist_params,
               T_C0Ci,
               true,
               true);
  }
  if (cam_params.size() == 0) {
    FATAL("Failed to parse any camera parameters...");
  }

  // Create loss functions
  if (enable_vision_loss_fn) {
    vision_loss = _create_loss_fn(vision_loss_fn_type, vision_loss_fn_param);
  }
  if (enable_imu_loss_fn) {
    imu_loss = _create_loss_fn(imu_loss_fn_type, imu_loss_fn_param);
  }

  // AprilGrid detector
  detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);
}

void calib_vi_t::add_imu(const imu_params_t &imu_params_,
                         const mat4_t &T_BS,
                         const double td,
                         const bool fix_extrinsics,
                         const bool fix_time_delay) {
  assert(imu_params_.rate > 0);
  assert(fltcmp(imu_params_.sigma_a_c, 0.0) != 0);
  assert(fltcmp(imu_params_.sigma_g_c, 0.0) != 0);
  assert(fltcmp(imu_params_.sigma_aw_c, 0.0) != 0);
  assert(fltcmp(imu_params_.sigma_gw_c, 0.0) != 0);

  // Imu parameters
  imu_params = imu_params_;

  // Imu extrinsics
  imu_exts = std::make_shared<extrinsics_t>(T_BS);
  problem->AddParameterBlock(imu_exts->param.data(), 7);
  problem->SetParameterization(imu_exts->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(imu_exts->param.data());
    imu_exts->fixed = true;
  }

  // Imu time delay
  time_delay = std::make_shared<time_delay_t>(td);
  problem->AddParameterBlock(time_delay->param.data(), 1);
  if (fix_time_delay) {
    problem->SetParameterBlockConstant(time_delay->param.data());
    time_delay->fixed = true;
  }
}

void calib_vi_t::add_camera(const int cam_idx,
                            const int cam_res[2],
                            const std::string &proj_model,
                            const std::string &dist_model,
                            const vecx_t &proj_params,
                            const vecx_t &dist_params,
                            const mat4_t &T_BCi,
                            const bool fix_params,
                            const bool fix_extrinsics) {
  // Camera parameters
  cam_params[cam_idx] = std::make_shared<camera_params_t>(cam_idx,
                                                          cam_res,
                                                          proj_model,
                                                          dist_model,
                                                          proj_params,
                                                          dist_params);
  if (fix_params) {
    auto data_ptr = cam_params[cam_idx]->param.data();
    auto block_size = cam_params[cam_idx]->global_size;
    cam_params[cam_idx]->fixed = true;
    problem->AddParameterBlock(data_ptr, block_size);
    problem->SetParameterBlockConstant(data_ptr);
  }

  // Camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    cam_geoms[cam_idx] = std::make_shared<pinhole_radtan4_t>();
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    cam_geoms[cam_idx] = std::make_shared<pinhole_equi4_t>();
  } else {
    FATAL("Invalid [%s-%s] camera model!",
          proj_model.c_str(),
          dist_model.c_str());
  }

  // Camera extrinsics
  cam_exts[cam_idx] = std::make_shared<extrinsics_t>(T_BCi);
  problem->AddParameterBlock(cam_exts[cam_idx]->param.data(), 7);
  problem->SetParameterization(cam_exts[cam_idx]->param.data(), &pose_plus);
  if (fix_extrinsics) {
    problem->SetParameterBlockConstant(cam_exts[cam_idx]->param.data());
    cam_exts[cam_idx]->fixed = true;
  }
}

int calib_vi_t::nb_cams() const { return cam_params.size(); }

int calib_vi_t::nb_views() const { return calib_views.size(); }

std::vector<int> calib_vi_t::get_camera_indices() const {
  std::vector<int> cam_idxs;
  for (const auto &[cam_idx, cam] : cam_params) {
    cam_idxs.push_back(cam_idx);
  }
  return cam_idxs;
}

real_t calib_vi_t::get_camera_rate() const {
  // Pre-check
  if (calib_views.size() < 2) {
    FATAL("calib_views.size() < 2");
  }

  // Calculate time difference in seconds between views
  std::vector<real_t> time_diff;
  for (size_t k = 1; k < calib_views.size(); k++) {
    auto ts_km1 = calib_views[k - 1]->ts;
    auto ts_k = calib_views[k]->ts;
    time_diff.push_back(ts2sec(ts_k - ts_km1));
  }

  // Return camera rate in Hz
  return 1.0 / median(time_diff);
}

real_t calib_vi_t::get_imu_rate() const {
  // Pre-check
  if (imu_buf.size() < 2) {
    FATAL("imu_buf.size() < 2");
  }

  // Calculate time difference in seconds between views
  std::vector<real_t> time_diff;
  for (size_t k = 1; k < imu_buf.size(); k++) {
    auto ts_km1 = imu_buf.timestamps[k - 1];
    auto ts_k = imu_buf.timestamps[k];
    time_diff.push_back(ts2sec(ts_k - ts_km1));
  }

  // Return camera rate in Hz
  return 1.0 / median(time_diff);
}

veci2_t calib_vi_t::get_camera_resolution(const int cam_idx) const {
  auto cam_res = cam_params.at(cam_idx)->resolution;
  return veci2_t{cam_res[0], cam_res[1]};
}

vecx_t calib_vi_t::get_camera_params(const int cam_idx) const {
  return cam_params.at(cam_idx)->param;
}

mat4_t calib_vi_t::get_camera_extrinsics(const int cam_idx) const {
  return cam_exts.at(cam_idx)->tf();
}

mat4_t calib_vi_t::get_imu_extrinsics() const { return imu_exts->tf(); }

real_t calib_vi_t::get_imucam_time_delay() const {
  return time_delay->param[0];
}

mat4_t calib_vi_t::get_fiducial_pose() const { return fiducial->estimate(); }

mat4_t calib_vi_t::get_imu_pose(const timestamp_t ts) const {
  for (const auto &view : calib_views) {
    if (view->ts == ts) {
      return view->pose.tf();
    }
  }

  FATAL("No IMU pose at [%ld]!", ts);
  return I(4);
}

mat4_t calib_vi_t::get_imu_pose() const {
  return calib_views.back()->pose.tf();
}

param_t *calib_vi_t::get_pose_param(const timestamp_t ts) const {
  for (auto view : calib_views) {
    if (view->ts == ts) {
      return &view->pose;
      break;
    }
  }

  return nullptr;
}

param_t *calib_vi_t::get_sb_param(const timestamp_t ts) const {
  for (auto view : calib_views) {
    if (view->ts == ts) {
      return &view->sb;
      break;
    }
  }

  return nullptr;
}

std::map<int, std::vector<real_t>> calib_vi_t::get_reproj_errors() const {
  std::map<int, std::vector<real_t>> errors;
  for (const auto &view : calib_views) {
    for (const auto cam_idx : view->get_camera_indices()) {
      extend(errors[cam_idx], view->get_reproj_errors(cam_idx));
    }
  }
  return errors;
}

mat4_t calib_vi_t::estimate_sensor_pose(const CamIdx2Grids &grids) {
  assert(grids.size() > 0);
  assert(initialized);

  bool detected = false;
  for (const auto &[cam_idx, grid] : grids) {
    UNUSED(cam_idx);
    if (grid.detected) {
      detected = true;
    }
  }
  if (detected == false) {
    FATAL("No fiducials detected!");
  }

  for (const auto &[cam_idx, grid] : grids) {
    // Skip if not detected
    if (grid.detected == false) {
      continue;
    }

    // Estimate relative pose
    const camera_geometry_t *cam = cam_geoms.at(cam_idx).get();
    const veci2_t cam_res_vec = get_camera_resolution(cam_idx);
    const int cam_res[2] = {cam_res_vec.x(), cam_res_vec.y()};
    const vecx_t cam_params = get_camera_params(cam_idx);
    mat4_t T_CiF = I(4);
    if (grid.estimate(cam, cam_res, cam_params, T_CiF) != 0) {
      FATAL("Failed to estimate relative pose!");
    }

    // Infer current pose T_WS using T_CiF, T_BCi and T_WF
    const mat4_t T_FCi_k = T_CiF.inverse();
    const mat4_t T_BCi = get_camera_extrinsics(cam_idx);
    const mat4_t T_BS = get_imu_extrinsics();
    const mat4_t T_CiS = T_BCi.inverse() * T_BS;
    const mat4_t T_WF = get_fiducial_pose();
    const mat4_t T_WS_k = T_WF * T_FCi_k * T_CiS;
    return T_WS_k;
  }

  // Should never reach here
  FATAL("Implementation Error!");
  return zeros(4, 4);
}

void calib_vi_t::initialize(const CamIdx2Grids &grids, imu_data_t &imu_buf) {
  assert(grids.size() > 0);
  assert(grids.at(0).detected);

  // Estimate relative pose - T_C0F
  const camera_geometry_t *cam = cam_geoms.at(0).get();
  const int *cam_res = cam_params.at(0)->resolution;
  const vecx_t params = get_camera_params(0);
  mat4_t T_C0F;
  if (grids.at(0).estimate(cam, cam_res, params, T_C0F) != 0) {
    FATAL("Failed to estimate relative pose!");
    return;
  }

  // Estimate initial IMU attitude
  const mat3_t C_WS = imu_buf.initial_attitude();

  // Sensor pose - T_WS
  mat4_t T_WS = tf(C_WS, zeros(3, 1));

  // Fiducial pose - T_WF
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_BS = get_imu_extrinsics();
  const mat4_t T_SC0 = T_BS.inverse() * T_BC0;
  mat4_t T_WF = T_WS * T_SC0 * T_C0F;

  // Set:
  // 1. Fiducial target as origin (with r_WF (0, 0, 0))
  // 2. Calculate the sensor pose offset relative to target origin
  const vec3_t offset = -1.0 * tf_trans(T_WF);
  const mat3_t C_WF = tf_rot(T_WF);
  const vec3_t r_WF{0.0, 0.0, 0.0};
  T_WF = tf(C_WF, r_WF);
  T_WS = tf(C_WS, offset);

  // Set fiducial
  if (fiducial == nullptr) {
    fiducial = std::make_shared<fiducial_t>(T_WF);
    problem->AddParameterBlock(fiducial->param.data(), FIDUCIAL_PARAMS_SIZE);
    if (fiducial->param.size() == 7) {
      problem->SetParameterization(fiducial->param.data(), &pose_plus);
    }
  } else {
#if FIDUCIAL_PARAMS_SIZE == 2
    const vec3_t rpy = quat2euler(tf_quat(T_WF));
    fiducial->param = vec2_t{rpy.x(), rpy.y()};
    fiducial->T_WF = T_WF;
#elif FIDUCIAL_PARAMS_SIZE == 3
    fiducial->param = quat2euler(tf_quat(T_WF));
    fiducial->T_WF = T_WF;
#elif FIDUCIAL_PARAMS_SIZE == 7
    fiducial->set_tf(T_WF);
#endif
  }

  // Print to screen
  if (verbose) {
    printf("Initial estimates\n");
    print_matrix("T_WF", T_WF);
    print_matrix("T_WS", T_WS);
    print_matrix("T_BS", T_BS);
  }

  // First calibration view
  const timestamp_t ts = grids.at(0).timestamp;
  const vec_t<9> sb = zeros(9, 1);
  calib_views.push_back(std::make_shared<calib_vi_view_t>(ts,
                                                          grids,
                                                          T_WS,
                                                          sb,
                                                          cam_geoms,
                                                          cam_params,
                                                          cam_exts,
                                                          imu_exts,
                                                          fiducial,
                                                          problem,
                                                          vision_loss,
                                                          imu_loss,
                                                          &pose_plus));

  initialized = true;
  prev_grids = grids;
}

void calib_vi_t::add_view(const CamIdx2Grids &grids) {
  // Pre-check
  if (initialized == false) {
    return;
  }

  // Check aprilgrids, make sure there is atleast 1 detected grid
  bool grid_detected = false;
  timestamp_t grid_ts = 0;
  for (const auto &[cam_idx, grid] : grids) {
    if (grid.detected) {
      grid_ts = grid.timestamp;
      grid_detected = true;
      break;
    }
  }
  if (grid_detected == false) {
    return;
  }

  // Estimate current pose
  const mat4_t T_WS_k = estimate_sensor_pose(grids);

  // Get previous calibration view
  auto &view_km1 = calib_views.back();
  const timestamp_t ts_km1 = view_km1->ts;

  // Infer velocity from two poses T_WS_k and T_WS_km1
  const timestamp_t ts_k = grid_ts;
  const real_t dt = ((ts_k - ts_km1) * 1e-9);
  const mat4_t T_WS_km1 = view_km1->pose.tf();
  const vec3_t r_WS_km1 = tf_trans(T_WS_km1);
  const vec3_t r_WS_k = tf_trans(T_WS_k);
  const vec3_t v_WS_k = (r_WS_k - r_WS_km1) / dt;

  // Form current speed and biases vector sb_k
  const vec_t<9> sb_km1 = view_km1->sb.param;
  const vec3_t ba_km1 = sb_km1.segment<3>(3);
  const vec3_t bg_km1 = sb_km1.segment<3>(6);
  vec_t<9> sb_k;
  sb_k << v_WS_k, ba_km1, bg_km1;

  // Form new calibration view
  // Note: instead of using the propagated sensor pose T_WS_k using imu
  // measurements, we are estimating `T_WS_k` via vision. This is because
  // vision estimate is better in this case.
  calib_view_counter++;
  calib_views.push_back(std::make_shared<calib_vi_view_t>(ts_k,
                                                          grids,
                                                          T_WS_k,
                                                          sb_k,
                                                          cam_geoms,
                                                          cam_params,
                                                          cam_exts,
                                                          imu_exts,
                                                          fiducial,
                                                          problem,
                                                          vision_loss,
                                                          imu_loss,
                                                          &pose_plus));

  // Form imu factor between view km1 and k
  if (imu_buf.size() < 5) {
    FATAL("imu_buf.size() < 5");
  }
  auto view_k = calib_views.back();
  view_km1->form_imu_residual(imu_params, imu_buf, &view_k->pose, &view_k->sb);
  imu_buf.trim(view_k->pose.ts);

  prev_grids = grids;
}

bool calib_vi_t::add_measurement(const timestamp_t ts,
                                 const int cam_idx,
                                 const cv::Mat &cam_image) {
  std::lock_guard<std::mutex> guard(mtx);

  // Do not add vision data before first imu measurement
  if (imu_started == false) {
    return false;
  }

  // Add image to image buffer
  img_buf[cam_idx] = {ts, cam_image};

  // Make sure timestamps in in image buffer all the same
  bool ready = true;
  for (auto &[cam_idx, data] : img_buf) {
    const auto img_ts = data.first;
    if (ts > img_ts) {
      ready = false;
    }
  }
  if (ready == false) {
    return false;
  }

  // Detect AprilGrids
  prof.start("detection");
  if (fltcmp(img_scale, 1.0) != 0) {
    // Detect with downsampled images
    // -- Make a copy of down sampled images
    std::map<int, std::pair<timestamp_t, cv::Mat>> buffer;
    for (const auto &[cam_idx, data] : img_buf) {
      auto ts = data.first;
      auto img = data.second;
      cv::resize(img, img, cv::Size(), img_scale, img_scale);
      buffer[cam_idx] = {ts, img};
    }
    // -- Detect aprilgrids
    grid_buf = detector->detect(buffer);
    // -- Rescale the detected keypoints
    const auto tag_rows = calib_target.tag_rows;
    const auto tag_cols = calib_target.tag_cols;
    for (auto &[cam_idx, grid] : grid_buf) {
      for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
        if (grid.data(i, 0) > 0) {
          grid.data(i, 1) /= img_scale;
          grid.data(i, 2) /= img_scale;
        }
      }
    }
  } else {
    // Detect at full image resolution
    grid_buf = detector->detect(img_buf);
  }
  prof.stop("detection");

  return true;
}

void calib_vi_t::add_measurement(const int cam_idx, const aprilgrid_t &grid) {
  std::lock_guard<std::mutex> guard(mtx);

  // Do not add vision data before first imu measurement
  if (imu_started == false) {
    return;
  }

  grid_buf[cam_idx] = grid;
  vision_started = true;
}

void calib_vi_t::add_measurement(const timestamp_t imu_ts,
                                 const vec3_t &a_m,
                                 const vec3_t &w_m) {
  std::lock_guard<std::mutex> guard(mtx);

  // Add imu measuremrent
  imu_buf.add(imu_ts, a_m, w_m);
  imu_started = true;
  // if (imu_buf.time_span() > 1.0) {
  //   LOG_WARN("IMU buffer time span > 1 second?!");
  // }

  // Check if AprilGrid buffer is filled
  if (static_cast<int>(grid_buf.size()) != nb_cams()) {
    return;
  }

  // Copy AprilGrid buffer data
  CamIdx2Grids grids = grid_buf;
  grid_buf.clear();

  // Initialize T_WS and T_WF
  // Conditions:
  // - Aprilgrid was observed by cam0
  // - There are more than 2 IMU measurements
  timestamp_t det_ts = 0;
  auto detection_ok = [&](const CamIdx2Grids &data) {
    for (const auto &[cam_idx, grid] : data) {
      if (grid.detected) {
        det_ts = grid.timestamp;
        return true;
      }
    }
    return false;
  };
  if (initialized == false && imu_buf.size() > 2) {
    if (detection_ok(grids)) {
      initialize(grids, imu_buf);
      return;
    }
  }

  // Add new view
  add_view(grids);

  // Marginalize
  if (enable_marginalization && calib_views.size() > (size_t)window_size) {
    // Solve then marginalize
    prof.start("solve");
    ceres::Solver::Options options;
    options.max_num_iterations = max_iter;
    options.num_threads = max_num_threads;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);
    prof.stop("solve");
    // if (verbose) {
    //   std::cout << summary.BriefReport() << std::endl;
    // }

    // Calculate calib_info
    // const int rate = get_camera_rate() * 5;
    // if (calib_view_counter % rate == 0) {
    //   if (recover_calib_info(calib_info) == 0) {
    //     calib_info_ok = true;
    //   } else {
    //     calib_info_ok = false;
    //   }
    // }
    // if (recover_calib_info(calib_info) == 0) {
    //   calib_info_ok = true;
    // } else {
    //   calib_info_ok = false;
    // }

    // Marginalize oldest view
    prof.start("marginalize");
    marginalize();
    prof.stop("marginalize");
    running = true;
  }

  imu_started = true;
}

std::shared_ptr<calib_loss_t> calib_vi_t::_create_loss_fn(
    const std::string &loss_fn_type, const real_t loss_param) const {
  if (loss_fn_type == "BLAKE-ZISSERMAN") {
    return std::make_shared<BlakeZissermanLoss>((int)loss_param);
  } else if (loss_fn_type == "CAUCHY") {
    return std::make_shared<CauchyLoss>(loss_param);
  } else if (loss_fn_type == "HUBER") {
    return std::make_shared<HuberLoss>(loss_param);
  }

  FATAL("Unsupported loss function type [%s]!", loss_fn_type.c_str());
}

int calib_vi_t::recover_calib_covar(matx_t &calib_covar) const {
  // Recover calibration covariance
  // -- Setup covariance blocks to estimate
  auto T_BS = imu_exts->param.data();
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  covar_blocks.push_back({T_BS, T_BS});
  // -- Estimate covariance
  ::ceres::Covariance::Options options;
  ::ceres::Covariance covar_est(options);
  if (covar_est.Compute(covar_blocks, problem.get()) == false) {
    LOG_ERROR("Failed to estimate covariance!");
    LOG_ERROR("Maybe Hessian is not full rank?");
    LOG_ERROR("Num Views: %ld", calib_views.size());
    LOG_ERROR("Num Residual Blocks: %d", problem->NumResidualBlocks());
    LOG_ERROR("Num Parameter Blocks: %d", problem->NumParameterBlocks());
    LOG_ERROR("");
    for (const auto &view : calib_views) {
      LOG_ERROR("VIEW [%ld]", view->ts);
      for (const auto &cam_idx : view->get_camera_indices()) {
        LOG_ERROR("  Num cam%d fiducial residuals: %ld",
                  cam_idx,
                  view->fiducial_residuals[cam_idx].size());
      }
      LOG_ERROR("  Has IMU residual?: %s",
                (view->imu_residual != nullptr) ? "true" : "false");
      if (view->imu_residual) {
        LOG_ERROR("  Num IMU measurements: %ld",
                  view->imu_residual->imu_data_.size());
        std::cout << view->imu_residual->imu_data_ << std::endl;
      }
      LOG_ERROR("");
    }

    return -1;
  }

  // for (const auto &view : calib_views) {
  //   LOG_INFO("VIEW [%ld]", view->ts);
  //   for (const auto &cam_idx : view->get_camera_indices()) {
  //     LOG_INFO("  Num cam%d fiducial residuals: %ld",
  //              cam_idx,
  //              view->fiducial_residuals[cam_idx].size());
  //   }
  //   if (view->imu_residual) {
  //     LOG_INFO("  Num IMU measurements: %ld",
  //              view->imu_residual->imu_data_.size());
  //     std::cout << view->imu_residual->imu_data_ << std::endl;
  //   }
  //   LOG_INFO("");
  // }

  // -- Extract covariances sub-blocks
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_BS_covar;
  covar_est.GetCovarianceBlockInTangentSpace(T_BS, T_BS, T_BS_covar.data());
  // -- Form covariance matrix block
  calib_covar = zeros(6, 6);
  calib_covar.block(0, 0, 6, 6) = T_BS_covar;
  // -- Check if calib_covar is full-rank?
  if (rank(calib_covar) != calib_covar.rows()) {
    LOG_ERROR("calib_covar is not full rank!");
    return -1;
  }

  return 0;
}

int calib_vi_t::recover_calib_info(matx_t &H) const {
  // Ceres-version
  matx_t covar;
  if (recover_calib_covar(covar) != 0) {
    return -1;
  }
  H = covar.inverse();

  // // Custom-version
  // {
  //   // Form full Hessian
  //   ParameterOrder param_order;
  //   matx_t H_full;
  //   form_hessian(param_order, H_full);

  //   // Marginalize out everything apart from T_BS
  //   schurs_complement(H_full, H_full.rows() - 6, 6, H);
  // }

  return 0;
}

void calib_vi_t::marginalize() {
  // Mark the pose to be marginalized
  auto view = calib_views.front();
  view->pose.marginalize = true;
  view->sb.marginalize = true;

  // Form new marg_residual_t
  if (marg_residual == nullptr) {
    // Initialize first marg_residual_t
    marg_residual = std::make_shared<marg_residual_t>();

  } else {
    // Add previous marg_residual_t to new
    auto new_marg_residual = std::make_shared<marg_residual_t>();
    new_marg_residual->add(marg_residual);

    // Delete old marg_residual_t
    problem->RemoveResidualBlock(marg_residual_id);

    // Point to new marg_residual_t
    marg_residual = std::move(new_marg_residual);
  }

  // Marginalize view
  marg_residual_id = view->marginalize(marg_residual.get());

  // Remove view
  calib_views.pop_front();
  // delete view;
}

void calib_vi_t::reset() {
  // Reset flags
  imu_started = false;
  vision_started = false;
  initialized = false;

  // Reset data
  grid_buf.clear();
  prev_grids.clear();
  imu_buf.clear();

  // Reset problem data
  calib_views.clear();
}

void calib_vi_t::eval_residuals(ParameterOrder &param_order,
                                std::vector<calib_residual_t *> &res_evaled,
                                size_t &residuals_length,
                                size_t &params_length) const {
  // Get all residuals
  std::unordered_set<calib_residual_t *> res_fns;
  for (const auto &view : calib_views) {
    if (view->imu_residual == nullptr) {
      continue;
    }

    // -- Fiducial Residuals
    for (const auto &[cam_idx, cam_res_fns] : view->fiducial_residuals) {
      for (const auto &res : cam_res_fns) {
        res_fns.insert(res.get());
      }
    }
    // -- IMU Residuals
    if (view->imu_residual) {
      res_fns.insert(view->imu_residual.get());
    }
  }
  // -- Marginalization Residual
  if (marg_residual) {
    res_fns.insert(marg_residual.get());
  }

  // Evaluate residuals
  residuals_length = 0;
  for (calib_residual_t *res_fn : res_fns) {
    if (res_fn->eval() == false) {
      FATAL("Residual evaulation failure!");
    }
    res_evaled.push_back(res_fn);
    residuals_length += res_fn->num_residuals();
  }

  // Track unique parameters
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> sb_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  std::vector<param_t *> fiducial_ptrs;
  params_length = 0;

  for (auto res_fn : res_evaled) {
    for (auto param_block : res_fn->param_blocks) {
      // Check if parameter block seen already
      if (params_seen.count(param_block)) {
        continue;
      }
      params_seen[param_block] = true;

      // Check if parameter is fixed
      if (param_block->fixed) {
        continue;
      }

      // Track parameter
      if (param_block->type == "pose_t") {
        pose_ptrs.push_back(param_block);
      } else if (param_block->type == "sb_params_t") {
        sb_ptrs.push_back(param_block);
      } else if (param_block->type == "camera_params_t") {
        cam_ptrs.push_back(param_block);
      } else if (param_block->type == "extrinsics_t") {
        extrinsics_ptrs.push_back(param_block);
      } else if (param_block->type == "fiducial_t") {
        fiducial_ptrs.push_back(param_block);
      }
      params_length += param_block->local_size;
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t idx = 0;
  std::vector<std::vector<param_t *> *> param_groups = {
      &extrinsics_ptrs,
      &pose_ptrs,
      &sb_ptrs,
      &cam_ptrs,
      &fiducial_ptrs,
  };
  param_order.clear();
  for (const auto &param_ptrs : param_groups) {
    for (const auto &param_block : *param_ptrs) {
      param_order.insert({param_block, idx});
      idx += param_block->local_size;
    }
  }
}

void calib_vi_t::form_hessian(ParameterOrder &param_order, matx_t &H) const {
  // Evaluate residuals
  std::vector<calib_residual_t *> res_evaled;
  size_t residuals_length = 0;
  size_t params_length = 0;
  eval_residuals(param_order, res_evaled, residuals_length, params_length);

  // Form Hessian H and R.H.S b
  H = zeros(params_length, params_length);

  for (calib_residual_t *res_fn : res_evaled) {
    const vecx_t r = res_fn->residuals;

    for (size_t i = 0; i < res_fn->param_blocks.size(); i++) {
      const auto &param_i = res_fn->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }
      const int idx_i = param_order[param_i];
      const int size_i = param_i->local_size;
      const matx_t &J_i = res_fn->min_jacobian_blocks[i];

      for (size_t j = i; j < res_fn->param_blocks.size(); j++) {
        const auto &param_j = res_fn->param_blocks[j];
        if (param_j->fixed) {
          continue;
        }
        const int idx_j = param_order[param_j];
        const int size_j = param_j->local_size;
        const matx_t &J_j = res_fn->min_jacobian_blocks[j];

        if (i == j) {
          // Form diagonals of H
          H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
        } else {
          // Form off-diagonals of H
          // clang-format off
            H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
            H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() * J_j).transpose();
          // clang-format on
        }
      }
    }
  }
}

void calib_vi_t::load_data(const std::string &data_path) {
  // Form timeline
  timeline_t timeline;
  // -- Load camera data
  std::map<int, std::string> cam_paths;
  for (const auto cam_idx : get_camera_indices()) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    cam_paths[cam_idx] = data_path + "/" + cam_str + "/data";
  }
  const auto grids_path = data_path + "/grid0";
  auto cam_grids = calib_data_preprocess(calib_target, cam_paths, grids_path);
  for (const auto cam_idx : get_camera_indices()) {
    for (const auto grid : cam_grids[cam_idx]) {
      timeline.add(grid.timestamp, cam_idx, grid);
    }
  }
  // -- Load imu data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  load_imu_data(data_path + "/imu0/data.csv", imu_ts, imu_acc, imu_gyr);
  for (size_t k = 0; k < imu_ts.size(); k++) {
    timeline.add(imu_ts[k], imu_acc[k], imu_gyr[k]);
  }

  // Process data
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        add_measurement(ts, acc, gyr);
      }
    }
  }
}

void calib_vi_t::solve() {
  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

  // Optimize problem - first pass
  {
    ceres::Solve(options, problem.get(), &summary);
    if (verbose) {
      std::cout << summary.BriefReport() << std::endl << std::endl;
    }
  }

  // Filter outliers
  if (enable_outlier_rejection) {
    LOG_INFO("Filter outliers");
    int nb_outliers = 0;
    // int view_idx = 0;
    for (auto &view : calib_views) {
      const auto view_outliers = view->filter_view(outlier_threshold);
      // printf("filtering view[%d]: %ld [removed %d outliers]\n",
      //        view_idx++,
      //        view->ts,
      //        view_outliers);
      nb_outliers += view_outliers;
    }
    LOG_INFO("Removed %d outliers!", nb_outliers);

    // Optimize problem - second pass
    {
      LOG_INFO("Optimize problem - second pass\n");

      // Solve
      ceres::Solve(options, problem.get(), &summary);
      if (verbose) {
        std::cout << summary.BriefReport() << std::endl << std::endl;
      }
    }
  }

  // Show results
  if (verbose) {
    show_results();
  }
}

void calib_vi_t::print_settings(FILE *os) const {
  // clang-format off
  fprintf(os, "settings:\n");
  fprintf(os, "  max_num_threads: %d\n", max_num_threads);
  fprintf(os, "  max_iter: %d\n", max_iter);
  fprintf(os, "  enable_outlier_rejection: %s\n", enable_outlier_rejection ? "true": "false");
  fprintf(os, "  enable_marginalization: %s\n", enable_marginalization ? "true": "false");
  fprintf(os, "  enable_vision_loss_fn: %s\n", enable_vision_loss_fn ? "true" : "false");
  fprintf(os, "  vision_loss_fn_type: \"%s\"\n", vision_loss_fn_type.c_str());
  fprintf(os, "  vision_loss_fn_param: %f\n", vision_loss_fn_param);
  fprintf(os, "  enable_imu_loss_fn: %s\n", enable_imu_loss_fn ? "true" : "false");
  fprintf(os, "  imu_loss_fn_type: \"%s\"\n", imu_loss_fn_type.c_str());
  fprintf(os, "  imu_loss_fn_param: %f\n", imu_loss_fn_param);
  fprintf(os, "  outlier_threshold: %f\n", outlier_threshold);
  fprintf(os, "  window_size: %d\n", window_size);
  fprintf(os, "  img_scale: %f\n", img_scale);
  fprintf(os, "\n");
  // clang-format on
}

void calib_vi_t::print_calib_target(FILE *os) const {
  fprintf(os, "calib_target:\n");
  fprintf(os, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(os, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(os, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(os, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(os, "\n");
}

void calib_vi_t::print_stats(FILE *os) const {
  // Stats - Reprojection Errors
  const auto reproj_errors = get_reproj_errors();
  std::vector<real_t> reproj_errors_all;
  for (const auto &[cam_idx, cam_errors] : reproj_errors) {
    extend(reproj_errors_all, cam_errors);
  }

  // Calibration metrics
  fprintf(os, "total_reproj_error:\n");
  fprintf(os, "  nb_views: %d\n", nb_views());
  fprintf(os, "  nb_corners: %ld\n", reproj_errors_all.size());
  fprintf(os, "  rmse:   %.4f # [px]\n", rmse(reproj_errors_all));
  fprintf(os, "  mean:   %.4f # [px]\n", mean(reproj_errors_all));
  fprintf(os, "  median: %.4f # [px]\n", median(reproj_errors_all));
  fprintf(os, "  stddev: %.4f # [px]\n", stddev(reproj_errors_all));
  fprintf(os, "\n");

  for (const auto &[cam_idx, errors] : get_reproj_errors()) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    fprintf(os, "%s_reproj_error:\n", cam_str.c_str());
    fprintf(os, "  nb_corners: %ld\n", errors.size());
    fprintf(os, "  rmse:   %.4f # [px]\n", rmse(errors));
    fprintf(os, "  mean:   %.4f # [px]\n", mean(errors));
    fprintf(os, "  median: %.4f # [px]\n", median(errors));
    fprintf(os, "  stddev: %.4f # [px]\n", stddev(errors));
    fprintf(os, "\n");
  }
}

void calib_vi_t::print_imu(FILE *os) const {
  // IMU parameters
  vec3s_t bias_acc;
  vec3s_t bias_gyr;
  for (auto view : calib_views) {
    const auto sb = view->sb;
    bias_acc.push_back(sb.param.segment<3>(3));
    bias_gyr.push_back(sb.param.segment<3>(6));
  }
  const vec3_t mu_ba = mean(bias_acc);
  const vec3_t mu_bg = mean(bias_gyr);
  fprintf(os, "imu0:\n");
  fprintf(os, "  rate: %f\n", imu_params.rate);
  fprintf(os, "  sigma_a_c: %e\n", imu_params.sigma_a_c);
  fprintf(os, "  sigma_g_c: %e\n", imu_params.sigma_g_c);
  fprintf(os, "  sigma_aw_c: %e\n", imu_params.sigma_aw_c);
  fprintf(os, "  sigma_gw_c: %e\n", imu_params.sigma_gw_c);
  fprintf(os, "  sigma_ba: %e\n", imu_params.sigma_ba);
  fprintf(os, "  sigma_bg: %e\n", imu_params.sigma_bg);
  fprintf(os, "  g: %f\n", imu_params.g);
  fprintf(os, "  ba: [%f, %f, %f]\n", mu_ba.x(), mu_ba.y(), mu_ba.z());
  fprintf(os, "  bg: [%f, %f, %f]\n", mu_bg.x(), mu_bg.y(), mu_bg.z());
  fprintf(os, "\n");
}

void calib_vi_t::print_cameras(FILE *os) const {
  // Setup
  bool max_prec = (os == stdout) ? false : true;

  // Camera parameters
  for (auto &kv : cam_params) {
    const auto cam_idx = kv.first;
    const auto cam = cam_params.at(cam_idx);
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const std::string proj_params = vec2str(cam->proj_params(), true, max_prec);
    const std::string dist_params = vec2str(cam->dist_params(), true, max_prec);

    fprintf(os, "cam%d:\n", cam_idx);
    fprintf(os, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(os, "  proj_model: \"%s\"\n", proj_model);
    fprintf(os, "  dist_model: \"%s\"\n", dist_model);
    fprintf(os, "  proj_params: %s\n", proj_params.c_str());
    fprintf(os, "  dist_params: %s\n", dist_params.c_str());
    fprintf(os, "\n");
  }
}

void calib_vi_t::print_extrinsics(FILE *os) const {
  // Setup
  bool max_prec = (os == stdout) ? false : true;

  // Camera-Camera extrinsics
  const mat4_t T_BC0 = get_camera_extrinsics(0);
  const mat4_t T_C0B = T_BC0.inverse();
  if (nb_cams() >= 2) {
    for (int i = 1; i < nb_cams(); i++) {
      const mat4_t T_BCi = get_camera_extrinsics(i);
      const mat4_t T_C0Ci = T_C0B * T_BCi;
      fprintf(os, "T_cam0_cam%d:\n", i);
      fprintf(os, "  rows: 4\n");
      fprintf(os, "  cols: 4\n");
      fprintf(os, "  data: [\n");
      fprintf(os, "%s\n", mat2str(T_C0Ci, "    ", max_prec).c_str());
      fprintf(os, "  ]\n");
      fprintf(os, "\n");
    }
  }

  // Sensor-Camera extrinsics
  const mat4_t T_BS = get_imu_extrinsics();
  const mat4_t T_SB = T_BS.inverse();
  const mat4_t T_BCi = get_camera_extrinsics(0);
  const mat4_t T_SCi = T_SB * T_BCi;
  fprintf(os, "T_imu0_cam%d:\n", 0);
  fprintf(os, "  rows: 4\n");
  fprintf(os, "  cols: 4\n");
  fprintf(os, "  data: [\n");
  fprintf(os, "%s\n", mat2str(T_SCi, "    ", max_prec).c_str());
  fprintf(os, "  ]\n");
  fprintf(os, "\n");
}

void calib_vi_t::show_results() const {
  print_settings(stdout);
  print_stats(stdout);
  print_calib_target(stdout);
  print_imu(stdout);
  print_cameras(stdout);
  print_extrinsics(stdout);
}

int calib_vi_t::save_results(const std::string &save_path) const {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }
  print_settings(outfile);
  print_stats(outfile);
  print_calib_target(outfile);
  print_imu(outfile);
  print_cameras(outfile);
  print_extrinsics(outfile);
  fclose(outfile);

  return 0;
}

void calib_vi_t::save_estimates(const std::string &dir_path) const {
  // Paths
  const auto poses_path = dir_path + "/poses_est.csv";
  const auto sb_path = dir_path + "/speed_biases_est.csv";

  // Setup csv files
  FILE *poses_csv = fopen(poses_path.c_str(), "w");
  FILE *sb_csv = fopen(sb_path.c_str(), "w");

  // Headers
  fprintf(poses_csv, "#ts,rx,ry,rz,qx,qy,qz,qw\n");
  fprintf(sb_csv, "#ts,vx,vy,vz,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n");

  // Write poses and speed and biases
  for (const auto &view : calib_views) {
    // Poses
    const auto pose = view->pose;
    const auto ts = pose.ts;
    const vec3_t r = pose.trans();
    const quat_t q = pose.rot();
    fprintf(poses_csv, "%ld,", ts);
    fprintf(poses_csv, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(poses_csv, "%f,%f,%f,%f\n", q.x(), q.y(), q.z(), q.w());

    // Speed and biases
    const auto sb = view->sb;
    const vec3_t v = sb.param.segment<3>(0);
    const vec3_t ba = sb.param.segment<3>(3);
    const vec3_t bg = sb.param.segment<3>(6);
    fprintf(sb_csv, "%ld,", ts);
    fprintf(sb_csv, "%f,%f,%f,", v.x(), v.y(), v.z());
    fprintf(sb_csv, "%f,%f,%f,", ba.x(), ba.y(), ba.z());
    fprintf(sb_csv, "%f,%f,%f\n", bg.x(), bg.y(), bg.z());
  }

  // Close csv files
  fclose(poses_csv);
  fclose(sb_csv);
}

} // namespace yac
