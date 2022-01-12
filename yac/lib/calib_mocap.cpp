#include "calib_mocap.hpp"

namespace yac {

// MOCAP MARKER - CAMERA RESIDUAL //////////////////////////////////////////////

mocap_residual_t::mocap_residual_t(const camera_geometry_t *cam_geom,
                                   const int cam_idx,
                                   const int cam_res[2],
                                   const vec3_t &r_FFi,
                                   const vec2_t &z,
                                   const mat2_t &covar)
    : cam_geom_{cam_geom_}, cam_idx_{cam_idx}, cam_res_{cam_res[0], cam_res[1]},
      r_FFi_{r_FFi}, z_{z}, covar_{covar}, info_{covar.inverse()},
      sqrt_info_{info_.llt().matrixL().transpose()} {
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // Fiducial pose
  block_sizes->push_back(7); // Marker pose
  block_sizes->push_back(7); // Marker-camera extrinsics
}

bool mocap_residual_t::Evaluate(double const *const *params,
                                double *residuals,
                                double **jacobians) const {
  // Map optimization variables to Eigen
  const mat4_t T_WF = tf(params[0]);
  const mat4_t T_WM = tf(params[1]);
  const mat4_t T_MC0 = tf(params[2]);
  Eigen::Map<const vecx_t> cam_params(params[3], 8);

  // Transform and project point to image plane
  const mat4_t T_MW = T_WM.inverse();
  const mat4_t T_C0M = T_MC0.inverse();
  const vec3_t r_MFi = tf_point(T_MW * T_WF, r_FFi_);
  const vec3_t r_C0Fi = tf_point(T_C0M * T_MW * T_WF, r_FFi_);
  const vec2_t p{r_C0Fi(0) / r_C0Fi(2), r_C0Fi(1) / r_C0Fi(2)};
  mat_t<2, 3> Jh;
  matx_t J_params;
  vec2_t z_hat;
  bool valid = true;

  if (cam_geom_->project(cam_res_, cam_params, r_C0Fi, z_hat) != 0) {
    valid = false;
  }

  // Residual
  const vec2_t r = sqrt_info_ * (z_ - z_hat);
  residuals[0] = r(0);
  residuals[1] = r(1);

  // Jacobians
  const matx_t J_cam_params = cam_geom_->params_jacobian(cam_params, r_C0Fi);
  const matx_t Jh_weighted = sqrt_info_ * Jh;
  const mat3_t C_C0W = tf_rot(T_C0M * T_MW);
  const mat3_t C_MC0 = tf_rot(T_MC0);
  const mat3_t C_C0M = C_MC0.transpose();

  if (jacobians) {
    // Jacobians w.r.t T_WF
    if (jacobians[0]) {
      const mat3_t C_WF = tf_rot(T_WF);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_C0W * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_C0W * -skew(C_WF * r_FFi_);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_WM
    if (jacobians[1]) {
      const mat3_t C_WM = tf_rot(T_WM);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0W * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0W * -skew(C_WM * r_MFi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_MC0
    if (jacobians[2]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
      J.setZero();
      J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0M * I(3);
      J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0M * -skew(C_MC0 * r_C0Fi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t camera params
    if (jacobians[3]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
      J = -1 * sqrt_info_ * J_params;
      if (valid == false) {
        J.setZero();
      }
    }
  }

  return true;
}

// MOCAP MARKER - CAMERA CALIBRATION DATA //////////////////////////////////////

calib_mocap_data_t::calib_mocap_data_t(const std::string &config_file) {
  // Load calib config file
  vec2_t resolution{0.0, 0.0};
  std::string proj_model;
  std::string dist_model;

  config_t config{config_file};
  parse(config, "settings.fix_intrinsics", fix_intrinsics);
  parse(config, "settings.fix_mocap_poses", fix_mocap_poses);
  parse(config, "settings.fix_fiducial_pose", fix_fiducial_pose);
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.sigma_vision", sigma_vision);
  parse(config, "settings.imshow", imshow);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);

  // Setup paths
  results_fpath = paths_join(data_path, "calib_results.yaml");
  cam0_path = data_path + "/cam0/data";
  grid0_path = data_path + "/grid0/cam0/data";
  body0_csv_path = data_path + "/body0/data.csv";
  target0_csv_path = data_path + "/target0/data.csv";

  // Load calibration target
  if (calib_target.load(config_file, "calib_target") != 0) {
    FATAL("Failed to load calib target in [%s]!", config_file.c_str());
  }

  // Set covariance matrix
  covar = I(2) * (1 / pow(sigma_vision, 2));

  // Setup AprilGrid detector
  aprilgrid_detector_t detector(calib_target.tag_rows,
                                calib_target.tag_cols,
                                calib_target.tag_size,
                                calib_target.tag_spacing);
  std::vector<std::string> image_paths;
  if (list_dir(cam0_path, image_paths) != 0) {
    FATAL("Failed to traverse dir [%s]!", cam0_path.c_str());
  }
  std::sort(image_paths.begin(), image_paths.end());

  // Detect AprilGrids
  aprilgrids_t cam0_grids;
  for (const auto &image_path : image_paths) {
    const auto ts = std::stoull(parse_fname(image_path));
    const auto image = cv::imread(paths_join(cam0_path, image_path));
    const auto grid = detector.detect(ts, image);
    grid.save(grid0_path + "/" + std::to_string(ts) + ".csv");
    cam0_grids.push_back(grid);
  }

  // Load camera params
  if (yaml_has_key(config, "cam0.proj_params") == false) {
    // // Perform intrinsics calibration
    // LOG_INFO("Camera parameters unknown!");
    // LOG_INFO("Calibrating camera intrinsics!");
    //
    // // -- Initialize camera parameters
    // const int cam_idx = 0;
    // const int cam_res[2] = {(int)resolution[0], (int)resolution[1]};
    // this->cam0 =
    //     camera_params_t{cam_idx, cam_res, proj_model, dist_model, 4, 4};
    // this->cam0.initialize(cam0_grids);
    //
    // // -- Calibrate camera intrinsics
    // int retval = 0;
    // calib_mono_data_t data{cam0_grids, this->cam0};
    // if (proj_model == "pinhole" && dist_model == "radtan4") {
    //   calib_mono_solve<pinhole_radtan4_t>(data);
    // } else if (proj_model == "pinhole" && dist_model == "equi4") {
    //   calib_mono_solve<pinhole_equi4_t>(data);
    // } else {
    //   FATAL("Unsupported [%s-%s]!", proj_model.c_str(), dist_model.c_str());
    // }
    // if (retval != 0) {
    //   FATAL("Failed to calibrate [cam0] intrinsics!");
    // }

  } else {
    // // Camera parameters known - proceed
    // vecx_t proj_params;
    // vecx_t dist_params;
    // parse(config, "cam0.proj_params", proj_params);
    // parse(config, "cam0.dist_params", dist_params);
    //
    // const int cam_idx = 0;
    // const int cam_res[2] = {(int)resolution[0], (int)resolution[1]};
    // this->cam0 = camera_params_t{cam_idx,
    //                              cam_res,
    //                              proj_model,
    //                              dist_model,
    //                              proj_params,
    //                              dist_params};
  }

  // Load dataset
  // -- April Grid
  aprilgrids_t grids_raw = load_aprilgrids(grid0_path);
  // -- Mocap marker pose
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_poses(body0_csv_path, body_timestamps, body_poses);
  // -- Synchronize aprilgrids and body poses
  mat4s_t marker_poses;
  lerp_body_poses(grids_raw,
                  body_timestamps,
                  body_poses,
                  this->grids,
                  marker_poses);
  // -- Fiducial target pose
  const mat4_t fiducial_pose = load_pose(target0_csv_path);
  this->T_WF = pose_t(0, fiducial_pose);
  // -- Marker poses
  for (size_t i = 0; i < this->grids.size(); i++) {
    const auto ts = this->grids[i].timestamp;
    const mat4_t marker_pose = marker_poses[i];
    this->T_WM.emplace_back(ts, marker_pose);
  }
  // -- Mocap marker to camera transform
  // const vec3_t euler{-90.0, 0.0, -90.0};
  const vec3_t euler{-180.0, 0.0, -90.0};
  const mat3_t C = euler321(deg2rad(euler));
  const mat4_t marker_cam_pose = tf(C, zeros(3, 1));
  this->T_MC0 = pose_t(0, marker_cam_pose);
}

int calib_mocap_solve(const std::string &config_file) {
  // calib_mocap_data_t data{config_file};

  // std::string proj_model = data.cam0.proj_model;
  // std::string dist_model = data.cam0.dist_model;
  // if (proj_model == "pinhole" && dist_model == "radtan4") {
  //   return calib_mocap_solve<pinhole_radtan4_t>(data);
  // } else if (proj_model == "pinhole" && dist_model == "equi4") {
  //   return calib_mocap_solve<pinhole_equi4_t>(data);
  // } else {
  //   FATAL("Unsupported [%s-%s]!", proj_model.c_str(), dist_model.c_str());
  // }

  return -1;
}

} //  namespace yac
