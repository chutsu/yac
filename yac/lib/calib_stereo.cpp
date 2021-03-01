#include "calib_stereo.hpp"

namespace yac {

int calib_covar(ceres::Problem &problem,
                camera_params_t &cam0,
                camera_params_t &cam1,
                extrinsics_t &exts,
                matx_t &covar) {
  const auto cam0_size = cam0.param.size();
  const auto cam1_size = cam1.param.size();
  const auto exts_size = exts.param.size() - 1;
  const auto covar_size = cam0_size + cam1_size + exts_size;
  covar.resize(covar_size, covar_size);
  covar.setZero();

  const auto cam0_data = cam0.param.data();
  const auto cam1_data = cam1.param.data();
  const auto exts_data = exts.param.data();

  std::vector<std::pair<const double *, const double *>> covar_blocks;
  covar_blocks.push_back({cam0_data, cam0_data});
  covar_blocks.push_back({cam1_data, cam1_data});
  covar_blocks.push_back({exts_data, exts_data});

  ceres::Covariance::Options covar_opts;
  ceres::Covariance covar_est(covar_opts);
  if (covar_est.Compute(covar_blocks, &problem) == false) {
    return -1;
  }

  // cam0-cam0 covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_cam0;
    covar_cam0.resize(cam0_size, cam0_size);
    covar_est.GetCovarianceBlock(cam0_data, cam0_data, covar_cam0.data());

    const long rs = 0;
    const long cs = 0;
    const long nb_rows = cam0_size;
    const long nb_cols = cam0_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_cam0;
  }

  // cam1-cam1 covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_cam1;
    covar_cam1.resize(cam1_size, cam1_size);
    covar_est.GetCovarianceBlock(cam1_data, cam1_data, covar_cam1.data());

    const long rs = cam0_size;
    const long cs = cam0_size;
    const long nb_rows = cam1_size;
    const long nb_cols = cam1_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_cam1;
  }

  // extrinsics covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_ext;
    covar_ext.resize(exts_size, exts_size);
    covar_est.GetCovarianceBlockInTangentSpace(exts_data, exts_data, covar_ext.data());

    const long rs = cam0_size + cam1_size;
    const long cs = cam0_size + cam1_size;
    const long nb_rows = exts_size;
    const long nb_cols = exts_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_ext;
  }

  // printf("rank(covar): %ld\n", rank(covar));

  return 0;
}

int save_results(const std::string &save_path, const calib_data_t &data) {
  const camera_params_t cam0 = data.cam_params.at(0);
  const camera_params_t cam1 = data.cam_params.at(1);
  const mat4_t T_BC0 = data.cam_exts.at(0).tf();
  const mat4_t T_BC1 = data.cam_exts.at(1).tf();
  const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;
  const auto cam0_errs = data.vision_errors.at(0);
  const auto cam1_errs = data.vision_errors.at(1);

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  {
    fprintf(outfile, "calib_results:\n");
    fprintf(outfile, "  cam0_rmse_reproj_error: %f\n", rmse(cam0_errs));
    fprintf(outfile, "  cam1_rmse_reproj_error: %f\n", mean(cam1_errs));
    fprintf(outfile, "  cam0_mean_reproj_error: %f\n", rmse(cam0_errs));
    fprintf(outfile, "  cam1_mean_reproj_error: %f\n", mean(cam1_errs));
    fprintf(outfile, "\n");
  }
  {
    const auto target = data.target;
    fprintf(outfile, "calib_target:\n");
    fprintf(outfile, "  target_type: '%s'\n", target.target_type.c_str());
    fprintf(outfile, "  tag_rows: %d\n", target.tag_rows);
    fprintf(outfile, "  tag_cols: %d\n", target.tag_cols);
    fprintf(outfile, "  tag_size: %f\n", target.tag_size);
    fprintf(outfile, "  tag_spacing: %f\n", target.tag_spacing);
    fprintf(outfile, "\n");
  }
  {
    const auto imu_params = data.imu_params;
    fprintf(outfile, "imu0:\n");
    fprintf(outfile, "  rate: %f\n", imu_params.rate);
    fprintf(outfile, "  a_max: %f\n", imu_params.a_max);
    fprintf(outfile, "  g_max: %f\n", imu_params.g_max);
    fprintf(outfile, "  sigma_a_c: %f\n", imu_params.sigma_a_c);
    fprintf(outfile, "  sigma_g_c: %f\n", imu_params.sigma_g_c);
    fprintf(outfile, "  sigma_aw_c: %f\n", imu_params.sigma_aw_c);
    fprintf(outfile, "  sigma_gw_c: %f\n", imu_params.sigma_gw_c);
    fprintf(outfile, "  sigma_ba: %f\n", imu_params.sigma_ba);
    fprintf(outfile, "  sigma_bg: %f\n", imu_params.sigma_bg);
    fprintf(outfile, "  g: %f\n", imu_params.g);
    fprintf(outfile, "\n");
  }
  {
    const int *cam_res = cam0.resolution;
    const char *proj_model = cam0.proj_model.c_str();
    const char *dist_model = cam0.dist_model.c_str();
    const std::string proj_params = vec2str(cam0.proj_params(), 4);
    const std::string dist_params = vec2str(cam0.dist_params(), 4);
    fprintf(outfile, "cam0:\n");
    fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }
  {
    const int *cam_res = cam1.resolution;
    const char *proj_model = cam1.proj_model.c_str();
    const char *dist_model = cam1.dist_model.c_str();
    const std::string proj_params = vec2str(cam1.proj_params(), 4);
    const std::string dist_params = vec2str(cam1.dist_params(), 4);
    fprintf(outfile, "cam1:\n");
    fprintf(outfile, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }

  // Save camera extrinsics
  for (size_t cam_idx = 0; cam_idx < data.cam_exts.size(); cam_idx++) {
    const mat4_t cam_exts = data.cam_exts.at(cam_idx).tf();
    fprintf(outfile, "T_body0_cam%ld:\n", cam_idx);
    fprintf(outfile, "  rows: 4\n");
    fprintf(outfile, "  cols: 4\n");
    fprintf(outfile, "  data: [\n");
    fprintf(outfile, "%s\n", mat2str(cam_exts, "    ").c_str());
    fprintf(outfile, "  ]\n");
    fprintf(outfile, "\n");
  }

  // Finsh up
  fclose(outfile);

  return 0;
}

int calib_stereo_solve(const std::string &config_file) {
  // Calibration settings
  std::string data_path;
  std::string results_fpath;
  bool imshow = false;
  double sigma_vision = 0.0;

  std::vector<int> cam0_res;
  real_t cam0_lens_hfov = 90.0;
  real_t cam0_lens_vfov = 90.0;
  std::string cam0_proj_model;
  std::string cam0_dist_model;

  std::vector<int> cam1_res;
  real_t cam1_lens_hfov = 90.0;
  real_t cam1_lens_vfov = 90.0;
  std::string cam1_proj_model;
  std::string cam1_dist_model;

  // Parse calibration config
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow);
  parse(config, "settings.sigma_vision", sigma_vision);
  parse(config, "cam0.resolution", cam0_res);
  parse(config, "cam0.proj_model", cam0_proj_model);
  parse(config, "cam0.dist_model", cam0_dist_model);
  parse(config, "cam1.resolution", cam1_res);
  parse(config, "cam1.proj_model", cam1_proj_model);
  parse(config, "cam1.dist_model", cam1_dist_model);

  // Setup camera intrinsics and distortion
  // -- cam0
  camera_params_t cam0;
  {
    const id_t id = 0;
    const int cam0_idx = 0;
    const double fx = pinhole_focal(cam0_res[0], cam0_lens_hfov);
    const double fy = pinhole_focal(cam0_res[1], cam0_lens_vfov);
    const double cx = cam0_res[0] / 2.0;
    const double cy = cam0_res[1] / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
    cam0 = camera_params_t{id, cam0_idx, cam0_res.data(),
                          cam0_proj_model, cam0_dist_model,
                          proj_params, dist_params};
  }
  // -- cam1
  camera_params_t cam1;
  {
    const id_t id = 1;
    const int cam1_idx = 1;
    const double fx = pinhole_focal(cam1_res[0], cam1_lens_hfov);
    const double fy = pinhole_focal(cam1_res[1], cam1_lens_vfov);
    const double cx = cam1_res[0] / 2.0;
    const double cy = cam1_res[1] / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
    cam1 = camera_params_t{id, cam1_idx, cam1_res.data(),
                          cam1_proj_model, cam0_dist_model,
                          proj_params, dist_params};
  }

  // Load calibration target
  calib_target_t target;
  if (calib_target_load(target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", config_file.c_str());
    return -1;
  }

  // Prepare aprilgrid data directory
  const auto cam0_grid_path = data_path + "/grid0/cam0/data";
  if (dir_exists(cam0_grid_path) == false) {
    dir_create(cam0_grid_path);
  }
  const auto cam1_grid_path = data_path + "/grid0/cam1/data";
  if (dir_exists(cam1_grid_path) == false) {
    dir_create(cam1_grid_path);
  }

  // Preprocess data
  auto cam0_data_path = data_path + "/cam0/data";
  auto cam1_data_path = data_path + "/cam1/data";
  if (preprocess_camera_data(target, cam0_data_path, cam0_grid_path, true) != 0) {
    FATAL("Failed to preprocess cam0 data!");
  }
  if (preprocess_camera_data(target, cam0_data_path, cam1_grid_path, true) != 0) {
    FATAL("Failed to preprocess cam1 data!");
  }

  // Load calibration data
	aprilgrids_t grids0;
	aprilgrids_t grids1;
	if (load_stereo_calib_data(cam0_data_path, cam1_data_path, grids0, grids1) != 0) {
		LOG_ERROR("Failed to local calibration data!");
		return -1;
	}

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  std::map<timestamp_t, pose_t> poses;  // T_BF
  mat2_t covar = pow(sigma_vision, 2) * I(2);

  // -- Pinhole-Radtan4
  calib_data_t data;
  data.add_calib_target(data.target);
  data.add_camera(cam0);
  data.add_camera(cam1);
  data.add_camera_extrinsics(0);
  data.add_camera_extrinsics(1);
  data.add_grids(0, grids0);
  data.add_grids(1, grids1);
  data.preprocess_data();
  data.check_data();

  if (cam0_proj_model == "pinhole" && cam0_dist_model == "radtan4") {
    calib_stereo_solve<pinhole_radtan4_t>(data);
  } else if (cam0_proj_model == "pinhole" && cam0_dist_model == "equi4") {
    calib_stereo_solve<pinhole_radtan4_t>(data);
  } else {
    LOG_ERROR("[%s-%s] unsupported!", cam0_proj_model.c_str(),
                                      cam0_dist_model.c_str());
    return -1;

  }

  // Save results
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, data) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
