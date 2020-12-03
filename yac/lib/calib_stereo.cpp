#include "calib_stereo.hpp"

namespace yac {

int save_results(const std::string &save_path,
                 const camera_params_t &cam0,
                 const camera_params_t &cam1,
                 const mat4_t &T_C1C0,
                 const std::vector<double> &cam0_errs,
                 const std::vector<double> &cam1_errs) {
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
  fprintf(outfile, "T_C1C0:\n");
  fprintf(outfile, "  rows: 4\n");
  fprintf(outfile, "  cols: 4\n");
  fprintf(outfile, "  data: [\n");
  fprintf(outfile, "%s\n", mat2str(T_C1C0, "    ").c_str());
  fprintf(outfile, "  ]");

  // Finsh up
  fclose(outfile);

  return 0;
}

/* Estimate Monocular Camera Covariance */
int calib_stereo_covar(camera_params_t &cam0,
                       camera_params_t &cam1,
                       pose_t &extrinsic,
                       ceres::Problem &problem,
                       matx_t &covar) {
  const auto cam0_params_size = cam0.param.size();
  const auto cam1_params_size = cam1.param.size();
  const auto extrinsic_params_size = extrinsic.param.size();
  const auto covar_size = cam0_params_size + cam1_params_size + extrinsic_params_size;
  covar.resize(covar_size, covar_size);

  const auto cam0_data = cam0.param.data();
  const auto cam1_data = cam1.param.data();
  const auto extrinsic_data = extrinsic.param.data();

  std::vector<std::pair<const double *, const double *>> covar_blocks;
  covar_blocks.push_back({cam0_data, cam0_data});
  covar_blocks.push_back({cam1_data, cam1_data});
  covar_blocks.push_back({extrinsic_data, extrinsic_data});

  ceres::Covariance::Options covar_opts;
  ceres::Covariance covar_est(covar_opts);
  if (covar_est.Compute(covar_blocks, &problem) == false) {
    return -1;
  }

  // cam0-cam0 covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_cam0;
    covar_cam0.resize(cam0_params_size, cam0_params_size);
    covar_est.GetCovarianceBlock(cam0_data, cam0_data, covar_cam0.data());

    const long rs = 0;
    const long cs = 0;
    const long nb_rows = cam0_params_size;
    const long nb_cols = cam0_params_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_cam0;
  }

  // cam1-cam1 covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_cam1;
    covar_cam1.resize(cam1_params_size, cam1_params_size);
    covar_est.GetCovarianceBlock(cam1_data, cam1_data, covar_cam1.data());

    const long rs = cam0_params_size;
    const long cs = cam0_params_size;
    const long nb_rows = cam1_params_size;
    const long nb_cols = cam1_params_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_cam1;
  }

  // T_C1C0-T_C1C0 covar
  {
    mat_t<Eigen::Dynamic, Eigen::Dynamic, row_major_t> covar_extrinsic;
    covar_extrinsic.resize(extrinsic_params_size, extrinsic_params_size);
    covar_est.GetCovarianceBlock(extrinsic_data, extrinsic_data, covar_extrinsic.data());

    const long rs = cam0_params_size + cam1_params_size;
    const long cs = cam0_params_size + cam1_params_size;
    const long nb_rows = extrinsic_params_size;
    const long nb_cols = extrinsic_params_size;
    covar.block(rs, cs, nb_rows, nb_cols) = covar_extrinsic;
  }

  return 0;
}

int calib_stereo_solve(const std::string &config_file) {
  // Calibration settings
  std::string data_path;
  std::string results_fpath;
  bool imshow = false;
  double sigma_vision = 0.0;

  std::vector<int> cam0_res;
  real_t cam0_lens_hfov = 0.0;
  real_t cam0_lens_vfov = 0.0;
  std::string cam0_proj_model;
  std::string cam0_dist_model;

  std::vector<int> cam1_res;
  real_t cam1_lens_hfov = 0.0;
  real_t cam1_lens_vfov = 0.0;
  std::string cam1_proj_model;
  std::string cam1_dist_model;

  // Parse calibration config
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow);
  parse(config, "settings.sigma_vision", sigma_vision);
  parse(config, "cam0.resolution", cam0_res);
  parse(config, "cam0.lens_hfov", cam0_lens_hfov);
  parse(config, "cam0.lens_vfov", cam0_lens_vfov);
  parse(config, "cam0.proj_model", cam0_proj_model);
  parse(config, "cam0.dist_model", cam0_dist_model);
  parse(config, "cam1.resolution", cam1_res);
  parse(config, "cam1.lens_hfov", cam1_lens_hfov);
  parse(config, "cam1.lens_vfov", cam1_lens_vfov);
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
  const std::vector<std::string> data_dirs = {cam0_data_path, cam1_data_path};
  std::map<int, aprilgrids_t> grids;
  if (load_multicam_calib_data(2, data_dirs, grids) != 0) {
    LOG_ERROR("Failed to local calibration data!");
    return -1;
  }

  // Estimate initial guess for grid poses
  // -- cam0
  mat4s_t T_C0F;
  for (auto &grid : grids[0]) {
    mat4_t rel_pose;
    const auto proj_params = cam0.proj_params();
    const auto dist_params = cam0.dist_params();
    aprilgrid_calc_relative_pose(grid, proj_params, dist_params, rel_pose);
    T_C0F.push_back(rel_pose);
  }
  // -- cam1
  mat4s_t T_C1F;
  for (auto &grid : grids[1]) {
    mat4_t rel_pose;
    const auto proj_params = cam1.proj_params();
    const auto dist_params = cam1.dist_params();
    aprilgrid_calc_relative_pose(grid, proj_params, dist_params, rel_pose);
    T_C1F.push_back(rel_pose);
  }

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C1C0 = I(4);
  mat2_t covar = pow(sigma_vision, 2) * I(2);
  std::vector<double> cam0_errs;
  std::vector<double> cam1_errs;

  // -- Pinhole-Radtan4
  if (cam0_proj_model == "pinhole" && cam0_dist_model == "radtan4") {
    calib_stereo_solve<pinhole_radtan4_t>(grids[0], grids[1], covar,
                                          cam0, cam1,
                                          T_C1C0, T_C0F, T_C1F);
    reproj_errors<pinhole_radtan4_t>(grids[0], cam0, T_C0F, cam0_errs);
    reproj_errors<pinhole_radtan4_t>(grids[1], cam1, T_C1F, cam1_errs);

  // -- Pinhole-Equi4
  } else if (cam0_proj_model == "pinhole" && cam0_dist_model == "equi4") {
    calib_stereo_solve<pinhole_equi4_t>(grids[0], grids[1], covar,
                                        cam0, cam1,
                                        T_C1C0, T_C0F, T_C1F);
    reproj_errors<pinhole_equi4_t>(grids[0], cam0, T_C0F, cam0_errs);
    reproj_errors<pinhole_equi4_t>(grids[1], cam1, T_C1F, cam1_errs);

  // -- Unsupported
  } else {
    LOG_ERROR("[%s-%s] unsupported!", cam0_proj_model.c_str(),
                                      cam0_dist_model.c_str());
    return -1;

  }

  // Save results
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, cam0, cam1, T_C1C0, cam0_errs, cam1_errs) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
