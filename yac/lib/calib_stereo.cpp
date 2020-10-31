#include "calib_stereo.hpp"

namespace yac {

static int save_results(const std::string &save_path,
                        const camera_params_t &cam0,
                        const camera_params_t &cam1,
                        const mat4_t &T_C0C1,
                        const calib_stereo_results_t &results) {
  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  {
    fprintf(outfile, "calib_results:\n");
    fprintf(outfile, "  cam0_rmse_reproj_error: %f\n", results.cam0_rmse_error);
    fprintf(outfile, "  cam1_rmse_reproj_error: %f\n", results.cam1_rmse_error);
    fprintf(outfile, "  cam0_mean_reproj_error: %f\n", results.cam0_mean_error);
    fprintf(outfile, "  cam1_mean_reproj_error: %f\n", results.cam1_mean_error);
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
  fprintf(outfile, "T_C0C1:\n");
  fprintf(outfile, "  rows: 4\n");
  fprintf(outfile, "  cols: 4\n");
  fprintf(outfile, "  data: [\n");
  fprintf(outfile, "%s\n", mat2str(T_C0C1, "    ").c_str());
  fprintf(outfile, "  ]");

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

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
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

  // Preprocess calibration data
  int retval = preprocess_stereo_data(calib_target,
                                      data_path + "/cam0/data",
                                      data_path + "/cam1/data",
                                      vec2_t{cam0_res[0], cam0_res[1]},
                                      vec2_t{cam1_res[0], cam1_res[1]},
                                      cam0_lens_hfov,
                                      cam0_lens_vfov,
                                      cam1_lens_hfov,
                                      cam1_lens_vfov,
                                      cam0_grid_path,
                                      cam1_grid_path,
                                      imshow);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load stereo calibration data
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  retval = load_stereo_calib_data(cam0_grid_path,
                                  cam1_grid_path,
                                  cam0_grids,
                                  cam1_grids);
  if (retval != 0) {
    LOG_ERROR("Failed to load calibration data!");
    return -1;
  }

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

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C0C1 = I(4);
  mat4s_t T_C0F;
  mat4s_t T_C1F;
  mat2_t covar = pow(sigma_vision, 2) * I(2);
  calib_stereo_results_t results;

  // -- Pinhole-Radtan4
  if (cam0_proj_model == "pinhole" && cam0_dist_model == "radtan4") {
    calib_stereo_solve<pinhole_radtan4_t>(cam0_grids, cam1_grids, covar,
                                          &cam0, &cam1,
                                          &T_C0C1, &T_C0F, &T_C1F);
    calib_mono_stats<pinhole_radtan4_t>(cam0_grids, cam0, T_C0F,
                                        &results.cam0_errors,
                                        &results.cam0_rmse_error,
                                        &results.cam0_mean_error);
    calib_mono_stats<pinhole_radtan4_t>(cam1_grids, cam1, T_C1F,
                                        &results.cam1_errors,
                                        &results.cam1_rmse_error,
                                        &results.cam1_mean_error);

  // -- Pinhole-Equi4
  } else if (cam0_proj_model == "pinhole" && cam0_dist_model == "equi4") {
    calib_stereo_solve<pinhole_equi4_t>(cam0_grids, cam1_grids, covar,
                                        &cam0, &cam1,
                                        &T_C0C1, &T_C0F, &T_C1F);
    calib_mono_stats<pinhole_equi4_t>(cam0_grids, cam0, T_C0F,
                                      &results.cam0_errors,
                                      &results.cam0_rmse_error,
                                      &results.cam0_mean_error);
    calib_mono_stats<pinhole_equi4_t>(cam1_grids, cam1, T_C1F,
                                      &results.cam1_errors,
                                      &results.cam1_rmse_error,
                                      &results.cam1_mean_error);

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
  retval = save_results(results_fpath, cam0, cam1, T_C0C1, results);
  if (retval != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
