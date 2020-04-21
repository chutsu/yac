#include "calib_stereo.hpp"

namespace yac {

int calib_stereo_solve(const std::string &config_file) {
  // Calibration settings
  std::string data_path;
  std::string results_fpath;

  vec2_t cam0_resolution{0.0, 0.0};
  real_t cam0_lens_hfov = 0.0;
  real_t cam0_lens_vfov = 0.0;
  std::string cam0_camera_model;
  std::string cam0_distortion_model;

  vec2_t cam1_resolution{0.0, 0.0};
  real_t cam1_lens_hfov = 0.0;
  real_t cam1_lens_vfov = 0.0;
  std::string cam1_camera_model;
  std::string cam1_distortion_model;

  // Parse calibration config
  config_t config{config_file};

  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "cam0.resolution", cam0_resolution);
  parse(config, "cam0.lens_hfov", cam0_lens_hfov);
  parse(config, "cam0.lens_vfov", cam0_lens_vfov);
  parse(config, "cam0.camera_model", cam0_camera_model);
  parse(config, "cam0.distortion_model", cam0_distortion_model);
  parse(config, "cam1.resolution", cam1_resolution);
  parse(config, "cam1.lens_hfov", cam1_lens_hfov);
  parse(config, "cam1.lens_vfov", cam1_lens_vfov);
  parse(config, "cam1.camera_model", cam1_camera_model);
  parse(config, "cam1.distortion_model", cam1_distortion_model);

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
                                      cam0_resolution,
                                      cam1_resolution,
                                      cam0_lens_hfov,
                                      cam0_lens_vfov,
                                      cam1_lens_hfov,
                                      cam1_lens_vfov,
                                      cam0_grid_path,
                                      cam1_grid_path);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load stereo calibration data
  aprilgrids_t cam0_aprilgrids;
  aprilgrids_t cam1_aprilgrids;
  retval = load_stereo_calib_data(cam0_grid_path,
                                  cam1_grid_path,
                                  cam0_aprilgrids,
                                  cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to load calibration data!");
    return -1;
  }

  // Setup initial cam0 intrinsics and distortion
  // -- cam0
  pinhole_t<radtan4_t> cam0;
  {
    const int img_w = cam0_resolution(0);
    const int img_h = cam0_resolution(1);
    const real_t fx = pinhole_focal(img_w, cam0_lens_hfov);
    const real_t fy = pinhole_focal(img_h, cam0_lens_vfov);
    const real_t cx = img_w / 2.0;
    const real_t cy = img_h / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
    cam0 = pinhole_t<radtan4_t>{img_w, img_h, proj_params, dist_params};
  }
  // -- cam1
  pinhole_t<radtan4_t> cam1;
  {
    const int img_w = cam1_resolution(0);
    const int img_h = cam1_resolution(1);
    const real_t fx = pinhole_focal(img_w, cam1_lens_hfov);
    const real_t fy = pinhole_focal(img_h, cam1_lens_vfov);
    const real_t cx = img_w / 2.0;
    const real_t cy = img_h / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
    cam1 = pinhole_t<radtan4_t>{img_w, img_h, proj_params, dist_params};
  }

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C0C1 = I(4);
  mat4s_t poses;
  retval = calib_stereo_solve(cam0_aprilgrids,
                              cam1_aprilgrids,
                              cam0,
                              cam1,
                              T_C0C1,
                              poses);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  // -- cam0
  std::cout << "cam0:" << std::endl;
  std::cout << cam0 << std::endl;
  std::cout << cam0.distortion << std::endl;
  // -- cam1
  std::cout << "cam1:" << std::endl;
  std::cout << cam1 << std::endl;
  std::cout << cam1.distortion << std::endl;
  // -- cam0-cam1 extrinsics
  std::cout << "T_C0C1:\n" << T_C0C1 << std::endl;
  std::cout << std::endl;

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  retval = save_results(results_fpath,
                        cam0_resolution,
                        cam1_resolution,
                        cam0,
                        cam1,
                        T_C0C1);
  if (retval != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
