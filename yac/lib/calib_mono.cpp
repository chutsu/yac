#include "calib_mono.hpp"

namespace yac {

int calib_mono_covar(const camera_params_t &cam_params,
                     ceres::Problem &problem,
                     matx_t &covar) {
  const auto param_data = cam_params.param.data();

  std::vector<std::pair<const double *, const double *>> covar_blocks;
  covar_blocks.push_back({param_data, param_data});

  ceres::Covariance::Options covar_opts;
  // covar_opts.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covar_est(covar_opts);
  if (covar_est.Compute(covar_blocks, &problem) == false) {
    return -1;
  }

  const auto params_size = cam_params.param.size();
  double *covar_raw = (double *) malloc(sizeof(double) * params_size * params_size);
  covar_est.GetCovarianceBlock(param_data, param_data, covar_raw);
  covar = Eigen::Map<mat_t<8, 8, row_major_t>>(covar_raw);
  free(covar_raw);

  return 0;
}

int calib_mono_solve(const std::string &config_file) {
  // Calibration config data
  std::string data_path;
  std::string results_fpath;
  bool imshow = true;
  double sigma_vision = 0.0;
  vec2_t resolution{0.0, 0.0};
  real_t lens_hfov = 0.0;
  real_t lens_vfov = 0.0;
  std::string proj_model;
  std::string dist_model;

  // Parse calib config file
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "settings.imshow", imshow);
  parse(config, "settings.sigma_vision", sigma_vision);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.lens_hfov", lens_hfov);
  parse(config, "cam0.lens_vfov", lens_vfov);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);

  // Setup camera intrinsics and distortion
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {(int) resolution[0], (int) resolution[1]};
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      proj_params, dist_params};

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target in [%s]!", config_file.c_str());
    return -1;
  }

  // Prepare aprilgrid data directory
  const auto grid_path = data_path + "/grid0/cam0/data";
  if (dir_exists(grid_path) == false) {
    dir_create(grid_path);
  }

  // Preprocess calibration data
  const auto cam_path = data_path + "/cam0/data";
  if (preprocess_camera_data(calib_target, cam_path, grid_path, imshow) != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load calibration data
  aprilgrids_t grids;
  if (load_camera_calib_data(grid_path, grids) != 0) {
    LOG_ERROR("Failed to load camera calibration data!");
    return -1;
  }

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  std::vector<double> errs;

	calib_mono_data_t data{grids, cam, I(2) * pow(sigma_vision, 2)};
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    calib_mono_solve<pinhole_radtan4_t>(data);
    reproj_errors<pinhole_radtan4_t>(data.grids, data.cam_params, data.poses, errs);
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    calib_mono_solve<pinhole_equi4_t>(data);
    reproj_errors<pinhole_equi4_t>(data.grids, data.cam_params, data.poses, errs);
  } else {
    LOG_ERROR("[%s-%s] unsupported!", proj_model.c_str(), dist_model.c_str());
    return -1;
  }

  // Save results
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, cam, rmse(errs), mean(errs)) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

// void estimate_covar() {
//   // Push calibration params to recover covariance
//   std::vector<std::shared_ptr<ParameterBlock>> recover_params;
//   recover_params.push_back(T_SC_blocks_[0]);
//   recover_params.push_back(T_SC_blocks_[1]);
//   recover_params.push_back(cam_blocks_[0]);
//   recover_params.push_back(cam_blocks_[1]);
//
//   std::vector<std::pair<const double *, const double *>> covar_blocks;
//   for (size_t i = 0; i < recover_params.size(); i++) {
//     auto param_i = recover_params[i]->parameters();
//     for (size_t j = i; j < recover_params.size(); j++) {
//       auto param_j = recover_params[j]->parameters();
//       covar_blocks.push_back({param_i, param_j});
//     }
//   }
//
//   // Estimate covariance options
//   ::ceres::Covariance::Options options;
//   options.num_threads = 1;
//   options.algorithm_type = ::ceres::SPARSE_QR;
//
//   // Estimate covariance
//   ::ceres::Covariance covar_est(options);
//   auto problem_ptr = problem_->problem_.get();
//   if (covar_est.Compute(covar_blocks, problem_ptr) == false) {
//     LOG_ERROR("Failed to estimate covariance!");
//     LOG_ERROR("Maybe Hessian is not full rank?");
//     return -1;
//   }
//
//   // Form covariance matrix
//   calib_covar = zeros(28, 28);
//   size_t idx_i = 0;
//   size_t idx_j = 0;
//   for (size_t i = 0; i < recover_params.size(); i++) {
//     auto param_i = recover_params[i]->parameters();
//     auto size_i = recover_params[i]->minimalDimension();
//
//     for (size_t j = i; j < recover_params.size(); j++) {
//       auto param_j = recover_params[j]->parameters();
//       auto size_j = recover_params[j]->minimalDimension();
//
//       // Get covariance block
//       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covar_block;
//       covar_block.resize(size_i, size_j);
//       covar_est.GetCovarianceBlockInTangentSpace(param_i, param_j, covar_block.data());
//
//       if (i == j) {
//         // Diagonal block
//         calib_covar.block(idx_i, idx_j, size_i, size_j) = covar_block;
//       } else {
//         // Off-diagonal block
//         calib_covar.block(idx_i, idx_j, size_i, size_j) = covar_block;
//         calib_covar.block(idx_j, idx_i, size_j, size_i) = covar_block.transpose();
//       }
//
//       idx_j += size_j;
//     }
//     idx_i += size_i;
//     idx_j = idx_i;
//   }
//
//   // Check if calib_covar is full-rank?
//   if (rank(calib_covar) != calib_covar.rows()) {
//     LOG_ERROR("calib_covar is not full rank!");
//     return -1;
//   }
// }

} //  namespace yac
