#include "calib_stereo.hpp"

namespace yac {

static int process_aprilgrid(const aprilgrid_t &cam0_aprilgrid,
                             const aprilgrid_t &cam1_aprilgrid,
                             calib_params_t &cam0_params,
                             calib_params_t &cam1_params,
                             calib_pose_t *T_C0C1,
                             calib_pose_t *T_C0F,
                             ceres::Problem *problem) {
  for (const auto &tag_id : cam0_aprilgrid.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_get(cam0_aprilgrid, tag_id, cam0_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_get(cam1_aprilgrid, tag_id, cam1_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(cam0_aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp0 = cam0_keypoints[i];
      const auto kp1 = cam1_keypoints[i];
      const auto obj_pt = object_points[i];
      const auto residual = new calib_stereo_residual_t{cam0_params, cam1_params,
                                                        kp0, kp1, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<calib_stereo_residual_t,
                                          4, // Size of: residual
                                          4, // Size of: cam0_intrinsics
                                          4, // Size of: cam0_distortion
                                          4, // Size of: cam1_intrinsics
                                          4, // Size of: cam1_distortion
                                          4, // Size of: q_C0C1
                                          3, // Size of: t_C0C1
                                          4, // Size of: q_C0F
                                          3  // Size of: t_C0F
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                cam0_params.proj_params.data(),
                                cam0_params.dist_params.data(),
                                cam1_params.proj_params.data(),
                                cam1_params.dist_params.data(),
                                T_C0C1->q,
                                T_C0C1->r,
                                T_C0F->q,
                                T_C0F->r);
    }
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const calib_params_t &cam0,
                        const calib_params_t &cam1,
                        const mat4_t &T_C0C1) {
  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  {
    const char *proj_model = cam0.proj_model.c_str();
    const char *dist_model = cam0.dist_model.c_str();
    const std::string proj_params = arr2str(cam0.proj_params.data(), 4);
    const std::string dist_params = arr2str(cam0.dist_params.data(), 4);
    fprintf(outfile, "cam0:\n");
    fprintf(outfile, "  resolution: [%d, %d]\n", cam0.img_w, cam0.img_h);
    fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
    fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
    fprintf(outfile, "  proj_params: %s\n", proj_params.c_str());
    fprintf(outfile, "  dist_params: %s\n", dist_params.c_str());
    fprintf(outfile, "\n");
  }
  {
    const char *proj_model = cam1.proj_model.c_str();
    const char *dist_model = cam1.dist_model.c_str();
    const std::string proj_params = arr2str(cam1.proj_params.data(), 4);
    const std::string dist_params = arr2str(cam1.dist_params.data(), 4);
    fprintf(outfile, "cam1:\n");
    fprintf(outfile, "  resolution: [%d, %d]\n", cam1.img_w, cam1.img_h);
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

int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_aprilgrids,
                       const std::vector<aprilgrid_t> &cam1_aprilgrids,
                       calib_params_t &cam0_params,
                       calib_params_t &cam1_params,
                       mat4_t &T_C0C1,
                       mat4s_t &T_C0F) {
  assert(cam0_aprilgrids.size() == cam1_aprilgrids.size());

  // Optimization variables
  calib_pose_t extrinsic_param{T_C0C1};
  std::vector<calib_pose_t> pose_params;
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    pose_params.emplace_back(cam0_aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  // clang-format off
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
  ceres::EigenQuaternionParameterization quaternion_parameterization;
  // clang-format on

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    int retval = process_aprilgrid(cam0_aprilgrids[i],
                                   cam1_aprilgrids[i],
                                   cam0_params,
                                   cam1_params,
                                   &extrinsic_param,
                                   &pose_params[i],
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(pose_params[i].q,
                                 &quaternion_parameterization);
  }
  problem->SetParameterization(extrinsic_param.q,
                               &quaternion_parameterization);

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Finish up
  T_C0C1 = extrinsic_param.T();
  for (auto pose_param : pose_params) {
    T_C0F.emplace_back(pose_param.T());
  }

  return 0;
}


int calib_stereo_solve(const std::string &config_file) {
  // Calibration settings
  std::string data_path;
  std::string results_fpath;

  vec2_t cam0_resolution{0.0, 0.0};
  real_t cam0_lens_hfov = 0.0;
  real_t cam0_lens_vfov = 0.0;
  std::string cam0_proj_model;
  std::string cam0_dist_model;

  vec2_t cam1_resolution{0.0, 0.0};
  real_t cam1_lens_hfov = 0.0;
  real_t cam1_lens_vfov = 0.0;
  std::string cam1_proj_model;
  std::string cam1_dist_model;

  // Parse calibration config
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", results_fpath);
  parse(config, "cam0.resolution", cam0_resolution);
  parse(config, "cam0.lens_hfov", cam0_lens_hfov);
  parse(config, "cam0.lens_vfov", cam0_lens_vfov);
  parse(config, "cam0.proj_model", cam0_proj_model);
  parse(config, "cam0.dist_model", cam0_dist_model);
  parse(config, "cam1.resolution", cam1_resolution);
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
  calib_params_t cam0_params(cam0_proj_model, cam0_dist_model,
                             cam0_resolution(0), cam0_resolution(1),
                             cam0_lens_hfov, cam0_lens_vfov);
  calib_params_t cam1_params(cam1_proj_model, cam1_dist_model,
                             cam1_resolution(0), cam1_resolution(1),
                             cam1_lens_hfov, cam1_lens_vfov);

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C0C1 = I(4);
  mat4s_t T_C0F;
  retval = calib_stereo_solve(cam0_aprilgrids,
                              cam1_aprilgrids,
                              cam0_params,
                              cam1_params,
                              T_C0C1,
                              T_C0F);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << cam0_params.toString(0) << std::endl;
  std::cout << cam1_params.toString(1) << std::endl;
  std::cout << "T_C0C1:\n" << T_C0C1 << std::endl;
  std::cout << std::endl;
  // calib_mono_stats(cam0_aprilgrids, cam0_params, T_C0F);

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_fpath.c_str());
  retval = save_results(results_fpath, cam0_params, cam1_params, T_C0C1);
  if (retval != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  return 0;
}

} //  namespace yac
