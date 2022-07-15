#include "calib_mocap.hpp"

namespace yac {

calib_mocap_t::calib_mocap_t(const std::string &config_file_,
                             const std::string &data_path_)
    : config_file{config_file_}, data_path{data_path_} {
  // Load calib config file
  std::vector<int> resolution;
  std::string proj_model;
  std::string dist_model;
  vecx_t proj_params;
  vecx_t dist_params;
  config_t config{config_file};
  parse(config, "settings.fix_intrinsics", fix_intrinsics);
  parse(config, "settings.fix_mocap_poses", fix_mocap_poses);
  parse(config, "settings.fix_fiducial_pose", fix_fiducial_pose);
  parse(config, "settings.imshow", imshow);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.proj_model", proj_model);
  parse(config, "cam0.dist_model", dist_model);
  parse(config, "cam0.proj_params", proj_params, true);
  parse(config, "cam0.dist_params", dist_params, true);

  // Setup paths
  const auto cam0_path = data_path + "/cam0/data";
  const auto grid0_path = data_path + "/grid0/cam0/data";
  const auto body0_csv_path = data_path + "/body0/data.csv";
  const auto target0_csv_path = data_path + "/target0/data.csv";

  // Setup calibration target
  if (calib_target.load(config_file, "calib_target") != 0) {
    FATAL("Failed to load calib target in [%s]!", config_file.c_str());
  }

  // Setup camera geometry
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    camera_geometry = std::make_shared<pinhole_radtan4_t>();
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    camera_geometry = std::make_shared<pinhole_equi4_t>();
  } else {
    FATAL("Unsupported [%s]-[%s]!", proj_model.c_str(), dist_model.c_str());
  }

  // Initialize / load camera parameters
  if (yaml_has_key(config, "cam0.proj_params") == false) {
    LOG_INFO("Camera parameters unknown!");
    LOG_INFO("Calibrating camera intrinsics!");
    const auto cam_grids = _preprocess(calib_target, cam0_path, grid0_path);
    calib_camera_t calib{calib_target};
    calib.add_camera_data(0, cam_grids);
    calib.add_camera(0, resolution.data(), proj_model, dist_model);
    calib.solve();
    camera =
        camera_params_t::init(0, resolution.data(), proj_model, dist_model);
    camera.param = calib.get_camera_params(0);
  } else {
    camera = camera_params_t{0,
                             resolution.data(),
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};
  }

  // Load dataset
  // -- Fiducial target pose
  const mat4_t T_WF = load_pose(target0_csv_path);
  fiducial_pose = pose_t{0, T_WF};
  // -- Mocap poses
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_poses(body0_csv_path, body_timestamps, body_poses);
  // -- Synchronize aprilgrids and body poses
  mat4s_t T_WM;
  const aprilgrids_t grids_raw = load_aprilgrids(grid0_path);
  lerp_body_poses(grids_raw, body_timestamps, body_poses, grids, T_WM);
  for (size_t k = 0; k < grids.size(); k++) {
    const auto ts = grids[k].timestamp;
    mocap_poses[ts] = pose_t{ts, T_WM[k]};
  }
  // -- Mocap marker to camera transform
  mat4_t T_MC0;
  for (const auto &grid : grids) {
    const auto ts = grid.timestamp;
    const mat4_t T_MW = mocap_poses[ts].tf().inverse();

    const auto cam_res = camera.resolution;
    const auto cam_param = camera.param;
    mat4_t T_C0F;
    if (grid.estimate(camera_geometry.get(), cam_res, cam_param, T_C0F) != 0) {
      continue;
    }

    T_MC0 = T_MW * T_WF * T_C0F.inverse();
    break;
  }
  mocap_camera_extrinsics = extrinsics_t{T_MC0};
}

int calib_mocap_t::get_num_views() const { return mocap_poses.size(); }

std::vector<real_t> calib_mocap_t::get_reproj_errors() const {
  std::vector<real_t> reproj_errors;
  for (const auto &res_fn : residuals) {
    real_t error;
    res_fn->get_reproj_error(error);
    reproj_errors.push_back(error);
  }
  return reproj_errors;
}

mat4_t calib_mocap_t::get_fiducial_pose() const { return fiducial_pose.tf(); }

mat4_t calib_mocap_t::get_mocap_camera_extrinsics() const {
  return mocap_camera_extrinsics.tf();
}

vecx_t calib_mocap_t::get_camera_params() const { return camera.param; }

aprilgrids_t calib_mocap_t::_preprocess(const calib_target_t &calib_target,
                                        const std::string &cam_path,
                                        const std::string &grid_path) {
  LOG_INFO("Preprocessing camera data [%s]", cam_path.c_str());

  // Setup AprilGrid detector
  const aprilgrid_detector_t detector(calib_target.tag_rows,
                                      calib_target.tag_cols,
                                      calib_target.tag_size,
                                      calib_target.tag_spacing);

  // Detect AprilGrids
  // -- Get images
  std::vector<std::string> image_paths;
  if (list_dir(cam_path, image_paths) != 0) {
    FATAL("Failed to traverse dir [%s]!", cam_path.c_str());
  }
  std::sort(image_paths.begin(), image_paths.end());

  // -- Detect AprilGrids
  aprilgrids_t grids;
  for (auto &image_path : image_paths) {
    image_path = paths_join(cam_path, image_path);
    const auto ts = std::stoull(parse_fname(image_path));
    const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    const auto grid = detector.detect(ts, image);
    grid.save(grid_path + "/" + std::to_string(ts) + ".csv");
    grids.push_back(grid);

    printf(".");
    fflush(stdout);
  }
  printf("\n");

  return grids;
}

void calib_mocap_t::_add_view(const aprilgrid_t &grid) {
  // Add mocap pose to problem
  const auto ts = grid.timestamp;
  problem->AddParameterBlock(mocap_poses[ts].param.data(), 7);
  problem->SetParameterization(mocap_poses[ts].param.data(), &pose_plus);
  if (fix_mocap_poses) {
    mocap_poses[ts].fixed = true;
    problem->SetParameterBlockConstant(mocap_poses[ts].param.data());
  }

  // Add reprojection error to problem
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  const mat2_t covar = I(2);
  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = keypoints[i];
    const vec3_t r_FFi = object_points[i];

    auto res_fn = std::make_shared<mocap_residual_t>(camera_geometry.get(),
                                                     &camera,
                                                     &fiducial_pose,
                                                     &mocap_poses[ts],
                                                     &mocap_camera_extrinsics,
                                                     tag_id,
                                                     corner_idx,
                                                     r_FFi,
                                                     z,
                                                     covar);
    residuals.push_back(res_fn);

    problem->AddResidualBlock(res_fn.get(),
                              NULL,
                              fiducial_pose.param.data(),
                              mocap_poses[ts].param.data(),
                              mocap_camera_extrinsics.param.data(),
                              camera.param.data());
  }
}

int calib_mocap_t::solve() {
  assert(grids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == grids.size());

  // Setup optimization problem
  ceres::Problem::Options options;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  // options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.enable_fast_removal = true;
  problem = std::make_unique<ceres::Problem>(options);

  // Set ficuial and marker-cam pose parameterization
  problem->AddParameterBlock(camera.param.data(), camera.global_size);
  problem->AddParameterBlock(fiducial_pose.param.data(), 7);
  problem->AddParameterBlock(mocap_camera_extrinsics.param.data(), 7);
  problem->SetParameterization(fiducial_pose.param.data(), &pose_plus);
  problem->SetParameterization(mocap_camera_extrinsics.param.data(),
                               &pose_plus);

  // Fix camera parameters
  if (fix_intrinsics) {
    camera.fixed = true;
    problem->SetParameterBlockConstant(camera.param.data());
  }

  // Fix fiducial pose - assumes camera intrincs and PnP is good
  if (fix_fiducial_pose) {
    fiducial_pose.fixed = true;
    problem->SetParameterBlockConstant(fiducial_pose.param.data());
  }

  // Build problem
  for (const auto grid : grids) {
    _add_view(grid);
  }

  // Set solver options
  ceres::Solver::Options solver_options;
  solver_options.minimizer_progress_to_stdout = show_progress;
  solver_options.max_num_iterations = max_iter;
  // solver_options.check_gradients = true;

  // Solve
  LOG_INFO("Calibrating mocap-marker to camera extrinsics ...");
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, problem.get(), &summary);
  // std::cout << summary.BriefReport() << std::endl;
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  // Show results
  show_results();

  return 0;
}

void calib_mocap_t::print_settings(FILE *out) const {
  // clang-format off
  fprintf(out, "settings:\n");
  fprintf(out, "  fix_intrinsics: %s\n", fix_intrinsics ? "true" : "false");
  fprintf(out, "  fix_mocap_poses: %s\n", fix_mocap_poses ? "true" : "false");
  fprintf(out, "  fix_fiducial_pose: %s\n", fix_fiducial_pose ? "true" : "false");
  fprintf(out, "  imshow: %s\n", imshow ? "true" : "false");
  fprintf(out, "  show_progress: %s\n", show_progress ? "true" : "false");
  fprintf(out, "  max_iter: %d\n", max_iter);
  fprintf(out, "\n");
  // clang-format on
}

void calib_mocap_t::print_calib_target(FILE *out) const {
  fprintf(out, "calib_target:\n");
  fprintf(out, "  target_type: \"%s\"\n", calib_target.target_type.c_str());
  fprintf(out, "  tag_rows: %d\n", calib_target.tag_rows);
  fprintf(out, "  tag_cols: %d\n", calib_target.tag_cols);
  fprintf(out, "  tag_size: %f\n", calib_target.tag_size);
  fprintf(out, "  tag_spacing: %f\n", calib_target.tag_spacing);
  fprintf(out, "\n");
}

void calib_mocap_t::print_metrics(FILE *out) const {
  const auto reproj_errors = get_reproj_errors();
  fprintf(out, "total_reproj_error:\n");
  fprintf(out, "  nb_views: %d\n", get_num_views());
  fprintf(out, "  nb_corners: %ld\n", reproj_errors.size());
  fprintf(out, "  rmse:   %.4f # [px]\n", rmse(reproj_errors));
  fprintf(out, "  mean:   %.4f # [px]\n", mean(reproj_errors));
  fprintf(out, "  median: %.4f # [px]\n", median(reproj_errors));
  fprintf(out, "  stddev: %.4f # [px]\n", stddev(reproj_errors));
  fprintf(out, "\n");
}

void calib_mocap_t::print_fiducial_pose(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const mat4_t T_WF = get_fiducial_pose();
  fprintf(out, "T_world_fiducial:\n");
  fprintf(out, "  fixed: %s\n", fiducial_pose.fixed ? "true" : "false");
  fprintf(out, "  rows: 4\n");
  fprintf(out, "  cols: 4\n");
  fprintf(out, "  data: [\n");
  fprintf(out, "%s\n", mat2str(T_WF, "    ", max_digits).c_str());
  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::print_mocap_camera_extrinsics(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const mat4_t T_MC0 = get_mocap_camera_extrinsics();
  const bool fixed = mocap_camera_extrinsics.fixed;
  fprintf(out, "T_mocap_camera:\n");
  fprintf(out, "  fixed: %s\n", fixed ? "true" : "false");
  fprintf(out, "  rows: 4\n");
  fprintf(out, "  cols: 4\n");
  fprintf(out, "  data: [\n");
  fprintf(out, "%s\n", mat2str(T_MC0, "    ", max_digits).c_str());
  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::print_camera_params(FILE *out) const {
  const bool max_digits = (out == stdout) ? false : true;
  const bool fixed = camera.fixed;
  const int *cam_res = camera.resolution;
  const char *proj_model = camera.proj_model.c_str();
  const char *dist_model = camera.dist_model.c_str();
  const auto proj_params = vec2str(camera.proj_params(), true, max_digits);
  const auto dist_params = vec2str(camera.dist_params(), true, max_digits);

  fprintf(out, "cam%d:\n", 0);
  fprintf(out, "  fixed: %s\n", fixed ? "true" : "false");
  fprintf(out, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
  fprintf(out, "  proj_model: \"%s\"\n", proj_model);
  fprintf(out, "  dist_model: \"%s\"\n", dist_model);
  fprintf(out, "  proj_params: %s\n", proj_params.c_str());
  fprintf(out, "  dist_params: %s\n", dist_params.c_str());
  fprintf(out, "\n");
}

void calib_mocap_t::print_mocap_poses(FILE *out) const {
  fprintf(out, "mocap_poses:\n");
  fprintf(out, "  rows: %d\n", get_num_views());
  fprintf(out, "  cols: 8\n");
  fprintf(out, "\n");
  fprintf(out, "  # ts, rx, ry, rz, qx, qy, qz, qw\n");
  fprintf(out, "  data: [\n");

  auto iter = mocap_poses.begin();
  while (iter != mocap_poses.end()) {
    const auto ts = iter->first;
    const auto pose = iter->second;
    const mat4_t T_C0F = pose.tf();
    const vec3_t r = tf_trans(T_C0F);
    const quat_t q = tf_quat(T_C0F);
    fprintf(out, "    ");
    fprintf(out, "%ld,", ts);
    fprintf(out, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(out, "%f,%f,%f,%f", q.x(), q.y(), q.z(), q.w());

    iter++;
    if (iter != mocap_poses.end()) {
      fprintf(out, ",\n");
    } else {
      fprintf(out, "\n");
    }
  }

  fprintf(out, "  ]\n");
  fprintf(out, "\n");
}

void calib_mocap_t::show_results() const {
  printf("Optimization results:\n");
  printf("---------------------\n");
  print_settings(stdout);
  print_calib_target(stdout);
  print_metrics(stdout);
  print_camera_params(stdout);
  print_mocap_camera_extrinsics(stdout);
  print_fiducial_pose(stdout);
}

int calib_mocap_t::save_results(const std::string &save_path) const {
  LOG_INFO(KGRN "Saved results to [%s]" KNRM, save_path.c_str());

  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  print_settings(outfile);
  print_calib_target(outfile);
  print_metrics(outfile);
  print_mocap_camera_extrinsics(outfile);
  print_camera_params(outfile);
  print_fiducial_pose(outfile);
  print_mocap_poses(outfile);
  fclose(outfile);

  return 0;
}

} //  namespace yac
