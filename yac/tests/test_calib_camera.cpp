#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

// GLOBAL VARIABLES
std::map<int, aprilgrids_t> test_data;
std::map<int, aprilgrids_t> valid_data;

void load_test_dataset() {
  // Calibration data
  const calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  const std::string data_path = "/data/euroc/cam_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  test_data = calib_data_preprocess(calib_target, cam_paths, grids_path);
}

void load_validation_dataset() {
  // Load test data for validation
  const calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  valid_data = calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int test_calib_view() {
  // Setup
  ceres::Problem problem;
  ceres::CauchyLoss loss(1.0);

  timestamp_t ts = 0;
  CamIdx2Grids grids;
  calib_target_t calib_target;
  fiducial_corners_t corners{calib_target};
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;
  const mat4_t T_C0F = I(4);
  pose_t rel_pose{0, T_C0F};

  calib_view_t(ts,
               grids,
               &corners,
               &problem,
               &loss,
               &cam_geoms,
               &cam_params,
               &cam_exts,
               &rel_pose);

  return 0;
}

int test_calib_camera_add_camera_data() {
  calib_target_t calib_target;
  calib_camera_t calib{calib_target};
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  MU_CHECK(calib.calib_data.size() > 0);
  MU_CHECK(calib.timestamps.size() > 0);
  MU_CHECK(calib.timestamps.size() == calib.calib_data.size());

  return 0;
}

int test_calib_camera_add_camera() {
  calib_target_t calib_target;
  calib_camera_t calib{calib_target};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);

  MU_CHECK(calib.cam_params.size() == 2);
  MU_CHECK(calib.cam_params[0]->resolution[0] == 752);
  MU_CHECK(calib.cam_params[0]->resolution[1] == 480);
  MU_CHECK(calib.cam_params[0]->proj_model == proj_model);
  MU_CHECK(calib.cam_params[0]->dist_model == dist_model);
  MU_CHECK(calib.cam_params[0]->fixed == false);

  MU_CHECK(calib.cam_params[1]->resolution[0] == 752);
  MU_CHECK(calib.cam_params[1]->resolution[1] == 480);
  MU_CHECK(calib.cam_params[1]->proj_model == proj_model);
  MU_CHECK(calib.cam_params[1]->dist_model == dist_model);
  MU_CHECK(calib.cam_params[1]->fixed == false);

  return 0;
}

int test_calib_camera_add_camera_extrinsics() {
  mat4_t cam0_exts = tf(quat_t{1.0, 0.0, 0.0, 0.0}, vec3_t{0.1, 0.2, 0.3});
  mat4_t cam1_exts = tf(quat_t{1.0, 0.0, 0.0, 0.0}, vec3_t{0.4, 0.5, 0.6});
  calib_target_t calib_target;
  calib_camera_t calib{calib_target};
  calib.add_camera_extrinsics(0, cam0_exts);
  calib.add_camera_extrinsics(1, cam1_exts);

  MU_CHECK(calib.cam_exts.size() == 2);
  MU_CHECK(calib.cam_exts[0]->tf().isApprox(cam0_exts));
  MU_CHECK(calib.cam_exts[0]->fixed == true);
  MU_CHECK(calib.cam_exts[1]->tf().isApprox(cam1_exts));
  MU_CHECK(calib.cam_exts[1]->fixed == false);

  return 0;
}

int test_calib_camera_add_pose() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  aprilgrid_t test_grid;
  for (const auto grid : test_data.at(0)) {
    if (grid.detected) {
      test_grid = grid;
      break;
    }
  }

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_pose(0, test_grid, false);
  MU_CHECK(calib.timestamps.count(test_grid.timestamp));
  MU_CHECK(calib.calib_data.count(test_grid.timestamp));
  MU_CHECK(calib.calib_data[test_grid.timestamp].count(0));
  MU_CHECK(calib.poses.count(test_grid.timestamp));

  return 0;
}

int test_calib_camera_add_and_remove_view() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  // Add view
  timestamp_t view_ts = 0;
  for (const auto ts : calib.timestamps) {
    size_t detected = 0;
    for (const auto [cam_idx, grid] : calib.calib_data[ts]) {
      detected += (grid.detected) ? 1 : 0;
    }
    if (detected != 2) {
      continue;
    }

    calib.add_view(calib.calib_data[ts]);
    view_ts = ts;
    break;
  }
  MU_CHECK(calib.poses.count(view_ts));
  MU_CHECK(calib.window.size() == 1);
  MU_CHECK(calib.calib_views.size() == 1);
  MU_CHECK(calib.calib_views.front()->get_camera_indices().size() == 2);
  MU_CHECK(calib.problem->NumResidualBlocks() > 0);
  MU_CHECK(calib.problem->NumParameterBlocks() == 149);
  // 144 corners + 2 camera parameters + 2 camera extrinsics + 1 pose

  // Remove view
  calib.remove_view(view_ts);
  MU_CHECK(calib.poses.count(view_ts) == 0);
  MU_CHECK(calib.window.size() == 0);
  MU_CHECK(calib.calib_views.size() == 0);
  MU_CHECK(calib.problem->NumResidualBlocks() == 0);
  MU_CHECK(calib.problem->NumParameterBlocks() == 148);
  // 144 corners + 2 camera parameters + 2 camera extrinsics

  return 0;
}

int test_calib_camera_recover_calib_covar() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  for (const auto ts : calib.timestamps) {
    calib.add_view(calib.calib_data[ts]);
    if (calib.nb_views() > 10) {
      break;
    }
  }

  matx_t calib_covar;
  MU_CHECK(calib.recover_calib_covar(calib_covar) == 0);

  return 0;
}

int test_calib_camera_find_nbv() {
  // Calibration target and camera parameters
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibration data
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  for (const auto grid : test_data.at(0)) {
    if (grid.detected) {
      cam0_grids.push_back(grid);
    }
    if (cam0_grids.size() >= 5) {
      break;
    }
  }
  for (const auto grid : test_data.at(1)) {
    if (grid.detected) {
      cam1_grids.push_back(grid);
    }
    if (cam1_grids.size() >= 5) {
      break;
    }
  }

  // Calibrator
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.enable_outlier_filter = false;
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_data(0, cam0_grids);
  calib.add_camera_data(1, cam1_grids);
  calib.solve();

  // Next best view poses
  auto nbv_poses = calib_nbv_poses(calib_target,
                                   calib.cam_geoms,
                                   calib.cam_params,
                                   calib.cam_exts);

  int cam_idx = 0;
  int nbv_idx = 0;
  calib.find_nbv(nbv_poses, cam_idx, nbv_idx);
  printf("cam_idx: %d\n", cam_idx);
  printf("nbv_idx: %d\n", nbv_idx);
  calib.solve();

  return 0;
}

int test_calib_camera_filter_view() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  timestamp_t last_ts = 0;
  for (const auto ts : calib.timestamps) {
    last_ts = ts;
    calib.add_view(calib.calib_data[ts]);
    if (calib.nb_views() > 10) {
      break;
    }
  }

  calib._filter_view(last_ts);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, calib.problem, &summary);

  return 0;
}

int test_calib_camera_filter_all_views() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  for (const auto ts : calib.timestamps) {
    calib.add_view(calib.calib_data[ts]);
    if (calib.nb_views() > 10) {
      break;
    }
  }
  calib._filter_all_views();
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, calib.problem, &summary);

  return 0;
}

int test_calib_camera_eval_nbv() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const calib_target_t calib_target;

  calib_camera_t calib{calib_target};
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera_data(1, test_data.at(1));

  timestamp_t last_ts = 0;
  for (const auto ts : calib.timestamps) {
    calib.add_view(calib.calib_data[ts]);
    if (calib.nb_views() > 10) {
      break;
    }
  }
  calib._eval_nbv(last_ts);

  return 0;
}

int test_calib_camera_solve_batch() {
  // Setup
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib._solve_batch(true);
  calib.show_results();

  return 0;
}

int test_calib_camera_solve_inc() {
  // Setup
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera_extrinsics(0);
  calib._solve_inc();

  return 0;
}

int test_calib_camera_mono() {
  // Setup
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.add_camera_data(0, test_data.at(0));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.solve();

  return 0;
}

int test_calib_camera_stereo() {
  // Load training data
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate
  calib_camera_t calib{target};
  calib.enable_nbv = false;
  calib.add_camera_data(test_data);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  // calib.save_results("/tmp/calib-results.yaml");
  // calib.save_estimates("/tmp/calib-estimates.yaml");
  // calib.validate(valid_data);

  return 0;
}

int test_marg_error() {
  const size_t max_num_views = 20;

  // Calibration data
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  for (const auto grid : test_data.at(0)) {
    if (grid.detected) {
      cam0_grids.push_back(grid);
    }
    if (cam0_grids.size() >= max_num_views) {
      break;
    }
  }
  for (const auto grid : test_data.at(1)) {
    if (grid.detected) {
      cam1_grids.push_back(grid);
    }
    if (cam1_grids.size() >= max_num_views) {
      break;
    }
  }

  // Calibrator
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  calib_camera_t calib{target};
  calib.enable_nbv = false;
  calib.add_camera_data(0, cam0_grids);
  calib.add_camera_data(1, cam1_grids);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  // calib._initialize_intrinsics();
  // calib._initialize_extrinsics();
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);

  // Build problem
  for (const auto ts : calib.timestamps) {
    if (calib.calib_data[ts].size() != 2) {
      continue;
    }

    calib.add_view(calib.calib_data[ts]);
    if (calib.nb_views() > max_num_views) {
      break;
    }
  }

  // Test marginalization error
  int nb_res_blocks = calib.problem->NumResidualBlocks();
  int nb_param_blocks = calib.problem->NumParameterBlocks();
  marg_error_t marg_error;
  auto view = calib.calib_views.front();
  view->marginalize(&marg_error);
  // -- Asserts
  nb_res_blocks -= (marg_error.res_blocks_.size() - 1);
  nb_param_blocks -= 1;
  MU_CHECK(nb_res_blocks == calib.problem->NumResidualBlocks());
  MU_CHECK(nb_param_blocks == calib.problem->NumParameterBlocks());

  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 50;
  // options.check_gradients = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, calib.problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  calib.show_results();

  return 0;
}

// int test_calib_camera_kalibr_data() {
//   // Load AprilGrid data
//   const std::string grid0_path = "/tmp/yac_data/cam0";
//   const std::string grid1_path = "/tmp/yac_data/cam1";
//   std::vector<std::string> grid0_csvs;
//   std::vector<std::string> grid1_csvs;
//   list_files(grid0_path, grid0_csvs);
//   list_files(grid1_path, grid1_csvs);
//
//   std::map<int, aprilgrids_t> cam_grids;
//   for (const auto csv_path : grid0_csvs) {
//     aprilgrid_t grid;
//     grid.load(grid0_path + "/" + csv_path);
//     cam_grids[0].push_back(grid);
//   }
//   for (const auto csv_path : grid1_csvs) {
//     aprilgrid_t grid;
//     grid.load(grid1_path + "/" + csv_path);
//     cam_grids[1].push_back(grid);
//   }
//
//   // Calibrate
//   const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
//   calib_camera_t calib{calib_target};
//   calib.enable_nbv = false;
//   calib.enable_outlier_filter = false;
//   calib.initialized = true;
//   calib.add_camera_data(0, cam_grids.at(0));
//   calib.add_camera_data(1, cam_grids.at(1));
//
//   // clang-format off
//   const int cam_res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t cam0_proj_params{460.285798,
//   459.066980,368.299812,244.660910}; const vec4_t
//   cam0_dist_params{-0.276158,0.067779,0.000757,-0.000299}; const vec4_t
//   cam1_proj_params{458.931838, 457.621130, 378.783646, 251.341323}; const
//   vec4_t cam1_dist_params{-0.272847, 0.064882, 0.000506, -0.000210}; const
//   vec3_t r_C1C0{-0.109948,0.000514,-0.000241}; const quat_t q_C1C0{0.999970,
//   -0.007147,0.002650,-0.001263}; const mat4_t T_C1C0 = tf(q_C1C0, r_C1C0);
//   const mat4_t T_C0C1 = T_C1C0.inverse();
//
//   calib.add_camera(0,
//                    cam_res,
//                    proj_model,
//                    dist_model,
//                    cam0_proj_params,
//                    cam0_dist_params);
//   calib.add_camera(1,
//                    cam_res,
//                    proj_model,
//                    dist_model,
//                    cam1_proj_params,
//                    cam1_dist_params);
//   calib.add_camera_extrinsics(0, I(4), true);
//   calib.add_camera_extrinsics(1, T_C0C1, false);
//   // clang-format on
//
//   calib.solve();
//   // calib.save_results("/tmp/calib-results.yaml");
//   calib.validate(valid_data);
//
//   return 0;
// }

void test_suite() {
  load_test_dataset();
  // load_validation_dataset();

  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_calib_camera_add_camera_data);
  MU_ADD_TEST(test_calib_camera_add_camera);
  MU_ADD_TEST(test_calib_camera_add_camera_extrinsics);
  MU_ADD_TEST(test_calib_camera_add_pose);
  MU_ADD_TEST(test_calib_camera_add_and_remove_view);
  MU_ADD_TEST(test_calib_camera_recover_calib_covar);
  // MU_ADD_TEST(test_calib_camera_find_nbv);
  MU_ADD_TEST(test_calib_camera_filter_view);
  MU_ADD_TEST(test_calib_camera_filter_all_views);
  MU_ADD_TEST(test_calib_camera_eval_nbv);
  MU_ADD_TEST(test_calib_camera_solve_batch);
  MU_ADD_TEST(test_calib_camera_solve_inc);
  MU_ADD_TEST(test_calib_camera_mono);
  MU_ADD_TEST(test_calib_camera_stereo);
  MU_ADD_TEST(test_marg_error);
  // MU_ADD_TEST(test_calib_camera_kalibr_data);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
