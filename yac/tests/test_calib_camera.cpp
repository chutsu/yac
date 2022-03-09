#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_CAM_CALIB_PATH TEST_PATH "/test_data/calib/cam_april"
#define MONO_CONF TEST_PATH "/test_data/calib/cam_april/mono_config.yaml"
#define STEREO_CONF TEST_PATH "/test_data/calib/cam_april/stereo_config.yaml"
#define TEST_IMUCAM_DATA TEST_PATH "/test_data/calib/imu_april"

// GLOBAL VARIABLES
std::map<int, aprilgrids_t> test_data;
std::map<int, aprilgrids_t> valid_data;

void load_test_dataset() {
  std::vector<std::string> cam0_files;
  std::vector<std::string> cam1_files;
  list_files(TEST_CAM_CALIB_PATH "/cam0", cam0_files);
  list_files(TEST_CAM_CALIB_PATH "/cam1", cam1_files);

  for (auto grid_path : cam0_files) {
    grid_path = std::string(TEST_CAM_CALIB_PATH "/cam0/") + grid_path;
    test_data[0].emplace_back(grid_path);
  }
  for (auto grid_path : cam1_files) {
    grid_path = std::string(TEST_CAM_CALIB_PATH "/cam1/") + grid_path;
    test_data[1].emplace_back(grid_path);
  }
}

void load_validation_dataset() {
  std::vector<std::string> cam0_files;
  std::vector<std::string> cam1_files;
  list_files(TEST_IMUCAM_DATA "/cam0", cam0_files);
  list_files(TEST_IMUCAM_DATA "/cam1", cam1_files);

  for (auto grid_path : cam0_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam0/") + grid_path;
    valid_data[0].emplace_back(grid_path);
  }
  for (auto grid_path : cam1_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam1/") + grid_path;
    valid_data[1].emplace_back(grid_path);
  }
}

int test_calib_view() {
  // Setup
  ceres_solver_t solver;

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
               &solver,
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

  const timestamp_t ts = test_grid.timestamp;
  const std::map<int, aprilgrid_t> cam_grids = {{0, test_grid}};
  calib.add_pose(ts, cam_grids);
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

    calib.add_view(calib.calib_data[ts], true);
    view_ts = ts;
    if (calib.nb_views() >= 5) {
      break;
    }
  }
  const size_t nb_res = calib.solver->num_residuals();
  const size_t nb_params = calib.solver->num_params();
  MU_CHECK(calib.poses.count(view_ts));
  MU_CHECK(calib.calib_views.size() == 5);
  MU_CHECK(calib.calib_views.begin()->second->get_camera_indices().size() == 2);
  // 144 corners + 2 camera parameters + 2 camera extrinsics + 1 pose

  // Remove view
  calib.remove_view(view_ts);
  MU_CHECK(calib.poses.count(view_ts) == 0);
  MU_CHECK(calib.calib_views.size() == 4);
  MU_CHECK(calib.solver->num_residuals() < nb_res);
  MU_CHECK(calib.solver->num_params() < nb_params);

  // Remove all views
  calib.remove_all_views();
  MU_CHECK(calib.poses.size() == 0);
  MU_CHECK(calib.calib_views.size() == 0);
  MU_CHECK(calib.solver->num_residuals() == 0);
  MU_CHECK(calib.solver->num_params() == 148);
  // 144 corners + 2 camera parameters + 2 camera extrinsics

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
    calib.add_view(calib.calib_data[ts], true);
    if (calib.nb_views() > 10) {
      break;
    }
  }
  calib._filter_all_views();
  calib.solver->solve();

  return 0;
}

int test_calib_camera_remove_all_views() {
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
    calib.add_view(calib.calib_data[ts], true);
    if (calib.nb_views() > 10) {
      break;
    }
  }
  calib.remove_all_views();
  MU_CHECK(calib.nb_views() == 0);
  MU_CHECK(calib.solver->num_residuals() == 0);

  return 0;
}

int test_calib_camera_remove_outliers() {
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
    calib.add_view(calib.calib_data[ts], true);
    if (calib.nb_views() > 10) {
      break;
    }
  }
  calib._remove_outliers(true);

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
  calib.add_camera_data(test_data);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib._initialize_intrinsics();
  calib._initialize_extrinsics();
  calib._solve_inc();
  calib.inspect(valid_data);

  return 0;
}

int test_calib_camera_mono() {
  calib_camera_t calib{MONO_CONF};
  calib.enable_nbv = false;
  calib.enable_marginalization = false;
  calib.enable_nbv_filter = false;
  calib.enable_outlier_filter = false;
  calib.add_camera_data(0, test_data.at(0));
  calib.solve();

  return 0;
}

int test_calib_camera_stereo() {
  // Calibrate
  calib_camera_t calib{STEREO_CONF};
  calib.enable_nbv = true;
  calib.enable_marginalization = false;
  calib.enable_nbv_filter = true;
  calib.enable_outlier_filter = true;
  calib.add_camera_data(test_data);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_estimates("/tmp/calib-estimates.yaml");
  calib.inspect(valid_data);

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
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1);

  // Build problem
  for (const auto ts : calib.timestamps) {
    if (calib.calib_data[ts].size() != 2) {
      continue;
    }

    calib.add_view(calib.calib_data[ts], true);
    if (calib.nb_views() > max_num_views) {
      break;
    }
  }

  // // Test marginalization error
  // int nb_res_blocks = calib.problem->NumResidualBlocks();
  // int nb_param_blocks = calib.problem->NumParameterBlocks();
  // marg_error_t marg_error;
  // auto view = calib.calib_views.begin()->second;
  // view->marginalize(&marg_error);
  // // -- Asserts
  // MU_CHECK(nb_res_blocks > calib.problem->NumResidualBlocks());
  // MU_CHECK(nb_param_blocks > calib.problem->NumParameterBlocks());
  //
  // // Solve
  // ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  // options.max_num_iterations = 10;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, calib.problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;
  // calib.show_results();

  return 0;
}

void test_suite() {
  load_test_dataset();
  load_validation_dataset();

  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_calib_camera_add_camera_data);
  MU_ADD_TEST(test_calib_camera_add_camera);
  MU_ADD_TEST(test_calib_camera_add_camera_extrinsics);
  MU_ADD_TEST(test_calib_camera_add_pose);
  MU_ADD_TEST(test_calib_camera_add_and_remove_view);
  MU_ADD_TEST(test_calib_camera_find_nbv);
  MU_ADD_TEST(test_calib_camera_filter_all_views);
  MU_ADD_TEST(test_calib_camera_remove_all_views);
  MU_ADD_TEST(test_calib_camera_remove_outliers);
  MU_ADD_TEST(test_calib_camera_solve_batch);
  MU_ADD_TEST(test_calib_camera_solve_inc);
  MU_ADD_TEST(test_calib_camera_mono);
  MU_ADD_TEST(test_calib_camera_stereo);
  MU_ADD_TEST(test_marg_error);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
