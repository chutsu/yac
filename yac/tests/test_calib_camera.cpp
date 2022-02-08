#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

std::map<int, aprilgrids_t> setup_test_data() {
  // Calibration data
  const calib_target_t calib_target;
  const std::string data_path = "/data/euroc/cam_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  return calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int test_calib_view() {
  // Load data
  auto cam_grids = setup_test_data();

  // Setup
  ceres::Problem problem;
  ceres::CauchyLoss loss(1.0);

  calib_target_t calib_target;
  fiducial_corners_t corners{calib_target};

  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  pinhole_radtan4_t cam_geom;
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);

  auto grid = cam_grids[0][0];
  const mat4_t T_C0F = I(4);
  const mat4_t T_C0Ci = I(4);
  pose_t extrinsics{0, T_C0Ci};
  pose_t rel_pose{0, T_C0F};

  calib_view_t(&problem,
               &loss,
               &corners,
               &cam_geom,
               &cam0,
               &extrinsics,
               &rel_pose,
               grid);

  return 0;
}

int test_calib_camera_find_nbv() {
  // Calibration target and camera parameters
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibration data
  const auto grids = setup_test_data();

  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  for (const auto grid : grids.at(0)) {
    if (grid.detected) {
      cam0_grids.push_back(grid);
    }
    if (cam0_grids.size() >= 5) {
      break;
    }
  }
  for (const auto grid : grids.at(1)) {
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

int test_calib_camera_mono() {
  // Setup
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const auto cam_grids = setup_test_data();

  // Calibrate
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.add_camera_data(0, cam_grids.at(0));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.solve();

  return 0;
}

int test_calib_camera_stereo() {
  // Setup
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const auto cam_grids = setup_test_data();

  // Calibrate
  calib_camera_t calib{target};
  calib.add_camera_data(cam_grids);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_estimates("/tmp/calib-estimates.yaml");

  // Load test data for validation
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const auto imu_grids = calib_data_preprocess(target, cam_paths, grids_path);
  calib.validate(imu_grids);

  return 0;
}

int test_calib_camera_validate() {
  // Calibrate
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  vec4_t cam0_proj_params;
  vec4_t cam0_dist_params;
  vec4_t cam1_proj_params;
  vec4_t cam1_dist_params;

  // EuRoc
  // clang-format off
  // cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  // cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  // cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  // cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
  // mat4_t T_C0C1;
  // T_C0C1 << 1.00000, -0.00232, -0.00034,  0.11007,
  //           0.00231,  0.99990, -0.01409, -0.00016,
  //           0.00038,  0.01409,  0.99990,  0.00089,
  //           0.00000,  0.00000,  0.00000,  1.00000;
  // calib_camera_t calib{target};
  // calib.add_camera(0, cam_res, proj_model, dist_model, cam0_proj_params, cam0_dist_params);
  // calib.add_camera(1, cam_res, proj_model, dist_model, cam1_proj_params, cam1_dist_params);
  // calib.add_camera_extrinsics(0, I(4), true);
  // calib.add_camera_extrinsics(1, T_C0C1, true);
  // clang-format on

  // Kalibr
  // clang-format off
  // cam0_proj_params << 458.9100509555829, 457.56455132296185, 367.0070551981907, 248.41458738348737;
  // cam0_dist_params << -0.2857499507124168, 0.07588321191742696, 0.00025397687448367254, 3.780547330184374e-05;
  // cam1_proj_params << 457.79590953556277, 456.3300866555826, 379.2101853461348, 255.3626846707417;
  // cam1_dist_params << -0.28543696102317306, 0.07601141914226962, -4.042435645055041e-07, 2.1251482046669695e-05;
  // mat4_t T_C1C0;
  // T_C1C0 << 0.9999963398188968, 0.0021921950531668542, 0.0015857583841900123, -0.11000903460150067,
  //           -0.0022138992743579032, 0.9999020868324647, 0.01381721383201927, 0.00042598376811829857,
  //           -0.0015553130897525734, -0.013820673967850404, 0.9999032803087816, -0.0005920475406685743,
  //           0.0, 0.0, 0.0, 1.0;
  // calib_camera_t calib{target};
  // calib.add_camera(0, cam_res, proj_model, dist_model, cam0_proj_params, cam0_dist_params);
  // calib.add_camera(1, cam_res, proj_model, dist_model, cam1_proj_params, cam1_dist_params);
  // calib.add_camera_extrinsics(0, I(4), true);
  // calib.add_camera_extrinsics(1, T_C1C0.inverse(), true);
  // clang-format on

  // YAC
  const auto cam_grids = setup_test_data();
  calib_camera_t calib{target};
  calib.enable_nbv = true;
  calib.enable_nbv_filter = true;
  calib.enable_outlier_filter = true;
  calib.outlier_threshold = 3.0;
  calib.min_nbv_views = 10;
  calib.info_gain_threshold = 0.05;
  calib.add_camera_data(cam_grids);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-results-nbv_full.yaml");
  // calib_camera_t calib{"/tmp/calib-results.yaml"};

  // Load test data for validation
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const auto imu_grids = calib_data_preprocess(target, cam_paths, grids_path);
  calib.validate(imu_grids);

  return 0;
}

int test_calib_camera_kalibr_data() {
  // Load AprilGrid data
  const std::string grid0_path = "/tmp/yac_data/cam0";
  const std::string grid1_path = "/tmp/yac_data/cam1";
  std::vector<std::string> grid0_csvs;
  std::vector<std::string> grid1_csvs;
  list_files(grid0_path, grid0_csvs);
  list_files(grid1_path, grid1_csvs);

  std::map<int, aprilgrids_t> cam_grids;
  for (const auto csv_path : grid0_csvs) {
    aprilgrid_t grid;
    grid.load(grid0_path + "/" + csv_path);
    cam_grids[0].push_back(grid);
  }
  for (const auto csv_path : grid1_csvs) {
    aprilgrid_t grid;
    grid.load(grid1_path + "/" + csv_path);
    cam_grids[1].push_back(grid);
  }

  // Calibrate
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  calib_camera_t calib{calib_target};
  calib.enable_nbv = true;
  calib.enable_outlier_filter = true;
  calib.initialized = true;
  calib.add_camera_data(0, cam_grids.at(0));
  calib.add_camera_data(1, cam_grids.at(1));

  // clang-format off
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  vec4_t cam0_proj_params;
  vec4_t cam0_dist_params;
  vec4_t cam1_proj_params;
  vec4_t cam1_dist_params;

  cam0_proj_params << 460.285798, 459.066980,368.299812,244.660910;
  cam0_dist_params << -0.276158,0.067779,0.000757,-0.000299;
  cam1_proj_params << 458.931838, 457.621130, 378.783646, 251.341323;
  cam1_dist_params << -0.272847, 0.064882, 0.000506, -0.000210;

  vec3_t r_C1C0{-0.109948,0.000514,-0.000241};
  quat_t q_C1C0{0.999970, -0.007147,0.002650,-0.001263};
  mat4_t T_C1C0 = tf(q_C1C0, r_C1C0);
  mat4_t T_C0C1 = T_C1C0.inverse();

  calib.add_camera(0,
                   cam_res,
                   proj_model,
                   dist_model,
                   cam0_proj_params,
                   cam0_dist_params);
  calib.add_camera(1,
                   cam_res,
                   proj_model,
                   dist_model,
                   cam1_proj_params,
                   cam1_dist_params);
  calib.add_camera_extrinsics(0, I(4), true);
  calib.add_camera_extrinsics(1, T_C0C1, false);
  // clang-format on

  calib.solve();
  // calib.save_results("/tmp/calib-results.yaml");

  // Load test data for validation
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  calib.validate(calib_data_preprocess(calib_target, cam_paths, grids_path));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_calib_camera_find_nbv);
  MU_ADD_TEST(test_calib_camera_mono);
  MU_ADD_TEST(test_calib_camera_stereo);
  MU_ADD_TEST(test_calib_camera_validate);
  MU_ADD_TEST(test_calib_camera_kalibr_data);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
