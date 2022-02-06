#include "munit.hpp"
#include "util/euroc.hpp"
#include "calib_vi.hpp"

namespace yac {

timeline_t setup_test_data() {
  // Load calibration data
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> grid_paths = {
      {0, data_path + "/grid0/cam0"},
      {1, data_path + "/grid0/cam1"},
  };
  LOG_INFO("Loading EuRoC data [%s]", data_path.c_str());

  // Preprocess aprilgrids
  LOG_INFO("Preprocessing AprilGrid data ...");
  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    if (system(("mkdir -p " + grid_paths[cam_idx]).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", grid_paths[cam_idx].c_str());
    }
  }

  // Loop through timeline
  euroc_calib_t calib_data{data_path};
  auto timeline = calib_data.timeline();
  size_t i = 0;

  for (const auto &ts : timeline.timestamps) {
    if (i++ % 100 == 0) {
      printf(".");
      fflush(stdout);
    }

    for (const auto &event : timeline.get_events(ts)) {
      if (auto cam_event = dynamic_cast<camera_event_t *>(event)) {
        // Setup
        const timestamp_t ts = cam_event->ts;
        const int cam_idx = cam_event->cam_idx;
        const auto flag = cv::IMREAD_GRAYSCALE;
        const cv::Mat image = cv::imread(cam_event->img_path, flag);
        const std::string grid_fname = std::to_string(ts) + ".csv";
        const std::string grid_file = grid_paths[cam_idx] + "/" + grid_fname;

        // Detect or load
        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
          grid = detector.detect(ts, image);
          grid.save(grid_file);
        } else {
          grid.load(grid_file);
          if (grid.detected == false) {
            grid.timestamp = ts;
            grid.tag_rows = detector.tag_rows;
            grid.tag_cols = detector.tag_cols;
            grid.tag_size = detector.tag_size;
            grid.tag_spacing = detector.tag_spacing;
          }
        }

        // Add aprilgrid to timeline
        timeline.add(ts, cam_idx, grid);
      }
    }
  }
  printf("\n");

  return timeline;
}

int test_calib_vi_add_imu() {
  calib_vi_t calib;

  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_g_c = 12.0e-4;
  imu_params.sigma_a_c = 8.0e-3;
  imu_params.sigma_gw_c = 4.0e-6;
  imu_params.sigma_aw_c = 4.0e-5;
  imu_params.g = 9.81007;

  mat4_t T_BS = I(4);
  calib.add_imu(imu_params, T_BS);

  MU_CHECK(fltcmp(calib.imu_params.rate, imu_params.rate) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_g_c, imu_params.sigma_g_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_a_c, imu_params.sigma_a_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_gw_c, imu_params.sigma_gw_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_aw_c, imu_params.sigma_aw_c) == 0);
  MU_CHECK(calib.imu_exts != nullptr);
  MU_CHECK(calib.imu_exts->fixed == false);

  return 0;
}

int test_calib_vi_add_camera() {
  calib_vi_t calib;

  // Camera
  const int cam_idx = 0;
  const int res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[0], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const mat4_t T_BCi = I(4);
  calib.add_camera(cam_idx,
                   res,
                   proj_model,
                   dist_model,
                   proj_params,
                   dist_params,
                   T_BCi);

  MU_CHECK(calib.cam_params[cam_idx] != nullptr);
  MU_CHECK(calib.cam_params[cam_idx]->fixed == true);
  MU_CHECK(calib.cam_exts[cam_idx] != nullptr);
  MU_CHECK(calib.cam_exts[cam_idx]->fixed == true);

  return 0;
}

int test_calib_vi() {
  const auto timeline = setup_test_data();

  // Camera
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  vec4_t cam0_proj_params;
  vec4_t cam0_dist_params;
  vec4_t cam1_proj_params;
  vec4_t cam1_dist_params;
  mat4_t T_C0C1;
  mat4_t T_BC0;
  mat4_t T_BC1;
  // -- Custom calibration values
  config_t config{"/tmp/calib-results.yaml"};
  veci2_t cam_res;
  parse(config, "cam0.resolution", cam_res);
  parse(config, "cam0.proj_params", cam0_proj_params);
  parse(config, "cam0.dist_params", cam0_dist_params);
  parse(config, "cam1.proj_params", cam1_proj_params);
  parse(config, "cam1.dist_params", cam1_dist_params);
  parse(config, "T_cam0_cam1", T_C0C1);
  // -- Euroc Calibration values
  // cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  // cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  // cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  // cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.555e-05;
  // clang-format off
  // T_BC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //         -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //         0.0, 0.0, 0.0, 1.0;
  // T_BC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
  //         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
  //         -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
  //         0.0, 0.0, 0.0, 1.0;
  // T_C0C1 = T_BC0.inverse() * T_BC1; // Camera-Camera extrinsics
  // clang-format on
  T_BC0 = I(4);   // Set cam0 as body frame
  T_BC1 = T_C0C1; // Cam1 extrinsics

  // Imu
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_a_c = 0.002;
  imu_params.sigma_g_c = 1.6968e-04;
  imu_params.sigma_aw_c = 0.003;
  imu_params.sigma_gw_c = 1.9393e-05;
  imu_params.g = 9.81007;
  // clang-format off
  mat4_t T_BS;
  T_BS << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Setup VI calibrator
  calib_vi_t calib;
  calib.add_imu(imu_params, T_BS);
  calib.add_camera(0,
                   res,
                   proj_model,
                   dist_model,
                   cam0_proj_params,
                   cam0_dist_params,
                   T_BC0,
                   true,
                   true);
  calib.add_camera(1,
                   res,
                   proj_model,
                   dist_model,
                   cam1_proj_params,
                   cam1_dist_params,
                   T_BC1,
                   true,
                   true);

  LOG_INFO("Adding data to problem ...");
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }
  }
  calib.solve();
  calib.save_results("/tmp/calib-vi.yaml");
  calib.save_estimates("/tmp");

  return 0;
}

int test_marg_error() {
  const auto timeline = setup_test_data();

  // Camera
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  vec4_t cam0_proj_params;
  vec4_t cam0_dist_params;
  vec4_t cam1_proj_params;
  vec4_t cam1_dist_params;
  mat4_t T_C0C1;
  mat4_t T_BC0;
  mat4_t T_BC1;
  // -- Euroc Calibration values
  // clang-format off
  cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.555e-05;
  T_BC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
          0.0, 0.0, 0.0, 1.0;
  T_BC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
          0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
          -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
          0.0, 0.0, 0.0, 1.0;
  T_C0C1 = T_BC0.inverse() * T_BC1; // Camera-Camera extrinsics
  // clang-format on
  T_BC0 = I(4);   // Set cam0 as body frame
  T_BC1 = T_C0C1; // Cam1 extrinsics

  // Imu
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_a_c = 0.002;
  imu_params.sigma_g_c = 1.6968e-04;
  imu_params.sigma_aw_c = 0.003;
  imu_params.sigma_gw_c = 1.9393e-05;
  imu_params.g = 9.81007;
  // clang-format off
  mat4_t T_BS;
  T_BS << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Setup VI calibrator
  calib_vi_t calib;
  calib.add_imu(imu_params, T_BS);
  calib.add_camera(0,
                   res,
                   proj_model,
                   dist_model,
                   cam0_proj_params,
                   cam0_dist_params,
                   T_BC0,
                   true,
                   true);
  calib.add_camera(1,
                   res,
                   proj_model,
                   dist_model,
                   cam1_proj_params,
                   cam1_dist_params,
                   T_BC1,
                   true,
                   true);

  LOG_INFO("Adding data to problem ...");
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);

        if (calib.nb_views() > 10) {
          break;
        }
      }
    }
  }
  // calib.solve();

  // Test marginalization error add
  LOG_INFO("Add fiducial errors");
  auto &view = calib.calib_views.front();
  view.pose.marginalize = true;
  for (auto &[cam_idx, errors] : view.fiducial_errors) {
    for (auto &error : errors) {
      calib.marg_error.add(error.get());
    }
  }
  LOG_INFO("Add inertial error");
  calib.marg_error.add(view.imu_error.get());

  LOG_INFO("Form hessian");
  matx_t H;
  vecx_t b;
  calib.marg_error.form_hessian(H, b, true);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_vi_add_imu);
  MU_ADD_TEST(test_calib_vi_add_camera);
  MU_ADD_TEST(test_calib_vi);
  MU_ADD_TEST(test_marg_error);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
