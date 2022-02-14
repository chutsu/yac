#include "munit.hpp"
#include "calib_vi.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_IMUCAM_DATA TEST_PATH "/test_data/calib/imu_april"

void load_imu_data(const std::string &csv_path,
                   timestamps_t &timestamps,
                   vec3s_t &a_B,
                   vec3s_t &w_B) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(csv_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", csv_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    double a_x, a_y, a_z = 0.0;
    int retval = fscanf(fp,
                        "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &w_x,
                        &w_y,
                        &w_z,
                        &a_x,
                        &a_y,
                        &a_z);
    if (retval != 7) {
      FATAL("Failed to parse line in [%s]", csv_path.c_str());
    }

    timestamps.push_back(ts);
    a_B.emplace_back(a_x, a_y, a_z);
    w_B.emplace_back(w_x, w_y, w_z);
  }
  fclose(fp);
}

timeline_t setup_test_data() {
  // Load grid data
  timeline_t timeline;

  // -- Add cam0 grids
  std::vector<std::string> cam0_files;
  list_files(TEST_IMUCAM_DATA "/cam0", cam0_files);
  for (auto grid_path : cam0_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam0/") + grid_path;
    aprilgrid_t grid(grid_path);
    if (grid.detected == false) {
      const auto grid_fname = parse_fname(grid_path);
      const auto ts_str = grid_fname.substr(0, 19);
      grid.timestamp = std::stoull(ts_str);
      grid.tag_rows = 6;
      grid.tag_cols = 6;
      grid.tag_size = 0.088;
      grid.tag_spacing = 0.3;
    }
    timeline.add(grid.timestamp, 0, grid);
  }

  // -- Add cam1 grids
  std::vector<std::string> cam1_files;
  list_files(TEST_IMUCAM_DATA "/cam1", cam1_files);
  for (auto grid_path : cam1_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam1/") + grid_path;
    aprilgrid_t grid(grid_path);
    if (grid.detected == false) {
      const auto grid_fname = parse_fname(grid_path);
      const auto ts_str = grid_fname.substr(0, 19);
      grid.timestamp = std::stoull(ts_str);
      grid.tag_rows = 6;
      grid.tag_cols = 6;
      grid.tag_size = 0.088;
      grid.tag_spacing = 0.3;
    }
    timeline.add(grid.timestamp, 1, grid);
  }

  // -- Add imu events
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  load_imu_data(TEST_IMUCAM_DATA "/imu0.csv", imu_ts, imu_acc, imu_gyr);
  for (size_t k = 0; k < imu_ts.size(); k++) {
    timeline.add(imu_ts[k], imu_acc[k], imu_gyr[k]);
  }

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
  // // -- Custom calibration values
  // config_t config{"/tmp/calib-results.yaml"};
  // veci2_t cam_res;
  // parse(config, "cam0.resolution", cam_res);
  // parse(config, "cam0.proj_params", cam0_proj_params);
  // parse(config, "cam0.dist_params", cam0_dist_params);
  // parse(config, "cam1.proj_params", cam1_proj_params);
  // parse(config, "cam1.dist_params", cam1_dist_params);
  // parse(config, "T_cam0_cam1", T_C0C1);
  // -- Euroc Calibration values
  cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.555e-05;
  // clang-format off
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
      }
    }
  }
  calib.solve();
  calib.save_results("/tmp/calib-vi.yaml");
  calib.save_estimates("/tmp");

  return 0;
}

int test_calib_vi_online() {
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
  // // -- Custom calibration values
  // config_t config{"/tmp/calib-results.yaml"};
  // veci2_t cam_res;
  // parse(config, "cam0.resolution", cam_res);
  // parse(config, "cam0.proj_params", cam0_proj_params);
  // parse(config, "cam0.dist_params", cam0_dist_params);
  // parse(config, "cam1.proj_params", cam1_proj_params);
  // parse(config, "cam1.dist_params", cam1_dist_params);
  // parse(config, "T_cam0_cam1", T_C0C1);
  // -- Euroc Calibration values
  cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.555e-05;
  // clang-format off
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
      }
    }
  }
  calib.solve();
  calib.save_results("/tmp/calib-vi.yaml");
  calib.save_estimates("/tmp");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_vi_add_imu);
  MU_ADD_TEST(test_calib_vi_add_camera);
  MU_ADD_TEST(test_calib_vi);
  MU_ADD_TEST(test_calib_vi_online);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
