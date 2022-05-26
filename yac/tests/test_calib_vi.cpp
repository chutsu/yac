#include "munit.hpp"
#include "calib_vi.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_IMUCAM_DATA TEST_PATH "/test_data/calib/imu_april"
#define CALIB_CONFIG TEST_PATH "/test_data/calib/imu_april/config.yaml"

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
  calib_target_t calib_target;
  calib_vi_t calib{calib_target};

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
  calib_target_t calib_target;
  calib_vi_t calib{calib_target};

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

int test_calib_vi(const std::string &mode, const int max_views = -1) {
  const auto timeline = setup_test_data();

  // Setup VI calibrator
  calib_vi_t calib{CALIB_CONFIG};

  if (mode == "batch") {
    calib.enable_marginalization = false;
  } else if (mode == "online") {
    calib.enable_marginalization = true;
  }

  LOG_INFO("Adding data to problem ...");
  int nb_views = 0;
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

        if (cam_idx == 0) {
          nb_views++;
        }
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }

    // Break early?
    if (max_views != -1 && nb_views >= max_views) {
      break;
    }
  }
  calib.solve();
  calib.save_results("/tmp/calib-vi.yaml");
  calib.save_estimates("/tmp");

  return 0;
}

int test_calib_vi_batch() {
  MU_CHECK(test_calib_vi("batch") == 0);
  return 0;
}

int test_calib_vi_online() {
  int max_views = 10;
  MU_CHECK(test_calib_vi("online", max_views) == 0);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_vi_add_imu);
  MU_ADD_TEST(test_calib_vi_add_camera);
  MU_ADD_TEST(test_calib_vi_batch);
  MU_ADD_TEST(test_calib_vi_online);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
