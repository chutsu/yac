#include "calib_camera.hpp"
#include "calib_vi.hpp"

#define CAMERA_DATA "/data/euroc/cam_april"
#define IMU_DATA "/data/euroc/imu_april"
// #define CAMERA_RESULTS_PATH "/tmp/calib_camera-results.yaml"
#define CAMERA_RESULTS_PATH                                                    \
  "/data/euroc_results/configs/yac/calib_camera-results.yaml"
#define IMU_RESULTS_PATH "/tmp/calib_imu-results.yaml"

// CALIBRATE CAMERAS /////////////////////////////////////////////////////////

void calibrate_cameras(const std::string &train_path,
                       const std::string &valid_path,
                       const std::string &results_path) {
  // Lambda function to load data
  auto load_data = [](const std::string &data_path) {
    // Calibration data
    const yac::calib_target_t calib_target;
    const std::string grids_path = data_path + "/mav0/grid0";
    std::map<int, std::string> cam_paths;
    cam_paths[0] = data_path + "/mav0/cam0/data";
    cam_paths[1] = data_path + "/mav0/cam1/data";
    return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
  };

  // Setup
  const auto train_data = load_data(train_path);
  const auto valid_data = load_data(valid_path);
  const yac::calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate cameras
  yac::calib_camera_t calib{calib_target};
  calib.add_camera_data(train_data);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results(results_path);
  calib.inspect(train_data);
  calib.inspect(valid_data);
}

// CALIBRATE IMU /////////////////////////////////////////////////////////////

void load_euroc_imu_data(const std::string &csv_path,
                         yac::timestamps_t &timestamps,
                         yac::vec3s_t &a_B,
                         yac::vec3s_t &w_B) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = yac::file_open(csv_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", csv_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      yac::skip_line(fp);
      continue;
    }

    // Parse line
    yac::timestamp_t ts = 0;
    double ax, ay, az = 0.0;
    double wx, wy, wz = 0.0;
    int retval = fscanf(fp,
                        "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &wx,
                        &wy,
                        &wz,
                        &ax,
                        &ay,
                        &az);
    if (retval != 7) {
      LOG_ERROR("Invalid line: %d", i);
      FATAL("Failed to parse line in [%s]", csv_path.c_str());
    }

    timestamps.push_back(ts);
    a_B.emplace_back(ax, ay, az);
    w_B.emplace_back(wx, wy, wz);
  }
  fclose(fp);
}

yac::timeline_t load_calib_vi_data(const std::string &data_path) {
  // Load grid data
  yac::timeline_t timeline;

  // Preprocess camera data
  const yac::calib_target_t target;
  const std::string grid_path = data_path + "/mav0/grid0";
  std::map<int, std::string> paths{
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const auto &grid_data = yac::calib_data_preprocess(target, paths, grid_path);

  // Add camera events to timeline
  for (const auto &[cam_idx, cam_grids] : grid_data) {
    for (const auto &grid : cam_grids) {
      timeline.add(grid.timestamp, cam_idx, grid);
    }
  }

  // Add imu events to timeline
  const std::string imu0_path = data_path + "/mav0/imu0/data.csv";
  yac::timestamps_t imu_ts;
  yac::vec3s_t imu_acc;
  yac::vec3s_t imu_gyr;
  load_euroc_imu_data(imu0_path, imu_ts, imu_acc, imu_gyr);
  for (size_t k = 0; k < imu_ts.size(); k++) {
    timeline.add(imu_ts[k], imu_acc[k], imu_gyr[k]);
  }

  return timeline;
}

void calibrate_imu(const std::string &data_path,
                   const std::string &camera_results_path,
                   const std::string &imu_results_path) {
  // Setup calibrator
  yac::calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  yac::calib_vi_t calib{calib_target};

  // -- Add IMU
  yac::imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.a_max = 160.0;
  imu_params.g_max = 10.0;
  imu_params.sigma_a_c = 0.002;
  imu_params.sigma_g_c = 1.6968e-04;
  imu_params.sigma_aw_c = 0.003;
  imu_params.sigma_gw_c = 1.9393e-05;
  imu_params.sigma_bg = 0.0;
  imu_params.sigma_ba = 0.0;
  imu_params.g = 9.81;
  calib.add_imu(imu_params, yac::I(4));

  // -- Add cameras
  yac::config_t config{camera_results_path};
  for (int cam_idx = 0; cam_idx < 100; cam_idx++) {
    // Check if key exists
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    if (yac::yaml_has_key(config, cam_str) == 0) {
      continue;
    }

    // Add camera intrinsics + extrinsics
    yac::veci2_t cam_res;
    std::string proj_model;
    std::string dist_model;
    yac::vecx_t proj_params;
    yac::vecx_t dist_params;
    yac::mat4_t T_C0Ci = yac::I(4);
    yac::parse(config, cam_str + ".resolution", cam_res);
    yac::parse(config, cam_str + ".proj_model", proj_model);
    yac::parse(config, cam_str + ".dist_model", dist_model);
    yac::parse(config, cam_str + ".proj_params", proj_params);
    yac::parse(config, cam_str + ".dist_params", dist_params);
    if (cam_idx != 0) {
      yac::parse(config, "T_cam0_" + cam_str, T_C0Ci);
    }
    calib.add_camera(cam_idx,
                     cam_res.data(),
                     proj_model,
                     dist_model,
                     proj_params,
                     dist_params,
                     T_C0Ci,
                     true,
                     true);
  }

  // Load data
  const auto timeline = load_calib_vi_data(data_path);
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<yac::aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<yac::imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }
  }

  // Solve and save results
  calib.solve();
  calib.save_results(imu_results_path);
}

int main() {
  // calibrate_cameras(CAMERA_DATA, IMU_DATA, CAMERA_RESULTS_PATH);
  calibrate_imu(IMU_DATA, CAMERA_RESULTS_PATH, IMU_RESULTS_PATH);
  return 0;
}
