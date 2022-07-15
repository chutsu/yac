#include "calib_camera.hpp"
#include "calib_vi.hpp"

using namespace yac;

void print_usage(char *argv[]) {
  printf("usage: %s <mode> <config_file> <data_path> <output_path>\n", argv[0]);
  printf("examples:\n");
  printf("  %s camera config.yaml /data/euroc ./calib_info.csv\n", argv[0]);
  printf("  %s camera-imu config.yaml /data/euroc ./calib_info.csv\n", argv[0]);
}

camera_data_t load_camera_data(const std::string data_path) {
  const calib_target_t calib_target;
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/cam0/data";
  cam_paths[1] = data_path + "/cam1/data";
  const std::string grids_path = data_path + "/grid0";

  auto calib_data = calib_data_preprocess(calib_target, cam_paths, grids_path);
  camera_data_t camera_data;
  for (const auto &[cam_idx, cam_grids] : calib_data) {
    for (const auto &grid : cam_grids) {
      camera_data[grid.timestamp][cam_idx] = grid;
    }
  }

  return camera_data;
}

void eval_camera_info(const std::string &config_file,
                      const std::string &data_path,
                      std::map<timestamp_t, real_t> &calib_info,
                      std::map<timestamp_t, real_t> &calib_entropy) {
  calib_camera_t calib{config_file};
  const auto camera_data = load_camera_data(data_path);

  for (const auto &[ts, cam_data] : camera_data) {
    std::map<int, aprilgrid_t> view_data;
    for (const auto &[cam_idx, grid] : cam_data) {
      view_data[cam_idx] = grid;
    }

    printf(".");
    fflush(stdout);
    calib.add_view(view_data);

    real_t info_k = 0.0;
    real_t entropy_k = 0.0;
    if (calib._calc_info(&info_k, &entropy_k) == 0) {
      calib_info[ts] = info_k;
      calib_entropy[ts] = entropy_k;
    }
  }
  printf("\n");
}

void eval_camera_imu_info(const std::string &config_file,
                          const std::string &data_path,
                          std::map<timestamp_t, real_t> &calib_info,
                          std::map<timestamp_t, real_t> &calib_entropy) {
  calib_vi_t calib{config_file};
  calib.enable_marginalization = false;

  // Form timeline
  timeline_t timeline;
  // -- Load camera data
  std::map<int, std::string> cam_paths;
  for (const auto cam_idx : calib.get_camera_indices()) {
    const auto cam_str = "cam" + std::to_string(cam_idx);
    cam_paths[cam_idx] = data_path + "/" + cam_str + "/data";
  }
  const auto grids_path = data_path + "/grid0";
  const auto calib_target = calib.calib_target;
  auto cam_grids = calib_data_preprocess(calib_target, cam_paths, grids_path);
  for (const auto cam_idx : calib.get_camera_indices()) {
    for (const auto grid : cam_grids[cam_idx]) {
      timeline.add(grid.timestamp, cam_idx, grid);
    }
  }
  // -- Load imu data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  load_imu_data(data_path + "/imu0/data.csv", imu_ts, imu_acc, imu_gyr);
  for (size_t k = 0; k < imu_ts.size(); k++) {
    timeline.add(imu_ts[k], imu_acc[k], imu_gyr[k]);
  }

  // Add data to calibrator
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
        printf(".");
        fflush(stdout);
      }

      // Imu event
      bool new_view = false;
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        new_view = calib.add_measurement(ts, acc, gyr);
      }

      // Estimate information
      if (new_view) {
        matx_t calib_covar;
        if (calib.recover_calib_covar(calib_covar) == 0) {
          const auto info_k = std::log(calib_covar.determinant()) / log(2.0);
          const auto k = pow(2 * M_PI * exp(1), 6);
          const auto entropy_k = 0.5 * log(k * calib_covar.determinant());
          calib_info[ts] = info_k;
          calib_entropy[ts] = entropy_k;
        }
      }
    }
  }
}

void save_results(const std::string &save_path,
                  std::map<timestamp_t, real_t> &calib_info,
                  std::map<timestamp_t, real_t> &calib_entropy) {
  LOG_INFO("Saving results to [%s]", save_path.c_str());

  // Save stats to csv file
  FILE *csv = fopen(save_path.c_str(), "w");
  if (csv == NULL) {
    FATAL("Failed to create file for saving!");
  }

  fprintf(csv, "view_idx,ts,info,entropy\n");
  int view_idx = 0;
  for (const auto &[ts, info] : calib_info) {
    fprintf(csv, "%d,", view_idx++);
    fprintf(csv, "%ld,", ts);
    fprintf(csv, "%f,", calib_info[ts]);
    fprintf(csv, "%f\n", calib_entropy[ts]);
  }
  fclose(csv);

  LOG_INFO("Done!");
}

int main(int argc, char *argv[]) {
  // Check command-line args
  if (argc != 5) {
    print_usage(argv);
    return -1;
  }

  // Parse commandline args
  const std::string mode = argv[1];
  const std::string config_file = argv[2];
  const std::string data_path = argv[3];
  const std::string output_path = argv[4];

  // Obtain calibration info and entropy
  std::map<timestamp_t, real_t> calib_info;
  std::map<timestamp_t, real_t> calib_entropy;
  if (mode == "camera") {
    eval_camera_info(config_file, data_path, calib_info, calib_entropy);
  } else if (mode == "camera-imu") {
    eval_camera_imu_info(config_file, data_path, calib_info, calib_entropy);
  }

  // Save results
  save_results(output_path, calib_info, calib_entropy);

  return 0;
}
