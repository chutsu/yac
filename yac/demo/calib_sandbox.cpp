#include "calib_camera.hpp"

std::map<int, yac::aprilgrids_t> load_dataset() {
  // Load test data for validation
  const yac::calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";

  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int main(int argc, char *argv[]) {
  // Load AprilGrid data
  const std::string grid0_path = "/tmp/yac_data/cam0";
  const std::string grid1_path = "/tmp/yac_data/cam1";
  const std::string timestamps_file = "/tmp/yac_data/timestamps.csv";

  std::vector<std::string> grid0_csvs;
  std::vector<std::string> grid1_csvs;
  yac::list_files(grid0_path, grid0_csvs);
  yac::list_files(grid1_path, grid1_csvs);

  std::map<int, yac::aprilgrids_t> cam_grids;
  for (const auto csv_path : grid0_csvs) {
    yac::aprilgrid_t grid;
    grid.load(grid0_path + "/" + csv_path);
    cam_grids[0].push_back(grid);
  }
  for (const auto csv_path : grid1_csvs) {
    yac::aprilgrid_t grid;
    grid.load(grid1_path + "/" + csv_path);
    cam_grids[1].push_back(grid);
  }

  // Load timestamps
  int nb_rows = 0;
  FILE *fp = yac::file_open("/tmp/yac_data/timestamps.csv", "r", &nb_rows);
  if (fp == NULL) {
    FATAL("Failed to open[%s]!", "/tmp/yac_data/timestamps.csv");
  }

  std::set<yac::timestamp_t> timestamps;
  for (int i = 0; i < nb_rows; i++) {
    yac::timestamp_t ts = 0;
    fscanf(fp, "%ld", &ts);
    timestamps.insert(ts);
  }

  // Load inspection data
  const auto inspect_data = load_dataset();

  // Setup calibrator
  const std::string config_file = argv[1];
  yac::calib_camera_t calib{config_file};
  calib.add_camera_data(0, cam_grids[0]);
  calib.add_camera_data(1, cam_grids[1]);
  calib.validation_data = inspect_data;
  calib.timestamps = timestamps;
  calib.solve();
  // calib.print_estimates(stdout);

  // size_t ts_idx = 0;
  // int early_stop_counter = 0;
  // int nb_timestamps = timestamps.size();
  //
  // for (const auto &ts : timestamps) {
  //   // Add view
  //   if (calib.add_view(calib.calib_data[ts], false) == false) {
  //     early_stop_counter++;
  //     calib._print_stats(ts_idx++, nb_timestamps);
  //     continue;
  //   }
  //   early_stop_counter = 0;
  //
  //   // Remove outliers
  //   if (calib.nb_views() >= calib.min_nbv_views) {
  //     calib._remove_outliers(calib.filter_all);
  //     calib.filter_all = false;
  //   }
  //
  //   // Print stats
  //   calib._print_stats(ts_idx++, nb_timestamps);
  // }

  return 0;
}
