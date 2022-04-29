#include "calib_camera.hpp"

std::map<int, yac::aprilgrids_t> load_test_dataset() {
  // // Calibration data
  // const yac::calib_target_t calib_target;
  // const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  // const std::string data_path = "/data/euroc/cam_april";
  // std::map<int, std::string> cam_paths;
  // cam_paths[0] = data_path + "/mav0/cam0/data";
  // cam_paths[1] = data_path + "/mav0/cam1/data";
  // return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);

  const yac::calib_target_t calib_target;
  const std::string grids_path = "/tmp/yac_data";
  const std::string data_path = "/tmp/yac_data";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/cam0";
  cam_paths[1] = data_path + "/cam1";

  std::map<int, yac::aprilgrids_t> cam_grids;
  for (const auto &[cam_idx, cam_path] : cam_paths) {
    std::vector<std::string> csv_paths;
    yac::list_files(cam_path, csv_paths);

    for (const auto csv_path : csv_paths) {
      yac::aprilgrid_t grid;
      grid.load(cam_path + "/" + csv_path);

      if (grid.detected == false) {
        const std::string ts_str = csv_path.substr(0, 19);
        const yac::timestamp_t ts = std::stoull(ts_str);
        grid.timestamp = ts;
        grid.tag_rows = calib_target.tag_rows;
        grid.tag_cols = calib_target.tag_cols;
        grid.tag_size = calib_target.tag_size;
        grid.tag_spacing = calib_target.tag_spacing;
      }

      cam_grids[cam_idx].push_back(grid);
    }
  }
  return cam_grids;
}

std::map<int, yac::aprilgrids_t> load_validation_dataset() {
  // Load test data for validation
  const yac::calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int main() {
  // Setup
  const auto test_data = load_test_dataset();
  const auto valid_data = load_validation_dataset();
  const yac::calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibrate
  yac::calib_camera_t calib{target};
  calib.add_camera_data(test_data);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();

  // calib.show_results();
  calib.save_results("/tmp/calib-results.yaml");
  calib.inspect(test_data);
  calib.inspect(valid_data);

  return 0;
}
