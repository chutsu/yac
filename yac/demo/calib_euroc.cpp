#include "calib_camera.hpp"

std::map<int, yac::aprilgrids_t> load_test_dataset() {
  // Calibration data
  const yac::calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  const std::string data_path = "/data/euroc/cam_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
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
  calib.validation_data = valid_data;
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.validate(valid_data);

  return 0;
}
