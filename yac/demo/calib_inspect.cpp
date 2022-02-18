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
  const auto inspect_data = load_dataset();
  const std::string config_file = argv[1];
  yac::calib_camera_t calib{config_file};
  calib.inspect(inspect_data);
  return 0;
}
