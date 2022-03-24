#include "calib_camera.hpp"

std::map<int, yac::aprilgrids_t> load_dataset(const std::string data_path) {
  const yac::calib_target_t calib_target;
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  const std::string grids_path = data_path + "/mav0/grid0";
  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    printf("usage: %s <config_file> <dataset_path>\n", argv[0]);
    return -1;
  }

  const std::string config_file = argv[1];
  const std::string dataset_path = argv[2];
  const auto inspect_data = load_dataset(dataset_path);
  yac::calib_camera_t calib{config_file};
  calib.inspect(inspect_data);
  return 0;
}
