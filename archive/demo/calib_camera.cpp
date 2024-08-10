#include "calib_camera.hpp"

int main(int argc, char *argv[]) {
  // Check arguments
  if (argc != 3) {
    printf("usage: %s <config_file> <data_path>\n", argv[0]);
    return -1;
  }

  // Calibrate
  const std::string config_file = argv[1];
  const std::string data_path = argv[2];
  yac::calib_camera_t calib{config_file};
  calib.load_data(data_path);
  calib.solve();
  calib.save_results(data_path + "/calib_camera-results.yaml");

  return 0;
}
