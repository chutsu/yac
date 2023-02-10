#include "calib_vi.hpp"

int main(int argc, char *argv[]) {
  // Check arguments
  if (argc != 3) {
    printf("usage: %s <config_file> <dataset_path>\n", argv[0]);
    return -1;
  }

  // Calibrate
  const std::string config_file = argv[1];
  const std::string dataset_path = argv[2];
  yac::calib_vi_t calib{config_file};
  calib.load_data(dataset_path);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");

  return 0;
}
