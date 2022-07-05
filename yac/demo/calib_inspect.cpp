#include "calib_camera.hpp"
#include "calib_vi.hpp"

void print_usage(char *argv[]) {
  printf("usage: %s <mode> <config_file> <dataset_path>\n", argv[0]);
  printf("examples:\n");
  printf("  %s camera config.yaml /data/euroc\n", argv[0]);
  printf("  %s camera-imu config.yaml /data/euroc\n", argv[0]);
}

std::map<int, yac::aprilgrids_t> load_dataset(const std::string data_path) {
  const yac::calib_target_t calib_target;
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  const std::string grids_path = data_path + "/mav0/grid0";
  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int main(int argc, char *argv[]) {
  // Check command-line args
  if (argc != 4) {
    print_usage(argv);
    return -1;
  }

  // Parse commandline args
  const std::string mode = argv[1];
  const std::string config_file = argv[2];
  const std::string dataset_path = argv[3];

  // Inspect
  if (mode == "camera") {
    const auto inspect_data = load_dataset(dataset_path);
    yac::calib_camera_t calib{config_file};
    calib.inspect(inspect_data);

  } else if (mode == "camera-imu") {
    yac::calib_vi_t calib{config_file};
    calib.max_iter = 0;
    calib.enable_outlier_rejection = false;
    calib.load_data(dataset_path);
    calib.solve();
    calib.print_stats(stdout);

  } else {
    FATAL("Unrecognized inspection mode [%s]!", mode.c_str());
  }

  return 0;
}
