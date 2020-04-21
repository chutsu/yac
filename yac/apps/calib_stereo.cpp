#include <yac/yac.hpp>

using namespace yac;

void print_usage() {
  const std::string usage = R"EOF(
Usage: calib_stereo <calib_config.yaml>

The `calib_config.yaml` file is expected to have the following format:

  settings:
    data_path: "/data"
    results_fpath: "/data/calib.yaml"

  calib_target:
    target_type: 'aprilgrid'  # Target type
    tag_rows: 6               # Number of rows
    tag_cols: 6               # Number of cols
    tag_size: 0.085           # Size of apriltag, edge to edge [m]
    tag_spacing: 0.3          # Ratio of space between tags to tagSize
                              # Example: tagSize=2m, spacing=0.5m
                              # --> tagSpacing=0.25[-]

  cam0:
    resolution: [752, 480]
    lens_hfov: 98.0
    lens_vfov: 73.0
    camera_model: "pinhole"
    distortion_model: "radtan"

  cam1:
    resolution: [752, 480]
    lens_hfov: 98.0
    lens_vfov: 73.0
    camera_model: "pinhole"
    distortion_model: "radtan"
)EOF";

  std::cout << usage << std::endl;
}

int main(int argc, char *argv[]) {
  // Parse command line arguments
  if (argc != 2) {
      print_usage();
    return -1;
  }

  // Calibrate stereo camera
  const std::string config_file{argv[1]};
  if (calib_stereo_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  return 0;
}
