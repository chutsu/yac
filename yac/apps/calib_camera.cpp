#include <yac/yac.hpp>

using namespace yac;

void print_usage() {
  const std::string usage = R"EOF(
Usage: calib_camera <calib_config.yaml>

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
)EOF";

  std::cout << usage << std::endl;
}

// struct calib_config_t {
//   std::string data_path;
//   bool imshow = true;
//
//   vec2_t resolution{0.0, 0.0};
//   std::string camera_model;
//   std::string distortion_model;
//   vec4_t intrinsics;
//   vec4_t distortion;
//
//   calib_config_t(const std::string &config_file) {
//     config_t config{config_file};
//
//     parse(config, "settings.data_path", data_path);
//     parse(config, "settings.imshow", imshow);
//
//     parse(config, "cam0.resolution", resolution);
//     parse(config, "cam0.camera_model", camera_model);
//     parse(config, "cam0.distortion_model", distortion_model);
//     parse(config, "cam0.intrinsics", intrinsics);
//     parse(config, "cam0.distortion", distortion);
//   }
// };

// static int get_camera_image_paths(const std::string &image_dir,
//                                   std::vector<std::string> &image_paths) {
//   // Check image dir
//   if (dir_exists(image_dir) == false) {
//     LOG_ERROR("Image dir [%s] does not exist!", image_dir.c_str());
//     return -1;
//   }
//
//   // Get image paths
//   if (list_dir(image_dir, image_paths) != 0) {
//     LOG_ERROR("Failed to traverse dir [%s]!", image_dir.c_str());
//     return -1;
//   }
//   std::sort(image_paths.begin(), image_paths.end());
//
//   return 0;
// }

// static pinhole_radtan4_t setup_camera_geometry(const calib_config_t &config) {
//   const pinhole_t camera_model{config.intrinsics};
//   const radtan4_t distortion_model{config.distortion};
//   pinhole_radtan4_t camera{camera_model, distortion_model};
//   return camera;
// }

int main(int argc, char *argv[]) {
  // Parse command line arguments
  if (argc != 2) {
      print_usage();
    return -1;
  }

  // Calibrate camera
  const std::string config_file{argv[1]};
  if (calib_camera_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  // // Load calibration target
  // const calib_config_t config{config_file};
  // calib_target_t calib_target;
  // if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
  //   LOG_ERROR("Failed to load calib target [%s]!", config_file.c_str());
  //   return -1;
  // }
  //
  // // Get camera image paths
  // std::vector<std::string> image_paths;
  // if (get_camera_image_paths(config.data_path, image_paths) != 0) {
  //   return -1;
  // }
  //
  // // Detect AprilGrid
  // LOG_INFO("Processing images:");
  // const aprilgrid_detector_t detector;
  // // const auto camera_geometry = setup_camera_geometry(config);
  // const auto cam_K = pinhole_K(config.intrinsics.data());
  // const auto cam_D = config.distortion;
  //
  // for (size_t i = 0; i < image_paths.size(); i++) {
  //   print_progress((double) i / image_paths.size());
  //
  //   // Detect
  //   const auto image_path = paths_combine(config.data_path, image_paths[i]);
  //   const cv::Mat image = cv::imread(image_path);
  //   aprilgrid_t grid{0,
  //                    calib_target.tag_rows,
  //                    calib_target.tag_cols,
  //                    calib_target.tag_size,
  //                    calib_target.tag_spacing};
  //   aprilgrid_detect(grid, detector, image, cam_K, cam_D);
  //
  //   // Validate
  //   if (grid.detected && config.imshow) {
  //     const auto validation = validate_intrinsics(image,
  //                                                 grid.keypoints,
  //                                                 grid.points_CF,
  //                                                 camera_geometry);
  //     cv::imshow("Intrinsics Validation", validation);
  //     cv::waitKey(1);
  //   }
  // }
  //
  // // Print newline after print progress has finished
  // print_progress(1.0);
  // std::cout << std::endl;
  //
  // // Destroy all opencv windows
  // cv::destroyAllWindows();

  return 0;
}
