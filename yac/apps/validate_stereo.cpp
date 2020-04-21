#include <yac/yac.hpp>

using namespace yac;

struct calib_config_t {
  std::string cam0_image_path;
  std::string cam1_image_path;
  bool imshow = true;

  vec2_t cam0_resolution{0.0, 0.0};
  std::string cam0_camera_model;
  std::string cam0_distortion_model;
  vec4_t cam0_intrinsics;
  vec4_t cam0_distortion;

  vec2_t cam1_resolution{0.0, 0.0};
  std::string cam1_camera_model;
  std::string cam1_distortion_model;
  vec4_t cam1_intrinsics;
  vec4_t cam1_distortion;

  mat4_t T_C0C1 = I(4);
};

void print_usage() {
  const std::string usage = R"EOF(
Usage: validate_stereo_extrinsics <config.yaml>

The `config.yaml` file is expected to have the following format:

  settings:
    cam0_image_path: "/data/cam0/data"
    cam1_image_path: "/data/cam1/data"
    imshow: true

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
    camera_model: "pinhole"
    distortion_model: "radtan"
    intrinsics: [460.107, 458.985, 371.509, 244.215]
    distortion: [-0.268364, 0.0608506, 0.000938523, -0.000351249]

  cam1:
    resolution: [752, 480]
    camera_model: "pinhole"
    distortion_model: "radtan"
    intrinsics: [460.107, 458.985, 371.509, 244.215]
    distortion: [-0.268364, 0.0608506, 0.000938523, -0.000351249]

  T_C0C1:
    rows: 4
    cols: 4
    data: [
      1.0, 0.0, 0.0, 1.0,
      0.0, 1.0, 0.0, 2.0,
      0.0, 0.0, 1.0, 3.0,
      0.0, 0.0, 0.0, 1.0
    ]
)EOF";

  std::cout << usage << std::endl;
}

calib_config_t parse_config(const std::string &config_file) {
  config_t config{config_file};
  calib_config_t calib_config;

  parse(config, "settings.cam0_image_path", calib_config.cam0_image_path);
  parse(config, "settings.cam1_image_path", calib_config.cam1_image_path);
  parse(config, "settings.imshow", calib_config.imshow);

  parse(config, "cam0.resolution", calib_config.cam0_resolution);
  parse(config, "cam0.camera_model", calib_config.cam0_camera_model);
  parse(config, "cam0.distortion_model", calib_config.cam0_distortion_model);
  parse(config, "cam0.intrinsics", calib_config.cam0_intrinsics);
  parse(config, "cam0.distortion", calib_config.cam0_distortion);

  parse(config, "cam1.resolution", calib_config.cam1_resolution);
  parse(config, "cam1.camera_model", calib_config.cam1_camera_model);
  parse(config, "cam1.distortion_model", calib_config.cam1_distortion_model);
  parse(config, "cam1.intrinsics", calib_config.cam1_intrinsics);
  parse(config, "cam1.distortion", calib_config.cam1_distortion);

  parse(config, "T_C0C1", calib_config.T_C0C1);

  return calib_config;
}

static int get_camera_image_paths(const std::string &image_dir,
                                  std::vector<std::string> &image_paths) {
  // Check image dir
  if (dir_exists(image_dir) == false) {
    LOG_ERROR("Image dir [%s] does not exist!", image_dir.c_str());
    return -1;
  }

  // Get image paths
  std::vector<std::string> image_files;
  if (list_dir(image_dir, image_files) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", image_dir.c_str());
    return -1;
  }
  std::sort(image_files.begin(), image_files.end());

  // Prepend the image dir to the image paths
  image_paths.clear();
  for (const auto &image_file : image_files) {
    image_paths.push_back(paths_combine(image_dir, image_file));
  }

  return 0;
}

static void
sychronize_stereo_images(std::vector<std::string> &cam0_image_paths,
                         std::vector<std::string> &cam1_image_paths) {
  if (cam0_image_paths.size() > cam1_image_paths.size()) {
    cam0_image_paths.pop_back();
  } else if (cam0_image_paths.size() < cam1_image_paths.size()) {
    cam1_image_paths.pop_back();
  }
}

static pinhole_radtan4_t setup_camera(const vec4_t &intrinsics,
                                      const vec4_t &distortion) {
  const pinhole_t camera_model{intrinsics};
  const radtan4_t distortion_model{distortion};
  pinhole_radtan4_t camera{camera_model, distortion_model};
  return camera;
}

int main(int argc, char *argv[]) {
  // Parse command line arguments
  if (argc != 2) {
    print_usage();
    return -1;
  }

  // Parse calib config file
  const std::string config_file{argv[1]};
  const calib_config_t config = parse_config(config_file);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", config_file.c_str());
    return -1;
  }

  // Get camera image paths
  std::vector<std::string> cam0_image_paths;
  if (get_camera_image_paths(config.cam0_image_path, cam0_image_paths) != 0) {
    return -1;
  }
  std::vector<std::string> cam1_image_paths;
  if (get_camera_image_paths(config.cam1_image_path, cam1_image_paths) != 0) {
    return -1;
  }

  // Synchronize stereo image
  sychronize_stereo_images(cam0_image_paths, cam1_image_paths);

  // Setup AprilGrid and camera properties
  // -- AprilGrid properties
  const aprilgrid_detector_t detector;
  const auto tag_rows = calib_target.tag_rows;
  const auto tag_cols = calib_target.tag_cols;
  const auto tag_size = calib_target.tag_size;
  const auto tag_spacing = calib_target.tag_spacing;
  // -- cam0 properties
  const auto cam0_intrinsics = config.cam0_intrinsics;
  const auto cam0_distortion = config.cam0_distortion;
  const auto cam0_geometry = setup_camera(cam0_intrinsics, cam0_distortion);
  const auto cam0_K = pinhole_K(cam0_intrinsics.data());
  const auto cam0_D = config.cam0_distortion;
  // -- cam1 properties
  const auto cam1_intrinsics = config.cam1_intrinsics;
  const auto cam1_distortion = config.cam1_distortion;
  const auto cam1_geometry = setup_camera(cam1_intrinsics, cam1_distortion);
  const auto cam1_K = pinhole_K(cam1_intrinsics.data());
  const auto cam1_D = config.cam1_distortion;

  // Detect AprilGrid
  // -- Double check number of images from each camera
  size_t nb_images = cam0_image_paths.size();
  if (cam0_image_paths.size() != cam1_image_paths.size()) {
    FATAL("Mismatch umber of images between cam0 and cam1!");
  }

  // -- Visualize validation
  LOG_INFO("Processing images:");
  for (size_t i = 0; i < nb_images; i++) {
    print_progress((double) i / nb_images);

    // Detect
    const cv::Mat cam0_image = cv::imread(cam0_image_paths[i]);
    const cv::Mat cam1_image = cv::imread(cam1_image_paths[i]);
    aprilgrid_t grid0{(unsigned long) i,
                      tag_rows,
                      tag_cols,
                      tag_size,
                      tag_spacing};
    aprilgrid_t grid1{(unsigned long) i,
                      tag_rows,
                      tag_cols,
                      tag_size,
                      tag_spacing};
    aprilgrid_detect(grid0, detector, cam0_image, cam0_K, cam0_D);
    aprilgrid_detect(grid1, detector, cam1_image, cam1_K, cam1_D);
    aprilgrid_intersection(grid0, grid1);

    // Validate
    if ((grid0.detected || grid1.detected) && config.imshow) {
      const auto result = validate_stereo(cam0_image,
                                          cam1_image,
                                          grid0.keypoints,
                                          grid0.points_CF,
                                          grid1.keypoints,
                                          grid1.points_CF,
                                          cam0_geometry,
                                          cam1_geometry,
                                          config.T_C0C1);
      cv::imshow("Stereo Camera Validation", result);
      cv::waitKey(1);
    }
  }

  // Print newline after print progress has finished
  print_progress(1.0);
  std::cout << std::endl;

  // Destroy all opencv windows
  cv::destroyAllWindows();

  return 0;
}
