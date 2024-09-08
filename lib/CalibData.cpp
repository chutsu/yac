#include "CalibData.hpp"

namespace yac {

static void
parse_line(FILE *fp, const char *key, const char *value_type, void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = " ";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *)value = atol(value_str);
  } else if (strcmp(value_type, "int") == 0) {
    *(int *)value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *)value = atof(value_str);
  } else if (strcmp(value_type, "string") == 0) {
    *(std::string *)value = value_str;
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

static void parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  char *retval = fgets(buf, buf_len, fp);
  UNUSED(retval);
}

CalibData::CalibData(const std::string &config_path)
    : config_path_{config_path} {}

void CalibData::loadConfig(const std::string &config_path) {
  FILE *fp = fopen(config_path.c_str(), "r");
  if (fp == NULL) {
    FATAL("Failed to open [%s]!\n", config_path.c_str());
  }

  // Calibration settings
  std::string dataset_path;
  int num_cams;
  int num_imus;
  parse_line(fp, "dataset_path", "string", &dataset_path);
  parse_line(fp, "num_cams", "int", &num_cams);
  parse_line(fp, "num_imus", "int", &num_imus);

  // Calibration target settings
  parse_line(fp, "target_type", "string", &target_type_);
  parse_line(fp, "target_rows", "int", &target_rows_);
  parse_line(fp, "target_cols", "int", &target_cols_);
  parse_line(fp, "target_size", "double", &target_size_);
  parse_line(fp, "target_spacing", "double", &target_spacing_);
  parse_skip_line(fp);

  fclose(fp);
}

void CalibData::loadCameraData(const int camera_index) {
  // Setup detector
  const AprilGridDetector detector{target_rows_,
                                   target_cols_,
                                   target_size_,
                                   target_spacing_};

  // Form calibration target path
  const std::string camera_string = "cam" + std::to_string(camera_index);
  const std::string camera_path = dir_path_ + "/" + camera_string;
  std::string target_dir = dir_path_;
  target_dir += (dir_path_.back() == '/') ? "" : "/";
  target_dir += "calib_target/" + camera_string;

  // Create grids path
  if (system(("mkdir -p " + target_dir).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", target_dir.c_str());
  }

  // Get image files
  std::vector<std::string> image_paths;
  list_files(camera_path, image_paths);

  // Detect aprilgrids
#pragma omp parallel for
  for (size_t k = 0; k < image_paths.size(); k++) {
    const std::string image_fname = image_paths[k];
    const std::string image_path = camera_path + "/" + image_fname;
    const std::string ts_str = image_fname.substr(0, 19);
    if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
      LOG_WARN("Unexpected image file: [%s]!", image_path.c_str());
      continue;
    }
    const timestamp_t ts = std::stoull(ts_str);
    const std::string target_fname = ts_str + ".csv";
    const std::string target_path = target_dir + "/" + target_fname;
    const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

    // Load or detect apriltag
    if (file_exists(target_path)) {
      camera_data_[camera_index][ts] = AprilGrid::load(target_path);
    } else {
      // camera_data_[camera_index][ts] = detector.detect(ts, image);
      // camera_data_[camera_index][ts].save(target_path);
    }

    // Imshow
    // if (imshow) {
    //   grid.imshow("viz", image);
    //   cv::waitKey(1);
    // }

    // camera_data[camera_index][ts] = grid;
    if ((k % 100) == 0) {
      printf(".");
      fflush(stdout);
    }
  }
}

// void lerp_body_poses(const CalibTarget &grids,
//                      const timestamps_t &body_timestamps,
//                      const mat4s_t &body_poses,
//                      CalibTarget &lerped_grids,
//                      mat4s_t &lerped_poses,
//                      timestamp_t ts_offset) {
//   // Make sure AprilGrids are between body poses else we can't lerp poses
//   timestamps_t grid_timestamps;
//   for (const auto &grid : grids) {
//     if (grid.timestamp > body_timestamps.front() &&
//         grid.timestamp < body_timestamps.back()) {
//       lerped_grids.push_back(grid);
//       grid_timestamps.push_back(grid.timestamp);
//     }
//   }
//
//   // Lerp body poses using AprilGrid timestamps
//   assert(body_poses.size() == body_timestamps.size());
//   assert(body_timestamps.front() < grid_timestamps.front());
//   timestamp_t t0 = 0;
//   mat4_t pose0 = I(4);
//   timestamp_t t1 = 0;
//   mat4_t pose1 = I(4);
//
//   size_t grid_idx = 0;
//   for (size_t i = 0; i < body_timestamps.size(); i++) {
//     // Make sure we're not going out of bounds
//     if (grid_idx > (grid_timestamps.size() - 1)) {
//       break;
//     }
//
//     // Get time now and desired lerp time
//     const auto t_now = body_timestamps[i] + ts_offset;
//     const auto t_lerp = grid_timestamps[grid_idx];
//
//     // Update t0
//     if (t_now < t_lerp) {
//       t0 = t_now;
//       pose0 = body_poses[i];
//     }
//
//     // Update t1
//     if (t_now > t_lerp) {
//       // Lerp
//       t1 = t_now;
//       pose1 = body_poses[i];
//       const auto pose = lerp_pose(t0, pose0, t1, pose1, t_lerp);
//       lerped_poses.push_back(pose);
//       grid_idx++;
//
//       // Reset
//       t0 = t_now;
//       pose0 = body_poses[i];
//       t1 = 0;
//       pose1 = I(4);
//     }
//   }
// }

} // namespace yac
