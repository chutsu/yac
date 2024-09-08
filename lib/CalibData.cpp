#include "CalibData.hpp"

namespace yac {

CalibData::CalibData(const std::string &config_path)
    : config_path_{config_path} {}

void CalibData::addCamera(const int camera_index,
                          const std::string &camera_model,
                          const vec2i_t &resolution,
                          const vecx_t &intrinsic,
                          const vec7_t &extrinsic) {
  camera_geoms_[camera_index] = std::make_shared<CameraGeometry>(camera_index,
                                                                 camera_model,
                                                                 resolution,
                                                                 intrinsic,
                                                                 extrinsic);
}

void CalibData::loadConfig(const std::string &config_path) {
  // Parse config
  config_t config{config_path};

  // -- Parse data settings
  parse(config, "data_path", data_path_);

  // -- Parse target settings
  parse(config, "calibration_target.target_type", target_type_);
  parse(config, "calibration_target.tag_rows", tag_rows_);
  parse(config, "calibration_target.tag_cols", tag_cols_);
  parse(config, "calibration_target.tag_size", tag_size_);
  parse(config, "calibration_target.tag_spacing", tag_spacing_);

  // -- Parse camera settings
  for (int camera_index = 0; camera_index < 100; camera_index++) {
    // Check if key exists
    const std::string prefix = "cam" + std::to_string(camera_index);
    if (yaml_has_key(config, prefix) == 0) {
      continue;
    }

    // Load camera data
    loadCameraData(camera_index);

    // Parse
    vec2i_t resolution;
    std::string camera_model;
    parse(config, prefix + ".resolution", resolution);
    parse(config, prefix + ".camera_model", camera_model);

    vecx_t intrinsic;
    if (yaml_has_key(config, prefix + ".intrinsic")) {
      parse(config, prefix + ".intrinsic", intrinsic);
    }

    vec7_t extrinsic;
    if (yaml_has_key(config, prefix + ".extrinsic")) {
      parse(config, prefix + ".extrinsic", extrinsic);
    } else {
      extrinsic << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }

    // Add camera
    addCamera(camera_index, camera_model, resolution, intrinsic, extrinsic);
  }
}

void CalibData::loadCameraData(const int camera_index) {
  assert(target_type_ == "aprilgrid");
  assert(tag_rows_ > 0);
  assert(tag_cols_ > 0);
  assert(tag_size_ > 0);
  assert(tag_spacing_ > 0);

  // Setup detector
  AprilGridDetector detector{tag_rows_, tag_cols_, tag_size_, tag_spacing_};

  // Form calibration target path
  const std::string camera_string = "cam" + std::to_string(camera_index);
  const std::string camera_path = data_path_ + "/" + camera_string;
  std::string target_dir = data_path_;
  target_dir += (data_path_.back() == '/') ? "" : "/";
  target_dir += "calib_target/" + camera_string;

  // Create grids path
  if (system(("mkdir -p " + target_dir).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", target_dir.c_str());
  }

  // Get image files
  std::vector<std::string> image_paths;
  list_files(camera_path, image_paths);

  // Detect aprilgrids
  const std::string desc = "Loading " + camera_string + " data: ";
  for (size_t k = 0; k < image_paths.size(); k++) {
    print_progress((double) k / (double) image_paths.size(), desc);
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
    std::shared_ptr<AprilGrid> calib_target;
    if (file_exists(target_path)) {
      calib_target = AprilGrid::load(target_path);
    } else {
      calib_target = detector.detect(ts, image);
      calib_target->save(target_path);
    }
    camera_data_[camera_index][ts] = calib_target;
  }
  printf("\n");
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
