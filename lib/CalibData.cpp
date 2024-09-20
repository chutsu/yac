#include "CalibData.hpp"

namespace yac {

CalibData::CalibData(const std::string &config_path)
    : config_path_{config_path} {
  // Parse config
  config_t config{config_path_};

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
  const std::string camera_path = data_path_ + "/" + camera_string + "/data";
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
    print_progress((double)k / (double)image_paths.size(), desc);
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

    // Add camera measurement
    addCameraMeasurement(ts, camera_index, calib_target);
  }
  printf("\n");
}

bool CalibData::hasCameraMeasurement(const timestamp_t ts,
                                     const int camera_index) const {
  if (camera_data_.at(camera_index).count(ts) == 0) {
    return false;
  }
  return true;
}

void CalibData::addCamera(const int camera_index,
                          const std::string &camera_model,
                          const vec2i_t &resolution,
                          const vecx_t &intrinsic,
                          const vec7_t &extrinsic) {
  camera_geometries_[camera_index] =
      std::make_shared<CameraGeometry>(camera_index,
                                       camera_model,
                                       resolution,
                                       intrinsic,
                                       extrinsic);
}

void CalibData::addCameraMeasurement(const timestamp_t ts,
                                     const int camera_index,
                                     const CalibTargetPtr &calib_target) {
  camera_data_[camera_index][ts] = calib_target;
}

void CalibData::addTargetPoint(const int point_id, const vec3_t &point) {
  if (target_points_.count(point_id) == 0) {
    target_points_[point_id] = point;
  }
}

int CalibData::getNumCameras() const { return camera_geometries_.size(); }

std::map<int, CameraData> &CalibData::getAllCameraData() {
  return camera_data_;
}

CameraData &CalibData::getCameraData(const int camera_index) {
  return camera_data_.at(camera_index);
}

CameraGeometryPtr &CalibData::getCameraGeometry(const int camera_index) {
  return camera_geometries_.at(camera_index);
}

vec3_t &CalibData::getTargetPoint(const int point_id) {
  return target_points_[point_id];
}

} // namespace yac
