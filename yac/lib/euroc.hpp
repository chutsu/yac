#ifndef YAC_EUROC_HPP
#define YAC_EUROC_HPP

#include <string>
#include <libgen.h>
#include <inttypes.h>

#include "core.hpp"
#include "timeline.hpp"

namespace yac {

/*****************************************************************************
 * euroc_imu_t
 ****************************************************************************/

/**
 * EuRoC IMU data
 */
struct euroc_imu_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  vec3s_t w_B;
  vec3s_t a_B;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;

  euroc_imu_t() {}

  euroc_imu_t(const std::string &data_dir_) : data_dir{data_dir_} {
    const std::string data_path = data_dir + "/data.csv";
    const std::string sensor_path = data_dir + "/sensor.yaml";

    // Open file for loading
    int nb_rows = 0;
    FILE *fp = file_open(data_path, "r", &nb_rows);
    if (fp == nullptr) {
      FATAL("Failed to open [%s]!", data_path.c_str());
    }

    // Parse file
    for (int i = 0; i < nb_rows; i++) {
      // Skip first line
      if (i == 0) {
        skip_line(fp);
        continue;
      }

      // Parse line
      timestamp_t ts = 0;
      double w_x, w_y, w_z = 0.0;
      double a_x, a_y, a_z = 0.0;
      int retval = fscanf(fp,
                          "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                          &ts,
                          &w_x,
                          &w_y,
                          &w_z,
                          &a_x,
                          &a_y,
                          &a_z);
      if (retval != 7) {
        FATAL("Failed to parse line in [%s]", data_path.c_str());
      }

      timestamps.push_back(ts);
      w_B.emplace_back(w_x, w_y, w_z);
      a_B.emplace_back(a_x, a_y, a_z);
    }
    fclose(fp);

    // Load calibration data
    config_t config{sensor_path};
    if (config.ok != true) {
      FATAL("Failed to load sensor file [%s]!", sensor_path.c_str());
    }
    parse(config, "sensor_type", sensor_type);
    parse(config, "comment", comment);
    parse(config, "T_BS", T_BS);
    parse(config, "rate_hz", rate_hz);
    parse(config, "gyroscope_noise_density", gyro_noise_density);
    parse(config, "gyroscope_random_walk", gyro_random_walk);
    parse(config, "accelerometer_noise_density", accel_noise_density);
    parse(config, "accelerometer_random_walk", accel_random_walk);

    ok = true;
  }

  ~euroc_imu_t() {}
};

/**
 * EuRoC IMU data to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_imu_t &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "gyroscope_noise_density: " << data.gyro_noise_density << std::endl;
  os << "gyroscope_random_walk: " << data.gyro_random_walk << std::endl;
  os << "accelerometer_noise_density: " << data.accel_noise_density << std::endl;
  os << "accelerometer_random_walk: " << data.accel_random_walk << std::endl;
  // clang-format on

  return os;
}

/*****************************************************************************
 * euroc_camera_t
 ****************************************************************************/

/**
 * EuRoC camera data
 */
struct euroc_camera_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  std::vector<std::string> image_paths;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  vec2_t resolution = zeros(2, 1);
  std::string camera_model;
  vec4_t intrinsics = zeros(4, 1);
  std::string distortion_model;
  vec4_t distortion_coefficients = zeros(4, 1);

  euroc_camera_t() {}

  euroc_camera_t(const std::string &data_dir_, bool is_calib_data=false)
    : data_dir{data_dir_} {
    const std::string data_path = data_dir + "/data.csv";
    const std::string sensor_path = data_dir + "/sensor.yaml";

    // Open file for loading
    int nb_rows = 0;
    FILE *fp = file_open(data_path, "r", &nb_rows);
    if (fp == nullptr) {
      FATAL("Failed to open [%s]!", data_path.c_str());
    }

    // Parse file
    for (int i = 0; i < nb_rows; i++) {
      // Skip first line
      if (i == 0) {
        skip_line(fp);
        continue;
      }

      // Parse line
      timestamp_t ts = 0;
      char filename[50] = {0};
      int retval = fscanf(fp, "%" SCNu64 ",%s", &ts, filename);
      if (retval != 2) {
        FATAL("Failed to parse line in [%s]", data_path.c_str());
      }

      // Check if file exists
      const std::string image_file{filename};
      const auto image_path = data_dir + "/data/" + image_file;
      if (file_exists(image_path) == false) {
        FATAL("File [%s] does not exist!", image_path.c_str());
      }

      // Add data
      timestamps.emplace_back(ts);
      image_paths.emplace_back(image_path);
    }
    fclose(fp);

    // Load calibration data
    config_t config{sensor_path};
    if (config.ok == false) {
      FATAL("Failed to load senor file [%s]!", sensor_path.c_str());
    }
    parse(config, "sensor_type", sensor_type);
    parse(config, "comment", comment);
    parse(config, "T_BS", T_BS);
    parse(config, "rate_hz", rate_hz);
    parse(config, "resolution", resolution);
    parse(config, "camera_model", camera_model, is_calib_data);
    parse(config, "intrinsics", intrinsics, is_calib_data);
    parse(config, "distortion_model", distortion_model, is_calib_data);
    parse(config,
          "distortion_coefficients",
          distortion_coefficients,
          is_calib_data);

    ok = true;
  }

  ~euroc_camera_t() {}
};

/**
 * EuRoC camera to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_camera_t &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "resolution: " << data.resolution.transpose() << std::endl;
  os << "camera_model: " << data.camera_model << std::endl;
  os << "intrinsics: " << data.intrinsics.transpose() << std::endl;
  os << "distortion_model: " << data.distortion_model << std::endl;
  os << "distortion_coefficients: " << data.distortion_coefficients.transpose() << std::endl;
  // clang-format on

  return os;
}

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

/**
 * EuRoC ground truth
 */
struct euroc_ground_truth_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  vec3s_t p_RS_R;
  vec4s_t q_RS;
  vec3s_t v_RS_R;
  vec3s_t b_w_RS_S;
  vec3s_t b_a_RS_S;

  euroc_ground_truth_t() {}

  euroc_ground_truth_t(const std::string &data_dir_)
      : data_dir{data_dir_} {
    // Open file for loading
    const std::string data_path = data_dir + "/data.csv";
    int nb_rows = 0;
    FILE *fp = file_open(data_path, "r", &nb_rows);
    if (fp == nullptr) {
      FATAL("Failed to open [%s]!", data_path.c_str());
    }

    // Parse file
    std::string str_format;
    str_format += "%" SCNu64 ",";     // Timestamp
    str_format += "%lf,%lf,%lf,";     // Position
    str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
    str_format += "%lf,%lf,%lf,";     // Velocity
    str_format += "%lf,%lf,%lf,";     // Gyro bias
    str_format += "%lf,%lf,%lf";      // Accel bias

    for (int i = 0; i < nb_rows; i++) {
      // Skip first line
      if (i == 0) {
        skip_line(fp);
        continue;
      }

      // Parse line
      timestamp_t ts = 0;
      double p_x, p_y, p_z = 0.0;
      double q_x, q_y, q_z, q_w = 0.0;
      double v_x, v_y, v_z = 0.0;
      double b_w_x, b_w_y, b_w_z = 0.0;
      double b_a_x, b_a_y, b_a_z = 0.0;
      int retval = fscanf(fp,
                          str_format.c_str(),
                          &ts,
                          &p_x,
                          &p_y,
                          &p_z,
                          &q_x,
                          &q_y,
                          &q_z,
                          &q_w,
                          &v_x,
                          &v_y,
                          &v_z,
                          &b_w_x,
                          &b_w_y,
                          &b_w_z,
                          &b_a_x,
                          &b_a_y,
                          &b_a_z);
      if (retval != 17) {
        FATAL("Failed to parse line in [%s]", data_path.c_str());
      }

      timestamps.push_back(ts);
      p_RS_R.emplace_back(p_x, p_y, p_z);
      q_RS.emplace_back(q_w, q_x, q_y, q_z);
      v_RS_R.emplace_back(v_x, v_y, v_z);
      b_w_RS_S.emplace_back(b_w_x, b_w_y, b_w_z);
      b_a_RS_S.emplace_back(b_a_x, b_a_y, b_a_z);
    }
    fclose(fp);

    ok = true;
  }

  ~euroc_ground_truth_t() {}
};

/*****************************************************************************
 * euroc_data_t
 ****************************************************************************/

/**
 * EuRoC data
 */
struct euroc_data_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool ok = false;
  std::string data_path;

  euroc_imu_t imu_data;
  euroc_camera_t cam0_data;
  euroc_camera_t cam1_data;
  euroc_ground_truth_t ground_truth;
  cv::Size image_size;

  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  timestamp_t ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, double> time;
  std::multimap<timestamp_t, timeline_event_t> timeline;

  euroc_data_t() {}

  euroc_data_t(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {
    // Load IMU data
    imu_data = euroc_imu_t{data_path + "/mav0/imu0"};
    for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
      const timestamp_t ts = imu_data.timestamps[i];
      const vec3_t a_B = imu_data.a_B[i];
      const vec3_t w_B = imu_data.w_B[i];
      const auto imu_event = timeline_event_t{ts, a_B, w_B};
      timeline.insert({ts, imu_event});
    }

    // Load camera data
    // -- Load cam0 data
    const auto cam0_path = data_path + "/mav0/cam0";
    cam0_data = euroc_camera_t{cam0_path};
    for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
      const timestamp_t ts = cam0_data.timestamps[i];
      const auto image_path = cam0_data.image_paths[i];
      const auto cam0_event = timeline_event_t(ts, 0, image_path);
      timeline.insert({ts, cam0_event});
    }
    // -- Load cam1 data
    const auto cam1_path = data_path + "/mav0/cam1";
    cam1_data = euroc_camera_t{cam0_path};
    for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
      const timestamp_t ts = cam1_data.timestamps[i];
      const auto image_path = cam1_data.image_paths[i];
      const auto cam1_event = timeline_event_t(ts, 1, image_path);
      timeline.insert({ts, cam1_event});
    }
    // -- Set camera image size
    cv::Mat image = cv::imread(cam0_data.image_paths[0]);
    image_size = cv::Size(image.size());

    // Load ground truth
    const auto gt_path = data_path + "/mav0/state_groundtruth_estimate0";
    ground_truth = euroc_ground_truth_t{gt_path};

    // Process timestamps
    ts_start = min_timestamp();
    ts_end = max_timestamp();
    ts_now = ts_start;

    // Get timestamps and calculate relative time
    auto it = timeline.begin();
    auto it_end = timeline.end();
    while (it != it_end) {
      const timestamp_t ts = it->first;
      timestamps.insert(ts);
      time[ts] = ((double) ts - ts_start) * 1e-9;

      // Advance to next non-duplicate entry.
      do {
        ++it;
      } while (ts == it->first);
    }

    ok = true;
  }

  ~euroc_data_t() {}

  void reset() {
    ts_start = min_timestamp();
    ts_end = max_timestamp();
    ts_now = ts_start;
    time_index = 0;
    imu_index = 0;
    frame_index = 0;
  }

  timestamp_t min_timestamp() {
    const timestamp_t cam0_ts0 = cam0_data.timestamps.front();
    const timestamp_t imu_ts0 = imu_data.timestamps.front();

    timestamps_t first_ts{cam0_ts0, imu_ts0};
    auto ts0 = std::min_element(first_ts.begin(), first_ts.end());
    const size_t first_ts_index = std::distance(first_ts.begin(), ts0);

    return first_ts[first_ts_index];
  }

  timestamp_t max_timestamp() {
    const timestamp_t cam0_last_ts = cam0_data.timestamps.back();
    const timestamp_t imu_last_ts = imu_data.timestamps.back();

    timestamps_t last_ts{cam0_last_ts, imu_last_ts};
    auto last_result = std::max_element(last_ts.begin(), last_ts.end());
    const timestamp_t last_ts_index = std::distance(last_ts.begin(), last_result);
    const timestamp_t max_ts = last_ts[last_ts_index];

    return max_ts;
  }
};

/*****************************************************************************
 * euroc_target_t
 ****************************************************************************/

/**
 * EuRoC calibration target
 */
struct euroc_target_t {
  bool ok = false;
  std::string file_path;

  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  euroc_target_t() {}

  euroc_target_t(const std::string &target_file) : file_path{target_file} {
    config_t config{target_file};
    if (config.ok != true) {
      FATAL("Failed to load target file [%s]!", target_file.c_str());
    }
    parse(config, "target_type", type);
    parse(config, "tagRows", tag_rows);
    parse(config, "tagCols", tag_cols);
    parse(config, "tagSize", tag_size);
    parse(config, "tagSpacing", tag_spacing);

    ok = true;
  }

  ~euroc_target_t() {}
};

/**
 * EuRoC target to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_target_t &target) {
  os << "target_type: " << target.type << std::endl;
  os << "tag_rows: " << target.tag_rows << std::endl;
  os << "tag_cols: " << target.tag_cols << std::endl;
  os << "tag_size: " << target.tag_size << std::endl;
  os << "tag_spacing: " << target.tag_spacing << std::endl;
  return os;
}

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

/**
 * EuRoC calibration data
 */
struct euroc_calib_t {
  bool ok = false;

  // Settings
  std::string data_path;
  bool imshow = false;

  // Data
  euroc_imu_t imu_data;
  euroc_camera_t cam0_data;
  euroc_camera_t cam1_data;
  euroc_target_t calib_target;
  cv::Size image_size;

  euroc_calib_t(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {
    // Load IMU data
    const std::string imu_data_dir = data_path + "/mav0/imu0";
    imu_data = euroc_imu_t{imu_data_dir};

    // Load cam0 data
    const std::string cam0_dir = data_path + "/mav0/cam0";
    cam0_data = euroc_camera_t{cam0_dir, true};

    // Load cam1 data
    const std::string cam1_dir = data_path + "/mav0/cam1";
    cam1_data = euroc_camera_t{cam1_dir, true};

    // Check if cam0 has same amount of images as cam1
    const size_t cam0_nb_images = cam0_data.image_paths.size();
    const size_t cam1_nb_images = cam1_data.image_paths.size();
    if (cam0_nb_images != cam1_nb_images) {
      if (cam0_nb_images > cam1_nb_images) {
        cam0_data.timestamps.pop_back();
        cam0_data.image_paths.pop_back();
      } else if (cam0_nb_images < cam1_nb_images) {
        cam1_data.timestamps.pop_back();
        cam1_data.image_paths.pop_back();
      }
    }

    // Get image size
    const cv::Mat image = cv::imread(cam0_data.image_paths[0]);
    image_size = cv::Size(image.size());

    // Load calibration target data
    const std::string target_path = data_path + "/april_6x6.yaml";
    calib_target = euroc_target_t{target_path};

    ok = true;
  }

  ~euroc_calib_t() {}

  timeline_t timeline() {
    // Create timeline
    timeline_t timeline;

    // -- Add cam0 events
    for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
      const auto ts = cam0_data.timestamps[i];
      const auto img_path = cam0_data.image_paths[i];
      const timeline_event_t event{ts, 0, img_path};
      timeline.add(event);
    }
    // -- Add cam1 events
    for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
      const auto ts = cam1_data.timestamps[i];
      const auto img_path = cam1_data.image_paths[i];
      const timeline_event_t event{ts, 1, img_path};
      timeline.add(event);
    }
    // -- Add imu events
    for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
      const timestamp_t ts = imu_data.timestamps[i];
      const auto a_B = imu_data.a_B[i];
      const auto w_B = imu_data.w_B[i];
      const timeline_event_t event{ts, a_B, w_B};
      timeline.add(event);
    }

    return timeline;
  }
};

} // namespace yac
#endif /* YAC_EUROC_HPP */
