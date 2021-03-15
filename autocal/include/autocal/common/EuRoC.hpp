#ifndef AUTOCAL_DATASET_EUROC_HPP
#define AUTOCAL_DATASET_EUROC_HPP

#include <string>
#include <libgen.h>
#include <inttypes.h>

#include "core.hpp"
// #include "./Core.hpp"
#include "./AprilGrid.hpp"
#include "./CalibData.hpp"
#include "./Timeline.hpp"

namespace autocal {

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

  euroc_imu_t();
  euroc_imu_t(const std::string &data_dir_);
  ~euroc_imu_t();
};

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

  euroc_camera_t();
  euroc_camera_t(const std::string &data_dir_, bool is_calib_data_ = false);
  ~euroc_camera_t();
};

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

  euroc_ground_truth_t();
  euroc_ground_truth_t(const std::string &data_dir_);
  ~euroc_ground_truth_t();
};

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
  std::multimap<timestamp_t, timeline_event_t<timestamp_t>> timeline;

  euroc_data_t();
  euroc_data_t(const std::string &data_path);
  ~euroc_data_t();
};

struct euroc_target_t {
  bool ok = false;
  std::string file_path;

  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  euroc_target_t();
  euroc_target_t(const std::string &file_path);
  ~euroc_target_t();
};

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

  euroc_calib_t();
  euroc_calib_t(const std::string &data_path);
  ~euroc_calib_t();
};

/**
 * Load IMU data
 *
 * @param[in] data IMU data
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int euroc_imu_load(euroc_imu_t &data, const std::string &data_dir);

/**
 * Load Camera data
 *
 * @param[in,out] data Camera data
 * @param[in] data_dir Data directory
 * @param[in] is_calib_data Is camera data for calibration?
 * @returns 0 for success, -1 for failure
 */
int euroc_camera_load(euroc_camera_t &data,
                      const std::string &data_dir,
                      const bool is_calib_data = false);

/**
 * Load ground truth data
 *
 * @param[in,out] data Ground truth data
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int euroc_ground_truth_load(euroc_ground_truth_t &data,
                            const std::string &data_dir);

/**
 * Load dataset
 *
 * @param[in,out] data Dataset
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int euroc_data_load(euroc_data_t &data, const std::string &data_path);

/**
 * Reset
 *
 * @param[in,out] data Dataset
 * @returns 0 for success, -1 for failure
 */
void euroc_data_reset(euroc_data_t &data);

/**
 * Return min timestamp
 *
 * @param[in] data Dataset
 * @returns Minimum timestamp
 */
timestamp_t euroc_data_min_timestamp(const euroc_data_t &data);

/**
 * Return max timestamp
 * @returns Maximum timestamp
 */
timestamp_t euroc_data_max_timestamp(const euroc_data_t &data);

/**
 * Load calibration target settings
 *
 * @param target_file Path to target yaml file
 * @returns 0 for success, -1 for failure
 */
int euroc_target_load(euroc_target_t &target, const std::string &target_file);

/**
 * Load calibration data
 * @returns 0 for success, -1 for failure
 */
int euroc_calib_load(euroc_calib_t &data, const std::string &data_path);

/**
 * Process EuRoC calibration data and detect the AprilGrid detected from both
 * cameras. The detected AprilGrids will be outputted as files into the
 * `preprocess_path`, and as `aprilgrids_t` in `cam0_grids` and `cam1_grids`.
 */
int process_stereo_images(const euroc_calib_t &calib_data,
                          const std::string &preprocess_path,
                          const mat3_t &cam0_K,
                          const vec4_t &cam0_D,
                          const mat3_t &cam1_K,
                          const vec4_t &cam1_D,
                          aprilgrids_t &cam0_grids,
                          aprilgrids_t &cam1_grids);

/**
 * Create timeline from EuRoC calibration data `calib_data`. The timeline data
 * will put the detected aprilgrids from cam0 and cam1 inplace of the cam0 and
 * cam1 image paths. Using the aprilgrid data and initial sensor-camera
 * extrinsics `T_SC0`, the initial sensor poses in world frame `T_WS`, fiducial
 * pose in world frame `T_WF` and finally first timestamp `t0` will be
 * calculated.
 */
timeline_t<timestamp_t> create_timeline(const euroc_calib_t &calib_data,
                                        const aprilgrids_t &cam0_grids,
                                        const aprilgrids_t &cam1_grids,
                                        const mat4_t &T_SC0,
                                        mat4s_t &T_WS,
                                        mat4_t &T_WF,
                                        timestamp_t &t0);

/**
 * `euroc_imu_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_imu_t &data);

/**
 * `euroc_camera_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_camera_t &data);

/**
 * `calib_target_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_target_t &target);

} // namespace autocal
#endif /* AUTOCAL_DATASET_EUROC_HPP */
