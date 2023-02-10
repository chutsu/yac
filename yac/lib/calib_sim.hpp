#ifndef YAC_CALIB_SIM_HPP
#define YAC_CALIB_SIM_HPP

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

enum sim_event_type_t {
  NOT_SET,
  CAMERA,
  IMU,
};

struct sim_event_t {
  sim_event_type_t type = NOT_SET;
  int sensor_id = 0;
  timestamp_t ts = 0;
  imu_meas_t imu;
  cam_frame_t frame;

  // Camera event
  sim_event_t(const int sensor_id_,
              const timestamp_t &ts_,
              const vec2s_t &kps_,
              const std::vector<size_t> &feature_idxs_)
      : type{CAMERA}, sensor_id{sensor_id_}, ts{ts_}, frame{ts_,
                                                            kps_,
                                                            feature_idxs_} {}

  // IMU event
  sim_event_t(const int sensor_id_,
              const timestamp_t &ts_,
              const vec3_t &accel_,
              const vec3_t &gyro_)
      : type{IMU}, sensor_id{sensor_id_}, ts{ts_}, imu{ts_, accel_, gyro_} {}
};

//////////////////
// VIO SIM DATA //
//////////////////

struct vio_sim_data_t {
  // Settings
  real_t sensor_velocity = 0.3;
  real_t cam_rate = 30;
  real_t imu_rate = 400;

  // Scene data
  vec3s_t features;

  // Camera data
  timestamps_t cam_ts;
  vec3s_t cam_pos_gnd;
  quats_t cam_rot_gnd;
  mat4s_t cam_poses_gnd;
  vec3s_t cam_pos;
  quats_t cam_rot;
  mat4s_t cam_poses;
  std::vector<std::vector<size_t>> observations;
  std::vector<vec2s_t> keypoints;

  // IMU data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  vec3s_t imu_pos;
  quats_t imu_rot;
  mat4s_t imu_poses;
  vec3s_t imu_vel;

  // Simulation timeline
  std::multimap<timestamp_t, sim_event_t> timeline;

  // Sim
  vio_sim_data_t(const real_t circle_r = 5.0);
  ~vio_sim_data_t() = default;

  // Add IMU measurement to timeline
  void _add(const int sensor_id,
            const timestamp_t &ts,
            const vec3_t &accel,
            const vec3_t &gyro);

  // Add camera frame to timeline
  void _add(const int sensor_id,
            const timestamp_t &ts,
            const vec2s_t &keypoints,
            const std::vector<size_t> &feature_idxs);

  // Save
  void save(const std::string &dir);
};

////////////////////
// CALIB SIM DATA //
////////////////////

struct calib_sim_data_t {
  // Settings
  real_t sensor_velocity = 0.3;
  real_t cam_rate = 30;
  real_t imu_rate = 400;

  // Scene data
  vec3s_t features;

  // Camera data
  timestamps_t cam_ts;
  vec3s_t cam_pos_gnd;
  quats_t cam_rot_gnd;
  mat4s_t cam_poses_gnd;
  vec3s_t cam_pos;
  quats_t cam_rot;
  mat4s_t cam_poses;
  std::vector<std::vector<size_t>> observations;
  std::vector<vec2s_t> keypoints;

  // IMU data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  vec3s_t imu_pos;
  quats_t imu_rot;
  mat4s_t imu_poses;
  vec3s_t imu_vel;

  // Simulation timeline
  std::multimap<timestamp_t, sim_event_t> timeline;

  // Sim
  calib_sim_data_t(const real_t circle_r = 5.0);
  ~calib_sim_data_t() = default;

  // Add IMU measurement to timeline
  void _add(const int sensor_id,
            const timestamp_t &ts,
            const vec3_t &accel,
            const vec3_t &gyro);

  // Add camera frame to timeline
  void _add(const int sensor_id,
            const timestamp_t &ts,
            const vec2s_t &keypoints,
            const std::vector<size_t> &feature_idxs);

  // Save
  void save(const std::string &dir);
};

} //  namespace yac
#endif // YAC_CALIB_SIM_HPP
