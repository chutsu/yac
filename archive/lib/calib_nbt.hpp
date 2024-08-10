#ifndef YAC_CALIB_NBT_HPP
#define YAC_CALIB_NBT_HPP

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_nbv.hpp"
#include "calib_vi.hpp"

namespace yac {

/** TRAJECTORY GENERATION ****************************************************/

/**
 * Lissajous Trajectory
 */
struct lissajous_traj_t {
  const std::string traj_type = "figure8";
  timestamp_t ts_start; // Start timestamp
  mat4_t T_WF;          // Fiducial pose
  mat4_t T_FO;          // Calibration origin
  real_t calib_width;   // Calibration width
  real_t calib_height;  // Calibration height

  real_t R;     // Distance from calibration target
  real_t A;     // Amplitude in x-axis
  real_t B;     // Amplitude in y-axis
  real_t a;     // Angular velocity in x-axis
  real_t b;     // Angular velocity in y-axis
  real_t delta; // Phase offset

  real_t T; // Period - Time it takes to complete [secs]
  real_t f; // Frequency
  real_t w; // Angular velocity

  real_t yaw_scale;   // Yaw scale
  real_t pitch_scale; // Pitch scale

  real_t pitch_bound;
  real_t yaw_bound;

  lissajous_traj_t() = delete;
  lissajous_traj_t(const std::string &traj_type_,
                   const timestamp_t ts_start_,
                   const mat4_t &T_WF_,
                   const mat4_t &T_FO_,
                   const real_t R_,
                   const real_t A_,
                   const real_t B_,
                   const real_t T_);
  ~lissajous_traj_t() = default;

  quat_t get_q_OS(const timestamp_t ts_k) const;
  quat_t get_q_OS_dot(const timestamp_t ts_k) const;

  mat4_t get_pose(const timestamp_t ts_k) const;
  vec3_t get_velocity(const timestamp_t ts_k) const;
  vec3_t get_acceleration(const timestamp_t ts_k) const;
  vec3_t get_angular_velocity(const timestamp_t ts_k) const;
  int save(const std::string &save_path) const;
};

// Vector container for lissajous trajectories
using lissajous_trajs_t = std::vector<lissajous_traj_t>;

/**
 * NBT Lassojous Trajectories
 *
 * @param[in] ts_start Start timestamp
 * @param[in] ts_end End timestamp
 * @param[in] target Calibration target configuration
 * @param[in] cam0_geoms Camera0 geometry
 * @param[in] cam0_params Camera0 parameters
 * @param[in] imu_exts Imu-Camera extrinsics T_C0S
 * @param[in] T_WF Fiducial world pose
 * @param[in] T_FO Fiducial-Calibration origin relative pose
 * @param[out] trajs Simulated trajectories
 */
void nbt_lissajous_trajs(const timestamp_t &ts_start,
                         const timestamp_t &ts_end,
                         const calib_target_t &target,
                         const mat4_t &T_WF,
                         lissajous_trajs_t &trajs);

/** SIMULATION ****************************************************************/

/**
 * Simulate camera view
 *
 * @param[in] ts Timestamp
 * @param[in] target Calibration target configuration
 * @param[in] T_FC0 Fiducial-Camera relative pose
 * @param[in] cam_geom Camera geometry
 * @param[in] cam_params Camera parameters
 * @param[in] cam_exts Camera extrinsics
 *
 * @return AprilGrid
 */
aprilgrid_t simulate_aprilgrid(const timestamp_t ts,
                               const calib_target_t &target,
                               const mat4_t &T_FC0,
                               const camera_geometry_t *cam_geom,
                               const camera_params_t *cam_params,
                               const extrinsics_t *cam_exts);

/**
 * Simulate cameras
 *
 * @param[in] ts_start Start timestamp
 * @param[in] ts_end End timestamp
 * @param[in] traj Continuous trajectory
 * @param[in] target Calibration target configuration
 * @param[in] cam_geoms Camera geometries
 * @param[in] cam_params Camera parameters
 * @param[in] cam_exts Camera extrinsics
 * @param[in] cam_rate Camera frame rate
 * @param[in] T_WF Fiducial pose in world frame
 * @param[out] cam_grids Camera observed AprilGrids
 * @param[out] T_WC0_sim Simulated camera world pose
 */
void simulate_cameras(const timestamp_t &ts_start,
                      const timestamp_t &ts_end,
                      const lissajous_traj_t &traj,
                      const calib_target_t &target,
                      const CamIdx2Geometry &cam_geoms,
                      const CamIdx2Parameters &cam_params,
                      const CamIdx2Extrinsics &cam_exts,
                      const double cam_rate,
                      const mat4_t &T_WF,
                      camera_data_t &cam_grids,
                      std::map<timestamp_t, mat4_t> &T_WC0_sim);

/**
 * Simulate IMU
 *
 * @param[in] ts_start Start timestamp
 * @param[in] ts_end End timestamp
 * @param[in] traj Continuous trajectory
 * @param[out] imu_params IMU parameters
 * @param[out] imu_time Simulated IMU timestamps
 * @param[out] imu_accel Simulated IMU accelerometer measurements
 * @param[out] imu_gyro Simulated IMU gyroscope measurements
 * @param[out] imu_poses Simulated IMU poses
 * @param[out] imu_vels Simulated IMU velocities
 */
void simulate_imu(const timestamp_t &ts_start,
                  const timestamp_t &ts_end,
                  const lissajous_traj_t &traj,
                  const imu_params_t &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels);

/** NBT EVALUATION ***********************************************************/

/**
 * Next-Best-Trajectory Data
 */
struct nbt_data_t {
  // Calibration target
  calib_target_t calib_target;

  // IMU
  real_t imu_rate;
  imu_params_t imu_params;
  std::unique_ptr<extrinsics_t> imu_exts;
  std::unique_ptr<time_delay_t> time_delay;

  // Cameras
  real_t cam_rate;
  CamIdx2Geometry cam_geoms;
  CamIdx2Parameters cam_params;
  CamIdx2Extrinsics cam_exts;

  // Fiducial pose
  mat4_t T_WF;

  nbt_data_t() = default;
  nbt_data_t(const calib_vi_t &calib);
  ~nbt_data_t() = default;
};

/**
 * Create timeline for NBT
 *
 * @param[in] cam_grids Camera data
 * @param[in] imu_ts IMU timestamps
 * @param[in] imu_acc IMU accelerometer measurments
 * @param[in] imu_gyr IMU gyroscope measurements
 * @param[out] timeline Timeline
 */
void nbt_create_timeline(const camera_data_t &cam_grids,
                         const timestamps_t &imu_ts,
                         const vec3s_t &imu_acc,
                         const vec3s_t &imu_gyr,
                         timeline_t &timeline);

/**
 * Evaluate NBT
 *
 * @param[in] calib Calibrator
 * @param[in] traj NBT Trajectory
 * @param[out] calib_covar Calibration covariance matrix
 *
 * @returns 0 for success or -1 for failure
 */
int nbt_eval(const lissajous_traj_t &traj,
             const nbt_data_t &nbt_data,
             const matx_t &H,
             matx_t &H_nbt);

/**
 * Find NBT
 *
 * @param[in] calib Calibrator
 * @param[in] traj NBT Trajectory
 * @param[in] verbose Verbose
 *
 * @returns 0 for success or -1 for failure
 */
int nbt_find(const lissajous_trajs_t &trajs,
             const nbt_data_t &nbt_data,
             const matx_t &H,
             const bool verbose = false,
             real_t *info_k = nullptr,
             real_t *info_kp1 = nullptr);

} //  namespace yac
#endif // YAC_CALIB_NBT_HPP
