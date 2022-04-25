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
 * NBT Orbit Trajectories
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
void nbt_orbit_trajs(const timestamp_t &ts_start,
                     const timestamp_t &ts_end,
                     const calib_target_t &target,
                     const camera_geometry_t *cam0_geoms,
                     const camera_params_t *cam0_params,
                     const extrinsics_t *imu_exts,
                     const mat4_t &T_WF,
                     const mat4_t &T_FO,
                     ctrajs_t &trajs);

/**
 * NBT Pan Trajectories
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
void nbt_pan_trajs(const timestamp_t &ts_start,
                   const timestamp_t &ts_end,
                   const calib_target_t &target,
                   const camera_geometry_t *cam0_geoms,
                   const camera_params_t *cam0_params,
                   const extrinsics_t *imu_exts,
                   const mat4_t &T_WF,
                   const mat4_t &T_FO,
                   ctrajs_t &trajs);

/**
 * NBT Figure-8 Trajectories
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
void nbt_figure8_trajs(const timestamp_t &ts_start,
                       const timestamp_t &ts_end,
                       const calib_target_t &target,
                       const camera_geometry_t *cam0_geoms,
                       const camera_params_t *cam0_params,
                       const extrinsics_t *imu_exts,
                       const mat4_t &T_WF,
                       const mat4_t &T_FO,
                       ctrajs_t &trajs);

/**
 * Lisajous Trajectory
 */
struct lisajous_traj_t {
  std::string traj_type = "figure8";

  real_t R; // Distance from calibration target
  real_t A; // Amplitude in x-axis
  real_t B; // Amplitude in y-axis
  real_t a;
  real_t b;
  real_t delta;
  real_t T; // Period - Time it takes to complete [secs]
  real_t f; // Frequency
  real_t w; // Angular velocity
  real_t phase_offset;

  real_t pitch_bound;
  real_t yaw_bound;

  lisajous_traj_t();
  ~lisajous_traj_t();

  vec3_t get_position(const real_t t) const;
  mat3_t get_attitude(const real_t t) const;
  mat4_t get_pose(const real_t t) const;
  vec3_t get_velocity(const real_t t) const;
  vec3_t get_acceleration(const real_t t) const;
  vec3_t get_angular_velocity(const real_t t) const;
};

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
                      const ctraj_t &traj,
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
                  const ctraj_t &traj,
                  const imu_params_t &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels);

/** NBT EVALUATION ***********************************************************/

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
int nbt_eval(const ctraj_t &traj, const calib_vi_t &calib, matx_t &calib_covar);

/**
 * Find NBT
 *
 * @param[in] calib Calibrator
 * @param[in] traj NBT Trajectory
 * @param[in] verbose Verbose
 *
 * @returns 0 for success or -1 for failure
 */
int nbt_find(const ctrajs_t &trajs,
             const calib_vi_t &calib,
             const bool verbose = false);

} //  namespace yac
#endif // YAC_CALIB_NBT_HPP
