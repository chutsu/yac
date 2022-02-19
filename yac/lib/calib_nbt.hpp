#ifndef YAC_CALIB_NBT_HPP
#define YAC_CALIB_NBT_HPP

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_nbv.hpp"

namespace yac {

/** TRAJECTORY GENERATION ****************************************************/

/**
 * Calibration Orbit Trajectories
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
void calib_orbit_trajs(const timestamp_t &ts_start,
                       const timestamp_t &ts_end,
                       const calib_target_t &target,
                       const camera_geometry_t *cam0_geoms,
                       const camera_params_t *cam0_params,
                       const extrinsics_t *imu_exts,
                       const mat4_t &T_WF,
                       const mat4_t &T_FO,
                       ctrajs_t &trajs);

/**
 * Calibration Pan Trajectories
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
void calib_pan_trajs(const timestamp_t &ts_start,
                     const timestamp_t &ts_end,
                     const calib_target_t &target,
                     const camera_geometry_t *cam0_geoms,
                     const camera_params_t *cam0_params,
                     const extrinsics_t *imu_exts,
                     const mat4_t &T_WF,
                     const mat4_t &T_FO,
                     ctrajs_t &trajs);

/**
 * Calibration Figure-8 Trajectories
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
void calib_figure8_trajs(const timestamp_t &ts_start,
                         const timestamp_t &ts_end,
                         const calib_target_t &target,
                         const camera_geometry_t *cam0_geoms,
                         const camera_params_t *cam0_params,
                         const extrinsics_t *imu_exts,
                         const mat4_t &T_WF,
                         const mat4_t &T_FO,
                         ctrajs_t &trajs);

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
aprilgrid_t sim_cam_view(const timestamp_t ts,
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
 * @param[in] traj NBT Trajectory
 * @param[in] ts_start Timestamp start
 * @param[in] ts_end Timestamp end
 * @returns 0 for success or -1 for failure
 */
int nbt_eval(const ctraj_t &traj,
             const timestamp_t &ts_start,
             const timestamp_t &ts_end);

} //  namespace yac
#endif // YAC_CALIB_NBT_HPP
