#ifndef YAC_CALIB_NBT_HPP
#define YAC_CALIB_NBT_HPP

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_nbv.hpp"

namespace yac {

void calib_orbit_trajs(const calib_target_t &target,
                       const camera_geometry_t *cam0_geom,
                       const camera_params_t *cam0,
                       const camera_geometry_t *cam1_geom,
                       const camera_params_t *cam1,
                       const mat4_t &T_BC0,
                       const mat4_t &T_BC1,
                       const mat4_t &T_WF,
                       const mat4_t &T_FO,
                       const timestamp_t &ts_start,
                       const timestamp_t &ts_end,
                       ctrajs_t &trajs);

// void calib_pan_trajs(const calib_target_t &target,
//                      const camera_geometry_t *cam0_geom,
//                      const camera_params_t *cam0,
//                      const camera_geometry_t *cam1_geom,
//                      const camera_params_t *cam1,
//                      const mat4_t &T_BC0,
//                      const mat4_t &T_BC1,
//                      const mat4_t &T_WF,
//                      const mat4_t &T_FO,
//                      const timestamp_t &ts_start,
//                      const timestamp_t &ts_end,
//                      ctrajs_t &trajs);
//
// void calib_figure8_trajs(const calib_target_t &target,
//                          const camera_geometry_t *cam0_geom,
//                          const camera_params_t *cam0,
//                          const camera_geometry_t *cam1_geom,
//                          const camera_params_t *cam1,
//                          const mat4_t &T_BC0,
//                          const mat4_t &T_BC1,
//                          const mat4_t &T_WF,
//                          const mat4_t &T_FO,
//                          const timestamp_t &ts_start,
//                          const timestamp_t &ts_end,
//                          ctrajs_t &trajs);
//
// aprilgrid_t calib_simulate(const calib_target_t &target,
//                            const mat4_t &T_FC0,
//                            const camera_geometry_t *cam_geom,
//                            const camera_params_t *cam_params,
//                            const mat4_t &T_C0Ci = I(4));
//
// aprilgrids_t calib_simulate(const calib_target_t &target,
//                             const mat4s_t &rel_poses,
//                             const camera_geometry_t *cam_geom,
//                             const camera_params_t *cam_params,
//                             const mat4_t &T_C0Ci = I(4));
//
// void simulate_cameras(const ctraj_t &traj,
//                       const calib_target_t &target,
//                       const camera_geometry_t *cam0_geom,
//                       const camera_params_t *cam0,
//                       const camera_geometry_t *cam1_geom,
//                       const camera_params_t *cam1,
//                       const double cam_rate,
//                       const mat4_t &T_WF,
//                       const mat4_t &T_BC0,
//                       const mat4_t &T_BC1,
//                       const timestamp_t &ts_start,
//                       const timestamp_t &ts_end,
//                       aprilgrids_t &grids0,
//                       aprilgrids_t &grids1,
//                       mat4s_t &T_WC0_sim);
//
// void simulate_imu(const ctraj_t &traj,
//                   const timestamp_t &ts_start,
//                   const timestamp_t &ts_end,
//                   const mat4_t &T_BC0,
//                   const mat4_t &T_BS,
//                   const imu_params_t &imu_params,
//                   timestamps_t &imu_time,
//                   vec3s_t &imu_accel,
//                   vec3s_t &imu_gyro,
//                   mat4s_t &imu_poses,
//                   vec3s_t &imu_vels);
//
// void nbt_create_timeline(const timestamps_t &imu_ts,
//                          const vec3s_t &imu_gyr,
//                          const vec3s_t &imu_acc,
//                          const std::vector<aprilgrids_t> grid_data,
//                          timeline_t &timeline);

} //  namespace yac
#endif // YAC_CALIB_NBT_HPP
