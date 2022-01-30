#ifndef YAC_CALIB_NBV_HPP
#define YAC_CALIB_NBV_HPP

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_vi.hpp"

namespace yac {

struct nbv_test_grid_t {
  const int grid_rows = 5;
  const int grid_cols = 5;
  const double grid_depth = 1.5;
  const size_t nb_points = grid_rows * grid_cols;
  vec3s_t object_points;
  vec2s_t keypoints;

  nbv_test_grid_t(const camera_geometry_t *cam_geom,
                  const camera_params_t &cam) {
    const int img_w = cam.resolution[0];
    const int img_h = cam.resolution[1];
    const double dx = img_w / (grid_cols + 1);
    const double dy = img_h / (grid_rows + 1);
    double kp_x = 0;
    double kp_y = 0;

    for (int i = 1; i < (grid_cols + 1); i++) {
      kp_y += dy;
      for (int j = 1; j < (grid_rows + 1); j++) {
        kp_x += dx;

        // Keypoint
        const vec2_t kp{kp_x, kp_y};
        keypoints.push_back(kp);

        // Object point
        vec3_t ray;
        cam_geom->back_project(cam.param, kp, ray);
        object_points.push_back(ray * grid_depth);
      }
      kp_x = 0;
    }
  }
};

double info_entropy(const matx_t &covar);
// double info_gain(const matx_t &calib_covar, const double info_prev);
double shannon_entropy(const matx_t &covar);

bool check_fully_observable(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const mat4_t &T_FC);

mat4_t calib_target_origin(const calib_target_t &target,
                           const vec2_t &cam_res,
                           const double hfov);

mat4_t calib_target_origin(const calib_target_t &target,
                           const camera_geometry_t *cam_geom,
                           const camera_params_t *cam_params,
                           const double target_scale = 0.5);

mat4s_t calib_init_poses(const calib_target_t &target,
                         const camera_geometry_t *cam_geom,
                         const camera_params_t *cam_params);

mat4s_t calib_nbv_poses(const calib_target_t &target,
                        const camera_geometry_t *cam_geom,
                        const camera_params_t *cam_params,
                        const int range_x_size = 5,
                        const int range_y_size = 5,
                        const int range_z_size = 2);

std::map<int, mat4s_t>
calib_nbv_poses(const calib_target_t &target,
                const std::map<int, camera_geometry_t *> &cam_geoms,
                const std::map<int, camera_params_t *> &cam_params,
                const std::map<int, extrinsics_t *> &cam_exts,
                const int range_x_size = 5,
                const int range_y_size = 5,
                const int range_z_size = 2);

aprilgrid_t nbv_target_grid(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const timestamp_t nbv_ts,
                            const mat4_t &nbv_pose);

vec2s_t nbv_draw(const calib_target_t &target,
                 const camera_geometry_t *cam_geom,
                 const camera_params_t *cam_params,
                 const mat4_t &T_FC,
                 cv::Mat &image);

// void calib_orbit_trajs(const calib_target_t &target,
//                        const camera_geometry_t *cam0_geom,
//                        const camera_params_t *cam0,
//                        const camera_geometry_t *cam1_geom,
//                        const camera_params_t *cam1,
//                        const mat4_t &T_BC0,
//                        const mat4_t &T_BC1,
//                        const mat4_t &T_WF,
//                        const mat4_t &T_FO,
//                        const timestamp_t &ts_start,
//                        const timestamp_t &ts_end,
//                        ctrajs_t &trajs);
//
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
#endif // YAC_CALIB_NBV_HPP
