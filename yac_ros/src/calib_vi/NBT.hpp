#ifndef YAC_NBT_HPP
#define YAC_NBT_HPP

#include <string>

#include "core.hpp"
#include "cv/CameraBase.hpp"
#include "common/timeline.hpp"
#include "common/operators.hpp"
#include "common/Parameters.hpp"
// #include "yac/calib/calib_data.hpp"
// #include "yac/ceres/Calibrator.hpp"
#include "Estimator.hpp"

namespace yac {

// Check number of apriltags observed by camera
template <typename T>
size_t nb_apriltag_corners_observed(const calib_target_t &target,
                                    const T &camera,
                                    const mat4_t &T_FC) {
  const mat4_t T_CF = T_FC.inverse();

  aprilgrid_t grid;
  grid.tag_rows = target.tag_rows;
  grid.tag_cols = target.tag_cols;
  grid.tag_size = target.tag_size;
  grid.tag_spacing = target.tag_spacing;
  const size_t nb_tags = (grid.tag_rows * grid.tag_cols);

  std::vector<bool> observable;
  for (size_t tag_id = 0; tag_id < nb_tags; tag_id++) {
    vec3s_t object_points;
    aprilgrid_object_points(grid, tag_id, object_points);

    for (const auto &p_F : object_points) {
      const vec4_t hp_F = p_F.homogeneous();
      const vec3_t p_C = (T_CF * hp_F).head(3);
      vec2_t kp;
      if (camera.project(p_C, &kp) == CameraBase::ProjectionStatus::Successful) {
        observable.push_back(true);
      }
    }
  }

  return observable.size();
}

// Check to see if aprilgrid is fully observable by camera
template <typename T>
bool check_aprilgrid_observable(const calib_target_t &target,
                                const T &camera,
                                const mat4_t &T_FC) {
  const auto observable = nb_apriltag_corners_observed(target, camera, T_FC);
  const auto nb_tags = (target.tag_rows * target.tag_cols);

  // Verify
  if (observable != (nb_tags * 4)) {
    // LOG_ERROR("observed [%ld] expected [%ld]", observable.size(), (nb_tags * 4));
    return false;
  }
  return true;
}

/** Calculate target origin (O) w.r.t. fiducial (F) T_FO **/
template <typename T>
mat4_t calib_target_origin(const calib_target_t &target, const T &camera) {
  // Calculate target center
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double target_width = tag_cols * tag_size + spacing_x;
  const double target_height = tag_rows * tag_size + spacing_y;
  const vec2_t center{target_width / 2.0, target_height / 2.0};

  double scale = 0.5; // Target width to be 50% of image space at T_FO
  int retry = 5;
start:
  // Calculate distance away from target center
  const double image_width = camera.imageWidth();
  const double target_half_width =  target_width / 2.0;
  const double target_half_resolution_x = image_width / 2.0;
  const auto fx = camera.focalLengthU();
  const auto z_FO = fx * target_half_width / (target_half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  const mat4_t T_FO = tf(C_FO, r_FO);

  // Rotate the camera around to see if camera can actually observe the target
  const vec3_t rpy = deg2rad(vec3_t{-180.0, 0.0, 0.0});
  const mat3_t C_FC = euler321(rpy);
  const mat4_t T_FC = tf(C_FC, r_FO);
  if (check_aprilgrid_observable(target, camera, T_FC) == false) {
    scale -= 0.1;
    retry--;

    if (retry == 0) {
      FATAL("Failed to find calibration origin! Check camera params?");
    }
    goto start;
  }

  return T_FO;
}

template <typename T>
void generate_orbit_trajectories(const calib_target_t &target,
                                 const T &cam0, const T &cam1,
                                 const mat4_t &T_C0C1,
                                 const mat4_t &T_WF,
                                 const mat4_t &T_FO,
                                 const timestamp_t &ts_start,
                                 const timestamp_t &ts_end,
                                 ctrajs_t &trajs) {
  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Trajectory parameters
  double scale = 0.1;
  const double lat_min = deg2rad(0.0);
  const double lat_max = deg2rad(360.0);
  const double lon_min = deg2rad(0.0);
  const double lon_max = deg2rad(80.0);

  int retry = 20;
  ctrajs_t orbit_trajs;
start:
  double rho = (calib_width * scale);  // Sphere radius

  // Adjust the calibration origin such that trajectories are valid
  mat4_t calib_origin = T_FO;
  calib_origin(2, 3) = rho;

  // Orbit trajectories. Imagine a half sphere coming out from the
  // calibration target center. The trajectory would go from the pole of the
  // sphere to the sides of the sphere. While following the trajectory in a
  // tangent manner the camera view focuses on the target center.
	const auto nb_trajs = 8.0;
	const auto dlat = lat_max / nb_trajs;
	auto lat = lat_min;
  for (int i = 0; i < nb_trajs; i++) {
    vec3s_t positions;
    quats_t attitudes;

    // Create sphere point and transform it into world frame
    for (const auto &lon : linspace(lon_min, lon_max, 10)) {
      const vec3_t p = sphere(rho, lon, lat);
      const vec3_t r_FJ = tf_point(calib_origin, p);
      const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
      const mat4_t T_FC1 = T_FC0 * T_C0C1;
      const mat4_t T_WC0 = T_WF * T_FC0;

      if (check_aprilgrid_observable(target, cam0, T_FC0) == false) {
        orbit_trajs.clear();
        scale += 0.1;
        retry--;
        if (retry == 0){
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }
      if (check_aprilgrid_observable(target, cam1, T_FC1) == false) {
        orbit_trajs.clear();
        scale += 0.1;
        retry--;
        if (retry == 0){
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }

      positions.push_back(tf_trans(T_WC0));
      attitudes.emplace_back(tf_rot(T_WC0));
    }

    // Update
    const auto timestamps = linspace(ts_start, ts_end, 10);
    orbit_trajs.emplace_back(timestamps, positions, attitudes);
		lat += dlat;
  }

  // Append to results
  for (const auto &traj : orbit_trajs) {
    trajs.emplace_back(traj.timestamps, traj.positions, traj.orientations);
  }
}

template <typename T>
void generate_pan_trajectories(const calib_target_t &target,
                               const T &cam0, const T &cam1,
                               const mat4_t &T_C0C1,
                               const mat4_t &T_WF,
                               const mat4_t &T_FO,
                               const timestamp_t &ts_start,
                               const timestamp_t &ts_end,
                               ctrajs_t &trajs) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Calibration origin
  const mat4_t calib_origin = T_FO;

  // Create rotation matrix that is z-forard (i.e. camera frame)
  // const vec3_t rpy_WC{-90.0, 0.0, -90.0};
  // const mat3_t C_WC = euler321(deg2rad(rpy_WC));
  // const mat4_t T_WO = T_WF * T_FO;
  // const mat4_t T_FW = T_WF.inverse();
  // const mat3_t C_FW = tf_rot(T_FW);
  // const mat3_t C_FC = C_FW * C_WC;

  double scale = 1.0;
  ctrajs_t pan_trajs;
//   int retry = 20;
// start:
  // Trajectory parameters
  const int nb_trajs = 4;
  const int nb_control_points = 5;
  const double pan_length = (calib_width / 2.0) * scale;
  const double theta_min = deg2rad(0.0);
  const double theta_max = deg2rad(270.0);

  // Pan trajectories. Basically simple pan trajectories with the camera
  // looking forwards. No attitude changes.
  for (const auto &theta : linspace(theta_min, theta_max, nb_trajs)) {
    vec3s_t positions;
    quats_t attitudes;

		for (const auto &r: linspace(0.0, pan_length, nb_control_points)) {
			const vec2_t x = circle(r, theta);
			const vec3_t p{x(0), x(1), 0.0};

      const vec3_t r_FJ = tf_point(calib_origin, p);
      const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
      // const mat4_t T_FC1 = T_FC0 * T_C0C1;
      const mat4_t T_WC0 = T_WF * T_FC0;

      // if (check_aprilgrid_observable(target, cam0, T_FC0) == false) {
      //   pan_trajs.clear();
      //   scale -= 0.05;
      //   retry--;
      //   if (retry == 0){
      //     FATAL("Failed to generate orbit trajectory!");
      //   }
      //   goto start;
      // }
      // if (check_aprilgrid_observable(target, cam1, T_FC1) == false) {
      //   pan_trajs.clear();
      //   scale -= 0.05;
      //   retry--;
      //   if (retry == 0){
      //     FATAL("Failed to generate orbit trajectory!");
      //   }
      //   goto start;
      // }

			positions.emplace_back(tf_trans(T_WC0));
			attitudes.emplace_back(tf_rot(T_WC0));
		}

    // Add to trajectories
    const auto timestamps = linspace(ts_start, ts_end, nb_control_points);
    pan_trajs.emplace_back(timestamps, positions, attitudes);
  }

  // Append to results
  for (const auto &traj : pan_trajs) {
    trajs.emplace_back(traj.timestamps, traj.positions, traj.orientations);
  }
}

template <typename T>
void generate_inout_trajectory(const calib_target_t &target,
                               const T &cam0, const T &cam1,
                               const mat4_t &T_C0C1,
                               const mat4_t &T_WF,
                               const mat4_t &T_FO,
                               const timestamp_t &ts_start,
                               const timestamp_t &ts_end,
                               ctrajs_t &trajs) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Calibration origin
  const mat4_t calib_origin = T_FO;
  const vec3_t r_FO = tf_trans(T_FO);

  // Trajectory parameters
  const int nb_control_points = 5;
  const double inout_depth = r_FO(2) * 0.4;

  // In/out trajectory
  vec3s_t positions;
  quats_t attitudes;

  for (const auto &depth: linspace(0.0, inout_depth, nb_control_points)) {
    const vec3_t p{r_FO(0), r_FO(1), r_FO(2) - depth};
    const vec3_t r_FJ = tf_point(calib_origin, p);
    const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
    const mat4_t T_WC0 = T_WF * T_FC0;
    positions.emplace_back(tf_trans(T_WC0));
    attitudes.emplace_back(tf_rot(T_WC0));
  }

  // Add to trajectories
  const auto timestamps = linspace(ts_start, ts_end, nb_control_points);
  trajs.emplace_back(timestamps, positions, attitudes);
}

template <typename T>
void generate_roll_trajectories(const calib_target_t &target,
                                const T &cam0, const T &cam1,
                                const mat4_t &T_C0C1,
                                const mat4_t &T_WF,
                                const mat4_t &T_FO,
                                const timestamp_t &ts_start,
                                const timestamp_t &ts_end,
                                ctrajs_t &trajs) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Trajectory parameters
  const double circle_radius = calib_width / 4.0;
  const double theta_min = deg2rad(0.0);
  const double theta_max = deg2rad(180.0);

  // Roll left trajectory.
  {
    vec3s_t positions;
    quats_t attitudes;
    auto roll_deg = 0.0;
    auto droll = -135.0 / 10.0;

    for (const auto &theta : linspace(theta_min, theta_max, 10)) {
      const vec2_t p = circle(circle_radius, theta);
      const vec3_t r_FT = tf_point(T_FO, vec3_t{p(0), p(1), 0.0});
      const mat4_t T_FC0 = lookat(r_FT, r_FFc);
      const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
      const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);

      const mat3_t offset = euler321(deg2rad(vec3_t{roll_deg, 0.0, 0.0}));
      roll_deg += droll;

      positions.emplace_back(r_WC0);
      attitudes.emplace_back(offset * C_WC0);
    }

    // Add to trajectories
    const auto timestamps = linspace(ts_start, ts_end, 10);
    trajs.emplace_back(timestamps, positions, attitudes);
  }

  // Roll right trajectory.
  {
    vec3s_t positions;
    quats_t attitudes;
    auto roll_deg = 0.0;
    auto droll = 135.0 / 10.0;

    for (const auto &theta : linspace(M_PI, 0.0, 10)) {
      const vec2_t p = circle(circle_radius, theta);
      const vec3_t r_FT = tf_point(T_FO, vec3_t{p(0), p(1), 0.0});
      const mat4_t T_FC0 = lookat(r_FT, r_FFc);
      const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
      const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);

      const mat3_t offset = euler321(deg2rad(vec3_t{roll_deg, 0.0, 0.0}));
      roll_deg += droll;

      positions.emplace_back(r_WC0);
      attitudes.emplace_back(offset * C_WC0);
    }

    // Add to trajectories
    const auto timestamps = linspace(ts_start, ts_end, 10);
    trajs.emplace_back(timestamps, positions, attitudes);
  }

}

template <typename T>
void generate_figure8_trajectory(const calib_target_t &target,
                                 const T &cam0, const T &cam1,
                                 const mat4_t &T_C0C1,
                                 const mat4_t &T_WF,
                                 const mat4_t &T_FO,
                                 const timestamp_t &ts_start,
                                 const timestamp_t &ts_end,
                                 ctrajs_t &trajs) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Parameters for figure 8
  double a = calib_width / 2.0;

  // Create trajectory control points
  const size_t nb_control_points = 100;
  vec3s_t positions;
  quats_t attitudes;
  // -- R.H.S
  for (const auto t : linspace(0.0, M_PI, nb_control_points / 2.0)) {
    const auto x = a * sin(t);
    const auto y = a * sin(t) * cos(t);
    const auto z = 0.0;
    const vec3_t r_OC{x, y, z};  // Position of camera relative to calib origin

    const vec3_t r_FT = tf_point(T_FO, r_OC);
    const mat4_t T_FC0 = lookat(r_FT, r_FFc);
    const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
    const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);

    // Add control point to spline
    positions.emplace_back(r_WC0);
    attitudes.emplace_back(C_WC0);
  }
  // -- L.H.S
  for (const auto t : linspace(M_PI, 2 * M_PI, nb_control_points / 2.0)) {
    const auto x = a * sin(t);
    const auto y = a * sin(t) * cos(t);
    const auto z = 0.0;
    const vec3_t r_OC{x, y, z};  // Position of camera relative to calib origin

    const vec3_t r_FT = tf_point(T_FO, r_OC);
    const mat4_t T_FC0 = lookat(r_FT, r_FFc);
    const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
    const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);

    // Add control point to spline
    positions.emplace_back(r_WC0);
    attitudes.emplace_back(C_WC0);
  }
  // -- Create spline
  const auto timestamps = linspace(ts_start, ts_end, nb_control_points);
  trajs.emplace_back(timestamps, positions, attitudes);
}

void simulate_imu(const ctraj_t &traj,
                  const timestamp_t &ts_start,
                  const timestamp_t &ts_end,
                  const mat4_t &T_SC,
                  const yac::ImuParameters &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels);

template <typename T>
aprilgrid_t nbv_create_aprilgrid(const calib_target_t &target,
                                 const T &camera,
                                 const mat4_t &T_CF) {
  // Create AprilGrid
  aprilgrid_t grid;
  grid.configured = true;
  grid.tag_rows = target.tag_rows;
  grid.tag_cols = target.tag_cols;
  grid.tag_size = target.tag_size;
  grid.tag_spacing = target.tag_spacing;

  // For every AprilGrid tag
  grid.detected = true;
  int nb_tags = grid.tag_rows * grid.tag_cols;
  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    // Check if object points are observable
    vec3s_t object_points;
    aprilgrid_object_points(grid, tag_id, object_points);
    vec2s_t observed_keypoints;
    vec3s_t observed_points_CF;

    for (const auto &p_F : object_points) {
      const vec4_t hp_F = p_F.homogeneous();
      const vec3_t p_C = (T_CF * hp_F).head(3);
      vec2_t kp;
      if (camera.project(p_C, &kp) == CameraBase::ProjectionStatus::Successful) {
        observed_keypoints.push_back(kp);
        observed_points_CF.push_back(p_C);
      }
    }

    // Add keypoints and points
    if (observed_keypoints.size() == 4) {
      grid.ids.push_back(tag_id);
      for (size_t i = 0; i < 4; i++) {
        grid.keypoints.push_back(observed_keypoints[i]);
        grid.points_CF.push_back(observed_points_CF[i]);
      }
    }
  }
  grid.nb_detections = grid.points_CF.size() / 4.0;

  // Esimated pose
  grid.estimated = true;
  grid.T_CF = T_CF;

  return grid;
}

template <typename T>
void simulate_cameras(const ctraj_t &traj,
                      const calib_target_t &target,
                      const T &cam0,
                      const T &cam1,
                      const double cam_rate,
                      const mat4_t &T_WF,
                      const mat4_t &T_SC0,
                      const mat4_t &T_SC1,
                      const timestamp_t &ts_start,
                      const timestamp_t &ts_end,
                      aprilgrids_t &grids0,
                      aprilgrids_t &grids1,
                      mat4s_t &T_WC0_sim) {
  // Simulate camera measurements with AprilGrids that will be observed
  const timestamp_t cam_dt = sec2ts(1.0 / cam_rate);
  const mat4_t T_C1C0 = T_SC1.inverse() * T_SC0;
  timestamp_t ts_k = ts_start;

  while (ts_k <= ts_end) {
    // Calculate transform of fiducial (F) w.r.t. camera (C)
    const mat4_t T_WC0 = ctraj_get_pose(traj, ts_k);
    const mat4_t T_C0F = T_WC0.inverse() * T_WF;
    const mat4_t T_C1F = T_C1C0 * T_C0F;

    // Create an AprilGrid that represents what the camera would see if it was
    // positioned at T_C0F
    auto grid0 = nbv_create_aprilgrid(target, cam0, T_C0F);
    auto grid1 = nbv_create_aprilgrid(target, cam1, T_C1F);
    grid0.timestamp = ts_k;
    grid1.timestamp = ts_k;

    grids0.push_back(grid0);
    grids1.push_back(grid1);

    T_WC0_sim.push_back(T_WC0);
    ts_k += cam_dt;
  }
}

struct nbt_res_blocks_t {
  ceres::PoseErrorPtr pose_prior;
  ceres::FiducialErrorPtr fiducial_prior;
  ceres::TimeDelayErrorPtr timedelay_prior;
  std::map<int, ceres::CameraIntrinsicsErrorPtr> cam_priors;
  std::map<int, ceres::ExtrinsicsErrorPtr> ext_priors;

  std::vector<ceres::ImuErrorPtr> imu_errors;
  std::vector<ceres::SpeedAndBiasErrorPtr> sb_errors;
  std::vector<ceres::CalibReprojErrorPtr> reproj_errors;
};

void create_timeline(const timestamps_t &imu_ts,
                     const vec3s_t &imu_gyr,
                     const vec3s_t &imu_acc,
                     const std::vector<aprilgrids_t> grid_data,
                     timeline_t<timestamp_t> &timeline);

template <typename T>
int eval_traj(const int traj_idx,
						  const ctraj_t &traj,
              const calib_target_t &target,
              const timestamp_t &ts_start,
              const timestamp_t &ts_end,
              const yac::ImuParameters &imu_params,
              const T &cam0, const T &cam1,
              const double cam_rate,
              const mat4_t T_WF,
              const mat4_t T_SC0,
              const mat4_t T_SC1,
              matx_t &calib_info,
              bool save=false,
							const std::string save_dir="/tmp/nbt") {
	profiler_t profiler;

  // Save trajectories
	auto traj_save_dir = save_dir + "/traj_" + std::to_string(traj_idx);
  if (save) {
    remove_dir(traj_save_dir);
    dir_create(traj_save_dir);
		ctraj_save(traj, traj_save_dir + "/traj.csv");
  }

  // Simulate camera frames
	profiler.start("simulate_cameras");
  // clang-format off
  aprilgrids_t grids0;
  aprilgrids_t grids1;
  mat4s_t T_WC_sim;
  simulate_cameras(traj, target,
                   cam0, cam1, cam_rate,
                   T_WF, T_SC0, T_SC1,
                   ts_start, ts_end,
                   grids0, grids1,
                   T_WC_sim);
	profiler.stop("simulate_cameras");

  // -- Save data
  if (save) {
    const std::string cam0_save_dir = traj_save_dir + "/cam0";
    const std::string cam1_save_dir = traj_save_dir + "/cam1";
    dir_create(cam0_save_dir);
    dir_create(cam1_save_dir);

    for (const auto &grid : grids0) {
      const auto ts = grid.timestamp;
			auto save_path = cam0_save_dir + "/" + std::to_string(ts) + ".csv";
      aprilgrid_save(grid, save_path);
    }

    for (const auto &grid : grids1) {
      const auto ts = grid.timestamp;
			auto save_path = cam1_save_dir + "/" + std::to_string(ts) + ".csv";
      aprilgrid_save(grid, save_path);
    }
  }
  // clang-format on

  // Simulate imu measurements
	profiler.start("simulate_imu");
  // clang-format off
  timestamps_t imu_time;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  mat4s_t imu_poses;
  vec3s_t imu_vels;
  simulate_imu(traj, ts_start, ts_end,
               T_SC0, imu_params,
               imu_time, imu_accel, imu_gyro,
               imu_poses, imu_vels);
	profiler.stop("simulate_imu");

  // -- Save data
  if (save) {
    dir_create(traj_save_dir + "/imu");
    save_data(traj_save_dir + "/imu/accel.csv", imu_time, imu_accel);
    save_data(traj_save_dir + "/imu/gyro.csv", imu_time, imu_gyro);
    save_data(traj_save_dir + "/imu/vel.csv", imu_time, imu_vels);
    save_poses(traj_save_dir + "/imu/poses.csv", imu_time, imu_poses);
  }
  // clang-format on

  // Create timeline
	profiler.start("create_timeline");
  timeline_t<timestamp_t> timeline;
  create_timeline(imu_time, imu_gyro, imu_accel, {grids0, grids1}, timeline);
	profiler.stop("create_timeline");

  // Setup Estimator
	profiler.start("setup_estimator");
  yac::ceres::Estimator est;
  est.opt_config_.save_estimates = false;
  est.mode_ = "nbt";
  // -- Add calib target
  est.addCalibTarget(target);
  // -- Add IMU
  est.addImu(imu_params);
  est.addImuTimeDelay(0.0);
  // -- Add Camera
  est.addCamera(0, cam0);
  est.addCamera(1, cam1);
  // -- Add sensor camera extrinsics
  est.addSensorCameraExtrinsics(0, T_SC0);
  est.addSensorCameraExtrinsics(1, T_SC1);

  for (const auto &ts : timeline.timestamps) {
    const auto result = timeline.data.equal_range(ts);
    // -- Loop through events at timestamp ts since there could be two events
    // at the same timestamp
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;
      // Camera event
      if (event.type == APRILGRID_EVENT) {
        est.addMeasurement(event.camera_index, event.grid);
      }

      // Imu event
      if (event.type == IMU_EVENT) {
        const auto ts = Time(ns2sec(event.ts));
        const vec3_t w_m = event.w_m;
        const vec3_t a_m = event.a_m;
        est.addMeasurement(ts, w_m, a_m);
      }
    }
  }
	profiler.stop("setup_estimator");
	// est.optimize(5, 1, true);
  // std::cout << est.problem_->summary.FullReport() << std::endl;
	// est.optimizeBatch(5, 1, false);
  // std::cout << est.batch_problem_->summary.FullReport() << std::endl;
  // printf("cam0 reproj error: %f\n", est.rmseReprojectionError(0));
  // printf("cam1 reproj error: %f\n", est.rmseReprojectionError(1));

  // printf("marg error: %d\n", (est.marg_error_ != nullptr));
  // printf("nb imu errors: %ld\n", est.residual_collection_.imu_errors.size());
  // printf("nb sb errors: %ld\n", est.residual_collection_.sb_errors.size());
  // printf("nb reproj errors: %ld\n", est.residual_collection_.reproj_errors.size());

  matx_t H;
  vecx_t b;
  form_hessian(est.residual_collection_.imu_errors,
               // est.residual_collection_.sb_errors,
               {},
               est.residual_collection_.reproj_errors,
               nullptr,
               nullptr,
               nullptr,
               nullptr,
               {},
               {},
               H,
               &b);

  // const matx_t covar = H.ldlt().solve(I(H.rows()));
  // const matx_t calib_covar = covar.bottomRightCorner(28, 28);
  // calib_info = calib_covar.ldlt().solve(I(28));

  const size_t r = 28;
  const size_t m = H.rows() - r;
  schurs_complement(H, b, m, r);
  calib_info = H;

  if (full_rank(calib_info) == false) {
    printf("rows(calib_info): %ld\t", calib_info.rows());
    printf("rank(calib_info): %ld\n", rank(calib_info));
    LOG_ERROR("calib_info is not full rank!");
    return -1;
  }

  profiler.stop("recover_calib_covar");
  // profiler.print();

  return 0;
}

struct test_grid_t {
  const int grid_rows = 5;
  const int grid_cols = 5;
  const double grid_depth = 1.5;
  const size_t nb_points = grid_rows * grid_cols;
  vec3s_t object_points;
  vec2s_t keypoints;

  template <typename T>
  test_grid_t(const T &cam) {
    const double dx = cam.imageWidth() / (grid_cols + 1);
    const double dy = cam.imageHeight() / (grid_rows + 1);
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
        Eigen::Vector3d ray;
        cam.backProject(kp, &ray);
        object_points.push_back(ray * grid_depth);
      }
      kp_x = 0;
    }
  }
};

// Returns reprojection uncertainty stddev in pixels
template <typename T>
double eval_reproj_uncertainty(const int cam_idx,
                               const matx_t &calib_covar,
                               const T &cam,
                               const mat4_t &T_SC,
                               cv::Mat *covar_map=nullptr) {
  // Form test grid
  test_grid_t test_grid(cam);

  // Setup covar_x
  matx_t covar_x = zeros(6 + 8, 6 + 8);
	if (cam_idx == 0) {
		covar_x.block(0, 0, 6, 6) = calib_covar.block(0, 0, 6, 6);    // Sensor-camera0 covar
		covar_x.block(6, 6, 8, 8) = calib_covar.block(12, 12, 8, 8);  // Camera0 params covar
	} else if (cam_idx == 1) {
		covar_x.block(0, 0, 6, 6) = calib_covar.block(6, 6, 6, 6);    // Sensor-camera1 covar
		covar_x.block(6, 6, 8, 8) = calib_covar.block(20, 20, 8, 8);  // Camera1 params covar
	}

  // Evaluate reprojection uncertainty
  if (covar_map) {
    auto rows = cam.imageHeight();
    auto cols = cam.imageWidth();
    *covar_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  }

  double max_eigval = 0.0;
  auto C_SC = tf_rot(T_SC);
  for (size_t i = 0; i < test_grid.nb_points; i++) {
    const vec3_t p_C = test_grid.object_points[i];
    const vec2_t kp = test_grid.keypoints[i];

    // Project and camera params jacobian
    vec2_t image_point;
    mat_t<2, 3> Jh;
    Eigen::Matrix2Xd J_camera_params;
    auto retval = cam.project(p_C, &image_point, &Jh, &J_camera_params);
    if (retval != CameraBase::ProjectionStatus::Successful) {
      // FATAL("Failed to project point!");
      continue;
    }

    // Sensor-camera Jacobian
    Eigen::Matrix<double, 2, 6> J_sensor_camera;
    J_sensor_camera.block(0, 0, 2, 3) = Jh * -C_SC.transpose();
    J_sensor_camera.block(0, 3, 2, 3) = Jh * -C_SC.transpose() * -kinematics::crossMx(C_SC * p_C);

    // Input Jacobian
    mat_t<2, 14> Jx;
    Jx.block(0, 0, 2, 6) = J_sensor_camera;
    Jx.block(0, 6, 2, 8) = J_camera_params;

    // Output covariance (First order error progagtion)
    const matx_t covar_y = Jx * covar_x * Jx.transpose();

    // Check covar_y
    if (covar_y(0, 0) < 0 || covar_y(1, 1) < 0 || covar_y.determinant() < 0) {
      LOG_WARN("Bad output covar");
      print_matrix("covar_y", covar_y);
      return INFINITY;
    }

		// Find max eigen-value. The largest eigen-value represents the largest
		// "spread" or variance.
    Eigen::SelfAdjointEigenSolver<matx_t> eigensolver(covar_y);
    max_eigval = std::max(max_eigval, eigensolver.eigenvalues().maxCoeff());

    if (covar_map) {
      cv::Point p(kp(0), kp(1));
      int radius = 5;
      double percentage = 1.0 - eigensolver.eigenvalues().maxCoeff();

      cv::Scalar color;
      if (percentage < 0.4) {
        color = cv::Scalar(0, 0, 255);    // Red
      } else if (percentage < 0.9) {
        color = cv::Scalar(0, 128, 255);  // Orage
      } else if (percentage <= 1.0) {
        color = cv::Scalar(0, 255, 0);    // Green
      }

      cv::circle(*covar_map, p, radius, color, cv::FILLED);
    }
  }

  // Calculate sqrt(max variance), ie. stddev in pixels
  return sqrt(max_eigval);
}

template <typename T>
void nbt_compute(const calib_target_t &target,
                 const ImuParameters &imu_params,
                 const T &cam0,
                 const T &cam1,
                 const double cam_rate,
                 const mat4_t &T_WF,
                 const mat4_t &T_SC0,
                 const mat4_t &T_SC1,
                 ctrajs_t &trajs,
                 std::vector<matx_t> &calib_infos,
                 bool save=false) {
  // Generate trajectories
  const mat4_t T_FO = calib_target_origin(target, cam0);
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = sec2ts(1.0);
  const mat4_t T_C0C1 = T_SC0.inverse() * T_SC1;
  generate_orbit_trajectories(target, cam0, cam1, T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  generate_pan_trajectories(target, cam0, cam1, T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  generate_inout_trajectory(target, cam0, cam1, T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  generate_roll_trajectories(target, cam0, cam1, T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  // generate_figure8_trajectory(target, cam0, cam1, T_C0C1, T_WF, T_FO, ts_start, sec2ts(2.0), trajs);

  // Evaluate trajectory
  calib_infos.resize(trajs.size());
  #pragma omp parallel for
  for (size_t i = 0; i < trajs.size(); i++) {
    int retval = eval_traj(i, trajs[i], target, ts_start, ts_end,
                           imu_params, cam0, cam1, cam_rate,
                           T_WF, T_SC0, T_SC1, calib_infos[i], save);
    if (retval != 0) {
      FATAL("Failed to eval NBT");
    }
  }
}

template <typename T>
int nbt_find(const size_t frame_idx,
             const T cam0, const T cam1,
             const matx_t T_SC0, const matx_t T_SC1,
             const matx_t calib_info,
             const ctrajs_t &nbt_trajs,
             const std::vector<matx_t> &nbt_calib_infos) {
  // Pre-check
  if (nbt_trajs.size() != nbt_calib_infos.size()) {
    FATAL("nbt_trajs.size() != nbt_calib_infos.size()");
  }

  // Estimate calibration covariance
  if (rank(calib_info) != calib_info.rows()) {
    FATAL("calib_info is not full rank!");
  }

  // Obtaining the information by inverting the covariance matrix
  const matx_t H0 = calib_info;
  if (rank(H0) != H0.rows()) {
    FATAL("H0 is not full rank!");
  }

  // Find next best trajectory
	// -- Keep track of stddev before NBT
  cv::Mat covar_map0;
  const matx_t calib_covar = H0.ldlt().solve(I(H0.rows()));
  const auto score0_init = eval_reproj_uncertainty(0, calib_covar, cam0, T_SC0);
  const auto score1_init = eval_reproj_uncertainty(1, calib_covar, cam1, T_SC1);
	const auto score_init = score0_init + score1_init;

	// -- Find an NBT that reduces the stddev
  std::vector<cv::Mat> covar_maps;
  size_t best_index = 0;
  auto best_score = score_init;

  printf("--\n");
  for (size_t i = 0; i < nbt_trajs.size(); i++) {
		// Calculate information gain
		const matx_t H_nbt = nbt_calib_infos[i];
    const matx_t H_new = H0 + H_nbt;
    const matx_t calib_covar_nbt = H_new.ldlt().solve(I(H_new.rows()));

    // Evaluate NBT
    cv::Mat covar_map;
    const auto score0 = eval_reproj_uncertainty(0, calib_covar_nbt, cam0, T_SC0);
		const auto score1 = eval_reproj_uncertainty(1, calib_covar_nbt, cam1, T_SC1);
		const auto score = score0 + score1;
    covar_maps.push_back(covar_map);
    // printf("score[%ld]: %f\n", i, score);

    // Keep track of best stddev
    if (score < best_score) {
      best_index = i;
      best_score = score;
    }
  }
  printf("frame_idx: %ld\n", frame_idx);
  printf("best_index: %ld\n", best_index);
  printf("trace(calib_covar): %f\n", calib_covar.trace());
  printf("score[before nbt]: %f\n", score_init);
  printf("score[after nbt]: %f\n", best_score);
  printf("--\n");

  return best_index;
}

} // namespace yac
#endif // YAC_NBT_HPP
