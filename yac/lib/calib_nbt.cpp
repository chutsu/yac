#include "calib_nbt.hpp"

namespace yac {

/** TRAJECTORY GENERATION ****************************************************/

void calib_orbit_trajs(const timestamp_t &ts_start,
                       const timestamp_t &ts_end,
                       const calib_target_t &target,
                       const camera_geometry_t *cam0_geom,
                       const camera_params_t *cam0_params,
                       const extrinsics_t *imu_exts,
                       const mat4_t &T_WF,
                       const mat4_t &T_FO,
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
  const mat4_t T_C0S = imu_exts->tf();
start:
  double rho = (calib_width * scale); // Sphere radius

  // Adjust the calibration origin such that trajectories are valid
  mat4_t calib_origin = T_FO;
  calib_origin(2, 3) = rho;

  // Orbit trajectories. Imagine a half sphere coming out from the
  // calibration target center. The trajectory would go from the pole of the
  // sphere to the sides of the sphere. While following the trajectory in a
  // tangent manner the camera view focuses on the target center.
  const auto nb_trajs = 8;
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
      const mat4_t T_WC0 = T_WF * T_FC0;

      if (!check_fully_observable(target, cam0_geom, cam0_params, T_FC0)) {
        orbit_trajs.clear();
        scale += 0.1;
        retry--;
        if (retry == 0) {
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }

      const mat4_t T_WS = T_WC0 * T_C0S;
      const vec3_t r_WS = tf_trans(T_WS);
      const mat3_t C_WS = tf_rot(T_WS);
      positions.push_back(r_WS);
      attitudes.emplace_back(C_WS);
    }

    // Create return journey
    for (int i = (int)(positions.size() - 1); i >= 0; i--) {
      positions.push_back(positions[i]);
      attitudes.push_back(attitudes[i]);
    }

    // Update
    const auto timestamps = linspace(ts_start, ts_end, positions.size());
    orbit_trajs.emplace_back(timestamps, positions, attitudes);
    lat += dlat;
  }

  // Append to results
  for (const auto &traj : orbit_trajs) {
    trajs.emplace_back(traj.timestamps, traj.positions, traj.orientations);
  }
}

void calib_pan_trajs(const timestamp_t &ts_start,
                     const timestamp_t &ts_end,
                     const calib_target_t &target,
                     const camera_geometry_t *cam0_geom,
                     const camera_params_t *cam0_params,
                     const extrinsics_t *imu_exts,
                     const mat4_t &T_WF,
                     const mat4_t &T_FO,
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

  // Calibration origin
  const mat4_t calib_origin = T_FO;

  int retry = 20;
  double scale = 1.0;
  ctrajs_t pan_trajs;
  const mat4_t T_C0S = imu_exts->tf();
start:
  // Trajectory parameters
  const int nb_trajs = 4;
  const int nb_control_points = 5;
  const double pan_length = calib_width * scale;
  const double theta_min = deg2rad(0.0);
  const double theta_max = deg2rad(270.0);

  // Pan trajectories. Basically simple pan trajectories with the camera
  // looking forwards. No attitude changes.
  for (const auto &theta : linspace(theta_min, theta_max, nb_trajs)) {
    vec3s_t positions;
    quats_t attitudes;

    for (const auto &r : linspace(0.0, pan_length, nb_control_points)) {
      const vec2_t x = circle(r, theta);
      const vec3_t p{x(0), x(1), 0.0};
      const vec3_t r_FJ = tf_point(calib_origin, p);
      const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
      const mat4_t T_WC0 = T_WF * T_FC0;

      if (!check_fully_observable(target, cam0_geom, cam0_params, T_FC0)) {
        pan_trajs.clear();
        scale -= 0.05;
        retry--;
        if (retry == 0) {
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }

      const mat4_t T_WS = T_WC0 * T_C0S;
      const vec3_t r_WS = tf_trans(T_WS);
      const mat3_t C_WS = tf_rot(T_WS);
      positions.emplace_back(r_WS);
      attitudes.emplace_back(C_WS);
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

void calib_figure8_trajs(const timestamp_t &ts_start,
                         const timestamp_t &ts_end,
                         const calib_target_t &target,
                         const camera_geometry_t *cam0_geom,
                         const camera_params_t *cam0_params,
                         const extrinsics_t *imu_exts,
                         const mat4_t &T_WF,
                         const mat4_t &T_FO,
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

  // Parameters for figure 8
  double a = calib_width / 3.4;

  // Create trajectory control points
  const mat4_t T_C0S = imu_exts->tf();
  const size_t nb_control_points = 100;
  vec3s_t positions;
  quats_t attitudes;
  // -- R.H.S
  for (const auto t : linspace(0.0, M_PI, nb_control_points / 2.0)) {
    // Form position of camera relative to calib origin
    const auto x = a * sin(t);
    const auto y = a * sin(t) * cos(t);
    const auto z = 0.0;
    const vec3_t r_OC{x, y, z};

    // Form T_WC0
    const vec3_t r_FT = tf_point(T_FO, r_OC);
    const mat4_t T_FC0 = lookat(r_FT, r_FFc);
    const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
    const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);
    const mat4_t T_WC0 = tf(C_WC0, r_WC0);

    // Add control point to spline
    const mat4_t T_WS = T_WC0 * T_C0S;
    const vec3_t r_WS = tf_trans(T_WS);
    const mat3_t C_WS = tf_rot(T_WS);
    positions.emplace_back(r_WS);
    attitudes.emplace_back(C_WS);
  }
  // -- L.H.S
  for (const auto t : linspace(M_PI, 2 * M_PI, nb_control_points / 2.0)) {
    // Form position of camera relative to calib origin
    const auto x = a * sin(t);
    const auto y = a * sin(t) * cos(t);
    const auto z = 0.0;
    const vec3_t r_OC{x, y, z};

    // Form T_WC0
    const vec3_t r_FT = tf_point(T_FO, r_OC);
    const mat4_t T_FC0 = lookat(r_FT, r_FFc);
    const vec3_t r_WC0 = tf_trans(T_WF * T_FC0);
    const mat3_t C_WC0 = tf_rot(T_WF * T_FC0);
    const mat4_t T_WC0 = tf(C_WC0, r_WC0);

    // Add control point to spline
    const mat4_t T_WS = T_WC0 * T_C0S;
    const vec3_t r_WS = tf_trans(T_WS);
    const mat3_t C_WS = tf_rot(T_WS);
    positions.emplace_back(r_WS);
    attitudes.emplace_back(C_WS);
  }
  // -- Create spline
  const auto timestamps = linspace(ts_start, ts_end, nb_control_points);
  trajs.emplace_back(timestamps, positions, attitudes);
}

/** SIMULATION ***************************************************************/

aprilgrid_t simulate_aprilgrid(const timestamp_t ts,
                               const calib_target_t &target,
                               const mat4_t &T_FC0,
                               const camera_geometry_t *cam_geom,
                               const camera_params_t *cam_param,
                               const extrinsics_t *cam_exts) {
  // Calibration target
  const int tag_rows = target.tag_rows;
  const int tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  // Camera params and extrinsics
  const auto cam_res = cam_param->resolution;
  const vecx_t params = cam_param->param;
  const mat4_t T_CiC0 = cam_exts->tf().inverse();

  // Simulate AprilGrid observation
  const mat4_t T_C0F = T_FC0.inverse();
  aprilgrid_t grid(ts, tag_rows, tag_cols, tag_size, tag_spacing);

  for (int tag_id = 0; tag_id < (tag_rows * tag_cols); tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
      const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi);
      const vec3_t r_CiFi = tf_point(T_CiC0, r_C0Fi);

      vec2_t z_hat;
      if (cam_geom->project(cam_res, params, r_CiFi, z_hat) == 0) {
        grid.add(tag_id, corner_idx, z_hat);
      }
    }
  }

  return grid;
}

void simulate_cameras(const timestamp_t &ts_start,
                      const timestamp_t &ts_end,
                      const ctraj_t &traj,
                      const calib_target_t &target,
                      const CamIdx2Geometry &cam_geoms,
                      const CamIdx2Parameters &cam_params,
                      const CamIdx2Extrinsics &cam_exts,
                      const real_t cam_rate,
                      const mat4_t &T_WF,
                      camera_data_t &cam_grids,
                      std::map<timestamp_t, mat4_t> &T_WC0_sim) {
  // Simulate camera measurements with AprilGrids that will be observed
  const timestamp_t dt = sec2ts(1.0 / cam_rate);
  timestamp_t ts_k = ts_start;

  while (ts_k <= ts_end) {
    // Calculate transform of fiducial (F) w.r.t. camera (C)
    const mat4_t T_WC0 = ctraj_get_pose(traj, ts_k);
    const mat4_t T_C0F = T_WC0.inverse() * T_WF;
    const mat4_t T_FC0 = T_C0F.inverse(); // NBV pose

    // Create an AprilGrid that represents what the camera would see if it
    // was positioned at T_C0F
    for (auto &[cam_idx, _] : cam_geoms) {
      UNUSED(_);
      const auto &geom = cam_geoms.at(cam_idx);
      const auto &cam = cam_params.at(cam_idx);
      const auto &ext = cam_exts.at(cam_idx);
      const auto &grid =
          simulate_aprilgrid(ts_k, target, T_FC0, geom, cam, ext);
      cam_grids[ts_k][cam_idx] = grid;
    }

    T_WC0_sim[ts_k] = T_WC0;
    ts_k += dt;
  }
}

void simulate_imu(const timestamp_t &ts_start,
                  const timestamp_t &ts_end,
                  const ctraj_t &traj,
                  const imu_params_t &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels) {
  const timestamp_t imu_dt = sec2ts(1.0 / imu_params.rate);
  timestamp_t ts_k = ts_start;
  std::default_random_engine rndeng;

  sim_imu_t sim_imu;
  sim_imu.rate = imu_params.rate;
  sim_imu.sigma_g_c = imu_params.sigma_g_c;
  sim_imu.sigma_a_c = imu_params.sigma_a_c;
  sim_imu.sigma_gw_c = imu_params.sigma_gw_c;
  sim_imu.sigma_aw_c = imu_params.sigma_aw_c;
  sim_imu.g = imu_params.g;

  while (ts_k <= ts_end) {
    const mat4_t T_WS_W = ctraj_get_pose(traj, ts_k);
    const vec3_t v_WS_W = ctraj_get_velocity(traj, ts_k);
    const vec3_t a_WS_W = ctraj_get_acceleration(traj, ts_k);
    const vec3_t w_WS_W = ctraj_get_angular_velocity(traj, ts_k);

    vec3_t a_WS_S{0.0, 0.0, 0.0};
    vec3_t w_WS_S{0.0, 0.0, 0.0};
    sim_imu_measurement(sim_imu,
                        rndeng,
                        ts_k,
                        T_WS_W,
                        w_WS_W,
                        a_WS_W,
                        a_WS_S,
                        w_WS_S);

    imu_time.push_back(ts_k);
    imu_accel.push_back(a_WS_S);
    imu_gyro.push_back(w_WS_S);
    imu_poses.push_back(T_WS_W);
    imu_vels.push_back(v_WS_W);

    ts_k += imu_dt;
  }
}

/** NBT EVALUATION ***********************************************************/

void nbt_create_timeline(const camera_data_t &cam_grids,
                         const timestamps_t &imu_ts,
                         const vec3s_t &imu_acc,
                         const vec3s_t &imu_gyr,
                         timeline_t &timeline) {
  // Add imu events
  for (size_t i = 0; i < imu_ts.size(); i++) {
    const timestamp_t ts = imu_ts[i];
    const vec3_t a_B = imu_acc[i];
    const vec3_t w_B = imu_gyr[i];
    timeline.add(ts, a_B, w_B);
  }

  // Add aprilgrids observed from all cameras
  for (const auto &[ts, data] : cam_grids) {
    for (const auto &[cam_idx, grid] : data) {
      if (grid.detected) {
        timeline.add(ts, cam_idx, grid);
      }
    }
  }
}

int nbt_eval(const calib_vi_t &calib, const ctraj_t &traj, real_t *info) {
  // // Simulate camera frames
  // const timestamp_t ts_start;
  // const timestamp_t ts_end;
  // const calib_target_t target;
  // const CamIdx2Geometry cam_geoms;
  // const CamIdx2Parameters cam_params;
  // const CamIdx2Extrinsics cam_exts;
  // const double cam_rate;
  // const mat4_t T_WF;
  // camera_data_t cam_grids;
  // std::map<timestamp_t, mat4_t> T_WC0_sim;
  // simulate_cameras(ts_start,
  //                  ts_end,
  //                  traj,
  //                  target,
  //                  cam_geoms,
  //                  cam_params,
  //                  cam_exts,
  //                  cam_rate,
  //                  T_WF,
  //                  cam_grids,
  //                  T_WC0_sim);
  //
  // // Simulate imu measurements
  // timestamps_t imu_time;
  // vec3s_t imu_accel;
  // vec3s_t imu_gyro;
  // mat4s_t imu_poses;
  // vec3s_t imu_vels;
  // simulate_imu(ts_start,
  //              ts_end,
  //              traj,
  //              imu_params,
  //              imu_time,
  //              imu_accel,
  //              imu_gyro,
  //              imu_poses,
  //              imu_vels);

  //   // Create timeline
  //   timeline_t timeline;
  //   nbt_create_timeline(imu_time,
  //                       imu_gyro,
  //                       imu_accel,
  //                       {grids0, grids1},
  //                       timeline);
  //
  //   // Setup calibration
  //   calib_vi_t calib;
  //   calib.batch_max_iter = 1;
  //   calib.enable_outlier_rejection = false;
  //   // -- Add imu
  //   calib.add_imu(imu_params);
  //   // -- Add cameras
  //   {
  //     const int res[2] = {cam0.resolution[0], cam0.resolution[1]};
  //     const auto proj_model = cam0.proj_model;
  //     const auto dist_model = cam0.dist_model;
  //     const auto proj = cam0.proj_params();
  //     const auto dist = cam0.dist_params();
  //     const auto fix = true;
  //     calib.add_camera(0, res, proj_model, dist_model, proj, dist, fix);
  //   }
  //   {
  //     const int res[2] = {cam1.resolution[0], cam1.resolution[1]};
  //     const auto proj_model = cam1.proj_model;
  //     const auto dist_model = cam1.dist_model;
  //     const auto proj = cam1.proj_params();
  //     const auto dist = cam1.dist_params();
  //     const auto fix = true;
  //     calib.add_camera(1, res, proj_model, dist_model, proj, dist, fix);
  //   }
  //   // -- Add extrinsics
  //   calib.add_cam_extrinsics(0, T_BC0, true);
  //   calib.add_cam_extrinsics(1, T_BC1, true);
  //   calib.add_imu_extrinsics(T_BS);
  //
  //   for (const auto &ts : timeline.timestamps) {
  //     const auto result = timeline.data.equal_range(ts);
  //     // -- Loop through events at timestamp ts since there could be two
  //     events
  //     // at the same timestamp
  //     for (auto it = result.first; it != result.second; it++) {
  //       const auto event = it->second;
  //       // Camera event
  //       if (event.type == APRILGRID_EVENT) {
  //         calib.add_measurement(event.camera_index, event.grid);
  //       }
  //
  //       // Imu event
  //       if (event.type == IMU_EVENT) {
  //         const auto ts = event.ts;
  //         const vec3_t w_m = event.w_m;
  //         const vec3_t a_m = event.a_m;
  //         calib.add_measurement(ts, a_m, w_m);
  //       }
  //     }
  //   }
  //   calib.solve(false);
  //   return calib.recover_calib_covar(calib_covar);

  return 0;
}

} // namespace yac
