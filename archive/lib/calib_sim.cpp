#include "calib_sim.hpp"

namespace yac {

//////////////////
// VIO SIM DATA //
//////////////////

static vec3s_t create_3d_features(const real_t *x_bounds,
                                  const real_t *y_bounds,
                                  const real_t *z_bounds,
                                  const size_t nb_features) {
  vec3s_t features;
  for (size_t i = 0; i < nb_features; i++) {
    real_t x = randf(x_bounds[0], x_bounds[1]);
    real_t y = randf(y_bounds[0], y_bounds[1]);
    real_t z = randf(z_bounds[0], z_bounds[1]);
    features.emplace_back(x, y, z);
  }
  return features;
}

static vec3s_t create_3d_features_perimeter(const vec3_t &origin,
                                            const vec3_t &dim,
                                            const size_t nb_features) {
  // Dimension of the outskirt
  const real_t w = dim(0);
  const real_t l = dim(1);
  const real_t h = dim(2);

  // Features per side
  size_t nb_fps = nb_features / 4.0;
  vec3s_t features;

  // Features in the east side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) + l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the north side
  {
    const real_t x_bounds[2] = {origin(0) + w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the west side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) - l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the south side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) - w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  return features;
}

vio_sim_data_t::vio_sim_data_t(const real_t circle_r) {
  const real_t circle_dist = 2 * M_PI * circle_r;
  const real_t time_taken = circle_dist / sensor_velocity;
  const real_t f = 1.0 / time_taken;
  const real_t w = -2.0 * M_PI * f;
  const real_t w2 = w * w;

  // Create features
  const vec3_t origin{0.0, 0.0, 0.0};
  const vec3_t dim{5.0, 5.0, 5.0};
  const size_t nb_features = 100;
  features = create_3d_features_perimeter(origin, dim, nb_features);

  // Create camera
  const int cam_idx = 0;
  const int res[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[1], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const camera_params_t cam_params{cam_idx,
                                   res,
                                   proj_model,
                                   dist_model,
                                   proj_params,
                                   dist_params};
  pinhole_radtan4_t pinhole_radtan4;

  // Simulate camera
  {
    const int cam_id = 0;
    const real_t cam_rate = 30.0;
    const real_t dt = 1.0 / cam_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = 0.0;

    while (t <= t_end) {
      const real_t x = circle_r * cos(theta);
      const real_t y = circle_r * sin(theta);
      const real_t z = 0.0;
      const vec3_t r_WC{x, y, z};
      const vec3_t rpy{deg2rad(-90.0), 0.0, yaw};
      const mat3_t C_WC = euler321(rpy);
      const mat4_t T_WC = tf(C_WC, r_WC);

      cam_ts.push_back(t * 1e9);

      cam_pos_gnd.push_back(r_WC);
      cam_rot_gnd.emplace_back(C_WC);
      cam_poses_gnd.push_back(tf(C_WC, r_WC));

      const mat4_t T_WC_n = add_noise(T_WC, 0.1, 1.0);
      cam_pos.push_back(tf_trans(T_WC_n));
      cam_rot.push_back(tf_quat(T_WC_n));
      cam_poses.push_back(T_WC_n);

      // Check which features in the scene is observable by camera
      const mat4_t T_CW = T_WC.inverse();
      size_t feature_idx = 0;
      std::vector<size_t> frame_obs;
      vec2s_t frame_kps;

      for (const vec3_t &p_W : features) {
        vec2_t z;
        const vec3_t p_C = tf_point(T_CW, p_W);
        if (pinhole_radtan4.project(cam_params.resolution,
                                    cam_params.param,
                                    p_C,
                                    z) == 0) {
          frame_obs.push_back(feature_idx);
          frame_kps.push_back(z);
        }
        feature_idx++;
      }
      observations.push_back(frame_obs);
      keypoints.push_back(frame_kps);
      _add(cam_id, t * 1e9, frame_kps, frame_obs);

      // Update
      t += dt;
      theta += w * dt; // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;   // -ve to yaw the camera CW fashion
    }
  }

  // Simulate imu
  {
    const int imu_id = 0;
    const real_t dt = 1.0 / imu_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = deg2rad(90.0);

    std::default_random_engine rndeng;
    sim_imu_t imu;
    imu.rate = imu_rate;
    imu.tau_a = 3600;
    imu.tau_g = 3600;
    // imu.sigma_g_c = 0.0;
    // imu.sigma_a_c = 0.0;
    // imu.sigma_gw_c = 0.0;
    // imu.sigma_aw_c = 0.0;
    imu.sigma_g_c = 0.005;
    imu.sigma_a_c = 0.025;
    imu.sigma_gw_c = 1e-05;
    imu.sigma_aw_c = 0.001;
    imu.g = 9.81;

    while (t <= t_end) {
      // Form sensor pose
      // -- Orientation
      const vec3_t rpy{0.0, 0.0, wrapPi(yaw)};
      const mat3_t C_WS_W = euler321(rpy);
      // -- Position
      const real_t rx = circle_r * cos(theta);
      const real_t ry = circle_r * sin(theta);
      const real_t rz = 0.0;
      const vec3_t r_WS_W{rx, ry, rz};
      // -- Pose
      const mat4_t T_WS_W = tf(C_WS_W, r_WS_W);

      // Form sensor velocity
      const real_t vx = -circle_r * w * sin(theta);
      const real_t vy = circle_r * w * cos(theta);
      const real_t vz = 0.0;
      imu_vel.emplace_back(vx, vy, vz);

      // Form sensor angular velocity
      const timestamp_t ts_k = t * 1e9;
      const vec3_t w_WS_W{0.0, 0.0, w};

      // Form Sensor acceleration
      const real_t ax = -circle_r * w2 * cos(theta);
      const real_t ay = -circle_r * w2 * sin(theta);
      const real_t az = 0.0;
      const vec3_t a_WS_W{ax, ay, az};

      // Simulate imu measurements
      vec3_t a_WS_S;
      vec3_t w_WS_S;
      sim_imu_measurement(imu,
                          rndeng,
                          ts_k,
                          T_WS_W,
                          w_WS_W,
                          a_WS_W,
                          a_WS_S,
                          w_WS_S);
      imu_ts.push_back(ts_k);
      imu_acc.push_back(a_WS_S);
      imu_gyr.push_back(w_WS_S);
      imu_pos.push_back(tf_trans(T_WS_W));
      imu_poses.push_back(T_WS_W);
      imu_rot.push_back(tf_quat(T_WS_W));
      _add(imu_id, ts_k, a_WS_S, w_WS_S);

      // Update
      t += dt;
      theta += w * dt; // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;   // -ve to go from 180 to -180 in CW fashion
    }
  }
}

void vio_sim_data_t::_add(const int sensor_id,
                          const timestamp_t &ts,
                          const vec3_t &accel,
                          const vec3_t &gyro) {
  sim_event_t event(sensor_id, ts, accel, gyro);
  timeline.insert({ts, event});
}

void vio_sim_data_t::_add(const int sensor_id,
                          const timestamp_t &ts,
                          const vec2s_t &keypoints,
                          const std::vector<size_t> &feature_idxs) {
  sim_event_t event{sensor_id, ts, keypoints, feature_idxs};
  timeline.insert({event.ts, event});
}

void vio_sim_data_t::save(const std::string &dir) {
  if (dir_create(dir) == -1) {
    FATAL("Failed to create dir [%s]!", dir.c_str());
  }

  auto features_csv_path = dir + "/features.csv";
  auto cam0_obs_csv_path = dir + "/cam0_observations.csv";
  auto cam0_kps_csv_path = dir + "/cam0_keypoints.csv";
  auto cam0_pose_csv_path = dir + "/cam0_pose.csv";
  auto imu_csv_path = dir + "/imu.csv";
  auto imu_pose_csv_path = dir + "/imu_pose.csv";
  auto imu_vel_csv_path = dir + "/imu_vel.csv";

  FILE *features_csv = fopen(features_csv_path.c_str(), "w");
  FILE *cam0_obs_csv = fopen(cam0_obs_csv_path.c_str(), "w");
  FILE *cam0_kps_csv = fopen(cam0_kps_csv_path.c_str(), "w");
  FILE *imu_csv = fopen(imu_csv_path.c_str(), "w");
  FILE *imu_pose_csv = fopen(imu_pose_csv_path.c_str(), "w");
  FILE *imu_vel_csv = fopen(imu_vel_csv_path.c_str(), "w");

  // Save features
  for (const auto &f : features) {
    fprintf(features_csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fclose(features_csv);

  // Save observations
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_obs_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_obs_csv, "%zu,", observations[k].size());

    for (size_t i = 0; i < observations[k].size(); i++) {
      const auto obs = observations[k][i];
      fprintf(cam0_obs_csv, "%zu", obs);
      if ((i + 1) != observations[k].size()) {
        fprintf(cam0_obs_csv, ",");
      } else {
        fprintf(cam0_obs_csv, "\n");
      }
    }
  }
  fclose(cam0_obs_csv);

  // Save keypoints
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_kps_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_kps_csv, "%zu,", keypoints[k].size());

    for (size_t i = 0; i < keypoints[k].size(); i++) {
      const auto kp = keypoints[k][i];
      fprintf(cam0_kps_csv, "%f,%f", kp(0), kp(1));
      if ((i + 1) != keypoints[k].size()) {
        fprintf(cam0_kps_csv, ",");
      } else {
        fprintf(cam0_kps_csv, "\n");
      }
    }
  }
  fclose(cam0_kps_csv);

  // Save camera poses
  mat4s_t cam_poses;
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto q = cam_rot_gnd[k];
    const auto r = cam_pos_gnd[k];
    cam_poses.push_back(tf(q, r));
  }
  save_poses(cam0_pose_csv_path, cam_ts, cam_poses);

  // Save imu data
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto a = imu_acc[k];
    const auto w = imu_gyr[k];
    fprintf(imu_csv, "%" PRIu64 ",", ts);
    fprintf(imu_csv, "%f,%f,%f,", a(0), a(1), a(2));
    fprintf(imu_csv, "%f,%f,%f\n", w(0), w(1), w(2));
  }
  fclose(imu_csv);

  // Save imu poses
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto q = imu_rot[k];
    const auto r = imu_pos[k];
    fprintf(imu_pose_csv, "%" PRIu64 ",", ts);
    fprintf(imu_pose_csv, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(imu_pose_csv, "%f,%f,%f\n", r(0), r(1), r(2));
  }
  fclose(imu_pose_csv);

  // Save imu velocities
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto vel = imu_vel[k];
    fprintf(imu_vel_csv, "%" PRIu64 ",", ts);
    fprintf(imu_vel_csv, "%f,%f,%f\n", vel(0), vel(1), vel(2));
  }
  fclose(imu_vel_csv);
}

////////////////////
// CALIB SIM DATA //
////////////////////

calib_sim_data_t::calib_sim_data_t(const real_t circle_r) {
  const real_t circle_dist = 2 * M_PI * circle_r;
  const real_t time_taken = circle_dist / sensor_velocity;
  const real_t f = 1.0 / time_taken;
  const real_t w = -2.0 * M_PI * f;
  const real_t w2 = w * w;

  // Create features
  const vec3_t origin{0.0, 0.0, 0.0};
  const vec3_t dim{5.0, 5.0, 5.0};
  const size_t nb_features = 100;
  features = create_3d_features_perimeter(origin, dim, nb_features);

  // Create camera
  const int cam_idx = 0;
  const int res[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[1], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const camera_params_t cam_params{cam_idx,
                                   res,
                                   proj_model,
                                   dist_model,
                                   proj_params,
                                   dist_params};
  pinhole_radtan4_t pinhole_radtan4;

  // Simulate camera
  {
    const int cam_id = 0;
    const real_t cam_rate = 30.0;
    const real_t dt = 1.0 / cam_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = 0.0;

    while (t <= t_end) {
      const real_t x = circle_r * cos(theta);
      const real_t y = circle_r * sin(theta);
      const real_t z = 0.0;
      const vec3_t r_WC{x, y, z};
      const vec3_t rpy{deg2rad(-90.0), 0.0, yaw};
      const mat3_t C_WC = euler321(rpy);
      const mat4_t T_WC = tf(C_WC, r_WC);

      cam_ts.push_back(t * 1e9);

      cam_pos_gnd.push_back(r_WC);
      cam_rot_gnd.emplace_back(C_WC);
      cam_poses_gnd.push_back(tf(C_WC, r_WC));

      const mat4_t T_WC_n = add_noise(T_WC, 0.1, 1.0);
      cam_pos.push_back(tf_trans(T_WC_n));
      cam_rot.push_back(tf_quat(T_WC_n));
      cam_poses.push_back(T_WC_n);

      // Check which features in the scene is observable by camera
      const mat4_t T_CW = T_WC.inverse();
      size_t feature_idx = 0;
      std::vector<size_t> frame_obs;
      vec2s_t frame_kps;

      for (const vec3_t &p_W : features) {
        vec2_t z;
        const vec3_t p_C = tf_point(T_CW, p_W);
        if (pinhole_radtan4.project(cam_params.resolution,
                                    cam_params.param,
                                    p_C,
                                    z) == 0) {
          frame_obs.push_back(feature_idx);
          frame_kps.push_back(z);
        }
        feature_idx++;
      }
      observations.push_back(frame_obs);
      keypoints.push_back(frame_kps);
      _add(cam_id, t * 1e9, frame_kps, frame_obs);

      // Update
      t += dt;
      theta += w * dt; // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;   // -ve to yaw the camera CW fashion
    }
  }

  // Simulate imu
  {
    const int imu_id = 0;
    const real_t dt = 1.0 / imu_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = deg2rad(90.0);

    std::default_random_engine rndeng;
    sim_imu_t imu;
    imu.rate = imu_rate;
    imu.tau_a = 3600;
    imu.tau_g = 3600;
    // imu.sigma_g_c = 0.0;
    // imu.sigma_a_c = 0.0;
    // imu.sigma_gw_c = 0.0;
    // imu.sigma_aw_c = 0.0;
    imu.sigma_g_c = 0.005;
    imu.sigma_a_c = 0.025;
    imu.sigma_gw_c = 1e-05;
    imu.sigma_aw_c = 0.001;
    imu.g = 9.81;

    while (t <= t_end) {
      // Form sensor pose
      // -- Orientation
      const vec3_t rpy{0.0, 0.0, wrapPi(yaw)};
      const mat3_t C_WS_W = euler321(rpy);
      // -- Position
      const real_t rx = circle_r * cos(theta);
      const real_t ry = circle_r * sin(theta);
      const real_t rz = 0.0;
      const vec3_t r_WS_W{rx, ry, rz};
      // -- Pose
      const mat4_t T_WS_W = tf(C_WS_W, r_WS_W);

      // Form sensor velocity
      const real_t vx = -circle_r * w * sin(theta);
      const real_t vy = circle_r * w * cos(theta);
      const real_t vz = 0.0;
      imu_vel.emplace_back(vx, vy, vz);

      // Form sensor angular velocity
      const timestamp_t ts_k = t * 1e9;
      const vec3_t w_WS_W{0.0, 0.0, w};

      // Form Sensor acceleration
      const real_t ax = -circle_r * w2 * cos(theta);
      const real_t ay = -circle_r * w2 * sin(theta);
      const real_t az = 0.0;
      const vec3_t a_WS_W{ax, ay, az};

      // Simulate imu measurements
      vec3_t a_WS_S;
      vec3_t w_WS_S;
      sim_imu_measurement(imu,
                          rndeng,
                          ts_k,
                          T_WS_W,
                          w_WS_W,
                          a_WS_W,
                          a_WS_S,
                          w_WS_S);
      imu_ts.push_back(ts_k);
      imu_acc.push_back(a_WS_S);
      imu_gyr.push_back(w_WS_S);
      imu_pos.push_back(tf_trans(T_WS_W));
      imu_poses.push_back(T_WS_W);
      imu_rot.push_back(tf_quat(T_WS_W));
      _add(imu_id, ts_k, a_WS_S, w_WS_S);

      // Update
      t += dt;
      theta += w * dt; // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;   // -ve to go from 180 to -180 in CW fashion
    }
  }
}

void calib_sim_data_t::_add(const int sensor_id,
                            const timestamp_t &ts,
                            const vec3_t &accel,
                            const vec3_t &gyro) {
  sim_event_t event(sensor_id, ts, accel, gyro);
  timeline.insert({ts, event});
}

void calib_sim_data_t::_add(const int sensor_id,
                            const timestamp_t &ts,
                            const vec2s_t &keypoints,
                            const std::vector<size_t> &feature_idxs) {
  sim_event_t event{sensor_id, ts, keypoints, feature_idxs};
  timeline.insert({event.ts, event});
}

void calib_sim_data_t::save(const std::string &dir) {
  if (dir_create(dir) == -1) {
    FATAL("Failed to create dir [%s]!", dir.c_str());
  }

  auto features_csv_path = dir + "/features.csv";
  auto cam0_obs_csv_path = dir + "/cam0_observations.csv";
  auto cam0_kps_csv_path = dir + "/cam0_keypoints.csv";
  auto cam0_pose_csv_path = dir + "/cam0_pose.csv";
  auto imu_csv_path = dir + "/imu.csv";
  auto imu_pose_csv_path = dir + "/imu_pose.csv";
  auto imu_vel_csv_path = dir + "/imu_vel.csv";

  FILE *features_csv = fopen(features_csv_path.c_str(), "w");
  FILE *cam0_obs_csv = fopen(cam0_obs_csv_path.c_str(), "w");
  FILE *cam0_kps_csv = fopen(cam0_kps_csv_path.c_str(), "w");
  FILE *imu_csv = fopen(imu_csv_path.c_str(), "w");
  FILE *imu_pose_csv = fopen(imu_pose_csv_path.c_str(), "w");
  FILE *imu_vel_csv = fopen(imu_vel_csv_path.c_str(), "w");

  // Save features
  for (const auto &f : features) {
    fprintf(features_csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fclose(features_csv);

  // Save observations
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_obs_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_obs_csv, "%zu,", observations[k].size());

    for (size_t i = 0; i < observations[k].size(); i++) {
      const auto obs = observations[k][i];
      fprintf(cam0_obs_csv, "%zu", obs);
      if ((i + 1) != observations[k].size()) {
        fprintf(cam0_obs_csv, ",");
      } else {
        fprintf(cam0_obs_csv, "\n");
      }
    }
  }
  fclose(cam0_obs_csv);

  // Save keypoints
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_kps_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_kps_csv, "%zu,", keypoints[k].size());

    for (size_t i = 0; i < keypoints[k].size(); i++) {
      const auto kp = keypoints[k][i];
      fprintf(cam0_kps_csv, "%f,%f", kp(0), kp(1));
      if ((i + 1) != keypoints[k].size()) {
        fprintf(cam0_kps_csv, ",");
      } else {
        fprintf(cam0_kps_csv, "\n");
      }
    }
  }
  fclose(cam0_kps_csv);

  // Save camera poses
  mat4s_t cam_poses;
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto q = cam_rot_gnd[k];
    const auto r = cam_pos_gnd[k];
    cam_poses.push_back(tf(q, r));
  }
  save_poses(cam0_pose_csv_path, cam_ts, cam_poses);

  // Save imu data
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto a = imu_acc[k];
    const auto w = imu_gyr[k];
    fprintf(imu_csv, "%" PRIu64 ",", ts);
    fprintf(imu_csv, "%f,%f,%f,", a(0), a(1), a(2));
    fprintf(imu_csv, "%f,%f,%f\n", w(0), w(1), w(2));
  }
  fclose(imu_csv);

  // Save imu poses
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto q = imu_rot[k];
    const auto r = imu_pos[k];
    fprintf(imu_pose_csv, "%" PRIu64 ",", ts);
    fprintf(imu_pose_csv, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(imu_pose_csv, "%f,%f,%f\n", r(0), r(1), r(2));
  }
  fclose(imu_pose_csv);

  // Save imu velocities
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto vel = imu_vel[k];
    fprintf(imu_vel_csv, "%" PRIu64 ",", ts);
    fprintf(imu_vel_csv, "%f,%f,%f\n", vel(0), vel(1), vel(2));
  }
  fclose(imu_vel_csv);
}

} //  namespace yac
