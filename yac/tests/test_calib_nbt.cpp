#include "munit.hpp"
#include "calib_nbt.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_IMUCAM_DATA TEST_PATH "/test_data/calib/imu_april"
#define CALIB_CONFIG TEST_PATH "/test_data/calib/imu_april/config.yaml"

static void setup_cameras(std::map<int, camera_params_t> &cam_params) {
  const int img_w = 640;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  const double fx = pinhole_focal(img_w, 69.4);
  const double fy = pinhole_focal(img_h, 42.5);
  const double cx = 640.0 / 2.0;
  const double cy = 480.0 / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};

  // clang-format off
  camera_params_t cam0{0, cam_res, proj_model, dist_model, proj_params, dist_params};
  camera_params_t cam1{1, cam_res, proj_model, dist_model, proj_params, dist_params};
  // clang-format on

  cam_params[0] = cam0;
  cam_params[1] = cam1;
}

static void setup_imu_extrinsics(extrinsics_t &imu_exts) {
  // clang-format off
  mat4_t T_C0S;
  T_C0S <<
    0.014727, 0.999692, -0.019988, 0.066885,
    -0.999882, 0.014810, 0.004013, -0.017887,
    0.004308, 0.019927, 0.999792, -0.002524,
    0.000000, 0.000000, 0.000000, 1.000000;
  // clang-format on

  imu_exts = extrinsics_t{T_C0S};
}

static void setup_calib_target(const camera_params_t &cam,
                               calib_target_t &target,
                               mat4_t &T_FO,
                               mat4_t *T_WF = nullptr) {
  // Create calibration origin
  pinhole_radtan4_t cam_geom;
  target = calib_target_t{"aprilgrid", 6, 6, 0.088, 0.3};
  calib_target_origin(T_FO, target, &cam_geom, &cam);

  // Calibration target pose
  if (T_WF) {
    const vec3_t rpy = deg2rad(vec3_t{90.0, 0.0, -90.0});
    const mat3_t C_WF = euler321(rpy);
    *T_WF = tf(C_WF, zeros(3, 1));
  }
}

static void setup_test(std::map<int, camera_params_t> &cam_params,
                       extrinsics_t &imu_exts,
                       calib_target_t &target,
                       mat4_t &T_FO,
                       mat4_t &T_WF) {
  setup_cameras(cam_params);
  setup_imu_extrinsics(imu_exts);
  setup_calib_target(cam_params[0], target, T_FO, &T_WF);
}

static timeline_t setup_test_data() {
  // Load grid data
  timeline_t timeline;

  // -- Add cam0 grids
  std::vector<std::string> cam0_files;
  list_files(TEST_IMUCAM_DATA "/cam0", cam0_files);
  for (auto grid_path : cam0_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam0/") + grid_path;
    aprilgrid_t grid(grid_path);
    if (grid.detected == false) {
      const auto grid_fname = parse_fname(grid_path);
      const auto ts_str = grid_fname.substr(0, 19);
      grid.timestamp = std::stoull(ts_str);
      grid.tag_rows = 6;
      grid.tag_cols = 6;
      grid.tag_size = 0.088;
      grid.tag_spacing = 0.3;
    }
    timeline.add(grid.timestamp, 0, grid);
  }

  // -- Add cam1 grids
  std::vector<std::string> cam1_files;
  list_files(TEST_IMUCAM_DATA "/cam1", cam1_files);
  for (auto grid_path : cam1_files) {
    grid_path = std::string(TEST_IMUCAM_DATA "/cam1/") + grid_path;
    aprilgrid_t grid(grid_path);
    if (grid.detected == false) {
      const auto grid_fname = parse_fname(grid_path);
      const auto ts_str = grid_fname.substr(0, 19);
      grid.timestamp = std::stoull(ts_str);
      grid.tag_rows = 6;
      grid.tag_cols = 6;
      grid.tag_size = 0.088;
      grid.tag_spacing = 0.3;
    }
    timeline.add(grid.timestamp, 1, grid);
  }

  // -- Add imu events
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  load_imu_data(TEST_IMUCAM_DATA "/imu0.csv", imu_ts, imu_acc, imu_gyr);
  for (size_t k = 0; k < imu_ts.size(); k++) {
    timeline.add(imu_ts[k], imu_acc[k], imu_gyr[k]);
  }

  return timeline;
}

int test_nbt_trajs(const ctrajs_t &trajs,
                   const timestamp_t ts_start,
                   const timestamp_t ts_end,
                   const mat4_t &T_WF) {
  // Save trajectories
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  for (size_t index = 0; index < trajs.size(); index++) {
    const auto &traj = trajs[index];
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%ld.csv", index);
    printf("saving trajectory to [%s]\n", buffer);
    ctraj_save(traj, std::string{buffer});
  }

  // Save fiducial pose
  mat2csv("/tmp/nbt/fiducial_pose.csv", T_WF);

  // -- Simulate imu measurements
  std::default_random_engine rndeng;

  sim_imu_t imu;
  imu.rate = 400;
  imu.tau_a = 3600;
  imu.tau_g = 3600;
  imu.sigma_g_c = 0.0;
  imu.sigma_a_c = 0.0;
  imu.sigma_gw_c = 0;
  imu.sigma_aw_c = 0;
  imu.g = 9.81007;

  int index = 0;
  const timestamp_t dt = (1 / imu.rate) * 1e9;
  for (auto traj : trajs) {
    // Initialize position, velocity and attidue
    auto T_WS = ctraj_get_pose(traj, 0);
    vec3_t r_WS = tf_trans(T_WS);
    mat3_t C_WS = tf_rot(T_WS);
    vec3_t v_WS = ctraj_get_velocity(traj, 0);
    // print_matrix("T_WS", T_WS);
    // print_vector("v_WS", v_WS);

    timestamp_t ts_k = ts_start;
    while (ts_k <= ts_end) {
      const auto T_WS_W = ctraj_get_pose(traj, ts_k);
      const auto w_WS_W = ctraj_get_angular_velocity(traj, ts_k);
      const auto a_WS_W = ctraj_get_acceleration(traj, ts_k);
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

      // Propagate simulated IMU measurements
      const real_t dt_s = ts2sec(dt);
      const real_t dt_s_sq = dt_s * dt_s;
      const vec3_t g{0.0, 0.0, -imu.g};
      // -- Position at time k
      const vec3_t b_a = ones(3, 1) * imu.b_a;
      const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
      r_WS += v_WS * dt_s;
      r_WS += 0.5 * g * dt_s_sq;
      r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_s_sq;
      // -- velocity at time k
      v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_s + g * dt_s;
      // -- Attitude at time k
      const vec3_t b_g = ones(3, 1) * imu.b_g;
      const vec3_t n_g = ones(3, 1) * imu.sigma_g_c;
      C_WS = C_WS * lie::Exp((w_WS_S - b_g - n_g) * ts2sec(dt));
      // -- Normalize rotation
      quat_t q = quat_t{C_WS};
      q.normalize();
      C_WS = q.toRotationMatrix();

      // Update
      ts_k += dt;
    }
    index++;

    T_WS = tf(C_WS, r_WS);
    const vec3_t r_est = tf_trans(T_WS);
    const vec3_t r_gnd = tf_trans(ctraj_get_pose(traj, ts_end));
    const vec3_t rpy_est = quat2euler(tf_quat(T_WS));
    const vec3_t rpy_gnd = quat2euler(tf_quat(ctraj_get_pose(traj, ts_end)));
    printf("index: %d  ", index);
    printf("trans diff: %f\t", (r_est - r_gnd).norm());
    printf("rot   diff: %f\n", (rpy_est - rpy_gnd).norm());
  }

  return 0;
}

int test_nbt_orbit_trajs() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_orbit_trajs(ts_start,
                  ts_end,
                  target,
                  &cam_geom,
                  &cam_params[0],
                  &imu_exts,
                  T_WF,
                  T_FO,
                  trajs);
  test_nbt_trajs(trajs, ts_start, ts_end, T_WF);

  return 0;
}

int test_nbt_pan_trajs() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_pan_trajs(ts_start,
                ts_end,
                target,
                &cam_geom,
                &cam_params[0],
                &imu_exts,
                T_WF,
                T_FO,
                trajs);
  test_nbt_trajs(trajs, ts_start, ts_end, T_WF);

  return 0;
}

int test_nbt_figure8_trajs() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_figure8_trajs(ts_start,
                    ts_end,
                    target,
                    &cam_geom,
                    &cam_params[0],
                    &imu_exts,
                    T_WF,
                    T_FO,
                    trajs);
  test_nbt_trajs(trajs, ts_start, ts_end, T_WF);

  return 0;
}

int test_simulate_cameras() {
  // Setup
  calib_vi_t calib{CALIB_CONFIG};

  // Form circle trajectory
  const auto circle_r = 5.0;
  const auto circle_v = 2.0;
  const auto circle_dist = 2.0 * M_PI * circle_r;
  const auto time_taken = circle_dist / circle_v;
  const auto w = -2.0 * M_PI * (1.0 / time_taken);
  const auto dt = time_taken / 10;

  auto time = 0.0;
  auto theta = 0.0;
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t attitudes;

  while (time <= time_taken) {
    const auto x = circle_r * sin(theta);
    const auto y = circle_r * cos(theta);
    const auto z = 0.0;

    timestamps.push_back(sec2ts(time));
    positions.emplace_back(x, y, z);
    attitudes.emplace_back(1.0, 0.0, 0.0, 0.0);

    time += dt;
    theta += w * dt;
  }
  ctraj_t traj{timestamps.front(), timestamps.back(), positions, attitudes};

  // Simulate cameras
  const timestamp_t ts_start = sec2ts(0.0);
  const timestamp_t ts_end = sec2ts(5.0);
  const real_t cam_rate = 20.0;
  const mat4_t T_WF = I(4);
  camera_data_t cam_grids;
  std::map<timestamp_t, mat4_t> T_WC0_sim;
  simulate_cameras(ts_start,
                   ts_end,
                   traj,
                   calib.calib_target,
                   calib.cam_geoms,
                   calib.cam_params,
                   calib.cam_exts,
                   cam_rate,
                   T_WF,
                   cam_grids,
                   T_WC0_sim);

  return 0;
}

int test_simulate_imu() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Form circle trajectory
  const auto circle_r = 5.0;
  const auto circle_v = 2.0;
  const auto circle_dist = 2.0 * M_PI * circle_r;
  const auto time_taken = circle_dist / circle_v;
  const auto w = -2.0 * M_PI * (1.0 / time_taken);
  const auto dt = time_taken / 10;

  auto time = 0.0;
  auto theta = 0.0;
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t attitudes;

  while (time <= time_taken) {
    const auto x = circle_r * sin(theta);
    const auto y = circle_r * cos(theta);
    const auto z = 0.0;

    timestamps.push_back(sec2ts(time));
    positions.emplace_back(x, y, z);
    attitudes.emplace_back(1.0, 0.0, 0.0, 0.0);

    time += dt;
    theta += w * dt;
  }
  ctraj_t traj{timestamps.front(), timestamps.back(), positions, attitudes};

  // Simulate IMU measurements
  imu_params_t imu_params;
  imu_params.rate = 200.0;

  const timestamp_t ts_start = sec2ts(0.001);
  const timestamp_t ts_end = sec2ts(15.0);
  timestamps_t imu_time;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  mat4s_t imu_poses;
  vec3s_t imu_vels;

  simulate_imu(ts_start,
               ts_end,
               traj,
               imu_params,
               imu_time,
               imu_accel,
               imu_gyro,
               imu_poses,
               imu_vels);

  // Save imu measurements
  FILE *data_csv = fopen("/tmp/imu_data.csv", "w");

  fprintf(data_csv, "ts,");
  fprintf(data_csv, "ax,ay,az,");
  fprintf(data_csv, "wx,wy,wz,");
  fprintf(data_csv, "rx,ry,rz,");
  fprintf(data_csv, "qx,qy,qz,qw,");
  fprintf(data_csv, "vx,vy,vz\n");

  for (size_t k = 0; k < imu_time.size(); k++) {
    const auto ts = imu_time[k];
    const auto acc = imu_accel[k];
    const auto gyr = imu_gyro[k];
    const auto pose = imu_poses[k];
    const auto vel = imu_vels[k];
    const vec3_t pos = tf_trans(pose);
    const quat_t rot = tf_quat(pose);

    fprintf(data_csv, "%ld,", ts);
    fprintf(data_csv, "%f,%f,%f,", acc.x(), acc.y(), acc.z());
    fprintf(data_csv, "%f,%f,%f,", gyr.x(), gyr.y(), gyr.z());
    fprintf(data_csv, "%f,%f,%f,", pos.x(), pos.y(), pos.z());
    fprintf(data_csv, "%f,%f,%f,%f,", rot.x(), rot.y(), rot.z(), rot.w());
    fprintf(data_csv, "%f,%f,%f\n", vel.x(), vel.y(), vel.z());
  }
  fclose(data_csv);

  return 0;
}

int test_nbt_eval() {
  // Setup
  timeline_t timeline = setup_test_data();
  calib_vi_t calib{CALIB_CONFIG};
  calib.enable_marginalization = false;

  LOG_INFO("Adding data to problem ...");
  int nb_views = 0;
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);

        if (cam_idx == 0) {
          nb_views++;
        }
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }
  }

  LOG_INFO("Solve calibration problem");
  calib.solve();

  // Generate trajectories
  LOG_INFO("Generate NBT orbit trajectory");
  const int cam_idx = 0;
  ctrajs_t trajs;
  const timestamp_t ts_start = calib.calib_views.back()->ts + 1;
  const timestamp_t ts_end = ts_start + sec2ts(2.0);
  mat4_t T_FO;
  calib_target_origin(T_FO,
                      calib.calib_target,
                      calib.cam_geoms[cam_idx],
                      calib.cam_params[cam_idx]);

  nbt_orbit_trajs(ts_start,
                  ts_end,
                  calib.calib_target,
                  calib.cam_geoms[cam_idx],
                  calib.cam_params[cam_idx],
                  calib.imu_exts,
                  calib.get_fiducial_pose(),
                  T_FO,
                  trajs);

  // Evaluate NBT trajectories
  LOG_INFO("Evaluate NBT orbit trajectory");
  const int traj_idx = 0;
  matx_t calib_covar;
  if (nbt_eval(trajs[traj_idx], calib, calib_covar) != 0) {
    return -1;
  }

  return 0;
}

int test_nbt_find() {
  // Setup
  timeline_t timeline = setup_test_data();
  calib_vi_t calib{CALIB_CONFIG};
  calib.enable_marginalization = true;

  LOG_INFO("Adding data to problem ...");
  int nb_views = 0;
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);

        if (cam_idx == 0) {
          nb_views++;
        }
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }

    // Sliding window is initialized
    if (calib.marg_residual != nullptr) {
      break;
    }
  }

  LOG_INFO("Solve calibration problem");
  calib.solve();

  // Generate trajectories
  LOG_INFO("Generate NBT orbit trajectory");
  const int cam_idx = 0;
  ctrajs_t trajs;
  const timestamp_t ts_start = calib.calib_views.back()->ts + 1;
  const timestamp_t ts_end = ts_start + sec2ts(2.0);
  mat4_t T_FO;
  calib_target_origin(T_FO,
                      calib.calib_target,
                      calib.cam_geoms[cam_idx],
                      calib.cam_params[cam_idx]);

  nbt_orbit_trajs(ts_start,
                  ts_end,
                  calib.calib_target,
                  calib.cam_geoms[cam_idx],
                  calib.cam_params[cam_idx],
                  calib.imu_exts,
                  calib.get_fiducial_pose(),
                  T_FO,
                  trajs);

  // Evaluate NBT trajectories
  LOG_INFO("Evaluate NBT orbit trajectories");
  profiler_t prof;
  prof.start("NBT Find");
  const int best_index = nbt_find(trajs, calib, true);
  MU_CHECK(best_index != -1);
  prof.stop("NBT Find");
  prof.print("NBT Find");

  return 0;
}

int test_lissajous_trajs() {
  // Fiducial pose
  const vec3_t r_WF{0.0, 0.0, 0.0};
  const vec3_t rpy_WF{deg2rad(90.0), 0.0, deg2rad(-90.0)};
  const mat3_t C_WF = euler321(rpy_WF);
  const mat4_t T_WF = tf(C_WF, r_WF);

  // Calculate calib width and height
  const calib_target_t target;
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const mat4_t T_FO = tf(I(3), calib_center);

  // Lissajous parameters
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = sec2ts(3.0);
  const real_t R = std::max(calib_width, calib_height) * 1.0;
  const real_t A = calib_width * 0.5;
  const real_t B = calib_height * 0.5;
  const real_t T = ts2sec(ts_end - ts_start);

  // Lissajous
  lissajous_traj_t traj{"figure8", ts_start, T_WF, T_FO, R, A, B, T};
  traj.save("/tmp/traj.csv");

  return 0;
}

int test_nbt_lissajous_trajs() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  lissajous_trajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  nbt_lissajous_trajs(ts_start, ts_end, target, T_WF, trajs);

  // Save trajectories
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  for (size_t index = 0; index < trajs.size(); index++) {
    const auto &traj = trajs[index];
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%ld.csv", index);
    printf("saving trajectory to [%s]\n", buffer);
    traj.save(std::string{buffer});
  }

  // Save fiducial pose
  mat2csv("/tmp/nbt/fiducial_pose.csv", T_WF);

  // -- Simulate imu measurements
  std::default_random_engine rndeng;

  sim_imu_t imu;
  imu.rate = 400;
  imu.tau_a = 3600;
  imu.tau_g = 3600;
  imu.sigma_g_c = 0.0;
  imu.sigma_a_c = 0.0;
  imu.sigma_gw_c = 0;
  imu.sigma_aw_c = 0;
  imu.g = 9.81007;

  int index = 0;
  const timestamp_t dt = (1 / imu.rate) * 1e9;
  for (auto traj : trajs) {
    // Initialize position, velocity and attidue
    mat4_t T_WS = traj.get_pose(0);
    vec3_t r_WS = tf_trans(T_WS);
    mat3_t C_WS = tf_rot(T_WS);
    vec3_t v_WS = traj.get_velocity(0);

    timestamp_t ts_k = 0;
    while (ts_k <= ts_end) {
      // Get sensor acceleration and angular velocity in world frame
      const auto a_WS_W = traj.get_acceleration(ts_k);
      const auto w_WS_W = traj.get_angular_velocity(ts_k);

      // Transform acceleration and angular velocity to body frame
      const vec3_t g{0.0, 0.0, imu.g};
      const vec3_t a_WS_S = C_WS.transpose() * (a_WS_W + g);
      const vec3_t w_WS_S = C_WS.transpose() * w_WS_W;
      // const auto T_WS_W = traj.get_pose(ts_k);
      // const vec3_t a_WS_S = tf_rot(T_WS_W).transpose() * (a_WS_W + g);
      // const vec3_t w_WS_S = tf_rot(T_WS_W).transpose() * w_WS_W;

      // Propagate simulated ideal IMU measurements
      const real_t dt_s = ts2sec(dt);
      const real_t dt_s_sq = dt_s * dt_s;
      const vec3_t g_W{0.0, 0.0, -imu.g};
      // -- Position at time k
      const vec3_t b_a = ones(3, 1) * imu.b_a;
      const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
      r_WS += v_WS * dt_s;
      r_WS += 0.5 * g_W * dt_s_sq;
      r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_s_sq;
      // -- velocity at time k
      v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_s + g_W * dt_s;
      // -- Attitude at time k
      const vec3_t b_g = ones(3, 1) * imu.b_g;
      const vec3_t n_g = ones(3, 1) * imu.sigma_g_c;
      C_WS = C_WS * lie::Exp((w_WS_S - b_g - n_g) * ts2sec(dt));
      // -- Normalize rotation
      quat_t q = quat_t{C_WS};
      q.normalize();
      C_WS = q.toRotationMatrix();

      // // Calculate angle difference using axis-angle equation
      // const mat3_t dC = tf_rot(T_WS_W).transpose() * C_WS;
      // const vec3_t ypr = quat2euler(quat_t{dC});
      // print_vector("ypr", rad2deg(ypr));

      // Update
      ts_k += dt;
    }
    index++;

    T_WS = tf(C_WS, r_WS);
    const vec3_t r_est = tf_trans(T_WS);
    const vec3_t r_gnd = tf_trans(traj.get_pose(ts_end));
    const vec3_t rpy_est = quat2euler(tf_quat(T_WS));
    const vec3_t rpy_gnd = quat2euler(tf_quat(traj.get_pose(ts_end)));
    printf("index: %d  ", index);
    printf("trans diff: %f\t", (r_est - r_gnd).norm());
    printf("rot   diff: %f\n", (rpy_est - rpy_gnd).norm());
  }

  return 0;
}

int test_simulate_cameras_lissajous() {
  // Setup calibrator
  const calib_target_t target;
  calib_vi_t calib{target};
  // -- Add IMU
  imu_params_t imu_params;
  imu_params.rate = 200;
  const mat4_t T_BS = I(4);
  calib.add_imu(imu_params, T_BS);
  // -- Add camera
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t fx = pinhole_focal(cam_res[0], 90.0);
  const real_t fy = pinhole_focal(cam_res[0], 90.0);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  calib.add_camera(cam_idx,
                   cam_res,
                   proj_model,
                   dist_model,
                   proj_params,
                   dist_params);

  // Fiducial pose
  const vec3_t r_WF{0.0, 0.0, 0.0};
  const vec3_t rpy_WF{deg2rad(90.0), 0.0, deg2rad(-90.0)};
  const mat3_t C_WF = euler321(rpy_WF);
  const mat4_t T_WF = tf(C_WF, r_WF);

  // Generate trajectories
  lissajous_trajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  nbt_lissajous_trajs(ts_start, ts_end, target, T_WF, trajs);

  // Simulate cameras
  const auto &traj = trajs[0];
  const real_t cam_rate = 20.0;
  camera_data_t cam_grids;
  std::map<timestamp_t, mat4_t> T_WC0;
  simulate_cameras(ts_start,
                   ts_end,
                   traj,
                   calib.calib_target,
                   calib.cam_geoms,
                   calib.cam_params,
                   calib.cam_exts,
                   cam_rate,
                   T_WF,
                   cam_grids,
                   T_WC0);

  // Save data
  remove_dir("/tmp/sim_cam");
  dir_create("/tmp/sim_cam");
  for (const auto &[ts, cam_data] : cam_grids) {
    for (const auto &[cam_idx, grid] : cam_data) {
      grid.save("/tmp/sim_cam/" + std::to_string(ts) + ".csv");
    }
  }

  return 0;
}

int test_simulate_imu_lissajous() {
  // Setup test
  std::map<int, camera_params_t> cam_params;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cam_params, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  lissajous_trajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  nbt_lissajous_trajs(ts_start, ts_end, target, T_WF, trajs);

  // Save trajectory
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  const auto &traj = trajs[0];
  char buffer[1024];
  snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%d.csv", 0);
  printf("saving trajectory to [%s]\n", buffer);
  traj.save(std::string{buffer});

  // Save fiducial pose
  mat2csv("/tmp/nbt/fiducial_pose.csv", T_WF);

  // -- Simulate imu measurements
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_g_c = 0.002;
  imu_params.sigma_a_c = 0.0001;
  imu_params.sigma_gw_c = 0.003;
  imu_params.sigma_aw_c = 0.00001;

  timestamps_t imu_time;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  mat4s_t imu_poses;
  vec3s_t imu_vels;
  simulate_imu(ts_start,
               ts_end,
               trajs[0],
               imu_params,
               imu_time,
               imu_accel,
               imu_gyro,
               imu_poses,
               imu_vels);

  // Save imu measurements
  FILE *data_csv = fopen("/tmp/imu_data.csv", "w");
  fprintf(data_csv, "ts,");
  fprintf(data_csv, "ax,ay,az,");
  fprintf(data_csv, "wx,wy,wz,");
  fprintf(data_csv, "rx,ry,rz,");
  fprintf(data_csv, "qx,qy,qz,qw,");
  fprintf(data_csv, "vx,vy,vz\n");

  imu_data_t imu_buf;
  for (size_t k = 0; k < imu_time.size(); k++) {
    const auto ts = imu_time[k];
    const auto acc = imu_accel[k];
    const auto gyr = imu_gyro[k];
    const auto pose = imu_poses[k];
    const auto vel = imu_vels[k];
    const vec3_t pos = tf_trans(pose);
    const quat_t rot = tf_quat(pose);
    imu_buf.add(ts, acc, gyr);

    fprintf(data_csv, "%ld,", ts);
    fprintf(data_csv, "%f,%f,%f,", acc.x(), acc.y(), acc.z());
    fprintf(data_csv, "%f,%f,%f,", gyr.x(), gyr.y(), gyr.z());
    fprintf(data_csv, "%f,%f,%f,", pos.x(), pos.y(), pos.z());
    fprintf(data_csv, "%f,%f,%f,%f,", rot.x(), rot.y(), rot.z(), rot.w());
    fprintf(data_csv, "%f,%f,%f\n", vel.x(), vel.y(), vel.z());
  }
  fclose(data_csv);

  const timestamp_t ts_i = imu_time.front();
  const timestamp_t ts_j = imu_time.back();
  const vec3_t v_i = imu_vels.front();
  const vec3_t v_j = imu_vels.back();
  const vec3_t ba_i{0.0, 0.0, 0.0};
  const vec3_t bg_i{0.0, 0.0, 0.0};
  const vec3_t ba_j{0.0, 0.0, 0.0};
  const vec3_t bg_j{0.0, 0.0, 0.0};
  pose_t pose_i{ts_i, imu_poses.front()};
  pose_t pose_j{ts_j, imu_poses.back()};
  sb_params_t sb_i{ts_i, v_i, ba_i, bg_i};
  sb_params_t sb_j{ts_j, v_j, ba_j, bg_j};
  imu_residual_t residual(imu_params, imu_buf, &pose_i, &sb_i, &pose_j, &sb_j);
  residual.eval();

  printf("imu_buf size: %ld\n", imu_buf.size());
  print_vector("r", residual.residuals);

  return 0;
}

static void test_setup_lissajous(calib_vi_t &calib, imu_params_t &imu_params) {
  // Add IMU
  const mat4_t T_BS = I(4);
  calib.add_imu(imu_params, T_BS);

  // Add camera
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t fx = pinhole_focal(cam_res[0], 90.0);
  const real_t fy = pinhole_focal(cam_res[0], 90.0);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  calib.add_camera(cam_idx,
                   cam_res,
                   proj_model,
                   dist_model,
                   proj_params,
                   dist_params);

  // Fiducial pose
  const vec3_t r_WF{0.0, 0.0, 0.0};
  const vec3_t rpy_WF{deg2rad(90.0), 0.0, deg2rad(-90.0)};
  const mat3_t C_WF = euler321(rpy_WF);
  const mat4_t T_WF = tf(C_WF, r_WF);

  // Generate trajectories
  LOG_INFO("Generate NBT trajectories");
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = ts_start + sec2ts(3.0);
  lissajous_trajs_t trajs;
  nbt_lissajous_trajs(ts_start, ts_end, calib.calib_target, T_WF, trajs);
  const auto &traj = trajs[0];

  // Simulate imu measurements
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  mat4s_t imu_poses;
  vec3s_t imu_vels;
  simulate_imu(ts_start,
               ts_end,
               traj,
               calib.imu_params,
               imu_ts,
               imu_acc,
               imu_gyr,
               imu_poses,
               imu_vels);

  // Simulate camera frames
  const auto cam_rate = 20.0;
  calib_target_t calib_target;
  camera_data_t cam_grids;
  std::map<timestamp_t, mat4_t> cam_poses;
  simulate_cameras(ts_start,
                   ts_end,
                   traj,
                   calib_target,
                   calib.cam_geoms,
                   calib.cam_params,
                   calib.cam_exts,
                   cam_rate,
                   T_WF,
                   cam_grids,
                   cam_poses);

  // Add NBT data into calibrator
  timeline_t timeline;
  nbt_create_timeline(cam_grids, imu_ts, imu_acc, imu_gyr, timeline);
  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib.add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const vec3_t &acc = imu_event->acc;
        const vec3_t &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }
  }
  calib.solve();

  // Save estimates
  FILE *pose_est_csv = fopen("/tmp/poses_est.csv", "w");
  fprintf(pose_est_csv, "#ts,rx,ry,rz,qx,qy,qz,qw\n");
  for (const auto &calib_view : calib.calib_views) {
    const auto ts = calib_view->ts;
    const mat4_t &pose = calib_view->pose.tf();
    const vec3_t r_WS = tf_trans(pose);
    const quat_t q_WS = tf_quat(pose);
    fprintf(pose_est_csv, "%ld,", ts);
    fprintf(pose_est_csv, "%f,%f,%f,", r_WS.x(), r_WS.y(), r_WS.z());
    fprintf(pose_est_csv,
            "%f,%f,%f,%f",
            q_WS.x(),
            q_WS.y(),
            q_WS.z(),
            q_WS.w());
    fprintf(pose_est_csv, "\n");
  }
  fclose(pose_est_csv);

  // Save ground-truth
  FILE *pose_gnd_csv = fopen("/tmp/poses_gnd.csv", "w");
  fprintf(pose_gnd_csv, "#ts,rx,ry,rz,qx,qy,qz,qw\n");
  for (size_t i = 0; i < imu_poses.size(); i++) {
    const auto ts = imu_ts[i];
    const auto &pose = imu_poses[i];
    const vec3_t r_WS = tf_trans(pose);
    const quat_t q_WS = tf_quat(pose);
    fprintf(pose_gnd_csv, "%ld,", ts);
    fprintf(pose_gnd_csv, "%f,%f,%f,", r_WS.x(), r_WS.y(), r_WS.z());
    fprintf(pose_gnd_csv,
            "%f,%f,%f,%f",
            q_WS.x(),
            q_WS.y(),
            q_WS.z(),
            q_WS.w());
    fprintf(pose_gnd_csv, "\n");
  }
  fclose(pose_gnd_csv);
}

static void schurs_complement(const matx_t &H,
                              const size_t m, // Marginalize
                              const size_t r, // Remain
                              matx_t &H_marg) {
  assert(m > 0 && r > 0);

  // Setup
  const long local_size = m + r;
  H_marg = zeros(local_size, local_size);

  // Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // entry by its reciprocal, leaving the zeros in place, and transposing
  // the resulting matrix.
  // clang-format off
  matx_t Hmm = H.block(r, r, m, m);
  Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  // const auto eigvals = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array(), 0);
  const auto eigvals_inv = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array().inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
  if (fabs(inv_check) > 1e-4) {
    LOG_WARN("Inverse identity check: %f", inv_check);
    LOG_WARN("This is bad ... Usually means marg_residual_t is bad!");
  }
  // clang-format on

  // Calculate Schur's complement
  // H = [Hrr, Hrm,
  //      Hmr, Hmm]
  const matx_t Hrr = H.block(0, 0, r, r);
  const matx_t Hrm = H.block(0, r, r, m);
  const matx_t Hmr = H.block(r, 0, m, r);

  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
}

int test_nbt_eval_lissajous() {
  // Imu params
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_g_c = 0.002;
  imu_params.sigma_a_c = 0.0001;
  imu_params.sigma_gw_c = 0.003;
  imu_params.sigma_aw_c = 0.00001;

  // Calibrator
  calib_target_t target;
  target.tag_size = 0.0375;

  calib_vi_t calib{target};
  test_setup_lissajous(calib, imu_params);

  // const real_t cam_rate = calib.get_camera_rate();
  // const real_t imu_rate = calib.get_imu_rate();
  // if ((imu_rate - calib.imu_params.rate) > 50) {
  //   LOG_ERROR("(imu_rate - imu_params.rate) > 50");
  //   LOG_ERROR("imu_rate: %f", imu_rate);
  //   LOG_ERROR("imu_params.rate: %f", calib.imu_params.rate);
  //   FATAL("Measured IMU rate is different to configured imu rate!");
  // }

  // Generate trajectories
  LOG_INFO("Generate NBT trajectories");
  const int cam_idx = 0;
  const timestamp_t ts_start = sec2ts(3.01);
  const timestamp_t ts_end = ts_start + sec2ts(3.0);
  const mat4_t T_WF = calib.get_fiducial_pose();
  lissajous_trajs_t trajs;
  nbt_lissajous_trajs(ts_start, ts_end, calib.calib_target, T_WF, trajs);

  // Calculate initial information
  matx_t H;
  if (calib.recover_calib_info(H) != 0) {
    return -1;
  }

  // Evaluate trajectory
  matx_t H_nbt;
  const nbt_data_t nbt_data{calib};
  const int retval = nbt_eval(trajs[0], nbt_data, H, H_nbt);
  const real_t info = -1.0 * log(H_nbt.inverse().determinant()) / log(2.0);
  printf("info: %f\n", info);

  return 0;
}

int test_nbt_find_lissajous() {
  // Imu params
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_g_c = 0.002;
  imu_params.sigma_a_c = 0.0001;
  imu_params.sigma_gw_c = 0.003;
  imu_params.sigma_aw_c = 0.00001;

  // Calibrator
  calib_target_t target;
  target.tag_size = 0.0375;

  calib_vi_t calib{target};
  // calib.enable_marginalization = true;
  test_setup_lissajous(calib, imu_params);

  // Generate trajectories
  LOG_INFO("Generate NBT trajectories");
  const int cam_idx = 0;
  const timestamp_t ts_start = sec2ts(3.01);
  const timestamp_t ts_end = ts_start + sec2ts(3.0);
  const mat4_t T_WF = calib.get_fiducial_pose();
  lissajous_trajs_t trajs;
  nbt_lissajous_trajs(ts_start, ts_end, calib.calib_target, T_WF, trajs);

  // Calculate initial information
  matx_t H;
  if (calib.recover_calib_info(H) != 0) {
    return -1;
  }

  // Evaluate NBT trajectories
  LOG_INFO("Evaluate NBT trajectories");
  profiler_t prof;
  prof.start("NBT Find");

  matx_t H_nbt;
  real_t info_k = 0.0;
  real_t info_kp1 = 0.0;
  const nbt_data_t nbt_data{calib};
  const int best_index = nbt_find(trajs, nbt_data, H, true, &info_k, &info_kp1);
  printf("info_k: %f\n", info_k);
  printf("info_kp1: %f\n", info_kp1);

  MU_CHECK(best_index != -1);
  prof.stop("NBT Find");
  prof.print("NBT Find");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_nbt_orbit_trajs);
  MU_ADD_TEST(test_nbt_pan_trajs);
  MU_ADD_TEST(test_nbt_figure8_trajs);
  MU_ADD_TEST(test_simulate_cameras);
  MU_ADD_TEST(test_simulate_imu);
  MU_ADD_TEST(test_nbt_eval);
  MU_ADD_TEST(test_nbt_find);

  MU_ADD_TEST(test_lissajous_trajs);
  MU_ADD_TEST(test_nbt_lissajous_trajs);
  MU_ADD_TEST(test_simulate_cameras_lissajous);
  MU_ADD_TEST(test_simulate_imu_lissajous);
  MU_ADD_TEST(test_nbt_eval_lissajous);
  MU_ADD_TEST(test_nbt_find_lissajous);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
