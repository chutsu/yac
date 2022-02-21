#include "munit.hpp"
#include "calib_nbt.hpp"

namespace yac {

static void setup_cameras(std::map<int, camera_params_t> &cams) {
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

  cams[0] = cam0;
  cams[1] = cam1;
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
  T_FO = calib_target_origin(target, &cam_geom, &cam);

  // Calibration target pose
  if (T_WF) {
    const vec3_t rpy = deg2rad(vec3_t{90.0, 0.0, -90.0});
    const mat3_t C_WF = euler321(rpy);
    *T_WF = tf(C_WF, zeros(3, 1));
  }
}

static void setup_test(std::map<int, camera_params_t> &cameras,
                       extrinsics_t &imu_exts,
                       calib_target_t &target,
                       mat4_t &T_FO,
                       mat4_t &T_WF) {
  setup_cameras(cameras);
  setup_imu_extrinsics(imu_exts);
  setup_calib_target(cameras[0], target, T_FO, &T_WF);
}

int test_nbt_trajs(const ctrajs_t &trajs,
                   const timestamp_t ts_start,
                   const timestamp_t ts_end) {
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

    timestamp_t ts_k = 0;
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
  std::map<int, camera_params_t> cameras;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cameras, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_orbit_trajs(ts_start,
                  ts_end,
                  target,
                  &cam_geom,
                  &cameras[0],
                  &imu_exts,
                  T_WF,
                  T_FO,
                  trajs);
  test_nbt_trajs(trajs, ts_start, ts_end);

  return 0;
}

int test_nbt_pan_trajs() {
  // Setup test
  std::map<int, camera_params_t> cameras;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cameras, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_pan_trajs(ts_start,
                ts_end,
                target,
                &cam_geom,
                &cameras[0],
                &imu_exts,
                T_WF,
                T_FO,
                trajs);
  test_nbt_trajs(trajs, ts_start, ts_end);

  return 0;
}

int test_nbt_figure8_trajs() {
  // Setup test
  std::map<int, camera_params_t> cameras;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cameras, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  pinhole_radtan4_t cam_geom;
  nbt_figure8_trajs(ts_start,
                    ts_end,
                    target,
                    &cam_geom,
                    &cameras[0],
                    &imu_exts,
                    T_WF,
                    T_FO,
                    trajs);
  test_nbt_trajs(trajs, ts_start, ts_end);

  return 0;
}

int test_simulate_cameras() {
  // Cameras
  std::map<int, camera_params_t> cameras;
  setup_cameras(cameras);
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const mat4_t T_BC0 = tf(I(3), zeros(3, 1));
  const mat4_t T_BC1 = T_BC0 * T_C0C1;

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  return 0;
}

int test_simulate_imu() {
  // Setup test
  std::map<int, camera_params_t> cameras;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cameras, imu_exts, target, T_FO, T_WF);

  // Form circle trajectory
  const auto imu_rate = 200.0;
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
  ctraj_t traj{timestamps, positions, attitudes};

  // Simulate IMU measurements
  imu_params_t imu_params;
  imu_params.rate = 200.0;

  const timestamp_t ts_start = sec2ts(1.0);
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
  // Setup test
  std::map<int, camera_params_t> cameras;
  extrinsics_t imu_exts;
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_test(cameras, imu_exts, target, T_FO, T_WF);

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 2e9;
  pinhole_radtan4_t cam_geom;
  nbt_orbit_trajs(ts_start,
                  ts_end,
                  target,
                  &cam_geom,
                  &cameras[0],
                  &imu_exts,
                  T_WF,
                  T_FO,
                  trajs);

  // Calibrator
  calib_target_t calib_target;
  calib_vi_t calib{calib_target};

  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    matx_t calib_covar;
    if (nbt_eval(trajs[traj_idx], calib, calib_covar) != 0) {
      return -1;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_nbt_orbit_trajs);
  MU_ADD_TEST(test_nbt_pan_trajs);
  MU_ADD_TEST(test_nbt_figure8_trajs);
  MU_ADD_TEST(test_simulate_cameras);
  MU_ADD_TEST(test_simulate_imu);
  // MU_ADD_TEST(test_nbt_eval);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
