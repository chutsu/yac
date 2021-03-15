#include <memory>
#include <fstream>
#include <sys/time.h>
#include <thread>
#include <future>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "autocal/core/core.hpp"
#include "autocal/calib/aprilgrid.hpp"
#include "autocal/calib/calib_data.hpp"
#include "autocal/ceres/nbt.hpp"
#include "autocal/cv/CameraGeometry.hpp"

#include "EuRoCTestSetup.hpp"

#define TEST_DATA "/data/euroc_mav/imu_april"
#define TEST_CONFIG "test_data/calib/test_config.yaml"

using namespace autocal;
using namespace autocal::ceres;

static std::vector<PinholeRadtan> setup_cameras() {
  // Setup cameras
  // const double cam_rate = 20;
  const int img_w = 640;
  const int img_h = 480;
  // -- cam0
  const double cam0_fx = pinhole_focal(img_w, 69.4);
  const double cam0_fy = pinhole_focal(img_h, 42.5);
  const double cam0_cx = 640.0 / 2.0;
  const double cam0_cy = 480.0 / 2.0;
  // const camera::RadialTangentialDistortion cam0_dist{0.01, 0.001, 0.001, 0.001};
  const camera::RadialTangentialDistortion cam0_dist{0.0, 0.0, 0.0, 0.0};
  PinholeRadtan cam0(img_w, img_h, cam0_fx, cam0_fy, cam0_cx, cam0_cy, cam0_dist);
  // -- cam1
  const double cam1_fx = pinhole_focal(img_w, 69.4);
  const double cam1_fy = pinhole_focal(img_h, 42.5);
  const double cam1_cx = 640.0 / 2.0;
  const double cam1_cy = 480.0 / 2.0;
  // const camera::RadialTangentialDistortion cam1_dist{0.01, 0.001, 0.001, 0.001};
  const camera::RadialTangentialDistortion cam1_dist{0.0, 0.0, 0.0, 0.0};
  PinholeRadtan cam1(img_w, img_h, cam1_fx, cam1_fy, cam1_cx, cam1_cy, cam1_dist);

  return {cam0, cam1};
}

static void setup_calib_target(const PinholeRadtan &cam,
                               calib_target_t &target,
                               mat4_t &T_FO,
                               mat4_t *T_WF=nullptr) {
  // Create calibration origin
  calib_target_load(target, TEST_CONFIG, "calib_target");
  T_FO = calib_target_origin(target, cam);

  // Calibration target pose
  if (T_WF) {
    const vec3_t rpy = deg2rad(vec3_t{90.0, 0.0, -90.0});
    const mat3_t C_WF = euler321(rpy);
    *T_WF = tf(C_WF, zeros(3, 1));
  }
}

TEST(NBT, calib_target_origin) {
  auto cameras = setup_cameras();

  calib_target_t target;
  calib_target_load(target, TEST_CONFIG, "calib_target");
  const mat4_t T_FO = calib_target_origin(target, cameras[0]);

  std::cout << T_FO << std::endl;
}

TEST(NBT, generate_orbit_trajectories) {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  generate_orbit_trajectories(target,
                              cameras[0], cameras[1],
                              T_C0C1,
                              T_WF, T_FO,
                              ts_start, ts_end,
                              trajs);

  // Save trajectories
  int index = 0;
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  for (const auto &traj : trajs) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%d.csv", index);
    printf("saving trajectory to [%s]\n", buffer);
    ctraj_save(traj, std::string{buffer});
    index++;
  }
}

TEST(NBT, generate_pan_trajectories) {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  generate_pan_trajectories(target,
                            cameras[0],
                            cameras[1],
                            T_C0C1,
                            T_WF,
                            T_FO,
                            ts_start, ts_end,
                            trajs);

  // Save trajectories
  int index = 0;
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  for (const auto &traj : trajs) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%d.csv", index);
    printf("saving trajectory to [%s]\n", buffer);
    ctraj_save(traj, std::string{buffer});
    index++;
  }
}

static real_t ts_normalize(const ctraj_t &ctraj, const timestamp_t ts) {
  const real_t ts_s_k = ts2sec(ts);
  const real_t ts_s_start = ctraj.ts_s_start;
  const real_t ts_s_end = ctraj.ts_s_end;
  return (ts_s_k - ts_s_start) / (ts_s_end - ts_s_start);
}

TEST(NBT, generate_roll_trajectories) {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  generate_pan_trajectories(target, cameras[0], cameras[1],
                            T_C0C1,
                            T_WF, T_FO,
                            ts_start, ts_end,
                            trajs);

  const auto ts = 0;
  const real_t u = ts_normalize(trajs[0], ts);
  const real_t scale = (1 / trajs[0].ts_s_gap);
  std::cout << trajs[0].pos_spline.derivatives(u, 1) << std::endl;
  std::cout << trajs[0].pos_spline.derivatives(u, 1).col(1) << std::endl;
  // trajs[0].pos_spline.derivatives(u, 1).col(1) * scale;

  // Save trajectories
  int index = 0;
  for (size_t i = 0; i < trajs.size(); i++) {
    remove_dir("/tmp/nbt/traj_" + std::to_string(i));
    dir_create("/tmp/nbt/traj_" + std::to_string(i));
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj_%ld/traj_%ld.csv", i, i);
    printf("saving trajectory to [%s]\n", buffer);
    ctraj_save(trajs[i], std::string{buffer});
    index++;
  }
}

TEST(NBT, generate_figure8_trajectory) {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  generate_figure8_trajectory(target, cameras[0], cameras[1],
                              T_C0C1,
                              T_WF, T_FO,
                              ts_start, ts_end,
                              trajs);

  // Save trajectories
  int index = 0;
  remove_dir("/tmp/nbt/traj");
  dir_create("/tmp/nbt/traj");
  for (const auto &traj : trajs) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "/tmp/nbt/traj/traj_%d.csv", index);
    printf("saving trajectory to [%s]\n", buffer);
    ctraj_save(traj, std::string{buffer});
    index++;
  }
}

TEST(NBT, simulate_cameras) {
  // Cameras
  auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const mat4_t T_SC0 = tf(I(3), zeros(3, 1));
  const mat4_t T_SC1 = T_SC0 * T_C0C1;

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  ctrajs_t trajs;
  // generate_orbit_trajectories(target, cameras[0], cameras[1], T_C0C1,
  //                             T_WF, T_FO, ts_start, ts_end, trajs);
  generate_pan_trajectories(target, cameras[0], cameras[1], T_C0C1,
                            T_WF, T_FO, ts_start, ts_end, trajs);
  EXPECT_TRUE(trajs.size() > 0);

  // Save trajectories
  int index = 0;
  std::string base_path = "/tmp/nbt";
  remove_dir(base_path);
  dir_create(base_path);

  for (const auto &traj : trajs) {
    const auto traj_dir = base_path + "/traj_" + std::to_string(index);
    dir_create(traj_dir);
    const auto save_path = traj_dir + "/traj.csv";
    printf("saving trajectory to [%s]\n", save_path.c_str());
    ctraj_save(traj, save_path);
    index++;
  }

  // clang-format off
  // const auto traj_idx = 0;
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    const double cam_rate = 15.0;
    aprilgrids_t grids0;
    aprilgrids_t grids1;
    mat4s_t T_WC0;
    simulate_cameras(trajs[traj_idx], target,
                    cameras[0], cameras[1], cam_rate,
                    T_WF, T_SC0, T_SC1,
                    ts_start, ts_end,
                    grids0, grids1,
                    T_WC0);
    EXPECT_EQ(T_WC0.size(), grids0.size());
    EXPECT_EQ(T_WC0.size(), grids1.size());
    // clang-format on

    // const auto nb_tags = target.tag_rows * target.tag_cols;
    // const auto nb_points = nb_tags * 4;
    // for (const auto &grid : grids0) {
    //   EXPECT_EQ(grid.nb_detections, nb_tags);
    //   EXPECT_EQ(grid.ids.size(), nb_tags);
    //   EXPECT_EQ(grid.keypoints.size(), nb_points);
    //   EXPECT_EQ(grid.points_CF.size(), nb_points);
    // }
    // for (const auto &grid : grids1) {
    //   EXPECT_EQ(grid.nb_detections, nb_tags);
    //   EXPECT_EQ(grid.ids.size(), nb_tags);
    //   EXPECT_EQ(grid.keypoints.size(), nb_points);
    //   EXPECT_EQ(grid.points_CF.size(), nb_points);
    // }

    // Save data
    for (const auto &grid : grids0) {
      const auto ts = grid.timestamp;
      auto save_path = base_path;
      save_path += "/traj_" + std::to_string(traj_idx);
      save_path += "/cam0/" + std::to_string(ts) + ".csv";
      // printf("saving grid to [%s]\n", save_path.c_str());
      aprilgrid_save(grid, save_path);
    }

    for (const auto &grid : grids1) {
      const auto ts = grid.timestamp;
      auto save_path = base_path;
      save_path += "/traj_" + std::to_string(traj_idx);
      save_path += "/cam1/" + std::to_string(ts) + ".csv";
      // printf("saving grid to [%s]\n", save_path.c_str());
      aprilgrid_save(grid, save_path);
    }

    // EXPECT_EQ(grids0.size(), 5 * nbt.cam0_.rate - 1);
    // EXPECT_EQ(grids1.size(), 5 * nbt.cam1_.rate - 1);
  }
}

TEST(NBT, simulate_imu) {
  // Cameras
  auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  ctrajs_t trajs;
  generate_orbit_trajectories(target, cameras[0], cameras[1], T_C0C1,
                              T_WF, T_FO, ts_start, ts_end, trajs);
  generate_pan_trajectories(target, cameras[0], cameras[1], T_C0C1,
                            T_WF, T_FO, ts_start, ts_end, trajs);
  EXPECT_TRUE(trajs.size() > 0);

  // Simulate IMU
  autocal::ImuParameters imu_params;
  autocal::ceres::load_imu_params(config_t{TEST_CONFIG}, 0, imu_params);
  imu_params.sigma_g_c  = 0.0;
  imu_params.sigma_bg = 0.0;
  imu_params.sigma_a_c  = 0.0;
  imu_params.sigma_ba  = 0.0;
  imu_params.sigma_gw_c = 0.0;
  imu_params.sigma_aw_c = 0.0;
  // imu_params.tau = 0.0;

  // const auto rpy_SC = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  const auto rpy_SC = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const auto C_SC = euler321(rpy_SC);
  mat4_t T_SC = tf(C_SC, zeros(3, 1));

  // const int traj_idx = 0;
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    timestamps_t imu_time;
    vec3s_t imu_accel;
    vec3s_t imu_gyro;
    mat4s_t imu_poses;
    vec3s_t imu_vels;
    simulate_imu(trajs[traj_idx], ts_start, ts_end, T_SC,
                imu_params, imu_time, imu_accel, imu_gyro,
                imu_poses, imu_vels);

    autocal::ImuMeasurementDeque imu_meas;
    for (size_t i = 0; i < imu_accel.size(); i++) {
      Time ts{ts2sec(imu_time[i])};
      vec3_t gyr = imu_gyro[i];
      vec3_t acc = imu_accel[i];
      imu_meas.emplace_back(ts, ImuSensorReadings{gyr, acc});
    }

    autocal::Transformation T_WS{imu_poses[0]};
    autocal::SpeedAndBias sb;
    sb << imu_vels.front(), 0, 0, 0, 0, 0, 0;

    Time t_start{ts2sec(ts_start)};
    Time t_end{ts2sec(ts_end)};
    ImuError::propagation(imu_meas, imu_params, T_WS, sb, t_start, t_end);

    print_matrix("pose at the end           ", imu_poses.back());
    print_matrix("propagated pose at the end", T_WS.T());

    // Save data
    auto base_path = "/tmp/nbt/traj_" + std::to_string(traj_idx) + "/imu";
    remove_dir(base_path);
    dir_create(base_path);
    save_data(base_path + "/accel.csv", imu_time, imu_accel);
    save_data(base_path + "/gyro.csv", imu_time, imu_gyro);
    save_poses(base_path + "/poses.csv", imu_time, imu_poses);

    EXPECT_EQ(imu_time.size(), 5 * imu_params.rate + 1);
    EXPECT_EQ(imu_accel.size(), 5 * imu_params.rate + 1);
    EXPECT_EQ(imu_gyro.size(), 5 * imu_params.rate + 1);
  }
}

TEST(NBT, test_grid) {
  auto cameras = setup_cameras();

  test_grid_t grid{cameras[0]};

  ASSERT_EQ(grid.grid_rows, 5);
  ASSERT_EQ(grid.grid_cols, 5);
  ASSERT_FLOAT_EQ(grid.grid_depth, 1.5);
  ASSERT_EQ(grid.nb_points, 25);
  ASSERT_EQ(grid.object_points.size(), 25);
  ASSERT_EQ(grid.keypoints.size(), 25);

  for (size_t i = 0; i < grid.nb_points; i++) {
    const vec2_t kp = grid.keypoints[i];
    const vec3_t p_C = grid.object_points[i];

    vec2_t image_point;
    const auto retval = cameras[0].project(p_C, &image_point);
    ASSERT_EQ(retval, CameraBase::ProjectionStatus::Successful);
    ASSERT_TRUE((kp - image_point).norm() < 1e-4);
  }
}

TEST(NBT, eval_traj) {
  // Cameras
  const double cam_rate = 15.0;
  const auto cams = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.01, 0.0, 0.0});
  const vec3_t rpy_SC = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const mat3_t C_SC = euler321(rpy_SC);
  const mat4_t T_SC0 = tf(C_SC, zeros(3, 1));
  const mat4_t T_SC1 = T_SC0 * T_C0C1;

  // IMU
  autocal::ImuParameters imu_params;
  autocal::ceres::load_imu_params(config_t{TEST_CONFIG}, 0, imu_params, false);

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cams[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  ctrajs_t trajs;
  generate_orbit_trajectories(target, cams[0], cams[1], T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  // generate_pan_trajectories(target, cams[0], cams[1], T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  // generate_roll_trajectories(target, cams[0], cams[1], T_C0C1, T_WF, T_FO, ts_start, ts_end, trajs);
  EXPECT_TRUE(trajs.size() > 0);

  // Evaluate trajectory
	profiler_t profiler;
	profiler.start("eval_traj");

  // #pragma omp parallel for
  // for (size_t i = 0; i < trajs.size(); i++) {
  {
    size_t i = 0;
    // printf("i: %ld\n", i);
    // clang-format off
    matx_t calib_info;
    int retval = eval_traj(i, trajs[i], target, ts_start, ts_end,
                           imu_params, cams[0], cams[1], cam_rate,
                           T_WF, T_SC0, T_SC1, calib_info, true);
    EXPECT_EQ(retval, 0);
    mat2csv("/tmp/calib_info-" + std::to_string(i) + ".csv", calib_info);
  }

	profiler.stop("eval_traj", true);
}

TEST(NBT, eval_reproj_uncertainty) {
  // Cameras
  const double cam_rate = 15.0;
  const auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.0, 0.0, 0.0});
  const vec3_t rpy_SC = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const mat3_t C_SC = euler321(rpy_SC);
  const mat4_t T_SC0 = tf(C_SC, zeros(3, 1));
  const mat4_t T_SC1 = T_SC0 * T_C0C1;

  // IMU
  autocal::ImuParameters imu_params;
  autocal::ceres::load_imu_params(config_t{TEST_CONFIG}, 0, imu_params, false);

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 3e9;
  ctrajs_t trajs;
  generate_orbit_trajectories(target, cameras[0], cameras[1], T_C0C1,
                              T_WF, T_FO, ts_start, ts_end, trajs);
  EXPECT_TRUE(trajs.size() > 0);

  const int traj_idx = 0;
  matx_t calib_info;
  int retval = eval_traj(traj_idx, trajs[traj_idx], target, ts_start, ts_end,
                         imu_params, cameras[0], cameras[1], cam_rate,
                         T_WF, T_SC0, T_SC1, calib_info, true);
  ASSERT_EQ(retval, 0);

  const Eigen::SelfAdjointEigenSolver<matx_t> eig(calib_info);
  const auto nb_cols = calib_info.cols();
  const auto nb_rows = calib_info.rows();
	const auto eps = std::numeric_limits<double>::epsilon();
  const auto tol = eps * nb_cols * eig.eigenvalues().array().maxCoeff();

  int rank = 0;
  for (int i = 0; i < nb_rows; ++i) {
    if (eig.eigenvalues()[i] > tol) {
      rank++;
    }
  }

  ASSERT_EQ(rank, nb_rows);

  // // Evaluate reprojection uncertainty
  // auto score = eval_reproj_uncertainty(0, calib_covar, cameras[0], T_SC0);
  // printf("score: %f\n", score);
}

TEST(NBT, nbt_compute) {
  // Cameras
  const double cam_rate = 15.0;
  const auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.0, 0.0, 0.0});
  const vec3_t rpy_SC = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const mat3_t C_SC = euler321(rpy_SC);
  const mat4_t T_SC0 = tf(C_SC, zeros(3, 1));
  const mat4_t T_SC1 = T_SC0 * T_C0C1;

  // IMU
  autocal::ImuParameters imu_params;
  autocal::ceres::load_imu_params(config_t{TEST_CONFIG}, 0, imu_params, false);

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Compute NBTs
  ctrajs_t trajs;
  std::vector<matx_t> calib_covars;
  nbt_compute(
    target, imu_params,
		cameras[0], cameras[1], cam_rate,
    T_WF, T_SC0, T_SC1,
    trajs, calib_covars,
    true
  );
}

TEST(NBT, nbt_find) {
  // Cameras
  const double cam_rate = 15.0;
  const auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const auto rpy_SC = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  const auto C_SC = euler321(rpy_SC);
  const auto T_SC0 = tf(C_SC, zeros(3, 1));
  const auto T_SC1 = T_SC0 * T_C0C1;

  // IMU
  autocal::ImuParameters imu_params;
  autocal::ceres::load_imu_params(config_t{TEST_CONFIG}, 0, imu_params, false);

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Load test data
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  // Setup Estimator
  autocal::ceres::Estimator est;
  est.opt_config_.save_estimates = true;
  est.opt_config_.window_size = 10;
  // -- Add calib target
  est.addCalibTarget(target);
  // -- Add IMU
  est.addImu(imu_params);
  est.addImuTimeDelay(0.0, true);
  // -- Add Camera
  est.addCamera(0, cameras[0], true);
  est.addCamera(1, cameras[1], true);
  // -- Add sensor camera extrinsics
  est.addSensorCameraExtrinsics(0, T_SC0, true);
  est.addSensorCameraExtrinsics(1, T_SC1, true);

  // Compute NBTs
  ctrajs_t trajs;
  std::vector<matx_t> calib_infos;
  nbt_compute(
    target, imu_params,
    cameras[0], cameras[1], cam_rate,
    T_WF, T_SC0, T_SC1,
    trajs, calib_infos
  );

  std::vector<int> traj_selected;

  std::map<int, std::string> cam_image_paths;
  for (const auto &ts : test_data.timeline.timestamps) {
    printf(".");
    const auto result = test_data.timeline.data.equal_range(ts);
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;

      // Camera event
      if (event.type == CAMERA_EVENT) {
        const auto ts = event.ts;
        const int cam_idx = event.camera_index;
        cam_image_paths[cam_idx] = event.image_path;

        if (cam_image_paths.size() == est.nb_cams()) {
          std::vector<cv::Mat> cam_images;
          for (size_t i = 0; i < est.nb_cams(); i++) {
            cam_images.push_back(cv::imread(cam_image_paths[i]));
          }
          est.addMeasurement(ts, cam_images);
          cam_image_paths.clear();
        }

        if (cam_idx == 0 && est.marg_error_ /*  && est.frame_index_ % 10 == 0 */) {
          // Compute NBTs
          // ctrajs_t trajs;
          // std::vector<matx_t> calib_covars;
          // nbt_compute(
          //   target, imu_params,
          //   cameras[0], cameras[1], cam_rate,
          //   T_WF, T_SC0, T_SC1,
          //   trajs, calib_covars
          // );

          // matx_t calib_covar;
          // est.recoverCalibCovariance(calib_covar, true);
          // if (full_rank(calib_covar) == false) {
          //   LOG_ERROR("calib covar is not full rank!");
          // }
          // printf("trace(calib_covar): %f\n", calib_covar.trace());

          // Estimate current information
          std::vector<ImuErrorPtr> imu_errs;
          std::vector<SpeedAndBiasErrorPtr> sb_errs;
          std::vector<CalibReprojErrorPtr> reproj_errs;
          for (const auto &pair : est.imu_errors_) imu_errs.push_back(pair.second);
          for (const auto &pair : est.sb_errors_) sb_errs.push_back(pair.second);
          for (const auto &pair : est.reproj_errors_) reproj_errs.push_back(pair.second);

          matx_t H;
          vecx_t b;
          form_hessian(imu_errs,
                       sb_errs,
                       reproj_errs,
                       est.marg_error_.get(),
                       nullptr,
                       nullptr,
                       nullptr, {}, {}, H, &b);

          const size_t r = 28;
          const size_t m = H.rows() - r;
          schurs_complement(H, b, m, r);
          matx_t calib_info = H;

          const auto cam0 = est.generateCamera(0);
          const auto cam1 = est.generateCamera(1);
          const auto T_SC0 = est.getSensorCameraPoseEstimate(0);
          const auto T_SC1 = est.getSensorCameraPoseEstimate(1);
          int retval = nbt_find(est.frame_index_,
                                cam0, cam1,
                                T_SC0, T_SC1,
                                calib_info,
                                trajs,
                                calib_infos);
          traj_selected.push_back(retval);

          // {
          //   matx_t H;
          //   vecx_t b;
          //   est.formHessian(H, &b);
          //   const matx_t covar = H.ldlt().solve(I(H.rows()));
          //   printf("rank: %ld\n", rank(H));
          //   printf("rows: %ld\n", H.rows());
          //   ASSERT_TRUE(rank(H) == H.rows());
          //
          //   mat2csv("/tmp/H.csv", H);
          //   mat2csv("/tmp/covar.csv", covar);
          //
          //   // const size_t nb_cams = est.nb_cams();
          //   // auto calib_param_size = (nb_cams * 6) + (nb_cams * 8);
          //   // matx_t calib_covar;
          //   // calib_covar.resize(calib_param_size, calib_param_size);
          //   // calib_covar = covar.bottomRightCorner(calib_param_size, calib_param_size);
          //   // mat2csv("/tmp/calib_covar.csv", calib_covar);
          //   //
          //   // const size_t r = (nb_cams * 6) + (nb_cams * 8);
          //   // const size_t m = H.rows() - r;
          //   // const bool precond = false;
          //   // const bool debug = false;
          //   // schurs_complement(H, b, m, r, precond, debug);
          //   //
          //   // mat2csv("/tmp/H_marg.csv", H);
          //   //
          //   // {
          //   //   const matx_t calib_covar = H.ldlt().solve(I(H.rows()));
          //   //   mat2csv("/tmp/calib_covar_marg.csv", covar);
          //   // }
          //   //
          //   // printf("rank: %ld\n", rank(H));
          //   // printf("rows: %ld\n", H.rows());
          //   // ASSERT_TRUE(rank(H) == H.rows());
          // }

          // goto find_nbt;
        }
      }

      // Imu event
      if (event.type == IMU_EVENT) {
        auto ts = Time(ns2sec(event.ts));
        auto w_m = event.w_m;
        auto a_m = event.a_m;
        est.addMeasurement(ts, w_m, a_m);
      }
    }
  }

  // {
  //   matx_t calib_covar;
  //   est.recoverCalibCovariance(calib_covar, true);
  //   if (full_rank(calib_covar) == false) {
  //     LOG_ERROR("calib covar is not full rank!");
  //   }
  //   printf("trace(calib_covar): %f\n", calib_covar.trace());
  //
  //   const auto cam0 = est.generateCamera(0);
  //   const auto cam1 = est.generateCamera(1);
  //   const auto T_SC0 = est.getSensorCameraPoseEstimate(0);
  //   const auto T_SC1 = est.getSensorCameraPoseEstimate(1);
  //   int retval = nbt_find(est.frame_index_,
  //                         cam0, cam1,
  //                         T_SC0, T_SC1,
  //                         calib_covar,
  //                         trajs,
  //                         calib_covars);
  //   traj_selected.push_back(retval);
  // }
  //
  // // Save NBT calib covars
  // for (size_t i = 0; i < calib_covars.size(); i++) {
  //   const matx_t covar_nbt = calib_covars[i];
  //   const std::string base_path = est.save_dir_ + "/nbt_covars";
  //   const std::string save_path = base_path + "/nbt_calib_covar-" + std::to_string(i + 1) + ".csv";
  //   mat2csv(save_path, covar_nbt);
  // }
  //
	// // Save NBT informations
  // for (size_t i = 0; i < calib_covars.size(); i++) {
  //   const matx_t nbt_covar = calib_covars[i];
	// 	int pinv_rank = 0;
	// 	const matx_t nbt_H = pinv_symm(nbt_covar, &pinv_rank);
	// 	if (pinv_rank != nbt_H.rows()) {
	// 		FATAL("nbt_H not full rank!");
	// 	}
  //
  //   const std::string base_path = est.save_dir_ + "/nbt_infos";
  //   const std::string save_path = base_path + "/nbt_info-" + std::to_string(i + 1) + ".csv";
  //   mat2csv(save_path, nbt_H);
  // }

  // Save selected trajectories
  FILE *csv = fopen("/tmp/calib/traj_selected.csv", "w");
  for (const int idx : traj_selected) {
    fprintf(csv, "%d\n", idx);
  }
  fclose(csv);

  // -- Find NBT
// // find_nbt:
//   // Compute NBTs
//   ctrajs_t trajs;
//   std::vector<matx_t> calib_covars;
//   nbt_compute(
//     target, imu_params,
// 		cameras[0], cameras[1], cam_rate,
//     T_WF, T_SC0, T_SC1,
//     trajs, calib_covars
//   );
//
//   printf("nb_trajs: %ld\n", trajs.size());
//   printf("nb_calib_covars: %ld\n", calib_covars.size());
//
//   // mat2csv("/tmp/calib_covar.csv", est.calib_covar_);
//   // exit(0);
//
//   int retval = nbt_find(est, trajs, calib_covars);
//   printf("retval: %d\n", retval);
}
