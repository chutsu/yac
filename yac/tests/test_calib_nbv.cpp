#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_mono.hpp"
#include "calib_nbv.hpp"

namespace yac {

static std::vector<camera_params_t> setup_cameras() {
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

  camera_params_t cam0{0, 0, cam_res,
                       proj_model, dist_model,
                       proj_params, dist_params};
  camera_params_t cam1{1, 1, cam_res,
                       proj_model, dist_model,
                       proj_params, dist_params};

  return {cam0, cam1};
}

static void setup_calib_target(const camera_params_t &cam,
                               calib_target_t &target,
                               mat4_t &T_FO,
                               mat4_t *T_WF=nullptr) {
  // Create calibration origin
  target = calib_target_t{"aprilgrid", 6, 6, 0.088, 0.3};
  T_FO = calib_target_origin<pinhole_radtan4_t>(target, cam);

  // Calibration target pose
  if (T_WF) {
    const vec3_t rpy = deg2rad(vec3_t{90.0, 0.0, -90.0});
    const mat3_t C_WF = euler321(rpy);
    *T_WF = tf(C_WF, zeros(3, 1));
  }
}

int test_calib_target_origin() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{0, 0, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};
  const mat4_t T_FO = calib_target_origin<pinhole_radtan4_t>(target, cam_params);
  // const mat4_t origin = calib_target_origin(target, cam_res, hfov);
  print_matrix("T_FO", T_FO);

  return 0;
}

int test_calib_init_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{0, 0, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  const mat4s_t poses = calib_init_poses<pinhole_radtan4_t>(target, cam_params);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  return 0;
}

int test_calib_nbv_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{0, 0, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  const mat4s_t poses = calib_nbv_poses<pinhole_radtan4_t>(target, cam_params);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  return 0;
}

int test_calib_orbit_trajs() {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  const mat4_t T_BC0 = tf(I(3), vec3_t{0.0, 0.0, 0.0});
  const mat4_t T_BC1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  calib_orbit_trajs<pinhole_radtan4_t>(target, cameras[0], cameras[1],
                                       T_BC0, T_BC1,
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

  return 0;
}

int test_calib_pan_trajs() {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  const mat4_t T_BC0 = tf(I(3), vec3_t{0.0, 0.0, 0.0});
  const mat4_t T_BC1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  calib_pan_trajs<pinhole_radtan4_t>(target, cameras[0], cameras[1],
                                     T_BC0, T_BC1,
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

  return 0;
}

int test_calib_figure8_trajs() {
  // Cameras
  auto cameras = setup_cameras();

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Stereo camera extrinsics
  const mat4_t T_BC0 = tf(I(3), vec3_t{0.0, 0.0, 0.0});
  const mat4_t T_BC1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});

  // Generate trajectories
  ctrajs_t trajs;
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  calib_figure8_trajs<pinhole_radtan4_t>(target, cameras[0], cameras[1],
                                     	 	 T_BC0, T_BC1,
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

  return 0;
}

int test_nbv_draw() {
  // Setup Camera
  const id_t id = 0;
  const int cam_idx = 0;
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 90.0;
  const double lens_vfov = 90.0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(img_w, lens_hfov);
  const double fy = pinhole_focal(img_h, lens_vfov);
  const double cx = img_w / 2.0;
  const double cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
  const camera_params_t cam_params{id, cam_idx, cam_res,
                                    proj_model, dist_model,
                                    proj_params, dist_params};

  // Setup nbv poses
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const mat4s_t poses = calib_init_poses<pinhole_radtan4_t>(target, cam_params);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  for (const auto &T_FC : poses) {
    cv::Mat image(cam_res[1], cam_res[0], CV_8UC3, cv::Scalar(255, 255, 255));
    nbv_draw<pinhole_radtan4_t>(target, cam_params, T_FC, image);
    cv::imshow("image", image);
    cv::waitKey(0);
  }

  return 0;
}

int test_nbv_test_grid() {
  // Setup Camera
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 90.0;
  const double lens_vfov = 90.0;
  const double fx = pinhole_focal(img_w, lens_hfov);
  const double fy = pinhole_focal(img_h, lens_vfov);
  const double cx = img_w / 2.0;
  const double cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
  pinhole_radtan4_t camera{cam_res, proj_params, dist_params};

  // NBV test grid
  nbv_test_grid_t test_grid(camera);

  for (size_t i = 0; i < test_grid.nb_points; i++) {
    const vec3_t p = test_grid.object_points[i];
    const vec2_t z = test_grid.keypoints[i];
    printf("r_FFi: [%f, %f, %f] ", p(0), p(1), p(2));
    printf("z: [%f, %f]\n", z(0), z(1));
  }

  return 0;
}

int test_nbv_find() {
  test_data_t test_data = setup_test_data();

  // Calibration target
  calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  // Camera
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{id, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  // Optimize
  aprilgrids_t grids0;
  for (const auto &grid : test_data.grids0) {
    if (grid.detected) {
      grids0.push_back(grid);
    }
    if (grids0.size() == 5) {
      break;
    }
  }
  calib_mono_data_t data{grids0, cam_params};
  calib_mono_solve<pinhole_radtan4_t>(data);

  // Find NBV
  mat4_t T_FC;
  nbv_find<pinhole_radtan4_t>(target, data, T_FC);
  print_matrix("T_FC", T_FC);

  return 0;
}

int test_nbv_find2() {
  test_data_t test_data = setup_test_data();

  // Calibration target
  calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  // Camera
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam0{0, 0, cam_res,
                       proj_model, dist_model,
                       proj_params, dist_params};
  camera_params_t cam1{1, 1, cam_res,
                       proj_model, dist_model,
                       proj_params, dist_params};
  extrinsics_t cam0_exts{2};
  extrinsics_t cam1_exts{3};

  // Prep data
  aprilgrids_t grids0;
  aprilgrids_t grids1;

  const size_t grids0_end = test_data.grids0.size();
  const size_t grids1_end = test_data.grids1.size();
  size_t grid0_idx = 0;
  size_t grid1_idx = 0;
  while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
    auto grid0 = test_data.grids0[grid0_idx];
    auto grid1 = test_data.grids1[grid1_idx];

    if (grid0.timestamp > grid1.timestamp) {
      grid1_idx++;
    } else if (grid0.timestamp < grid1.timestamp) {
      grid0_idx++;
    } else if (grid0.timestamp == grid1.timestamp) {
      if (grid0.detected && grid1.detected) {
        grids0.push_back(grid0);
        grids1.push_back(grid1);
      }
      if (grids0.size() == 10) {
        break;
      }
      grid0_idx++;
      grid1_idx++;
    }
  }

  // Optimize
  calib_data_t data;
  data.add_calib_target(target);
  data.add_camera(cam0);
  data.add_camera(cam1);
  data.add_camera_extrinsics(0);
  data.add_camera_extrinsics(1);
  data.add_grids(0, grids0);
  data.add_grids(1, grids1);
  data.preprocess_data();
  data.check_data();
  calib_stereo_solve<pinhole_radtan4_t>(data);

  // Find NBV
  mat4_t T_FC = I(4);
  nbv_find<pinhole_radtan4_t>(target, data, T_FC);
  print_matrix("T_FC", T_FC);

  return 0;
}

int test_simulate_cameras() {
// Cameras
  auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const mat4_t T_BC0 = tf(I(3), zeros(3, 1));
  const mat4_t T_BC1 = T_BC0 * T_C0C1;

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  ctrajs_t trajs;
  calib_orbit_trajs<pinhole_radtan4_t>(target,
                                                   cameras[0], cameras[1],
                                                   T_BC0, T_BC1, T_WF, T_FO,
                                                ts_start, ts_end, trajs);

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
    simulate_cameras<pinhole_radtan4_t>(trajs[traj_idx], target,
                                                 cameras[0], cameras[1], cam_rate,
                                                 T_WF, T_BC0, T_BC1,
                                                 ts_start, ts_end,
                                                 grids0, grids1,
                                                 T_WC0);
    MU_CHECK(T_WC0.size() == grids0.size());
    MU_CHECK(T_WC0.size() == grids1.size());
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
      printf("saving grid to [%s]\n", save_path.c_str());
      grid.save(save_path);
    }

    for (const auto &grid : grids1) {
      const auto ts = grid.timestamp;
      auto save_path = base_path;
      save_path += "/traj_" + std::to_string(traj_idx);
      save_path += "/cam1/" + std::to_string(ts) + ".csv";
      printf("saving grid to [%s]\n", save_path.c_str());
      grid.save(save_path);
    }

    // EXPECT_EQ(grids0.size(), 5 * nbt.cam0_.rate - 1);
    // EXPECT_EQ(grids1.size(), 5 * nbt.cam1_.rate - 1);
  }
  return 0;
}

int test_simulate_imu() {
  // Cameras
  auto cameras = setup_cameras();
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const mat4_t T_BC0 = tf(I(3), zeros(3, 1));
  const mat4_t T_BC1 = T_BC0 * T_C0C1;

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  ctrajs_t trajs;
  calib_orbit_trajs<pinhole_radtan4_t>(target,
                                                   cameras[0], cameras[1],
                                                   T_BC0, T_BC1, T_WF, T_FO,
                                                ts_start, ts_end, trajs);

  // Simulate IMU
  imu_params_t imu_params;
  imu_params.rate = 250.0;
  imu_params.sigma_g_c  = 0.0;
  imu_params.sigma_bg = 0.0;
  imu_params.sigma_a_c  = 0.0;
  imu_params.sigma_ba  = 0.0;
  imu_params.sigma_gw_c = 0.0;
  imu_params.sigma_aw_c = 0.0;
  // imu_params.tau = 0.0;

  // const auto rpy_SC = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  const auto rpy_BS = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const auto C_BS = euler321(rpy_BS);
  mat4_t T_BS = tf(C_BS, zeros(3, 1));

  // const int traj_idx = 0;
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    timestamps_t imu_time;
    vec3s_t imu_accel;
    vec3s_t imu_gyro;
    mat4s_t imu_poses;
    vec3s_t imu_vels;
    simulate_imu(trajs[traj_idx], ts_start, ts_end, T_BC0, T_BS,
                 imu_params, imu_time, imu_accel, imu_gyro,
                 imu_poses, imu_vels);

    // autocal::ImuMeasurementDeque imu_meas;
    // for (size_t i = 0; i < imu_accel.size(); i++) {
    //   Time ts{ts2sec(imu_time[i])};
    //   vec3_t gyr = imu_gyro[i];
    //   vec3_t acc = imu_accel[i];
    //   imu_meas.emplace_back(ts, ImuSensorReadings{gyr, acc});
    // }
    //
    // autocal::Transformation T_WS{imu_poses[0]};
    // autocal::SpeedAndBias sb;
    // sb << imu_vels.front(), 0, 0, 0, 0, 0, 0;
    //
    // Time t_start{ts2sec(ts_start)};
    // Time t_end{ts2sec(ts_end)};
    // ImuError::propagation(imu_meas, imu_params, T_WS, sb, t_start, t_end);
    //
    // print_matrix("pose at the end           ", imu_poses.back());
    // print_matrix("propagated pose at the end", T_WS.T());

    // Save data
    auto base_path = "/tmp/nbt/traj_" + std::to_string(traj_idx) + "/imu";
    remove_dir(base_path);
    dir_create(base_path);
    save_data(base_path + "/accel.csv", imu_time, imu_accel);
    save_data(base_path + "/gyro.csv", imu_time, imu_gyro);
    save_poses(base_path + "/poses.csv", imu_time, imu_poses);

    // EXPECT_EQ(imu_time.size(), 5 * imu_params.rate + 1);
    // EXPECT_EQ(imu_accel.size(), 5 * imu_params.rate + 1);
    // EXPECT_EQ(imu_gyro.size(), 5 * imu_params.rate + 1);
  }

  return 0;
}

int test_nbt_eval_traj() {
  // Cameras
  auto cameras = setup_cameras();
  auto cam0 = cameras[0];
  auto cam1 = cameras[1];
  auto cam_rate = 20.0;
  const mat4_t T_C0C1 = tf(I(3), vec3_t{0.1, 0.0, 0.0});
  const mat4_t T_BC0 = tf(I(3), zeros(3, 1));
  const mat4_t T_BC1 = T_BC0 * T_C0C1;

  // Imu
  // const auto rpy_SC = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  const auto rpy_BS = deg2rad(vec3_t{0.0, 0.0, 0.0});
  const auto C_BS = euler321(rpy_BS);
  const mat4_t T_BS = tf(C_BS, zeros(3, 1));

  // Calibration target
  calib_target_t target;
  mat4_t T_FO;
  mat4_t T_WF;
  setup_calib_target(cameras[0], target, T_FO, &T_WF);

  // Generate trajectory
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 2e9;
  ctrajs_t trajs;
  calib_orbit_trajs<pinhole_radtan4_t>(target, cameras[0], cameras[1],
                                                   T_BC0, T_BC1, T_WF, T_FO,
                                                ts_start, ts_end, trajs);

  // Simulate IMU
  imu_params_t imu_params;
  imu_params.rate = 250.0;
  imu_params.sigma_g_c = 0.00278;
  imu_params.sigma_a_c = 0.0252;
  imu_params.sigma_bg = 0.03;
  imu_params.sigma_ba = 0.1;
  imu_params.sigma_gw_c = 1.65e-05;
  imu_params.sigma_aw_c = 0.000441;

  #pragma omp parallel for
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    matx_t calib_covar;
    int retval = nbt_eval_traj<pinhole_radtan4_t>(trajs[traj_idx],
                                                             target,
                                                             ts_start, ts_end,
                                                             imu_params, cam0, cam1,
                                                             cam_rate,
                                                             T_WF, T_BC0, T_BC1, T_BS,
                                                             calib_covar);
    printf("retval: %d, full_rank(calib_covar): %d\n", retval, full_rank(calib_covar));
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_target_origin);
  MU_ADD_TEST(test_calib_init_poses);
  MU_ADD_TEST(test_calib_nbv_poses);
  MU_ADD_TEST(test_calib_orbit_trajs);
  MU_ADD_TEST(test_calib_pan_trajs);
  MU_ADD_TEST(test_calib_figure8_trajs);
  MU_ADD_TEST(test_nbv_draw);
  MU_ADD_TEST(test_nbv_test_grid);
  MU_ADD_TEST(test_nbv_find);
  MU_ADD_TEST(test_nbv_find2);
  MU_ADD_TEST(test_simulate_cameras);
  MU_ADD_TEST(test_simulate_imu);
  MU_ADD_TEST(test_nbt_eval_traj);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
