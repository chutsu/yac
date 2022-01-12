#include "munit.hpp"
#include "util/euroc.hpp"
#include "calib_vi.hpp"

namespace yac {

struct test_data_t {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/cam_april";
  const std::string grids0_path = data_path + "/grid0/cam0";
  const std::string grids1_path = data_path + "/grid0/cam1";
  euroc_calib_t data{data_path};
  aprilgrids_t grids0;
  aprilgrids_t grids1;
  camera_params_t cam0;
  camera_params_t cam1;
};

test_data_t setup_test_data() {
  test_data_t test_data;
  LOG_INFO("Loading EuRoC data [%s]", test_data.data_path.c_str());

  // preprocess aprilgrids
  LOG_INFO("Preprocessing AprilGrid data ...");
  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  const auto grids0_path = test_data.grids0_path;
  const auto grids1_path = test_data.grids1_path;
  if (system(("mkdir -p " + grids0_path).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", grids0_path.c_str());
  }
  if (system(("mkdir -p " + grids1_path).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", grids1_path.c_str());
  }

  auto timeline = test_data.data.timeline();
  size_t i = 0;
  for (const auto &ts : timeline.timestamps) {
    if (i++ % 100 == 0) {
      printf(".");
      fflush(stdout);
    }

    for (const auto &event : timeline.get_events(ts)) {
      if (auto cam_event = dynamic_cast<camera_event_t *>(event)) {
        const timestamp_t ts = cam_event->ts;
        const int cam_idx = cam_event->cam_idx;
        const cv::Mat image = cv::imread(cam_event->img_path);
        const std::string grid_fname = std::to_string(ts) + ".csv";

        std::string grid_file;
        if (cam_idx == 0) {
          grid_file = grids0_path + "/" + grid_fname;
        } else if (cam_idx == 1) {
          grid_file = grids1_path + "/" + grid_fname;
        }

        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
          grid = detector.detect(ts, image);
          grid.save(grid_file);
        } else {
          grid.load(grid_file);

          if (grid.detected == false) {
            grid.timestamp = ts;
            grid.tag_rows = detector.tag_rows;
            grid.tag_cols = detector.tag_cols;
            grid.tag_size = detector.tag_size;
            grid.tag_spacing = detector.tag_spacing;
          }
        }

        switch (cam_idx) {
          case 0:
            test_data.grids0.push_back(grid);
            break;
          case 1:
            test_data.grids1.push_back(grid);
            break;
        }
      }
    }
  }
  printf("\n");

  // Setup cameras
  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  test_data.cam0 = camera_params_t::init(0, res, proj_model, dist_model);
  test_data.cam1 = camera_params_t::init(1, res, proj_model, dist_model);

  return test_data;
}

// Load Test Data
test_data_t test_data = setup_test_data();

int test_fiducial_error() {
  assert(test_data.grids.size() > 2);

  const auto grid = test_data.grids0[400];
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t kps;
  vec3s_t obj_pts;
  grid.get_measurements(tag_ids, corner_idxs, kps, obj_pts);

  int i = 0;
  auto ts = grid.timestamp;
  const int tag_id = tag_ids[i];
  const int corner_idx = corner_idxs[i];
  const vec3_t r_FFi = obj_pts[i];
  const vec2_t z = kps[i];
  const mat2_t covar = I(2);

  // clang-format off
  mat4_t T_WF;
  T_WF << -0.121411, -0.234188, 0.964580, -0.858724,
           0.992576, -0.035726, 0.116261, -0.407895,
           0.007234, 0.971535, 0.236787, -0.486840,
           0.000000, 0.000000, 0.000000, 1.000000;
  // clang-format on

  // clang-format off
  mat4_t T_WS;
  T_WS << -0.324045, 0.036747, -0.945328, 0.100000,
           0.036747, 0.998980, 0.026236, 0.00000,
           0.945328, -0.026236, -0.325065, 0.00000,
           0.000000, 0.000000, 0.000000, 1.000000;
  // ^ Note: Due to numerical stability issues the translation component cannot
  // be 0 for checking jacobians
  // clang-format on

  // clang-format off
  mat4_t T_BC0;
  T_BC0 << 0.0, -1.0, 0.0, 0.001,
           1.0, 0.0, 0.0, 0.001,
           0.0, 0.0, 1.0, 0.001,
           0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // clang-format off
  mat4_t T_BS;
  T_BS << 0.0, -1.0, 0.0, 0.001,
          1.0, 0.0, 0.0, 0.001,
          0.0, 0.0, 1.0, 0.001,
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  const int cam_idx = 0;
  const pinhole_radtan4_t cam_geom;
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  fiducial_error_t err(ts,
                       &cam_geom,
                       cam_idx,
                       res,
                       tag_id,
                       corner_idx,
                       r_FFi,
                       z,
                       T_WF,
                       covar);

  pose_t fiducial_pose{0, T_WF};
  pose_t sensor_pose{ts, T_WS};
  extrinsics_t cam_extrinsics{T_BC0};
  extrinsics_t imu_extrinsics{T_BS};
  camera_params_t cam(cam_idx,
                      res,
                      proj_model,
                      dist_model,
                      proj_params,
                      dist_params);

  std::vector<double *> params = {fiducial_pose.param.data(),
                                  sensor_pose.param.data(),
                                  imu_extrinsics.param.data(),
                                  cam_extrinsics.param.data(),
                                  cam.param.data()};

  vec2_t r;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J2;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J3;
  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J4;
  std::vector<double *> jacobians = {J0.data(),
                                     J1.data(),
                                     J2.data(),
                                     J3.data(),
                                     J4.data()};

  // Baseline
  err.Evaluate(params.data(), r.data(), jacobians.data());
  print_vector("r", r);
  double step = 1e-8;
  double threshold = 1e-4;

  // // -- Test fiducial-pose jacobian
  // {
  //   mat_t<2, 6> fdiff;
  //
  //   for (int i = 0; i < 6; i++) {
  //     vec2_t r_fd;
  //     fiducial_pose.perturb(i, step);
  //     err.Evaluate(params.data(), r_fd.data(), nullptr);
  //     fdiff.col(i) = (r_fd - r) / step;
  //     fiducial_pose.perturb(i, -step);
  //   }
  //
  //   const mat_t<2, 6> J0_min = J0.block(0, 0, 2, 6);
  //   MU_CHECK(check_jacobian("J0", fdiff, J0_min, threshold, true) == 0);
  //   print_matrix("fdiff", fdiff);
  //   print_matrix("J0", J0);
  // }

  // // -- Test sensor-pose jacobian
  // {
  //   mat_t<2, 6> fdiff;
  //
  //   for (int i = 0; i < 6; i++) {
  //     // Forward difference
  //     // vec2_t r_fd;
  //     // sensor_pose.perturb(i, step);
  //     // err.Evaluate(params.data(), r_fd.data(), nullptr);
  //     // sensor_pose.perturb(i, -step);
  //     // fdiff.col(i) = (r_fd - r) / step;
  //
  //     // Central finite difference
  //     vec2_t r_fd;
  //     sensor_pose.perturb(i, 0.5 * step);
  //     err.Evaluate(params.data(), r_fd.data(), nullptr);
  //     sensor_pose.perturb(i, -0.5 * step);
  //
  //     vec2_t r_bd;
  //     sensor_pose.perturb(i, -0.5 * step);
  //     err.Evaluate(params.data(), r_bd.data(), nullptr);
  //     sensor_pose.perturb(i, 0.5 * step);
  //
  //     fdiff.col(i) = (r_fd - r_bd) / step;
  //   }
  //
  //   const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
  //   MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
  // }

  // -- Test imu-extrinsics jacobian
  {
    mat_t<2, 6> fdiff;
    for (int i = 0; i < 6; i++) {
      imu_extrinsics.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      imu_extrinsics.perturb(i, -step);
    }

    const mat_t<2, 6> J2_min = J2.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J2", fdiff, J2_min, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J2", J2_min);
  }

  // -- Test cam extrinsics jacobian
  {
    mat_t<2, 6> fdiff;
    for (int i = 0; i < 6; i++) {
      cam_extrinsics.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam_extrinsics.perturb(i, -step);
    }

    const mat_t<2, 6> J3_min = J3.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J3", fdiff, J3_min, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J3", J3_min);
  }

  // -- Test camera-parameters jacobian
  {
    mat_t<2, 8> fdiff;

    for (int i = 0; i < 8; i++) {
      cam.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J4", fdiff, J4, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J4", J4);
  }

  return 0;
}

int test_fiducial_td_error() {
  assert(test_data.grids.size() > 2);

  const auto grid_i = test_data.grids0[400];
  const auto grid_j = test_data.grids0[401];

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t kps_i;
  vec2s_t kps_j;
  vec3s_t object_points;
  aprilgrid_t::common_measurements(grid_i,
                                   grid_j,
                                   tag_ids,
                                   corner_indicies,
                                   kps_i,
                                   kps_j,
                                   object_points);

  int i = 0;
  auto ts_i = grid_i.timestamp;
  auto ts_j = grid_j.timestamp;
  const int tag_id = tag_ids[i];
  const int corner_idx = corner_indicies[i];
  const vec3_t r_FFi = object_points[i];
  const vec2_t z_i = kps_i[i];
  const vec2_t z_j = kps_j[i];
  const mat2_t covar = I(2);

  // clang-format off
  mat4_t T_WF;
  T_WF << -0.121411, -0.234188, 0.964580, -0.858724,
           0.992576, -0.035726, 0.116261, -0.407895,
           0.007234, 0.971535, 0.236787, -0.486840,
           0.000000, 0.000000, 0.000000, 1.000000;
  // clang-format on

  // clang-format off
  mat4_t T_WS;
  T_WS << -0.324045, 0.036747, -0.945328, 0.100000,
           0.036747, 0.998980, 0.026236, 0.00000,
           0.945328, -0.026236, -0.325065, 0.00000,
           0.000000, 0.000000, 0.000000, 1.000000;
  // ^ Note: Due to numerical stability issues the translation component cannot
  // be 0 for checking jacobians
  // clang-format on

  // clang-format off
  mat4_t T_BC0;
  T_BC0 << 0.0, -1.0, 0.0, 0.001,
           1.0, 0.0, 0.0, 0.001,
           0.0, 0.0, 1.0, 0.001,
           0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // clang-format off
  mat4_t T_BS;
  T_BS << 0.0, -1.0, 0.0, 0.001,
          1.0, 0.0, 0.0, 0.001,
          0.0, 0.0, 1.0, 0.001,
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  const int cam_idx = 0;
  const pinhole_radtan4_t cam_geom;
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

  fiducial_td_error_t err(ts_i,
                          ts_j,
                          &cam_geom,
                          cam_idx,
                          res,
                          tag_id,
                          corner_idx,
                          r_FFi,
                          z_i,
                          z_j,
                          T_WF,
                          covar);

  fiducial_t fiducial{T_WF};
  pose_t sensor_pose{ts_i, T_WS};
  extrinsics_t cam_extrinsics{T_BC0};
  extrinsics_t imu_extrinsics{T_BS};
  camera_params_t cam(cam_idx,
                      res,
                      proj_model,
                      dist_model,
                      proj_params,
                      dist_params);
  time_delay_t td(0.0);

  std::vector<double *> params = {fiducial.param.data(),
                                  sensor_pose.param.data(),
                                  imu_extrinsics.param.data(),
                                  cam_extrinsics.param.data(),
                                  cam.param.data(),
                                  td.param.data()};
  vec2_t r;
  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J2;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J3;
  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J4;
  Eigen::Vector2d J5;
  std::vector<double *> jacobians =
      {J0.data(), J1.data(), J2.data(), J3.data(), J4.data(), J5.data()};

  // Baseline
  err.Evaluate(params.data(), r.data(), jacobians.data());
  print_vector("r", r);

  double step = 1e-8;
  double threshold = 1e-5;

  // -- Test fiducial jacobian
  {
    mat_t<2, 2> fdiff;

    for (int i = 0; i < 2; i++) {
      vec2_t r_fd;
      fiducial.perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      fiducial.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J0", fdiff, J0, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J0", J0);
  }

  // -- Test sensor-pose jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      // Forward difference
      // vec2_t r_fd;
      // sensor_pose.perturb(i, step);
      // err.Evaluate(params.data(), r_fd.data(), nullptr);
      // sensor_pose.perturb(i, -step);
      // fdiff.col(i) = (r_fd - r) / step;

      // Central finite difference
      vec2_t r_fd;
      sensor_pose.perturb(i, 0.5 * step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      sensor_pose.perturb(i, -0.5 * step);

      vec2_t r_bd;
      sensor_pose.perturb(i, -0.5 * step);
      err.Evaluate(params.data(), r_bd.data(), nullptr);
      sensor_pose.perturb(i, 0.5 * step);

      fdiff.col(i) = (r_fd - r_bd) / step;
    }

    const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
  }

  // -- Test imu-extrinsics jacobian
  {
    const mat_t<2, 6> J2_min = J2.block(0, 0, 2, 6);

    mat_t<2, 6> fdiff;
    for (int i = 0; i < 6; i++) {
      imu_extrinsics.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      imu_extrinsics.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J2", fdiff, J2_min, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J2", J2_min);
  }

  // -- Test cam extrinsics jacobian
  {
    const mat_t<2, 6> J3_min = J3.block(0, 0, 2, 6);

    mat_t<2, 6> fdiff;
    for (int i = 0; i < 6; i++) {
      cam_extrinsics.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam_extrinsics.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J3", fdiff, J3_min, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J3", J3_min);
  }

  // -- Test camera-parameters jacobian
  {
    mat_t<2, 8> fdiff;

    for (int i = 0; i < 8; i++) {
      cam.perturb(i, step);
      vec2_t r_fd;
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J4", fdiff, J4, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J4", J4);
  }

  // -- Test time delay
  {
    mat_t<2, 1> fdiff;

    td.perturb(0, step);
    vec2_t r_fd;
    err.Evaluate(params.data(), r_fd.data(), nullptr);
    fdiff.col(0) = (r_fd - r) / step;
    td.perturb(0, -step);

    MU_CHECK(check_jacobian("J5", fdiff, J5, threshold, true) == 0);
    print_matrix("fdiff", fdiff);
    print_matrix("J5", J5);
  }

  return 0;
}

int test_calib_vi_add_imu() {
  calib_vi_t calib;

  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_g_c = 12.0e-4;
  imu_params.sigma_a_c = 8.0e-3;
  imu_params.sigma_gw_c = 4.0e-6;
  imu_params.sigma_aw_c = 4.0e-5;
  imu_params.g = 9.81007;

  mat4_t T_BS = I(4);
  calib.add_imu(imu_params, T_BS);

  MU_CHECK(fltcmp(calib.imu_params.rate, imu_params.rate) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_g_c, imu_params.sigma_g_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_a_c, imu_params.sigma_a_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_gw_c, imu_params.sigma_gw_c) == 0);
  MU_CHECK(fltcmp(calib.imu_params.sigma_aw_c, imu_params.sigma_aw_c) == 0);
  MU_CHECK(calib.imu_exts != nullptr);
  MU_CHECK(calib.imu_exts->fixed == false);

  return 0;
}

int test_calib_vi_add_camera() {
  calib_vi_t calib;

  // Camera
  const int cam_idx = 0;
  const int res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[0], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const mat4_t T_BCi = I(4);
  calib.add_camera(cam_idx,
                   res,
                   proj_model,
                   dist_model,
                   proj_params,
                   dist_params,
                   T_BCi);

  MU_CHECK(calib.cam_params[cam_idx] != nullptr);
  MU_CHECK(calib.cam_params[cam_idx]->fixed == false);
  MU_CHECK(calib.cam_exts[cam_idx] != nullptr);
  MU_CHECK(calib.cam_exts[cam_idx]->fixed == false);

  return 0;
}

int test_calib_vi_add_sensor_pose() {
  calib_vi_t calib;

  const timestamp_t ts = 0;
  const mat4_t T_WS = I(4);
  calib.add_sensor_pose(ts, T_WS);

  MU_CHECK(calib.sensor_poses.size() == 1);
  MU_CHECK(calib.sensor_poses[ts] != NULL);
  MU_CHECK(calib.sensor_poses[ts]->tf().isApprox(T_WS));

  return 0;
}

int test_calib_vi_add_speed_biases() {
  calib_vi_t calib;

  const timestamp_t ts = 0;
  vec_t<9> sb;
  sb << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  calib.add_speed_biases(ts, sb);

  MU_CHECK(calib.speed_biases.size() == 1);
  MU_CHECK(calib.speed_biases[ts] != NULL);
  MU_CHECK(calib.speed_biases[ts]->param.isApprox(sb));

  return 0;
}

int test_calib_vi_add_fiducial_pose() {
  calib_vi_t calib;

  const mat4_t T_WF = I(4);
  calib.add_fiducial_pose(T_WF);

  MU_CHECK(calib.fiducial != NULL);
  MU_CHECK(calib.fiducial->estimate().isApprox(T_WF));

  return 0;
}

int test_calib_vi_update_prev_grids() {
  calib_vi_t calib;

  std::map<int, aprilgrid_t> grids;
  grids[0] = aprilgrid_t();
  grids[1] = aprilgrid_t();

  calib.update_prev_grids(grids);
  MU_CHECK(calib.grids_prev.size() == 2);

  return 0;
}

// int test_calib_vi() {
//   test_data_t test_data = setup_test_data();
//
//   vec4_t cam0_proj_params;
//   vec4_t cam0_dist_params;
//   vec4_t cam1_proj_params;
//   vec4_t cam1_dist_params;
//   mat4_t T_BC0;
//   mat4_t T_BC1;
//   config_t config{"/tmp/calib-stereo.yaml"};
//   parse(config, "cam0.proj_params", cam0_proj_params);
//   parse(config, "cam0.dist_params", cam0_dist_params);
//   parse(config, "cam1.proj_params", cam1_proj_params);
//   parse(config, "cam1.dist_params", cam1_dist_params);
//   parse(config, "T_body0_cam0", T_BC0);
//   parse(config, "T_body0_cam1", T_BC1);
//
//   // Camera parameters
//   const int res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   // // -- Euroc Calibration values
//   // vec4_t cam0_proj_params{458.654, 457.296, 367.215, 248.375};
//   // vec4_t cam0_dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05};
//   // vec4_t cam1_proj_params{457.587, 456.134, 379.999, 255.238};
//   // vec4_t cam1_dist_params{-0.28368365, 0.07451284, -0.00010473,
//   -3.55590700e-05};
//
//   // Imu parameters
//   imu_params_t imu_params;
//   imu_params.rate = 200.0;
//   imu_params.sigma_a_c = 0.002;       // Accel noise density [m/s^2/sqrt(Hz)]
//   imu_params.sigma_g_c = 1.6968e-04;  // Gyro noise density [rad/s/sqrt(Hz)]
//   imu_params.sigma_aw_c = 0.003;      // Accel drift noise density
//   [m/s^2/sqrt(Hz)] imu_params.sigma_gw_c = 1.9393e-05; // Gyro drift noise
//   density [rad/s^s/sqrt(Hz)] imu_params.g = 9.81007;         // Earth's
//   acceleration due to gravity [m/s^2]
//
//   // Setup VI calibrator
//   calib_vi_t calib;
//   calib.add_imu(imu_params);
//   calib.add_camera(0, res, proj_model, dist_model, cam0_proj_params,
//   cam0_dist_params, true); calib.add_camera(1, res, proj_model, dist_model,
//   cam1_proj_params, cam1_dist_params, true); mat4_t T_BS; T_BS << 0.0, 1.0,
//   0.0, 0.0,
//           -1.0, 0.0, 0.0, 0.0,
//           0.0, 0.0, 1.0, 0.0,
//           0.0, 0.0, 0.0, 1.0;
//   // T_BS = T_imu0_cam0.inverse();
//   calib.add_imu_extrinsics(T_BS);
//   calib.add_cam_extrinsics(0, T_BC0, true);
//   calib.add_cam_extrinsics(1, T_BC1, true);
//   // clang-format on
//
//   LOG_INFO("Adding data to problem ...");
//   // int counter = 0;
//   auto timeline = test_data.data.timeline();
//   for (const auto &ts : timeline.timestamps) {
//     const auto kv = timeline.data.equal_range(ts);
//
//     for (auto it = kv.first; it != kv.second; it++) {
//       const auto event = it->second;
//
//       if (event.type == CAMERA_EVENT) {
//         const int cam_idx = event.cam_idx;
//         const auto ts = event.ts;
//         const auto grid_fname = std::to_string(ts) + ".csv";
//
//         std::string grid_fpath;
//         if (cam_idx == 0) {
//           grid_fpath = test_data.grids0_path + "/" + grid_fname;
//         } else if (cam_idx == 1) {
//           grid_fpath = test_data.grids1_path + "/" + grid_fname;
//         }
//         aprilgrid_t grid{grid_fpath};
//         calib.add_measurement(cam_idx, grid);
//
//       } else if (event.type == IMU_EVENT) {
//         const auto ts = event.ts;
//         const vec3_t a_m = event.a_m;
//         const vec3_t w_m = event.w_m;
//         calib.add_measurement(ts, a_m, w_m);
//       }
//     }
//   }
//   calib.solve();
//   calib.save();
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_fiducial_error);
  MU_ADD_TEST(test_fiducial_td_error);

  MU_ADD_TEST(test_calib_vi_add_imu);
  MU_ADD_TEST(test_calib_vi_add_camera);
  MU_ADD_TEST(test_calib_vi_add_sensor_pose);
  MU_ADD_TEST(test_calib_vi_add_speed_biases);
  MU_ADD_TEST(test_calib_vi_add_fiducial_pose);
  MU_ADD_TEST(test_calib_vi_update_prev_grids);

  // MU_ADD_TEST(test_calib_vi);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
