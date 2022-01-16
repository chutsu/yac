#include "munit.hpp"
#include "util/euroc.hpp"
#include "calib_vi.hpp"

namespace yac {

timeline_t setup_test_data() {
  // Load calibration data
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> grid_paths = {
      {0, data_path + "/grid0/cam0"},
      {1, data_path + "/grid0/cam1"},
  };
  LOG_INFO("Loading EuRoC data [%s]", data_path.c_str());

  // Preprocess aprilgrids
  LOG_INFO("Preprocessing AprilGrid data ...");
  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    if (system(("mkdir -p " + grid_paths[cam_idx]).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", grid_paths[cam_idx].c_str());
    }
  }

  // Loop through timeline
  euroc_calib_t calib_data{data_path};
  auto timeline = calib_data.timeline();
  size_t i = 0;

  for (const auto &ts : timeline.timestamps) {
    if (i++ % 100 == 0) {
      printf(".");
      fflush(stdout);
    }

    for (const auto &event : timeline.get_events(ts)) {
      if (auto cam_event = dynamic_cast<camera_event_t *>(event)) {
        // Setup
        const timestamp_t ts = cam_event->ts;
        const int cam_idx = cam_event->cam_idx;
        const cv::Mat image = cv::imread(cam_event->img_path);
        const std::string grid_fname = std::to_string(ts) + ".csv";
        const std::string grid_file = grid_paths[cam_idx] + "/" + grid_fname;

        // Detect or load
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

        // Add aprilgrid to timeline
        timeline.add(ts, cam_idx, grid);
      }
    }
  }
  printf("\n");

  return timeline;
}

// int test_fiducial_error() {
//   // Fiducial pose in world frame
//   // clang-format off
//   mat4_t T_WF;
//   T_WF << 0.0, 0.0, 1.0, -1.0,
//           1.0, 0.0, 0.0, -0.4,
//           0.0, 1.0, 0.0, -0.5,
//           0.0, 0.0, 0.0, 1.0;
//   T_WF = tf_perturb_rot(T_WF, 0.01, 0);
//   // clang-format on
//
//   // Imu pose in world frame
//   // clang-format off
//   mat4_t T_WS;
//   T_WS << -1.0, 0.0, 0.0, 0.1,
//           0.0, -1.0, 0.0, 0.0,
//           0.0, 0.0, 1.0, 0.0,
//           0.0, 0.0, 0.0, 1.0;
//   T_WS = tf_perturb_rot(T_WS, 0.01, 1);
//   // ^ Note: Due to numerical stability issues the translation component
//   cannot
//   // be 0 for checking jacobians
//   // clang-format on
//
//   // Camera extrinsics
//   // clang-format off
//   mat4_t T_BC0;
//   T_BC0 << 0.0, -1.0, 0.0, 0.001,
//            1.0, 0.0, 0.0, 0.001,
//            0.0, 0.0, 1.0, 0.001,
//            0.0, 0.0, 0.0, 1.0;
//   T_BC0 = tf_perturb_rot(T_BC0, 0.01, 2);
//   // clang-format on
//
//   // Imu extrinsics
//   // clang-format off
//   mat4_t T_BS;
//   T_BS << 0.0, 0.0, 1.0, 0.001,
//           0.0, -1.0, 0.0, 0.001,
//           1.0, 0.0, 0.0, 0.001,
//           0.0, 0.0, 0.0, 1.0;
//   T_BS = tf_perturb_rot(T_BS, -0.01, 2);
//   // clang-format on
//
//   // Get AprilGrid data
//   aprilgrid_t grid{0, 6, 6, 0.088, 0.3};
//   const int tag_id = 0;
//   const int corner_idx = 2;
//   const timestamp_t ts = 0;
//   const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
//   print_vector("r_FFi", r_FFi);
//
//   // Camera parameters
//   const int cam_idx = 0;
//   const pinhole_radtan4_t cam_geom;
//   const int res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; vec_t<8> params_data; params_data <<
//   proj_params, dist_params;
//
//   // Project image point
//   const mat2_t covar = I(2);
//   vec2_t z;
//   const mat4_t T_C0B = T_BC0.inverse();
//   const mat4_t T_SW = T_WS.inverse();
//   const vec3_t r_C0Fi = tf_point(T_C0B * T_BS * T_SW * T_WF, r_FFi);
//   cam_geom.project(res, params_data, r_C0Fi, z);
//
//   // Form fiducial error
//   fiducial_error_t err(ts,
//                        &cam_geom,
//                        &cam_params[cam_idx],
//                        &cam_exts[cam_idx],
//                        &imu_exts,
//                        &fiducial,
//                        tag_id,
//                        corner_idx,
//                        r_FFi,
//                        z,
//                        covar);
//
//   // Form parameters
//   pose_t fiducial_pose{0, T_WF};
//   pose_t sensor_pose{ts, T_WS};
//   extrinsics_t cam_extrinsics{T_BC0};
//   extrinsics_t imu_extrinsics{T_BS};
//   camera_params_t cam(cam_idx,
//                       res,
//                       proj_model,
//                       dist_model,
//                       proj_params,
//                       dist_params);
//   std::vector<double *> params = {fiducial_pose.param.data(),
//                                   sensor_pose.param.data(),
//                                   imu_extrinsics.param.data(),
//                                   cam_extrinsics.param.data(),
//                                   cam.param.data()};
//
//   // Form residual and jacobian data
//   vec2_t r;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J0;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J2;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J3;
//   Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J4;
//   std::vector<double *> jacobians = {J0.data(),
//                                      J1.data(),
//                                      J2.data(),
//                                      J3.data(),
//                                      J4.data()};
//
//   // Baseline
//   err.Evaluate(params.data(), r.data(), jacobians.data());
//   print_vector("r", r);
//   double step = 1e-8;
//   double threshold = 1e-4;
//
//   // -- Test fiducial-pose jacobian
//   {
//     mat_t<2, 6> fdiff;
//
//     for (int i = 0; i < 6; i++) {
//       vec2_t r_fd;
//       fiducial_pose.perturb(i, step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       fiducial_pose.perturb(i, -step);
//     }
//
//     const mat_t<2, 6> J0_min = J0.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J0", fdiff, J0_min, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J0", J0);
//   }
//
//   // -- Test sensor-pose jacobian
//   {
//     mat_t<2, 6> fdiff;
//
//     for (int i = 0; i < 6; i++) {
//       // Forward difference
//       // vec2_t r_fd;
//       // sensor_pose.perturb(i, step);
//       // err.Evaluate(params.data(), r_fd.data(), nullptr);
//       // sensor_pose.perturb(i, -step);
//       // fdiff.col(i) = (r_fd - r) / step;
//
//       // Central finite difference
//       vec2_t r_fd;
//       sensor_pose.perturb(i, 0.5 * step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       sensor_pose.perturb(i, -0.5 * step);
//
//       vec2_t r_bd;
//       sensor_pose.perturb(i, -0.5 * step);
//       err.Evaluate(params.data(), r_bd.data(), nullptr);
//       sensor_pose.perturb(i, 0.5 * step);
//
//       fdiff.col(i) = (r_fd - r_bd) / step;
//     }
//
//     const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
//   }
//
//   // -- Test imu-extrinsics jacobian
//   {
//     mat_t<2, 6> fdiff;
//     for (int i = 0; i < 6; i++) {
//       imu_extrinsics.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       imu_extrinsics.perturb(i, -step);
//     }
//
//     const mat_t<2, 6> J2_min = J2.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J2", fdiff, J2_min, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J2", J2_min);
//   }
//
//   // -- Test cam extrinsics jacobian
//   {
//     mat_t<2, 6> fdiff;
//     for (int i = 0; i < 6; i++) {
//       cam_extrinsics.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam_extrinsics.perturb(i, -step);
//     }
//
//     const mat_t<2, 6> J3_min = J3.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J3", fdiff, J3_min, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J3", J3_min);
//   }
//
//   // -- Test camera-parameters jacobian
//   {
//     mat_t<2, 8> fdiff;
//
//     for (int i = 0; i < 8; i++) {
//       cam.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J4", fdiff, J4, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J4", J4);
//   }
//
//   return 0;
// }

// int test_fiducial_td_error() {
//   assert(test_data.grids.size() > 2);
//
//   const auto grid_i = test_data.grids0[400];
//   const auto grid_j = test_data.grids0[401];
//
//   std::vector<int> tag_ids;
//   std::vector<int> corner_indicies;
//   vec2s_t kps_i;
//   vec2s_t kps_j;
//   vec3s_t object_points;
//   aprilgrid_t::common_measurements(grid_i,
//                                    grid_j,
//                                    tag_ids,
//                                    corner_indicies,
//                                    kps_i,
//                                    kps_j,
//                                    object_points);
//
//   int i = 0;
//   auto ts_i = grid_i.timestamp;
//   auto ts_j = grid_j.timestamp;
//   const int tag_id = tag_ids[i];
//   const int corner_idx = corner_indicies[i];
//   const vec3_t r_FFi = object_points[i];
//   const vec2_t z_i = kps_i[i];
//   const vec2_t z_j = kps_j[i];
//   const mat2_t covar = I(2);
//
//   // clang-format off
//   mat4_t T_WF;
//   T_WF << -0.121411, -0.234188, 0.964580, -0.858724,
//            0.992576, -0.035726, 0.116261, -0.407895,
//            0.007234, 0.971535, 0.236787, -0.486840,
//            0.000000, 0.000000, 0.000000, 1.000000;
//   // clang-format on
//
//   // clang-format off
//   mat4_t T_WS;
//   T_WS << -0.324045, 0.036747, -0.945328, 0.100000,
//            0.036747, 0.998980, 0.026236, 0.00000,
//            0.945328, -0.026236, -0.325065, 0.00000,
//            0.000000, 0.000000, 0.000000, 1.000000;
//   // ^ Note: Due to numerical stability issues the translation component
//   cannot
//   // be 0 for checking jacobians
//   // clang-format on
//
//   // clang-format off
//   mat4_t T_BC0;
//   T_BC0 << 0.0, -1.0, 0.0, 0.001,
//            1.0, 0.0, 0.0, 0.001,
//            0.0, 0.0, 1.0, 0.001,
//            0.0, 0.0, 0.0, 1.0;
//   // clang-format on
//
//   // clang-format off
//   mat4_t T_BS;
//   T_BS << 0.0, -1.0, 0.0, 0.001,
//           1.0, 0.0, 0.0, 0.001,
//           0.0, 0.0, 1.0, 0.001,
//           0.0, 0.0, 0.0, 1.0;
//   // clang-format on
//
//   const int cam_idx = 0;
//   const pinhole_radtan4_t cam_geom;
//   const int res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05};
//
//   fiducial_td_error_t err(ts_i,
//                           ts_j,
//                           &cam_geom,
//                           cam_idx,
//                           res,
//                           tag_id,
//                           corner_idx,
//                           r_FFi,
//                           z_i,
//                           z_j,
//                           T_WF,
//                           covar);
//
//   fiducial_t fiducial{T_WF};
//   pose_t sensor_pose{ts_i, T_WS};
//   extrinsics_t cam_extrinsics{T_BC0};
//   extrinsics_t imu_extrinsics{T_BS};
//   camera_params_t cam(cam_idx,
//                       res,
//                       proj_model,
//                       dist_model,
//                       proj_params,
//                       dist_params);
//   time_delay_t td(0.0);
//
//   std::vector<double *> params = {fiducial.param.data(),
//                                   sensor_pose.param.data(),
//                                   imu_extrinsics.param.data(),
//                                   cam_extrinsics.param.data(),
//                                   cam.param.data(),
//                                   td.param.data()};
//   vec2_t r;
//   Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J0;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J2;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J3;
//   Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J4;
//   Eigen::Vector2d J5;
//   std::vector<double *> jacobians =
//       {J0.data(), J1.data(), J2.data(), J3.data(), J4.data(), J5.data()};
//
//   // Baseline
//   err.Evaluate(params.data(), r.data(), jacobians.data());
//   print_vector("r", r);
//
//   double step = 1e-8;
//   double threshold = 1e-5;
//
//   // -- Test fiducial jacobian
//   {
//     mat_t<2, 2> fdiff;
//
//     for (int i = 0; i < 2; i++) {
//       vec2_t r_fd;
//       fiducial.perturb(i, step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       fiducial.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J0", fdiff, J0, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J0", J0);
//   }
//
//   // -- Test sensor-pose jacobian
//   {
//     mat_t<2, 6> fdiff;
//
//     for (int i = 0; i < 6; i++) {
//       // Forward difference
//       // vec2_t r_fd;
//       // sensor_pose.perturb(i, step);
//       // err.Evaluate(params.data(), r_fd.data(), nullptr);
//       // sensor_pose.perturb(i, -step);
//       // fdiff.col(i) = (r_fd - r) / step;
//
//       // Central finite difference
//       vec2_t r_fd;
//       sensor_pose.perturb(i, 0.5 * step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       sensor_pose.perturb(i, -0.5 * step);
//
//       vec2_t r_bd;
//       sensor_pose.perturb(i, -0.5 * step);
//       err.Evaluate(params.data(), r_bd.data(), nullptr);
//       sensor_pose.perturb(i, 0.5 * step);
//
//       fdiff.col(i) = (r_fd - r_bd) / step;
//     }
//
//     const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
//   }
//
//   // -- Test imu-extrinsics jacobian
//   {
//     const mat_t<2, 6> J2_min = J2.block(0, 0, 2, 6);
//
//     mat_t<2, 6> fdiff;
//     for (int i = 0; i < 6; i++) {
//       imu_extrinsics.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       imu_extrinsics.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J2", fdiff, J2_min, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J2", J2_min);
//   }
//
//   // -- Test cam extrinsics jacobian
//   {
//     const mat_t<2, 6> J3_min = J3.block(0, 0, 2, 6);
//
//     mat_t<2, 6> fdiff;
//     for (int i = 0; i < 6; i++) {
//       cam_extrinsics.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam_extrinsics.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J3", fdiff, J3_min, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J3", J3_min);
//   }
//
//   // -- Test camera-parameters jacobian
//   {
//     mat_t<2, 8> fdiff;
//
//     for (int i = 0; i < 8; i++) {
//       cam.perturb(i, step);
//       vec2_t r_fd;
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J4", fdiff, J4, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J4", J4);
//   }
//
//   // -- Test time delay
//   {
//     mat_t<2, 1> fdiff;
//
//     td.perturb(0, step);
//     vec2_t r_fd;
//     err.Evaluate(params.data(), r_fd.data(), nullptr);
//     fdiff.col(0) = (r_fd - r) / step;
//     td.perturb(0, -step);
//
//     MU_CHECK(check_jacobian("J5", fdiff, J5, threshold, true) == 0);
//     print_matrix("fdiff", fdiff);
//     print_matrix("J5", J5);
//   }
//
//   return 0;
// }

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
  MU_CHECK(calib.cam_params[cam_idx]->fixed == true);
  MU_CHECK(calib.cam_exts[cam_idx] != nullptr);
  MU_CHECK(calib.cam_exts[cam_idx]->fixed == true);

  return 0;
}

int test_calib_vi() {
  const auto timeline = setup_test_data();

  // Camera
  const int res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  vec4_t cam0_proj_params;
  vec4_t cam0_dist_params;
  vec4_t cam1_proj_params;
  vec4_t cam1_dist_params;
  mat4_t T_BC0;
  mat4_t T_BC1;
  // -- Custom calibration values
  // config_t config{"/tmp/calib-stereo.yaml"};
  // parse(config, "cam0.proj_params", cam0_proj_params);
  // parse(config, "cam0.dist_params", cam0_dist_params);
  // parse(config, "cam1.proj_params", cam1_proj_params);
  // parse(config, "cam1.dist_params", cam1_dist_params);
  // parse(config, "T_body0_cam0", T_BC0);
  // parse(config, "T_body0_cam1", T_BC1);
  // -- Euroc Calibration values
  cam0_proj_params << 458.654, 457.296, 367.215, 248.375;
  cam0_dist_params << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  cam1_proj_params << 457.587, 456.134, 379.999, 255.238;
  cam1_dist_params << -0.28368365, 0.07451284, -0.00010473, -3.555e-05;
  // clang-format off
  T_BC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
          0.0, 0.0, 0.0, 1.0;
  T_BC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
          0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
          -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
          0.0, 0.0, 0.0, 1.0;

  mat4_t T_C0C1;
  T_C0C1 = T_BC0.inverse() * T_BC1; // Camera-Camera extrinsics
  T_BC0 = I(4);                     // Set cam0 as body frame
  T_BC1 = T_C0C1;                   // Cam1 extrinsics
  // clang-format on

  // Imu
  imu_params_t imu_params;
  imu_params.rate = 200.0;
  imu_params.sigma_a_c = 0.002;
  imu_params.sigma_g_c = 1.6968e-04;
  imu_params.sigma_aw_c = 0.003;
  imu_params.sigma_gw_c = 1.9393e-05;
  imu_params.g = 9.81007;
  // clang-format off
  mat4_t T_BS;
  T_BS << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Setup VI calibrator
  calib_vi_t calib;
  calib.add_imu(imu_params, T_BS);
  calib.add_camera(0,
                   res,
                   proj_model,
                   dist_model,
                   cam0_proj_params,
                   cam0_dist_params,
                   T_BC0,
                   true,
                   true);
  calib.add_camera(1,
                   res,
                   proj_model,
                   dist_model,
                   cam1_proj_params,
                   cam1_dist_params,
                   T_BC1,
                   true,
                   true);

  LOG_INFO("Adding data to problem ...");
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
        const auto &acc = imu_event->acc;
        const auto &gyr = imu_event->gyr;
        calib.add_measurement(ts, acc, gyr);
      }
    }
  }
  calib.solve();
  calib.save_results("/tmp/calib_results.yaml");
  calib.save_estimates("/tmp");

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_fiducial_error);
  // MU_ADD_TEST(test_fiducial_td_error);

  MU_ADD_TEST(test_calib_vi_add_imu);
  MU_ADD_TEST(test_calib_vi_add_camera);
  MU_ADD_TEST(test_calib_vi);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
