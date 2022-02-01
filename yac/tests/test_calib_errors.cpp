#include "munit.hpp"
#include "calib_errors.hpp"

namespace yac {

std::map<int, aprilgrids_t> setup_test_data() {
  // Calibration data
  const calib_target_t calib_target;
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  return calib_data_preprocess(calib_target, cam_paths, grids_path);
}

timeline_t setup_camimu_test_data() {
  // Load calibration data
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> grid_paths = {
      {0, data_path + "/mav0/grid0/cam0"},
      {1, data_path + "/mav0/grid0/cam1"},
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
        const auto flag = cv::IMREAD_GRAYSCALE;
        const cv::Mat image = cv::imread(cam_event->img_path, flag);
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

int test_reproj_error() {
  // Load data
  auto cam_grids = setup_test_data();

  // Get 1 detected aprilgrid
  aprilgrid_t grid;
  for (size_t k = 0; k < cam_grids[0].size(); k++) {
    if (cam_grids[0][k].detected) {
      grid = cam_grids[0][k];
      break;
    }
  }

  // Form reprojection error
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{cam_idx,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};
  pinhole_radtan4_t cam_geom;

  mat4_t T_CiF;
  if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  const calib_target_t calib_target;
  mat4_t T_C0Ci = I(4);
  mat4_t T_C0F = T_C0Ci * T_CiF;
  pose_t cam_exts{grid.timestamp, T_C0Ci};
  pose_t rel_pose{grid.timestamp, T_C0F}; // Fiducial corners
  fiducial_corners_t corners{calib_target};
  auto corner = corners.get_corner(tag_ids[0], corner_indicies[0]);

  reproj_error_t err(&cam_geom,
                     &cam_params,
                     &cam_exts,
                     &rel_pose,
                     corner,
                     keypoints[0],
                     I(2));

  MU_CHECK(err.check_jacs(0, "J_cam_exts"));
  MU_CHECK(err.check_jacs(1, "J_rel_pose"));
  MU_CHECK(err.check_jacs(2, "J_fiducial"));
  MU_CHECK(err.check_jacs(3, "J_cam"));

  return 0;
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

int test_residual_info() {
  // Load data
  const calib_target_t calib_target;
  const std::string data_path = "/data/euroc/imu_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  auto cam_grids = calib_data_preprocess(calib_target, cam_paths, grids_path);

  // Get 1 detected aprilgrid
  aprilgrid_t grid;
  for (size_t k = 0; k < cam_grids[0].size(); k++) {
    if (cam_grids[0][k].detected) {
      grid = cam_grids[0][k];
      break;
    }
  }

  // Setup state-variables
  // -- Camera params
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string cam_proj_model = "pinhole";
  const std::string cam_dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  pinhole_radtan4_t cam_geom;
  camera_params_t cam_params{cam_idx,
                             cam_res,
                             cam_proj_model,
                             cam_dist_model,
                             proj_params,
                             dist_params};
  // -- Fiducial corner
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_idxs, keypoints, object_points);
  mat4_t T_CiF;
  if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }
  fiducial_corners_t corners{calib_target};
  auto corner = corners.get_corner(tag_ids[0], corner_idxs[0]);
  // -- Camera extrinsics
  mat4_t T_C0Ci = I(4);
  mat4_t T_C0F = T_C0Ci * T_CiF;
  pose_t cam_exts{grid.timestamp, T_C0Ci};
  // -- Relative pose
  pose_t rel_pose{grid.timestamp, T_C0F};

  // Form reprojection residual block
  reproj_error_t err(&cam_geom,
                     &cam_params,
                     &cam_exts,
                     &rel_pose,
                     corner,
                     keypoints[0],
                     I(2));

  // Marginalization block
  std::vector<param_t *> param_blocks;
  param_blocks.push_back((param_t *)&cam_params);
  param_blocks.push_back((param_t *)&cam_exts);
  param_blocks.push_back((param_t *)&rel_pose);
  param_blocks.push_back((param_t *)corner);
  residual_info_t residual_info{&err, param_blocks};

  // Assert
  MU_CHECK(residual_info.cost_fn == &err);
  MU_CHECK(residual_info.loss_fn == nullptr);
  MU_CHECK(residual_info.param_blocks.size() == 4);

  return 0;
}

// struct view_t {
//   std::vector<camera_params_t *> cam_params;
//   std::vector<extrinsics_t *> cam_exts;
//   pose_t *pose;
//   std::map<int, std::vector<reproj_error_t<pinhole_radtan4_t> *>>
//   reproj_errors;
// };

// int test_marg_error() {
//   // Test data
//   test_data_t test_data = setup_test_data();
//
//   // Filter out grids not detected
//   const size_t max_grids = 10;
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//
//   const size_t grids0_end = test_data.grids0.size();
//   const size_t grids1_end = test_data.grids1.size();
//   size_t grid0_idx = 0;
//   size_t grid1_idx = 0;
//   while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
//     auto grid0 = test_data.grids0[grid0_idx];
//     auto grid1 = test_data.grids1[grid1_idx];
//
//     if (grid0.timestamp > grid1.timestamp) {
//       grid1_idx++;
//     } else if (grid0.timestamp < grid1.timestamp) {
//       grid0_idx++;
//     } else if (grid0.timestamp == grid1.timestamp) {
//       if (grid0.detected && grid1.detected) {
//         cam0_grids.push_back(grid0);
//         cam1_grids.push_back(grid1);
//       }
//       if (cam0_grids.size() == max_grids) {
//         break;
//       }
//       grid0_idx++;
//       grid1_idx++;
//     }
//   }
//
//   // Setup camera
//   const int cam_res[2] = {752, 480};
//   id_t param_id = 0;
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam0_params{param_id++,
//                               0,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//   camera_params_t cam1_params{param_id++,
//                               1,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//
//   // Setup problem
//   ceres::Problem problem;
//   marg_error_t *marg_error = new marg_error_t();
//
//   mat4_t T_BC0 = I(4);
//   mat4_t T_BC1;
//   T_BC1 << 0.99998, -0.00254, -0.00498, 0.10995, 0.00247, 0.99989, -0.01440,
//       -0.00025, 0.00502, 0.01439, 0.99988, 0.00062, 0.00000, 0.00000,
//       0.00000, 1.00000;
//   extrinsics_t cam0_exts{param_id++, T_BC0};
//   extrinsics_t cam1_exts{param_id++, T_BC1};
//
//   std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
//   std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
//   std::map<timestamp_t, pose_t *> rel_poses;
//   std::vector<view_t> views;
//
//   for (size_t k = 0; k < max_grids; k++) {
//     std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
//     view_t view;
//     view.cam_params = cam_params;
//     view.cam_exts = cam_exts;
//
//     for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
//       const auto grid = cam_grids[cam_idx];
//       const vec4_t proj_params = cam0_params.proj_params();
//       const vec4_t dist_params = cam0_params.dist_params();
//       const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
//
//       // Estimate relative pose (T_C0F)
//       mat4_t T_CiF;
//       if (grid->estimate(cam, T_CiF) != 0) {
//         FATAL("Failed to estimate relative pose!");
//       }
//       mat4_t T_BCi = cam_exts[cam_idx]->tf();
//       mat4_t T_BF = T_BCi * T_CiF;
//
//       // Form relative pose
//       const auto ts = grid->timestamp;
//       pose_t *rel_pose;
//       if (rel_poses.count(ts) == 0) {
//         rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
//         rel_poses[ts] = rel_pose;
//       } else {
//         rel_pose = rel_poses[ts];
//       }
//       view.pose = rel_pose;
//
//       // Form reprojection residual block
//       std::vector<int> tag_ids;
//       std::vector<int> corner_indicies;
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       grid->get_measurements(tag_ids,
//                              corner_indicies,
//                              keypoints,
//                              object_points);
//
//       for (size_t i = 0; i < tag_ids.size(); i++) {
//         auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
//                                                          cam_res,
//                                                          tag_ids[i],
//                                                          corner_indicies[i],
//                                                          object_points[i],
//                                                          keypoints[i],
//                                                          I(2));
//
//         view.reproj_errors[cam_idx].push_back(err);
//         problem.AddResidualBlock(err,
//                                  nullptr,
//                                  rel_pose->param.data(),
//                                  cam_exts[cam_idx]->param.data(),
//                                  cam_params[cam_idx]->param.data());
//       }
//     }
//
//     views.push_back(view);
//   }
//
//   printf("[before marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[before marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   // Marginalize
//   const auto view = views[0];
//   for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
//     for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
//       view.pose->marginalize = true;
//       std::vector<param_t *> param_blocks;
//       param_blocks.push_back((param_t *)view.pose);
//       param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
//       param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
//       marg_error->add(reproj_error, param_blocks);
//     }
//   }
//   marg_error->marginalize(problem);
//   problem.AddResidualBlock(marg_error, nullptr,
//   marg_error->get_param_ptrs());
//
//   printf("[after marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[after marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   print_vector("cam0_params", cam_params[0]->param);
//   print_vector("cam0_exts", cam_exts[0]->param);
//   print_vector("cam1_params", cam_params[1]->param);
//   print_vector("cam1_exts", cam_exts[1]->param);
//
//   // Solve
//   problem.SetParameterBlockConstant(cam_exts[0]->param.data());
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = true;
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << std::endl;
//   std::cout << std::endl;
//
//   print_vector("cam0_params", cam_params[0]->param);
//   print_vector("cam0_exts", cam_exts[0]->param);
//   print_vector("cam1_params", cam_params[1]->param);
//   print_vector("cam1_exts", cam_exts[1]->param);
//
//   return 0;
// }
//
// int test_marg_error_check_gradients() {
//   // Test data
//   test_data_t test_data = setup_test_data();
//
//   // Filter out grids not detected
//   const size_t max_grids = 10;
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//
//   const size_t grids0_end = test_data.grids0.size();
//   const size_t grids1_end = test_data.grids1.size();
//   size_t grid0_idx = 0;
//   size_t grid1_idx = 0;
//   while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
//     auto grid0 = test_data.grids0[grid0_idx];
//     auto grid1 = test_data.grids1[grid1_idx];
//
//     if (grid0.timestamp > grid1.timestamp) {
//       grid1_idx++;
//     } else if (grid0.timestamp < grid1.timestamp) {
//       grid0_idx++;
//     } else if (grid0.timestamp == grid1.timestamp) {
//       if (grid0.detected && grid1.detected) {
//         cam0_grids.push_back(grid0);
//         cam1_grids.push_back(grid1);
//       }
//       if (cam0_grids.size() == max_grids) {
//         break;
//       }
//       grid0_idx++;
//       grid1_idx++;
//     }
//   }
//
//   // Setup camera
//   const int cam_res[2] = {752, 480};
//   id_t param_id = 0;
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam0_params{param_id++,
//                               0,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//   camera_params_t cam1_params{param_id++,
//                               1,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//
//   // Setup problem
//   ceres::Problem problem;
//   marg_error_t *marg_error = new marg_error_t();
//
//   mat4_t T_BC0 = I(4);
//   mat4_t T_BC1;
//   T_BC1 << 0.99998, -0.00254, -0.00498, 0.10995, 0.00247, 0.99989, -0.01440,
//       -0.00025, 0.00502, 0.01439, 0.99988, 0.00062, 0.00000, 0.00000,
//       0.00000, 1.00000;
//   extrinsics_t cam0_exts{param_id++, T_BC0};
//   extrinsics_t cam1_exts{param_id++, T_BC1};
//
//   std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
//   std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
//   std::map<timestamp_t, pose_t *> rel_poses;
//   std::vector<view_t> views;
//
//   for (size_t k = 0; k < max_grids; k++) {
//     std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
//     view_t view;
//     view.cam_params = cam_params;
//     view.cam_exts = cam_exts;
//
//     for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
//       const auto grid = cam_grids[cam_idx];
//       const vec4_t proj_params = cam0_params.proj_params();
//       const vec4_t dist_params = cam0_params.dist_params();
//       const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
//
//       // Estimate relative pose (T_C0F)
//       mat4_t T_CiF;
//       if (grid->estimate(cam, T_CiF) != 0) {
//         FATAL("Failed to estimate relative pose!");
//       }
//       mat4_t T_BCi = cam_exts[cam_idx]->tf();
//       mat4_t T_BF = T_BCi * T_CiF;
//
//       // Form relative pose
//       const auto ts = grid->timestamp;
//       pose_t *rel_pose;
//       if (rel_poses.count(ts) == 0) {
//         rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
//         rel_poses[ts] = rel_pose;
//       } else {
//         rel_pose = rel_poses[ts];
//       }
//       view.pose = rel_pose;
//
//       // Form reprojection residual block
//       std::vector<int> tag_ids;
//       std::vector<int> corner_indicies;
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       grid->get_measurements(tag_ids,
//                              corner_indicies,
//                              keypoints,
//                              object_points);
//
//       for (size_t i = 0; i < tag_ids.size(); i++) {
//         auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
//                                                          cam_res,
//                                                          tag_ids[i],
//                                                          corner_indicies[i],
//                                                          object_points[i],
//                                                          keypoints[i],
//                                                          I(2));
//
//         view.reproj_errors[cam_idx].push_back(err);
//         problem.AddResidualBlock(err,
//                                  nullptr,
//                                  rel_pose->param.data(),
//                                  cam_exts[cam_idx]->param.data(),
//                                  cam_params[cam_idx]->param.data());
//       }
//     }
//
//     views.push_back(view);
//   }
//
//   printf("[before marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[before marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   // Marginalize
//   const auto view = views[0];
//   for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
//     for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
//       view.pose->marginalize = true;
//       std::vector<param_t *> param_blocks;
//       param_blocks.push_back((param_t *)view.pose);
//       param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
//       param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
//       marg_error->add(reproj_error, param_blocks);
//     }
//   }
//   marg_error->marginalize(problem, true);
//   problem.AddResidualBlock(marg_error, nullptr,
//   marg_error->get_param_ptrs());
//
//   printf("[after marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[after marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   std::vector<double *> param_ptrs = marg_error->get_param_ptrs();
//
//   const auto r_size = marg_error->e0_.size();
//   vecx_t r = zeros(r_size, 1);
//
//   std::vector<param_t *> params = marg_error->get_params();
//   double **jacobians = new double *[params.size()];
//   for (size_t i = 0; i < params.size(); i++) {
//     auto param = params[i];
//     jacobians[i] =
//         (double *)malloc(sizeof(double) * r.size() * param->global_size);
//   }
//   marg_error->Evaluate(param_ptrs.data(), r.data(), jacobians);
//   for (const auto param : params) {
//     printf("param[%s]\n", param->type.c_str());
//   }
//
//   // Check jacobians
//   double step = 1e-8;
//   double threshold = 1e-4;
//
//   printf("marg_error->x0.size(): %ld\n", marg_error->x0_.size());
//   mat2csv("/tmp/e0.csv", marg_error->e0_);
//   mat2csv("/tmp/J0.csv", marg_error->J0_);
//
//   for (size_t param_idx = 0; param_idx < params.size(); param_idx++) {
//     auto &param = params[param_idx];
//
//     matx_t fdiff = zeros(r_size, param->local_size);
//     for (int i = 0; i < param->local_size; i++) {
//       vecx_t r_fd = zeros(r_size, 1);
//       param->perturb(i, step);
//       marg_error->Evaluate(param_ptrs.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       param->perturb(i, -step);
//     }
//
//     auto J_name = "J" + std::to_string(param_idx);
//     auto J_rows = r_size;
//     auto J_cols = param->global_size;
//     auto J_min_cols = param->local_size;
//
//     Eigen::Map<matx_row_major_t> J(jacobians[param_idx], J_rows, J_cols);
//     const matx_t J_min = J.block(0, 0, J_rows, J_min_cols);
//
//     MU_CHECK(check_jacobian(J_name, fdiff, J_min, threshold, true) == 0);
//   }
//
//   return 0;
// }
//
// int test_marg_error_swe() {
//   // Test data
//   test_data_t test_data = setup_test_data();
//
//   // Filter out grids not detected
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//
//   const size_t grids0_end = test_data.grids0.size();
//   const size_t grids1_end = test_data.grids1.size();
//   size_t grid0_idx = 0;
//   size_t grid1_idx = 0;
//   while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
//     auto grid0 = test_data.grids0[grid0_idx];
//     auto grid1 = test_data.grids1[grid1_idx];
//
//     if (grid0.timestamp > grid1.timestamp) {
//       grid1_idx++;
//     } else if (grid0.timestamp < grid1.timestamp) {
//       grid0_idx++;
//     } else if (grid0.timestamp == grid1.timestamp) {
//       if (grid0.detected && grid1.detected) {
//         cam0_grids.push_back(grid0);
//         cam1_grids.push_back(grid1);
//       }
//       grid0_idx++;
//       grid1_idx++;
//     }
//   }
//
//   // Setup camera
//   const int cam_res[2] = {752, 480};
//   id_t param_id = 0;
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam0_params{param_id++,
//                               0,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//   camera_params_t cam1_params{param_id++,
//                               1,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//
//   // Setup camera extrinsics
//   mat4_t T_BC0 = I(4);
//   mat4_t T_BC1 = I(4);
//   // T_BC1 << 0.99998, -0.00254, -0.00498,  0.10995,
//   //          0.00247,  0.99989, -0.01440, -0.00025,
//   //          0.00502,  0.01439,  0.99988,  0.00062,
//   //          0.00000,  0.00000,  0.00000,  1.00000;
//   extrinsics_t cam0_exts{param_id++, T_BC0};
//   extrinsics_t cam1_exts{param_id++, T_BC1};
//
//   // Sliding window estimation
//   std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
//   std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
//   std::map<timestamp_t, pose_t *> rel_poses;
//   std::deque<view_t> sliding_window;
//
//   ceres::Problem problem;
//   PoseLocalParameterization pose_plus;
//
//   problem.AddParameterBlock(cam_exts[0]->param.data(), 7);
//   problem.AddParameterBlock(cam_exts[1]->param.data(), 7);
//   problem.SetParameterBlockConstant(cam_exts[0]->param.data());
//   problem.SetParameterization(cam_exts[0]->param.data(), &pose_plus);
//   problem.SetParameterization(cam_exts[1]->param.data(), &pose_plus);
//
//   marg_error_t *marg_error = new marg_error_t();
//   ceres::ResidualBlockId marg_error_id = nullptr;
//
//   size_t max_window_size = 10;
//   auto nb_cams = 2;
//   auto nb_grids = cam0_grids.size();
//
//   for (size_t k = 0; k < nb_grids; k++) {
//     std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
//     view_t view;
//     view.cam_params = cam_params;
//     view.cam_exts = cam_exts;
//
//     // Keep sliding window bounded
//     if (sliding_window.size() >= max_window_size) {
//       // Remove previous residual block if it exists
//       if (marg_error_id != nullptr) {
//         problem.RemoveResidualBlock(marg_error_id);
//
//         // Replace old marginalization error, and keep track of previous info
//         auto marg_error_new = new marg_error_t();
//         marg_error_new->add(marg_error, marg_error->get_params());
//         delete marg_error;
//         marg_error = marg_error_new;
//       }
//
//       // Pop the oldest view from the sliding window
//       const auto view = sliding_window.front();
//       sliding_window.pop_front();
//       // problem.RemoveParameterBlock(view.pose->param.data());
//
//       // Marginalize the oldest pose and add marginalization prior to problem
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
//           view.pose->marginalize = true;
//           std::vector<param_t *> param_blocks;
//           param_blocks.push_back((param_t *)view.pose);
//           param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
//           param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
//           marg_error->add(reproj_error, param_blocks);
//         }
//       }
//       marg_error->marginalize(problem);
//       marg_error_id = problem.AddResidualBlock(marg_error,
//                                                nullptr,
//                                                marg_error->get_param_ptrs());
//     }
//
//     // Add view to sliding window
//     for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
//       // Estimate relative pose (T_C0F)
//       const auto grid = cam_grids[cam_idx];
//       const vec4_t proj_params = cam0_params.proj_params();
//       const vec4_t dist_params = cam0_params.dist_params();
//       const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
//       mat4_t T_CiF;
//       if (grid->estimate(cam, T_CiF) != 0) {
//         FATAL("Failed to estimate relative pose!");
//       }
//       mat4_t T_BCi = cam_exts[cam_idx]->tf();
//       mat4_t T_BF = T_BCi * T_CiF;
//
//       // Form relative pose
//       const auto ts = grid->timestamp;
//       pose_t *rel_pose;
//       if (rel_poses.count(ts) == 0) {
//         rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
//         rel_poses[ts] = rel_pose;
//       } else {
//         rel_pose = rel_poses[ts];
//       }
//       view.pose = rel_pose;
//
//       // Form reprojection residual block for each observation from each
//       camera std::vector<int> tag_ids; std::vector<int> corner_indicies;
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       grid->get_measurements(tag_ids,
//                              corner_indicies,
//                              keypoints,
//                              object_points);
//
//       for (size_t i = 0; i < tag_ids.size(); i++) {
//         auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
//                                                          cam_res,
//                                                          tag_ids[i],
//                                                          corner_indicies[i],
//                                                          object_points[i],
//                                                          keypoints[i],
//                                                          I(2));
//
//         view.reproj_errors[cam_idx].push_back(err);
//         problem.AddResidualBlock(err,
//                                  nullptr,
//                                  rel_pose->param.data(),
//                                  cam_exts[cam_idx]->param.data(),
//                                  cam_params[cam_idx]->param.data());
//       }
//       problem.SetParameterization(rel_pose->param.data(), &pose_plus);
//     }
//     sliding_window.push_back(view);
//
//     // Solve window
//     if (sliding_window.size() >= max_window_size) {
//       ceres::Solver::Options options;
//       options.max_num_iterations = 10;
//       ceres::Solver::Summary summary;
//       ceres::Solve(options, &problem, &summary);
//
//       // Obtain the calibration covariance matrix and calculate entropy. This
//       // is one way to make sure the marginalization error is working. If the
//       // marg_error_t() is working the entropy should be increasing over
//       time. matx_t covar; if (calib_covar(problem, cam0_params, cam1_params,
//       cam1_exts, covar) ==
//           0) {
//         printf("entropy: %f\t", entropy(covar));
//       }
//       std::cout << summary.BriefReport() << std::endl;
//     }
//   }
//
//   print_vector("cam0_params", cam_params[0]->param);
//   print_vector("cam0_exts", cam_exts[0]->param);
//   print_vector("cam1_params", cam_params[1]->param);
//   print_vector("cam1_exts", cam_exts[1]->param);
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_reproj_error);
  // MU_ADD_TEST(test_fiducial_error);
  // MU_ADD_TEST(test_fiducial_td_error);
  MU_ADD_TEST(test_residual_info);
  // MU_ADD_TEST(test_marg_error);
  // MU_ADD_TEST(test_marg_error_check_gradients);
  // MU_ADD_TEST(test_marg_error_swe);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
