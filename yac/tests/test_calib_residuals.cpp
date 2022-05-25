#include "munit.hpp"
#include "calib_residuals.hpp"

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

int test_eval() {
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
  mat_t<8, 8> covar = I(8);
  prior_t r(&cam_params, covar);
  r.eval();

  return 0;
}

int test_prior() {
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
  mat_t<8, 8> covar = I(8);
  prior_t r(&cam_params, covar);
  MU_CHECK(r.check_jacs(0, "J_prior"));

  return 0;
}

int test_pose_prior() {
  const vec3_t r{0.1, 0.2, 0.3};
  const vec3_t rpy{0.1, 0.2, 0.3};
  const quat_t q = euler2quat(rpy);

  pose_t pose{0, tf(q, r)};
  mat_t<6, 6> covar = I(6);
  pose_prior_t prior(&pose, covar);
  MU_CHECK(prior.check_jacs(0, "J_pose"));

  return 0;
}

int test_reproj_residual() {
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

  // Form reprojection residual
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

  reproj_residual_t r(&cam_geom,
                      &cam_params,
                      &cam_exts,
                      &rel_pose,
                      corner,
                      keypoints[0],
                      I(2));

  MU_CHECK(r.check_jacs(0, "J_cam_exts"));
  MU_CHECK(r.check_jacs(1, "J_rel_pose"));
  MU_CHECK(r.check_jacs(2, "J_cam"));

  return 0;
}

int test_fiducial_residual() {
  // Fiducial pose in world frame
  // clang-format off
  mat4_t T_WF;
  T_WF << 0.0, 0.0, 1.0, -1.0,
          1.0, 0.0, 0.0, -0.4,
          0.0, 1.0, 0.0, -0.5,
          0.0, 0.0, 0.0, 1.0;
  T_WF = tf_perturb_rot(T_WF, 0.01, 0);
  fiducial_t fiducial{T_WF};
  // clang-format on

  // Imu pose in world frame
  // clang-format off
  mat4_t T_WS;
  T_WS << -1.0, 0.0, 0.0, 0.1,
          0.0, -1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  T_WS = tf_perturb_rot(T_WS, 0.01, 1);
  pose_t pose{0, T_WS};
  // ^ Note: Due to numerical stability issues the translation component
  // cannot be 0 for checking jacobians
  // clang-format on

  // Camera extrinsics
  // clang-format off
  mat4_t T_BC0;
  T_BC0 << 0.0, -1.0, 0.0, 0.001,
           1.0, 0.0, 0.0, 0.001,
           0.0, 0.0, 1.0, 0.001,
           0.0, 0.0, 0.0, 1.0;
  T_BC0 = tf_perturb_rot(T_BC0, 0.01, 2);
  extrinsics_t cam_exts{T_BC0};
  // clang-format on

  // Imu extrinsics
  // clang-format off
  mat4_t T_BS;
  T_BS << 0.0, 0.0, 1.0, 0.001,
          0.0, -1.0, 0.0, 0.001,
          1.0, 0.0, 0.0, 0.001,
          0.0, 0.0, 0.0, 1.0;
  T_BS = tf_perturb_rot(T_BS, -0.01, 2);
  extrinsics_t imu_exts{T_BS};
  // clang-format on

  // Get AprilGrid data
  aprilgrid_t grid{0, 6, 6, 0.088, 0.3};
  const int tag_id = 0;
  const int corner_idx = 2;
  const timestamp_t ts = 0;
  const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);

  // Camera parameters
  pinhole_radtan4_t cam_geom;
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

  // Project image point
  const mat2_t covar = I(2);
  const mat4_t T_C0B = T_BC0.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_C0Fi = tf_point(T_C0B * T_BS * T_SW * T_WF, r_FFi);
  vec2_t z = zeros(2, 1);
  cam_geom.project(cam_res, cam_params.param, r_C0Fi, z);

  // Form fiducial residual
  fiducial_residual_t r(ts,
                        &cam_geom,
                        &cam_params,
                        &cam_exts,
                        &imu_exts,
                        &fiducial,
                        &pose,
                        tag_id,
                        corner_idx,
                        r_FFi,
                        z,
                        covar);

  MU_CHECK(r.check_jacs(0, "J_fiducial"));
  MU_CHECK(r.check_jacs(1, "J_pose"));
  MU_CHECK(r.check_jacs(2, "J_imu_exts"));
  MU_CHECK(r.check_jacs(2, "J_cam_exts"));
  MU_CHECK(r.check_jacs(3, "J_cam"));

  return 0;
}

int test_marg_residual() {
  // Fiducial corners
  const calib_target_t calib_target;
  fiducial_corners_t corners{calib_target};

  // Camera parameters
  pinhole_radtan4_t cam_geom;
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

  // Camera extrinsics
  mat4_t T_C0Ci = I(4);
  extrinsics_t cam_exts{T_C0Ci};

  // Form reprojection residuals
  std::vector<pose_t> rel_poses;
  std::vector<reproj_residual_t *> reproj_residuals;
  auto cam_grids = setup_test_data();

  for (size_t k = 0; k < cam_grids[0].size(); k++) {
    if (cam_grids[0][k].detected) {
      const auto &grid = cam_grids[0][k];

      // Estimate camera fiducial relative pose
      mat4_t T_CiF;
      if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      // Get measurements
      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;
      vec3s_t object_points;
      grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

      // Form reprojection residual
      mat4_t T_C0F = T_C0Ci * T_CiF;
      rel_poses.emplace_back(grid.timestamp, T_C0F);
      auto corner = corners.get_corner(tag_ids[0], corner_indicies[0]);
      const mat2_t covar = I(2);
      reproj_residuals.push_back(new reproj_residual_t{&cam_geom,
                                                       &cam_params,
                                                       &cam_exts,
                                                       &rel_poses.back(),
                                                       corner,
                                                       keypoints[0],
                                                       covar});
    }
  }

  // Form marginalization residual
  marg_residual_t *marg_residual = new marg_residual_t();
  for (auto res : reproj_residuals) {
    marg_residual->add(res);
  }

  marg_residual_t marg_copy;
  marg_copy.add(marg_residual);

  // // Clean up
  // for (auto res : reproj_residuals) {
  //   delete res;
  // }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_eval);
  MU_ADD_TEST(test_prior);
  MU_ADD_TEST(test_pose_prior);
  MU_ADD_TEST(test_reproj_residual);
  MU_ADD_TEST(test_fiducial_residual);
  MU_ADD_TEST(test_marg_residual);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
