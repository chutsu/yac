#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

int test_reproj_error() {
  // Load data
  const calib_target_t target;
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grids0";
  auto cam_grids = calib_data_preprocess(target, cam_paths, grids_path);

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
  const pinhole_radtan4_t cam_geom;

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

  // Evaluate reprojection error to form baseline
  std::vector<double *> params = {
      cam_exts.param.data(),
      rel_pose.param.data(),
      corner->param.data(),
      cam_params.param.data(),
  };
  vec2_t r;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J2;
  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J3;
  std::vector<double *> jacobians = {J0.data(),
                                     J1.data(),
                                     J2.data(),
                                     J3.data()};

  err.Evaluate(params.data(), r.data(), jacobians.data());

  // Check Jacobians
  double step = 1e-8;
  double threshold = 1e-4;

  // -- Test cam exts T_C0Ci jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      vec2_t r_fd;
      cam_exts.perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam_exts.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J0", fdiff, err.J0_min, threshold, true) == 0);
  }

  // -- Test relative pose T_C0F jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      vec2_t r_fd;
      rel_pose.perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      rel_pose.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J1", fdiff, err.J1_min, threshold, true) == 0);
  }

  // -- Test fiducial corner jacobian
  {
    mat_t<2, 3> fdiff;

    auto corner = corners.get_corner(tag_ids[0], corner_indicies[0]);
    for (int i = 0; i < 3; i++) {
      vec2_t r_fd;
      corner->perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      corner->perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J2", fdiff, err.J2_min, threshold, true) == 0);
  }

  // -- Test camera-parameters jacobian
  {
    mat_t<2, 8> fdiff;

    for (int i = 0; i < 8; i++) {
      vec2_t r_fd;
      cam_params.perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam_params.perturb(i, -step);
    }

    MU_CHECK(check_jacobian("J3", fdiff, err.J3_min, threshold, true) == 0);
  }

  return 0;
}

int test_calib_view() {
  // Load data
  const calib_target_t target;
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grids0";
  auto cam_grids = calib_data_preprocess(target, cam_paths, grids_path);

  // Setup
  ceres::Problem problem;
  ceres::CauchyLoss loss(1.0);

  calib_target_t calib_target;
  fiducial_corners_t corners{calib_target};

  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  pinhole_radtan4_t cam_geom;
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);

  auto grid = cam_grids[0][0];
  const mat4_t T_C0F = I(4);
  const mat4_t T_C0Ci = I(4);
  pose_t extrinsics{0, T_C0Ci};
  pose_t rel_pose{0, T_C0F};

  calib_view_t(&problem,
               &loss,
               &corners,
               &cam_geom,
               &cam0,
               &extrinsics,
               &rel_pose,
               grid);

  return 0;
}

int test_initialize_camera() {
  // Load data
  const calib_target_t target;
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grids0";
  auto cam_grids = calib_data_preprocess(target, cam_paths, grids_path);

  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);
  pinhole_radtan4_t cam_geom;

  initialize_camera(target, cam_grids[0], &cam_geom, &cam0, true);
  return 0;
}

int test_calib_camera_find_nbv() {
  // Calibration target and camera parameters
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibration data
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grids0";
  const auto grids = calib_data_preprocess(calib_target, cam_paths, grids_path);

  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  for (const auto grid : grids.at(0)) {
    if (grid.detected) {
      cam0_grids.push_back(grid);
    }
    if (cam0_grids.size() >= 20) {
      break;
    }
  }
  for (const auto grid : grids.at(1)) {
    if (grid.detected) {
      cam1_grids.push_back(grid);
    }
    if (cam1_grids.size() >= 20) {
      break;
    }
  }

  // Calibrator
  calib_camera_t calib{calib_target};
  calib.enable_nbv = false;
  calib.enable_outlier_rejection = false;
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.add_camera_data(0, cam0_grids);
  calib.add_camera_data(1, cam1_grids);
  calib.solve();

  // Next best view poses
  auto nbv_poses = calib_nbv_poses(calib_target,
                                   calib.cam_geoms,
                                   calib.cam_params,
                                   calib.cam_exts);

  int cam_idx = 0;
  int nbv_idx = 0;
  calib.find_nbv(nbv_poses, cam_idx, nbv_idx);

  return 0;
}

int test_calib_camera() {
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grids0";
  const auto grids = calib_data_preprocess(calib_target, cam_paths, grids_path);

  calib_camera_t calib{calib_target};
  calib.add_camera_data(0, grids.at(0));
  calib.add_camera_data(1, grids.at(1));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_stats("/tmp/calib-stats.csv");

  return 0;
}

int test_calib_camera2() {
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.0772, 0.3};
  const int cam_res[2] = {640, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  const std::map<int, std::string> cam_paths = {{0, "/tmp/cam0/data"}};
  const std::string save_path = "/tmp/cam0/grid0";
  const auto grids = calib_data_preprocess(calib_target, cam_paths, save_path);

  calib_camera_t calib{calib_target};
  calib.add_camera_data(0, grids.at(0));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_reproj_error);
  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_initialize_camera);
  MU_ADD_TEST(test_calib_camera_find_nbv);
  MU_ADD_TEST(test_calib_camera);
  MU_ADD_TEST(test_calib_camera2);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
