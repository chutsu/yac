#include "munit.hpp"
#include "calib_camera.hpp"

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

int test_calib_view() {
  // Load data
  auto cam_grids = setup_test_data();

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
  const auto cam_grids = setup_test_data();
  const calib_target_t calib_target;
  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);
  pinhole_radtan4_t cam_geom;
  initialize_camera(calib_target, cam_grids.at(0), &cam_geom, &cam0, true);
  return 0;
}

int test_calib_camera_find_nbv() {
  // Calibration target and camera parameters
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  // Calibration data
  const auto grids = setup_test_data();

  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  for (const auto grid : grids.at(0)) {
    if (grid.detected) {
      cam0_grids.push_back(grid);
    }
    if (cam0_grids.size() >= 5) {
      break;
    }
  }
  for (const auto grid : grids.at(1)) {
    if (grid.detected) {
      cam1_grids.push_back(grid);
    }
    if (cam1_grids.size() >= 5) {
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
  printf("cam_idx: %d\n", cam_idx);
  printf("nbv_idx: %d\n", nbv_idx);
  calib.solve();

  return 0;
}

int test_calib_camera() {
  // Setup
  const calib_target_t calib_target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const auto grids = setup_test_data();

  // Calibrate
  calib_camera_t calib{calib_target};
  // calib.enable_nbv = false;
  calib.add_camera_data(0, grids.at(0));
  calib.add_camera_data(1, grids.at(1));
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_stats("/tmp/calib-stats.csv");

  return 0;
}

// int test_calib_camera2() {
//   const calib_target_t calib_target{"aprilgrid", 6, 6, 0.0772, 0.3};
//   const int cam_res[2] = {640, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//
//   const std::map<int, std::string> cam_paths = {{0, "/tmp/cam0/data"}};
//   const std::string save_path = "/tmp/cam0/grid0";
//   const auto grids = calib_data_preprocess(calib_target, cam_paths,
//   save_path);
//
//   calib_camera_t calib{calib_target};
//   calib.add_camera_data(0, grids.at(0));
//   calib.add_camera(0, cam_res, proj_model, dist_model);
//   calib.solve();
//   calib.save_results("/tmp/calib-results.yaml");
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_initialize_camera);
  MU_ADD_TEST(test_calib_camera_find_nbv);
  MU_ADD_TEST(test_calib_camera);
  // MU_ADD_TEST(test_calib_camera2);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
