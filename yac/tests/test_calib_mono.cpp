#include "munit.hpp"
#include "calib_mono.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TARGET_CONFIG TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define CAM0_DATA "/data/euroc/calib/cam_april/mav0/cam0/data"
#define CAM0_GRIDS "/tmp/aprilgrid_test/mono/cam0"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, TARGET_CONFIG) != 0) {
    FATAL("Failed to load calib target [%s]!", TARGET_CONFIG);
  }

  // Test preprocess data
  auto t_start = tic();
  int retval = preprocess_camera_data(target, CAM0_DATA, CAM0_GRIDS);
  if (retval == -1) {
    FATAL("Failed to preprocess camera data!");
  }
  printf("Processing calibration data took: %f [s]\n", toc(&t_start));
}

// int test_calib_mono_residual() {
//   // Test load
//   aprilgrids_t aprilgrids;
//   int retval = load_camera_calib_data(CAM0_GRIDS, aprilgrids);
//   MU_CHECK(retval == 0);
//   MU_CHECK(aprilgrids.size() > 0);
//   MU_CHECK(aprilgrids[0].nb_detections > 0);
//
//   // Setup camera intrinsics and distortion
//   const id_t id = 0;
//   const int cam_idx = 0;
//   const int cam_res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const double fx = pinhole_focal(cam_res[0], 98.0);
//   const double fy = pinhole_focal(cam_res[1], 73.0);
//   const double cx = cam_res[0] / 2.0;
//   const double cy = cam_res[1] / 2.0;
//   const vec4_t proj_params{fx, fy, cx, cy};
//   const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
//   camera_params_t cam{id, cam_idx, cam_res,
//                       proj_model, dist_model,
//                       proj_params, dist_params};
//
//   // Measurement covariance
//   mat2_t covar = pow(0.5, 2) * I(2);
//
//   // Evaluate residuals
//   for (size_t i = 0; i < aprilgrids.size(); i++) {
//     const auto grid = aprilgrids[i];
//     const auto tag_id = grid.ids[0];
//     const int corner_id = 0;
//     const auto z = grid.keypoints[corner_id];
//     const auto T_CF = grid.T_CF;
//     pose_t rel_pose{0, 0, T_CF};
//
//     // Get object point
//     vec3_t r_FFi;
//     if (aprilgrid_object_point(grid, tag_id, corner_id, r_FFi) != 0) {
//       LOG_ERROR("Failed to calculate AprilGrid object point!");
//     }
//
//     // Form residual and call the functor
//     double residuals[2] = {0.0, 0.0};
//     const double *parameters[2] = {rel_pose.param.data(), cam.param.data()};
//     calib_mono_residual_t<pinhole_radtan4_t> residual{cam_res, r_FFi, z, covar};
//     residual.Evaluate(parameters, residuals, nullptr);
//
//     // Just some arbitrary test to make sure reprojection error is not larger
//     // than 100pixels in x or y direction. But often this can be the case ...
//     // printf("residuals: (%.2f, %.2f)\n", residuals[0], residuals[1]);
//     MU_CHECK(residuals[0] < 200.0);
//     MU_CHECK(residuals[1] < 200.0);
//   }
//
//   return 0;
// }

int test_calib_mono_solve() {
  // Setup camera intrinsics and distortion
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
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      proj_params, dist_params};

  // Load calibration data
  std::vector<aprilgrid_t> grids;
  MU_CHECK(load_camera_calib_data(CAM0_GRIDS, grids) == 0);
  MU_CHECK(grids.size() > 0);
  MU_CHECK(grids[0].nb_detections > 0);

  // Test
  mat4s_t T_CF;
  const mat2_t covar = I(2) * pow(0.5, 2);
  int retval = calib_mono_solve<pinhole_radtan4_t>(grids, covar, cam, T_CF);
  MU_CHECK(retval == 0);
  MU_CHECK(grids.size() == T_CF.size());

  // Make sure the estimated camera parameters are not far from EuRoC
  vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
  vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  vec4_t est_proj_params = cam.proj_params();
  vec4_t est_dist_params = cam.dist_params();

  print_vector("cam proj params [gnd]", gnd_proj_params);
  print_vector("cam proj params [est]", est_proj_params);
  print_vector("cam dist params [gnd]", gnd_dist_params);
  print_vector("cam dist params [est]", est_dist_params);

  MU_CHECK(fabs(gnd_proj_params[0] - est_proj_params[0]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[1] - est_proj_params[1]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[2] - est_proj_params[2]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[3] - est_proj_params[3]) < 5.0);

  MU_CHECK(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);

  return 0;
}

int test_calib_mono_inc_solve() {
  // Setup camera intrinsics and distortion
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
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      proj_params, dist_params};

  // Load calibration data
  std::vector<aprilgrid_t> grids;
  MU_CHECK(load_camera_calib_data(CAM0_GRIDS, grids) == 0);
  MU_CHECK(grids.size() > 0);
  MU_CHECK(grids[0].nb_detections > 0);

  // Test
  calib_mono_data_t data;
  data.covar = I(2) * pow(0.5, 2);
  data.cam_params = cam;
  data.grids = grids;

  MU_CHECK(calib_mono_inc_solve<pinhole_radtan4_t>(data) == 0);

  // Make sure the estimated camera parameters are not far from EuRoC
  vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
  vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  vec4_t est_proj_params = data.cam_params.proj_params();
  vec4_t est_dist_params = data.cam_params.dist_params();

  print_vector("cam proj params [gnd]", gnd_proj_params);
  print_vector("cam proj params [est]", est_proj_params);
  print_vector("cam dist params [gnd]", gnd_dist_params);
  print_vector("cam dist params [est]", est_dist_params);

  MU_CHECK(fabs(gnd_proj_params[0] - est_proj_params[0]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[1] - est_proj_params[1]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[2] - est_proj_params[2]) < 5.0);
  MU_CHECK(fabs(gnd_proj_params[3] - est_proj_params[3]) < 5.0);

  MU_CHECK(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
  MU_CHECK(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);

  return 0;
}

void test_suite() {
  test_setup();

  // MU_ADD_TEST(test_calib_mono_residual);
  MU_ADD_TEST(test_calib_mono_solve);
  // MU_ADD_TEST(test_calib_mono_inc_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
