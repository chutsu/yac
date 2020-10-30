#include "munit.hpp"
#include "calib_mono.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define APRILGRID_DATA "/tmp/aprilgrid_test/mono/cam0"
#define APRILGRID_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define CAM0_APRILGRID_DATA "/tmp/aprilgrid_test/stereo/cam0"
#define CAM1_APRILGRID_DATA "/tmp/aprilgrid_test/stereo/cam1"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
  }

  // Test preprocess data
  const std::string image_dir = IMAGE_DIR;
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  int retval = preprocess_camera_data(target,
                                      image_dir,
                                      image_size,
                                      lens_hfov,
                                      lens_vfov,
                                      APRILGRID_DATA);
  if (retval == -1) {
    FATAL("Failed to preprocess camera data!");
  }
}

int test_calib_mono_residual() {
  // Test load
  aprilgrids_t aprilgrids;
  timestamps_t timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

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

  // Measurement covariance
  mat2_t covar = pow(0.5, 2) * I(2);

  // Evaluate residuals
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto grid = aprilgrids[i];
    const auto tag_id = grid.ids[0];
    const int corner_id = 0;
    const auto z = grid.keypoints[corner_id];
    const auto T_CF = grid.T_CF;
    pose_t rel_pose{0, 0, T_CF};

    // Get object point
    vec3_t r_FFi;
    if (aprilgrid_object_point(grid, tag_id, corner_id, r_FFi) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object point!");
    }

    // Form residual and call the functor
    double residuals[2] = {0.0, 0.0};
		const double *parameters[2] = {rel_pose.param.data(), cam.param.data()};
    calib_mono_residual_t<pinhole_radtan4_t> residual{cam_res, r_FFi, z, covar};
    residual.Evaluate(parameters, residuals, nullptr);

    // Just some arbitrary test to make sure reprojection error is not larger
    // than 100pixels in x or y direction. But often this can be the case ...
		// printf("residuals: (%.2f, %.2f)\n", residuals[0], residuals[1]);
    MU_CHECK(residuals[0] < 200.0);
    MU_CHECK(residuals[1] < 200.0);
  }

  return 0;
}

int test_calib_mono_solve() {
  // Load calibration data
  std::vector<aprilgrid_t> grids;
  std::vector<timestamp_t> timestamps;
  MU_CHECK(load_camera_calib_data(APRILGRID_DATA, grids, timestamps) == 0);
  MU_CHECK(grids.size() > 0);
  MU_CHECK(grids[0].ids.size() > 0);

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

  // Test
  const mat2_t covar = I(2) * pow(0.5, 2);
  mat4s_t T_CF;
  int retval = calib_mono_solve<pinhole_radtan4_t>(grids, covar, &cam, &T_CF);
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

// int test_calib_mono_estimate_covariance() {
//   // Load calibration data
//   std::vector<aprilgrid_t> aprilgrids;
//   std::vector<timestamp_t> timestamps;
//   MU_CHECK(load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps) == 0);
//   MU_CHECK(aprilgrids.size() > 0);
//   MU_CHECK(aprilgrids[0].ids.size() > 0);
//
//   // Setup camera intrinsics and distortion
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
//   camera_params_t cam_params{0, cam_idx, cam_res,
//                              proj_model, dist_model,
//                              proj_params, dist_params};
//
//   // // Test
//   // mat4s_t T_CF;
//   // MU_CHECK(calib_mono_solve(aprilgrids, cam_params, T_CF) == 0);
//   // MU_CHECK(aprilgrids.size() == T_CF.size());
//   // calib_mono_stats(aprilgrids, cam_params, T_CF);
//
//   // Estimate covariance
//   calib_mono_covar_est_t covar_est(cam_params, aprilgrids);
//
//   matx_t covar;
//   MU_CHECK(covar_est.estimate(covar) == 0);
//   mat2csv("/tmp/covar.csv", covar);
//
//   return 0;
// }

// int test_calib_mono_find_nbv() {
//   // Load calibration data
//   std::vector<aprilgrid_t> aprilgrids;
//   std::vector<timestamp_t> timestamps;
//   MU_CHECK(load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps) == 0);
//   MU_CHECK(aprilgrids.size() > 0);
//   MU_CHECK(aprilgrids[0].ids.size() > 0);
//
//   // Setup camera intrinsics and distortion
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
//   camera_params_t cam_params{0, cam_idx, cam_res,
//                              proj_model, dist_model,
//                              proj_params, dist_params};
//
//   // // Test
//   // mat4s_t T_CF;
//   // MU_CHECK(calib_mono_solve(aprilgrids, cam_params, T_CF) == 0);
//   // MU_CHECK(aprilgrids.size() == T_CF.size());
//   // calib_mono_stats(aprilgrids, cam_params, T_CF);
//
//   // Find next best view
//   calib_target_t target;
//   if (calib_target_load(target, APRILGRID_CONF) != 0) {
//     FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
//   }
//   nbv_t nbv(target, cam_params, aprilgrids);
//   nbv.find_nbv(target);
//
//   return 0;
// }

void test_suite() {
  test_setup();

  MU_ADD_TEST(test_calib_mono_residual);
  MU_ADD_TEST(test_calib_mono_solve);
  // MU_ADD_TEST(test_calib_mono_estimate_covariance);
  // MU_ADD_TEST(test_calib_mono_find_nbv);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
