#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_mono.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TARGET_CONFIG TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define CAM0_DATA "/data/euroc/calib/cam_april/mav0/cam0/data"
#define CAM0_GRIDS "/tmp/aprilgrid_test/mono/cam0"

int test_calib_mono_solve() {
  // Setup test data
  test_data_t test_data = setup_test_data();

  // Setup camera intrinsics and distortion
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      4, 4};
  if (cam.initialize(test_data.grids0) == false) {
    FATAL("Failed to inialize camera!");
  }

  // Test
  calib_mono_data_t data{test_data.grids0, cam};
  MU_CHECK(calib_mono_solve<pinhole_radtan4_t>(data) == 0);

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

int test_calib_mono_inc_solve() {
  // Setup test data
  test_data_t test_data = setup_test_data();

  // Setup camera intrinsics and distortion
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam{id, cam_idx, cam_res,
                      proj_model, dist_model,
                      4, 4};
  if (cam.initialize(test_data.grids0) == false) {
    FATAL("Failed to inialize camera!");
  }

  // Test
  calib_mono_data_t data{test_data.grids0, cam};
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
  MU_ADD_TEST(test_calib_mono_solve);
  MU_ADD_TEST(test_calib_mono_inc_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
