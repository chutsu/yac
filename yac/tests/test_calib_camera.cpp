#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_camera.hpp"

namespace yac {

test_data_t test_data = setup_test_data();

int test_calib_view() {
  ceres::Problem problem;
  auto grid = test_data.grids0[0];
  auto cam0 = test_data.cam0;
  const mat4_t T_C0F = I(4);
  const mat4_t T_C0Ci = I(4);
  pose_t rel_pose{0, T_C0F};
  pose_t extrinsics{0, T_C0Ci};
  pinhole_radtan4_t cam_geom;
  calib_view_t(problem, grid, rel_pose, extrinsics, &cam_geom, cam0);

  return 0;
}

int test_initialize_camera() {
  pinhole_radtan4_t cam_geom;
  initialize_camera(test_data.grids1, &cam_geom, test_data.cam1, true);

  return 0;
}

int test_calib_camera() {
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  calib_camera_t calib;
  calib.add_calib_target(test_data.target);
  calib.add_camera_data(0, test_data.grids0);
  calib.add_camera_data(1, test_data.grids1);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();

  print_matrix("T_C0C0", calib.get_camera_extrinsics(0));
  print_matrix("T_C0C1", calib.get_camera_extrinsics(1));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_initialize_camera);
  MU_ADD_TEST(test_calib_camera);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
