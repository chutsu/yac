#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_camera.hpp"

namespace yac {

int test_reproj_error() {
  test_data_t test_data = setup_test_data();

  aprilgrid_t grid;
  for (size_t i = 0; i < test_data.grids0.size(); i++) {
    if (test_data.grids0[i].detected) {
      grid = test_data.grids0[i];
      break;
    }
  }

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  id_t param_id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{param_id++, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};
  const pinhole_radtan4_t cam_geom;

  mat4_t T_CiF;
  if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  mat4_t T_C0Ci = I(4);
  mat4_t T_C0F = T_C0Ci * T_CiF;
  pose_t cam_extrinsics{param_id++, grid.timestamp, T_C0Ci};
  pose_t rel_pose{param_id++, grid.timestamp, T_C0F};

  reproj_error_t err(&cam_geom,
                     cam_idx,
                     cam_res,
                     tag_ids[0],
                     corner_indicies[0],
                     object_points[0],
                     keypoints[0],
                     I(2));

  std::vector<double *> params = {
    rel_pose.param.data(),
    cam_extrinsics.param.data(),
    cam_params.param.data(),
  };
  vec2_t r;

  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J2;
  std::vector<double *> jacobians = {J0.data(), J1.data(), J2.data()};

  // Baseline
  err.Evaluate(params.data(), r.data(), jacobians.data());
  print_vector("r", r);

  double step = 1e-8;
  double threshold = 1e-4;

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

    const mat_t<2, 6> J0_min = J0.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J0", fdiff, J0_min, threshold, true) == 0);
  }

  // -- Test cam extrinsics T_C0Ci jacobian
  {
    mat_t<2, 6> fdiff;

    for (int i = 0; i < 6; i++) {
      vec2_t r_fd;
      cam_extrinsics.perturb(i, step);
      err.Evaluate(params.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      cam_extrinsics.perturb(i, -step);
    }

    const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
    MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
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

    MU_CHECK(check_jacobian("J2", fdiff, J2, threshold, true) == 0);
  }

  return 0;
}

// int test_calib_view() {
//   test_data_t test_data = setup_test_data();
//
//   return 0;
// }

int test_calib_camera() {
  // Setup test data
  test_data_t test_data = setup_test_data();

  calib_data_t calib_data;
  calib_data.add_calib_target(test_data.target);
  calib_data.add_camera(test_data.cam0);
  calib_data.add_camera_extrinsics(0);
  calib_data.add_grids(0, test_data.grids0);

  calib_camera_t calib_camera{calib_data};
  calib_camera.solve();

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_reproj_error);
  // MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_calib_camera);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
