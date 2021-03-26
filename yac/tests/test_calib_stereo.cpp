#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_stereo.hpp"
#include "marg_error.hpp"

namespace yac {

std::vector<camera_params_t> setup_cameras(const test_data_t &data) {
  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0{0, 0, res, proj_model, dist_model, 4, 4};
  camera_params_t cam1{1, 1, res, proj_model, dist_model, 4, 4};

  if (cam0.initialize(data.grids0) == false) {
    FATAL("Failed to inialize camera!");
  }
  if (cam1.initialize(data.grids1) == false) {
    FATAL("Failed to inialize camera!");
  }

  return {cam0, cam1};
}

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
  const int cam0_idx = 0;
  const int cam0_res[2] = {752, 480};
  const std::string cam0_proj_model = "pinhole";
  const std::string cam0_dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{param_id++, cam0_idx, cam0_res,
                             cam0_proj_model, cam0_dist_model,
                             proj_params, dist_params};
  const pinhole_radtan4_t cam{cam0_res, proj_params, dist_params};

  mat4_t T_CiF;
  if (grid.estimate(cam, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  mat4_t T_BCi = I(4);
  mat4_t T_BF = T_BCi * T_CiF;
  pose_t cam_extrinsics{param_id++, grid.timestamp, T_BCi};
  pose_t rel_pose{param_id++, grid.timestamp, T_BF};

  reproj_error_t<pinhole_radtan4_t> err(cam0_idx, cam0_res,
                                        tag_ids[1],
                                        corner_indicies[1],
                                        object_points[1],
                                        keypoints[1],
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

  // -- Test relative pose T_BF jacobian
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

  // -- Test cam extrinsics T_BCi jacobian
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

aprilgrids_t filter_aprilgrids(aprilgrids_t &grids) {

  aprilgrids_t filtered_grids;
  auto grid_i = grids[0];

  for (size_t j = 1; j < grids.size(); j++) {
    auto grid_j = grids[j];

    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t grid_i_keypoints;
    vec2s_t grid_j_keypoints;
    vec3s_t object_points;
    aprilgrid_t::common_measurements(grid_i, grid_j,
                                     tag_ids, corner_indicies,
                                     grid_i_keypoints,
                                     grid_j_keypoints,
                                     object_points);

    aprilgrid_t new_grid{grid_i.timestamp, 6, 6, 0.088, 0.3};
    for (size_t i = 0; i < tag_ids.size(); i++) {
      new_grid.add(tag_ids[i], corner_indicies[i], grid_i_keypoints[i]);
    }
    filtered_grids.push_back(new_grid);

    if ((j + 1) == grids.size()) {
      aprilgrid_t new_grid{grid_j.timestamp, 6, 6, 0.088, 0.3};
      for (size_t i = 0; i < tag_ids.size(); i++) {
        new_grid.add(tag_ids[i], corner_indicies[i], grid_j_keypoints[i]);
      }
      filtered_grids.push_back(new_grid);
    }

    grid_i = grid_j;
  }

  return filtered_grids;
}

int test_calib_stereo_solve() {
  // Test data
  test_data_t test_data = setup_test_data();
  auto cameras = setup_cameras(test_data);

  calib_data_t data;
  data.add_calib_target(test_data.target);
  data.add_camera(cameras[0]);
  data.add_camera(cameras[1]);
  data.add_camera_extrinsics(0);
  data.add_camera_extrinsics(1);
  data.add_grids(0, test_data.grids0);
  data.add_grids(1, test_data.grids1);
  data.preprocess_data();
  data.check_data();

  if (calib_stereo_solve<pinhole_radtan4_t>(data) != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  const std::string outpath = "/tmp/calib-stereo.yaml";
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", outpath.c_str());
  printf("\033[0m\n");
  if (save_results(outpath, data) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", outpath.c_str());
    return -1;
  }

  // Compare estimation to ground truth
  // -- cam0
  {
    vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
    vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    vec4_t est_proj_params = data.cam_params[0].proj_params();
    vec4_t est_dist_params = data.cam_params[0].dist_params();

    print_vector("cam0 proj params [gnd]", gnd_proj_params);
    print_vector("cam0 proj params [est]", est_proj_params);
    print_vector("cam0 dist params [gnd]", gnd_dist_params);
    print_vector("cam0 dist params [est]", est_dist_params);
    printf("\n");

    MU_CHECK(fabs(gnd_proj_params[0] - est_proj_params[0]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[1] - est_proj_params[1]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[2] - est_proj_params[2]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[3] - est_proj_params[3]) < 10.0);

    MU_CHECK(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);
  }
  // -- cam1
  {
    vec4_t gnd_proj_params{457.587, 456.134, 379.999, 255.238};
    vec4_t gnd_dist_params{-0.28368365,  0.07451284, -0.00010473, -3.55590700e-05};
    vec4_t est_proj_params = data.cam_params[1].proj_params();
    vec4_t est_dist_params = data.cam_params[1].dist_params();

    print_vector("cam1 proj params [gnd]", gnd_proj_params);
    print_vector("cam1 proj params [est]", est_proj_params);
    print_vector("cam1 dist params [gnd]", gnd_dist_params);
    print_vector("cam1 dist params [est]", est_dist_params);
    printf("\n");

    MU_CHECK(fabs(gnd_proj_params[0] - est_proj_params[0]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[1] - est_proj_params[1]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[2] - est_proj_params[2]) < 10.0);
    MU_CHECK(fabs(gnd_proj_params[3] - est_proj_params[3]) < 10.0);

    MU_CHECK(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
    MU_CHECK(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);
  }
  // -- cam1-cam0 extrinsics
  {
    // clang-format off
    mat4_t T_SC0;
    T_SC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0;
    mat4_t T_SC1;
    T_SC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0;
    const mat4_t T_C1C0_gnd = T_SC1.inverse() * T_SC0;

    // Autocal
    // mat4_t T_C0C1;
    // T_C0C1 <<
    // 0.999997, -0.002325, -0.001038, 0.109974,
    //   0.002310, 0.999900, -0.013935, -0.000233,
    //   0.001070, 0.013933, 0.999902, 0.000561,
    //   0.000000, 0.000000, 0.000000, 1.000000;
    // mat4_t T_C1C0 = T_C0C1.inverse();

    // // YAC
    // mat4_t T_C1C0;
    // T_C1C0 <<
    //   0.999996, 0.002270, 0.001470, -0.109979,
    //   -0.002290, 0.999900, 0.013957, 0.000455,
    //   -0.001438, -0.013960, 0.999902, -0.000306,
    //   0.000000, 0.000000, 0.000000, 1.000000;

    // Kalibr
    // mat4_t T_C1C0;
    // T_C1C0 <<
    //   0.9999974202424761, 0.002199344629743525, 0.000567795372244847, -0.11001351661213345,
    //   -0.0022069639325529495, 0.99990252413431, 0.013786643537443936, 0.0004142099766208941,
    //   -0.0005374184454731391, -0.01378786107515421, 0.999904798502527, -0.0006275221351493475,
    //   0.0, 0.0, 0.0, 1.0;

    // clang-format on
    const mat4_t T_BC0 = data.cam_exts[0].tf();
    const mat4_t T_BC1 = data.cam_exts[1].tf();
    const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;
    const vec3_t gnd_trans = tf_trans(T_C1C0_gnd);
    const vec3_t est_trans = tf_trans(T_C1C0);
    const vec3_t gnd_euler = rad2deg(quat2euler(tf_quat(T_C1C0_gnd)));
    const vec3_t est_euler = rad2deg(quat2euler(tf_quat(T_C1C0)));

    print_matrix("T_C1C0 [gnd]", T_C1C0_gnd);
    print_matrix("T_C1C0 [est]", T_C1C0);
    print_vector("trans (cam1-cam0) [gnd] [m]", gnd_trans);
    print_vector("trans (cam1-cam0) [est] [m]", est_trans);
    print_vector("rot (cam1-cam0) [gnd] [deg]", gnd_euler);
    print_vector("rot (cam1-cam0) [est] [deg]", est_euler);

    MU_CHECK(fabs(gnd_trans(0) - est_trans(0)) < 0.01);
    MU_CHECK(fabs(gnd_trans(1) - est_trans(1)) < 0.01);
    MU_CHECK(fabs(gnd_trans(2) - est_trans(2)) < 0.01);

    MU_CHECK(fabs(gnd_euler(0) - est_euler(0)) < 1.0);
    MU_CHECK(fabs(gnd_euler(1) - est_euler(1)) < 1.0);
    MU_CHECK(fabs(gnd_euler(2) - est_euler(2)) < 1.0);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_reproj_error);
  MU_ADD_TEST(test_calib_stereo_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
