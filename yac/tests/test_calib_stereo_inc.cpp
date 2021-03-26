#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_stereo_inc.hpp"

namespace yac {

std::vector<camera_params_t> setup_cameras(const aprilgrids_t &grids0,
                                           const aprilgrids_t &grids1) {
  // Setup cameras
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0{0, 0, cam_res, proj_model, dist_model, 4, 4};
  camera_params_t cam1{1, 1, cam_res, proj_model, dist_model, 4, 4};

  if (cam0.initialize(grids0) == false) {
    FATAL("Failed to inialize camera!");
  }
  if (cam1.initialize(grids1) == false) {
    FATAL("Failed to inialize camera!");
  }

  return {cam0, cam1};
}

int test_calib_stereo_inc_solve() {
  // Setup
  test_data_t test_data = setup_test_data();

  // Test
  const auto target = test_data.target;
  auto grids0 = test_data.grids0;
  auto grids1 = test_data.grids1;
  auto cameras = setup_cameras(grids0, grids1);
  std::map<timestamp_t, pose_t> poses;  // T_BF
  auto cam0 = cameras[0];
  auto cam1 = cameras[1];
  mat2_t covar = I(2);

  calib_data_t data;
  data.add_calib_target(data.target);
  data.add_camera(cameras[0]);
  data.add_camera(cameras[1]);
  data.add_camera_extrinsics(0);
  data.add_camera_extrinsics(1);
  data.add_grids(0, test_data.grids0);
  data.add_grids(1, test_data.grids1);
  data.preprocess_data();
  data.check_data();

  calib_stereo_inc_solver_t<pinhole_radtan4_t> solver(data);

  const std::string results_fpath = "/tmp/calib-stereo.yaml";
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, data) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
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
    // clang-format on
    const mat4_t T_BC0 = data.cam_exts.at(0).tf();
    const mat4_t T_BC1 = data.cam_exts.at(1).tf();
    const mat4_t T_C1C0_est = T_BC1.inverse() * T_BC0;

    const vec3_t gnd_trans = tf_trans(T_C1C0_gnd);
    const vec3_t est_trans = tf_trans(T_C1C0_est);
    const vec3_t gnd_euler = rad2deg(quat2euler(tf_quat(T_C1C0_gnd)));
    const vec3_t est_euler = rad2deg(quat2euler(tf_quat(T_C1C0_est)));

    print_matrix("T_C1C0 [gnd]", T_C1C0_gnd);
    print_matrix("T_C1C0 [est]", T_C1C0_est);
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
  MU_ADD_TEST(test_calib_stereo_inc_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
