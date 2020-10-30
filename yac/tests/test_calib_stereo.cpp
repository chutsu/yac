#include "munit.hpp"
#include "calib_stereo.hpp"

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

int test_calib_stereo_residual() {
  // Test load
  std::vector<aprilgrid_t> cam0_aprilgrids;
  std::vector<aprilgrid_t> cam1_aprilgrids;
  int retval = load_stereo_calib_data(CAM0_APRILGRID_DATA,
                                      CAM0_APRILGRID_DATA,
                                      cam0_aprilgrids,
                                      cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to local calibration data!");
    return -1;
  }
  MU_CHECK(retval == 0);
  MU_CHECK(cam0_aprilgrids.size() > 0);
  MU_CHECK(cam0_aprilgrids[0].ids.size() > 0);
  MU_CHECK(cam1_aprilgrids.size() > 0);
  MU_CHECK(cam1_aprilgrids[0].ids.size() > 0);
  MU_CHECK(cam0_aprilgrids.size() == cam1_aprilgrids.size());

  // Setup intrinsic and distortion initialization
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  // -- cam0 intrinsics and distortion
  const double cam0_fx = pinhole_focal(img_w, lens_hfov);
  const double cam0_fy = pinhole_focal(img_h, lens_vfov);
  const double cam0_cx = img_w / 2.0;
  const double cam0_cy = img_h / 2.0;
  const vec4_t cam0_proj_params{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
  const vec4_t cam0_dist_params{0.01, 0.0001, 0.0001, 0.0001};
  camera_params_t cam0{0, 0, cam_res,
                       "pinhole", "radtan4",
                       cam0_proj_params, cam0_dist_params};
  // -- cam1 intrinsics and distortion
  const double cam1_fx = pinhole_focal(img_w, lens_hfov);
  const double cam1_fy = pinhole_focal(img_h, lens_vfov);
  const double cam1_cx = img_w / 2.0;
  const double cam1_cy = img_h / 2.0;
  const vec4_t cam1_proj_params{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
  const vec4_t cam1_dist_params{0.01, 0.0001, 0.0001, 0.0001};
  camera_params_t cam1{1, 1, cam_res,
                       "pinhole", "radtan4",
                       cam1_proj_params, cam1_dist_params};

  // Setup cam0 cam1 extrinsics
  // clang-format off
  mat4_t T_C1C0;
  T_C1C0 << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
            -0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
            -0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
            0.0, 0.0, 0.0, 1.0;
  const pose_t extrinsics{0, 0, T_C1C0};
  // clang-format on

  // Measurement covariance matrix
  mat2_t covar = pow(0.5, 2) * I(2);

  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    // AprilGrid, keypoint and relative pose observed in cam0
    const auto &grid0 = cam0_aprilgrids[i];
    const auto &cam0_kp = grid0.keypoints[0];
    const auto &T_C0F = grid0.T_CF;
    const pose_t rel_pose{0, 0, T_C0F};

    // AprilGrid, keypoint and relative pose observed in cam1
    const auto &grid1 = cam1_aprilgrids[i];
    const auto &cam1_kp = grid1.keypoints[0];

    // Tag id and corner id
    const auto tag_id = grid0.ids[0];
    const int corner_id = 0;

    // Form residual and call the functor
    // -- Get the object point
    vec3_t r_FFi;
    if (aprilgrid_object_point(grid0, tag_id, corner_id, r_FFi) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object point!");
    }
    // -- Form residual
    const calib_stereo_residual_t<pinhole_radtan4_t> residual{
      cam_res, r_FFi, cam0_kp, cam1_kp, covar};

    // Calculate residual
    vec4_t r{0.0, 0.0, 0.0, 0.0};
    std::vector<const double *> params{rel_pose.param.data(),
                                       extrinsics.param.data(),
                                       cam0.param.data(),
                                       cam1.param.data()};
    residual.Evaluate(params.data(), r.data(), nullptr);

    // Just some arbitrary test to make sure reprojection error is not larger
    // than 300pixels in x or y direction. But often this can be the case ...
    // print_vector("r", r);
    MU_CHECK(fabs(r(0)) > 0.0);
    MU_CHECK(fabs(r(1)) > 0.0);
    MU_CHECK(fabs(r(2)) > 0.0);
    MU_CHECK(fabs(r(3)) > 0.0);
  }

  return 0;
}

int test_calib_stereo_solve() {
  // Load stereo calibration data
  aprilgrids_t cam0_aprilgrids;
  aprilgrids_t cam1_aprilgrids;
  int retval = 0;
  retval = load_stereo_calib_data(CAM0_APRILGRID_DATA,
                                  CAM1_APRILGRID_DATA,
                                  cam0_aprilgrids,
                                  cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to local calibration data!");
    return -1;
  }

  // Setup cameras
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  // -- cam0 intrinsics and distortion
  const double cam0_fx = pinhole_focal(img_w, lens_hfov);
  const double cam0_fy = pinhole_focal(img_h, lens_vfov);
  const double cam0_cx = img_w / 2.0;
  const double cam0_cy = img_h / 2.0;
  const vec4_t cam0_proj_params{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
  const vec4_t cam0_dist_params{0.01, 0.0001, 0.0001, 0.0001};
  camera_params_t cam0{0, 0, cam_res,
                       "pinhole", "radtan4",
                       cam0_proj_params, cam0_dist_params};
  // -- cam1 intrinsics and distortion
  const double cam1_fx = pinhole_focal(img_w, lens_hfov);
  const double cam1_fy = pinhole_focal(img_h, lens_vfov);
  const double cam1_cx = img_w / 2.0;
  const double cam1_cy = img_h / 2.0;
  const vec4_t cam1_proj_params{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
  const vec4_t cam1_dist_params{0.01, 0.0001, 0.0001, 0.0001};
  camera_params_t cam1{1, 1, cam_res,
                       "pinhole", "radtan4",
                       cam1_proj_params, cam1_dist_params};

  // Test
  const mat2_t covar = pow(0.5, 2) * I(2);
  mat4_t T_C1C0 = I(4);
  mat4s_t poses;
  retval = calib_stereo_solve<pinhole_radtan4_t>(cam0_aprilgrids,
                                                 cam1_aprilgrids,
                                                 covar,
                                                 &cam0,
                                                 &cam1,
                                                 &T_C1C0,
                                                 &poses);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Compare estimation to ground truth
  // -- cam0
  {
    vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
    vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    vec4_t est_proj_params = cam0.proj_params();
    vec4_t est_dist_params = cam0.dist_params();

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
    vec4_t est_proj_params = cam1.proj_params();
    vec4_t est_dist_params = cam1.dist_params();

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
  test_setup();
  MU_ADD_TEST(test_calib_stereo_residual);
  MU_ADD_TEST(test_calib_stereo_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
