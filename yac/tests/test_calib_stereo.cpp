#include "munit.hpp"
#include "calib_stereo.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TARGET_CONFIG TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define CAM0_DATA "/data/euroc/calib/cam_april/mav0/cam0/data"
#define CAM1_DATA "/data/euroc/calib/cam_april/mav0/cam1/data"
// #define CAM0_DATA "/data/tum_vi/calib/dataset-calib-cam1_512_16/mav0/cam0/data"
// #define CAM1_DATA "/data/tum_vi/calib/dataset-calib-cam1_512_16/mav0/cam1/data"
#define CAM0_GRIDS "/tmp/aprilgrid_test/stereo/cam0"
#define CAM1_GRIDS "/tmp/aprilgrid_test/stereo/cam1"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, TARGET_CONFIG) != 0) {
    FATAL("Failed to load calib target [%s]!", TARGET_CONFIG);
  }

  // Test preprocess data
  if (preprocess_camera_data(target, CAM0_DATA, CAM0_GRIDS) != 0) {
    FATAL("Failed to preprocess cam0 data!");
  }
  if (preprocess_camera_data(target, CAM1_DATA, CAM1_GRIDS) != 0) {
    FATAL("Failed to preprocess cam1 data!");
  }
}

// int test_calib_stereo_residual() {
//   // Setup intrinsic and distortion initialization
//   const int img_w = 752;
//   const int img_h = 480;
//   const int cam_res[2] = {img_w, img_h};
//   const double lens_hfov = 98.0;
//   const double lens_vfov = 73.0;
//   // -- cam0 intrinsics and distortion
//   const double cam0_fx = pinhole_focal(img_w, lens_hfov);
//   const double cam0_fy = pinhole_focal(img_h, lens_vfov);
//   const double cam0_cx = img_w / 2.0;
//   const double cam0_cy = img_h / 2.0;
//   const vec4_t cam0_proj_params{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
//   const vec4_t cam0_dist_params{0.01, 0.0001, 0.0001, 0.0001};
//   camera_params_t cam0{0, 0, cam_res,
//                        "pinhole", "radtan4",
//                        cam0_proj_params, cam0_dist_params};
//   // -- cam1 intrinsics and distortion
//   const double cam1_fx = pinhole_focal(img_w, lens_hfov);
//   const double cam1_fy = pinhole_focal(img_h, lens_vfov);
//   const double cam1_cx = img_w / 2.0;
//   const double cam1_cy = img_h / 2.0;
//   const vec4_t cam1_proj_params{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
//   const vec4_t cam1_dist_params{0.01, 0.0001, 0.0001, 0.0001};
//   camera_params_t cam1{1, 1, cam_res,
//                        "pinhole", "radtan4",
//                        cam1_proj_params, cam1_dist_params};
//
//   // Setup cam0 cam1 extrinsics
//   // clang-format off
//   mat4_t T_C1C0;
//   T_C1C0 << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
//             -0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
//             -0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
//             0.0, 0.0, 0.0, 1.0;
//   const pose_t extrinsics{0, 0, T_C1C0};
//   // clang-format on
//
//   // Test load
//   const std::vector<std::string> data_dirs = {CAM0_GRIDS, CAM1_GRIDS};
//   std::map<int, aprilgrids_t> grids;
//   if (load_multicam_calib_data(2, data_dirs, grids) != 0) {
//     LOG_ERROR("Failed to local calibration data!");
//     return -1;
//   }
//   MU_CHECK(grids[0].size() > 0);
//   MU_CHECK(grids[0][0].ids.size() > 0);
//   MU_CHECK(grids.size() > 0);
//   MU_CHECK(grids[1][0].ids.size() > 0);
//   MU_CHECK(grids[0].size() == grids[1].size());
//
//   // Estimate initial guess for grid poses
//   mat4s_t T_C0F;
//   for (auto &grid : grids[0]) {
//     mat4_t rel_pose;
//     aprilgrid_calc_relative_pose(grid, cam0_proj_params, cam0_dist_params, rel_pose);
//     T_C0F.push_back(rel_pose);
//   }
//
//   // Measurement covariance matrix
//   mat2_t covar = pow(1, 2) * I(2);
//
//   for (size_t i = 0; i < grids[0].size(); i++) {
//     // AprilGrid, keypoint and relative pose observed in cam0
//     const auto &cam0_kp = grids[0][i].keypoints[0];
//     const pose_t rel_pose{0, 0, T_C0F[i]};
//
//     // AprilGrid, keypoint and relative pose observed in cam1
//     const auto &cam1_kp = grids[1][i].keypoints[0];
//
//     // Tag id and corner id
//     const int tag_id = grids[0][i].ids[0];
//     const int corner_id = 0;
//
//     // Form residual and call the functor
//     // -- Get the object point
//     vec3_t r_FFi;
//     if (aprilgrid_object_point(grids[0][i], tag_id, corner_id, r_FFi) != 0) {
//       LOG_ERROR("Failed to calculate AprilGrid object point!");
//     }
//     // -- Form residual
//     const calib_stereo_residual_t<pinhole_radtan4_t> residual{
//       cam_res, tag_id, corner_id, r_FFi, cam0_kp, cam1_kp, covar};
//
//     // Calculate residual
//     vec4_t r{0.0, 0.0, 0.0, 0.0};
//     std::vector<const double *> params{rel_pose.param.data(),
//                                        extrinsics.param.data(),
//                                        cam0.param.data(),
//                                        cam1.param.data()};
//     residual.Evaluate(params.data(), r.data(), nullptr);
//
//     // Just some arbitrary test to make sure reprojection error is not larger
//     // than 300pixels in x or y direction. But often this can be the case ...
//     // print_vector("r", r);
//     MU_CHECK(fabs(r(0)) > 0.0);
//     MU_CHECK(fabs(r(1)) > 0.0);
//     MU_CHECK(fabs(r(2)) > 0.0);
//     MU_CHECK(fabs(r(3)) > 0.0);
//   }
//
//   return 0;
// }

int test_calib_stereo_solve() {
  // Setup cameras
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 90.0;
  const double lens_vfov = 90.0;
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

  // // Load calibration data
  // aprilgrids_t grids0;
  // aprilgrids_t grids1;
  // if (load_stereo_calib_data(CAM0_GRIDS, CAM1_GRIDS, grids0, grids1) != 0) {
  //   LOG_ERROR("Failed to local calibration data!");
  //   return -1;
  // }

  // Load cam0 calibration data
  aprilgrids_t grids0;
  if (load_camera_calib_data(CAM0_GRIDS, grids0) != 0) {
    return -1;
  }

  // Load cam1 calibration data
  aprilgrids_t grids1;
  if (load_camera_calib_data(CAM1_GRIDS, grids1) != 0) {
    return -1;
  }

  // Test
  const mat2_t covar = pow(1.0, 2) * I(2);
  mat4_t T_C1C0 = I(4);
  int retval = calib_stereo_solve<pinhole_radtan4_t>(grids0, grids1,
                                                     covar, cam0, cam1,
                                                     T_C1C0);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  const std::string results_fpath = "/tmp/calib_stereo.yaml";
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  std::vector<double> cam0_errs;
  std::vector<double> cam1_errs;
  if (save_results(results_fpath, cam0, cam1, T_C1C0, cam0_errs, cam1_errs) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
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

int test_calib_stereo_inc_solve() {
  // Setup cameras
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 90.0;
  const double lens_vfov = 90.0;
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

  // Load data
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  load_stereo_calib_data(CAM0_GRIDS, CAM1_GRIDS, cam0_grids, cam1_grids);

  // Test
  calib_stereo_data_t data{I(2), cam0_grids, cam1_grids, cam0, cam1};
  if (calib_stereo_inc_solve<pinhole_radtan4_t>(data) != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  const std::string results_fpath = "/tmp/calib-stereo.yaml";
  printf("\x1B[92m");
  printf("Saving optimization results to [%s]", results_fpath.c_str());
  printf("\033[0m\n");
  if (save_results(results_fpath, data.cam0, data.cam1, data.T_C1C0, data.cam0_errs, data.cam1_errs) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    return -1;
  }

  // Compare estimation to ground truth
  // -- cam0
  {
    vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
    vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    vec4_t est_proj_params = data.cam0.proj_params();
    vec4_t est_dist_params = data.cam0.dist_params();

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
    vec4_t est_proj_params = data.cam1.proj_params();
    vec4_t est_dist_params = data.cam1.dist_params();

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
    const vec3_t est_trans = tf_trans(data.T_C1C0);
    const vec3_t gnd_euler = rad2deg(quat2euler(tf_quat(T_C1C0_gnd)));
    const vec3_t est_euler = rad2deg(quat2euler(tf_quat(data.T_C1C0)));

    print_matrix("T_C1C0 [gnd]", T_C1C0_gnd);
    print_matrix("T_C1C0 [est]", data.T_C1C0);
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
  // MU_ADD_TEST(test_calib_stereo_residual);
  MU_ADD_TEST(test_calib_stereo_solve);
  MU_ADD_TEST(test_calib_stereo_inc_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
