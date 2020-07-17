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
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  // -- cam0 intrinsics and distortion
  const double cam0_fx = pinhole_focal(img_w, lens_hfov);
  const double cam0_fy = pinhole_focal(img_h, lens_vfov);
  const double cam0_cx = img_w / 2.0;
  const double cam0_cy = img_h / 2.0;
  const vec4_t cam0_intrinsics{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
  const vec4_t cam0_distortion{0.01, 0.0001, 0.0001, 0.0001};
  // -- cam1 intrinsics and distortion
  const double cam1_fx = pinhole_focal(img_w, lens_hfov);
  const double cam1_fy = pinhole_focal(img_h, lens_vfov);
  const double cam1_cx = img_w / 2.0;
  const double cam1_cy = img_h / 2.0;
  const vec4_t cam1_intrinsics{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
  const vec4_t cam1_distortion{0.01, 0.0001, 0.0001, 0.0001};

  // Setup cam0 cam1 extrinsics
  // clang-format off
  mat4_t T_C1C0_data;
  T_C1C0_data << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
                 -0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
                 -0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
                 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    // AprilGrid, keypoint and relative pose observed in cam0
    const auto &grid0 = cam0_aprilgrids[i];
    const auto &cam0_kp = grid0.keypoints[0];
    const auto &T_C0F = grid0.T_CF;

    // AprilGrid, keypoint and relative pose observed in cam1
    const auto &grid1 = cam1_aprilgrids[i];
    const auto &cam1_kp = grid1.keypoints[0];

    // Tag id and corner id
    const auto tag_id = grid0.ids[0];
    const int corner_id = 0;

    // Form pose of fiducial expressed in cam0
    const quat_t q_C0F{T_C0F.block<3, 3>(0, 0)};
    const vec3_t t_C0F{T_C0F.block<3, 1>(0, 3)};

    // Form cam0-cam1 extrinsics
    const quat_t q_C0C1{1.0, 0.0, 0.0, 0.0};
    const vec3_t t_C0C1{0.0, 0.0, 0.0};

    // Form residual and call the functor
    // -- Get the object point
    vec3_t p_F;
    if (aprilgrid_object_point(grid0, tag_id, corner_id, p_F) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object point!");
    }
    // -- Form residual
    // const calib_stereo_residual_t residual{cam0_kp, cam1_kp, p_F};

    // // Calculate residual
    // vec4_t result{0.0, 0.0, 0.0, 0.0};
    // residual(cam0_intrinsics.data(),
    //          cam0_distortion.data(),
    //          cam1_intrinsics.data(),
    //          cam1_distortion.data(),
    //          q_C0C1.coeffs().data(),
    //          t_C0C1.data(),
    //          q_C0F.coeffs().data(),
    //          t_C0F.data(),
    //          result.data());

    // Just some arbitrary test to make sure reprojection error is not larger
    // than 300pixels in x or y direction. But often this can be the case ...
    // MU_CHECK(fabs(result[0]) > 0.0);
    // MU_CHECK(fabs(result[1]) > 0.0);
    // MU_CHECK(fabs(result[2]) > 0.0);
    // MU_CHECK(fabs(result[3]) > 0.0);
  }

  return 0;
}

int test_calib_stereo_solve() {
  // Load stereo calibration data
  aprilgrids_t cam0_aprilgrids;
  aprilgrids_t cam1_aprilgrids;
  int retval = 0;
  load_stereo_calib_data(CAM0_APRILGRID_DATA,
                         CAM1_APRILGRID_DATA,
                         cam0_aprilgrids,
                         cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to local calibration data!");
    return -1;
  }

  // Setup camera intrinsics and distortion
  calib_params_t cam0_params("pinhole", "radtan4", 752, 480, 98.0, 73.0);
  calib_params_t cam1_params("pinhole", "radtan4", 752, 480, 98.0, 73.0);

  // Test
  mat4_t T_C0C1 = I(4);
  mat4s_t poses;
  retval = calib_stereo_solve(cam0_aprilgrids,
                              cam1_aprilgrids,
                              cam0_params,
                              cam1_params,
                              T_C0C1,
                              poses);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Show results
  std::cout << std::endl << cam0_params.toString(0) << std::endl;
  std::cout << std::endl << cam1_params.toString(1) << std::endl;
  std::cout << "T_C0C1:\n" << T_C0C1 << std::endl;

  return 0;
}

// int test_calib_stereo_stats() {
//   // Load stereo calibration data
//   std::vector<aprilgrid_t> cam0_aprilgrids;
//   std::vector<aprilgrid_t> cam1_aprilgrids;
//   int retval = 0;
//   load_stereo_calib_data(CAM0_APRILGRID_DATA,
//                          CAM1_APRILGRID_DATA,
//                          cam0_aprilgrids,
//                          cam1_aprilgrids);
//   if (retval != 0) {
//     LOG_ERROR("Failed to local calibration data!");
//     return -1;
//   }
//
//   // Setup cam0 intrinsics and distortion
//   const int img_w = 752;
//   const int img_h = 480;
//   const double lens_hfov = 98.0;
//   const double lens_vfov = 73.0;
//   const double fx = pinhole_focal(img_w, lens_hfov);
//   const double fy = pinhole_focal(img_h, lens_vfov);
//   const double cx = img_w / 2.0;
//   const double cy = img_h / 2.0;
//   const vec4_t proj_params{fx, fy, cx, cy};
//   const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
//   // -- cam0: pinhole radtan
//   pinhole_t<radtan4_t> cam0{img_w, img_h, proj_params, dist_params};
//   // -- cam1: pinhole radtan
//   pinhole_t<radtan4_t> cam1{img_w, img_h, proj_params, dist_params};
//
//   // Test
//   mat4_t T_C0C1 = I(4);
//   mat4s_t poses;
//   retval = calib_stereo_solve(cam0_aprilgrids, cam1_aprilgrids,
//                               cam0, cam1,
//                               T_C0C1, poses);
//   if (retval != 0) {
//     LOG_ERROR("Failed to calibrate stereo cameras!");
//     return -1;
//   }
//
//   return 0;
// }

void test_suite() {
  test_setup();

  // Stereo camera tests
  MU_ADD_TEST(test_calib_stereo_residual);
  MU_ADD_TEST(test_calib_stereo_solve);
  // MU_ADD_TEST(test_calib_stereo_stats);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
