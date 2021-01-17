#include "munit.hpp"
#include "calib_data.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define APRILGRID_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define APRILGRID_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define CAM0_IMAGE TEST_PATH "/test_data/calib/stereo/cam0_1403709395937837056.png"
#define CAM1_IMAGE TEST_PATH "/test_data/calib/stereo/cam1_1403709395937837056.png"
#define CAM0_IMAGE_DIR "/data/euroc/calib/cam_april/mav0/cam0/data"
#define CAM1_IMAGE_DIR "/data/euroc/calib/cam_april/mav0/cam1/data"

#define MONO_OUTPUT_DIR "/tmp/aprilgrid_test/mono"
#define STEREO_OUTPUT_DIR "/tmp/aprilgrid_test/stereo"

int test_preprocess_and_load_camera_data() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // Test preprocess data
  const std::string image_dir = CAM0_IMAGE_DIR;
  const std::string output_dir = MONO_OUTPUT_DIR "/cam0";
  preprocess_camera_data(target, image_dir, output_dir);

  // // Test load
  // aprilgrids_t aprilgrids;
  // MU_CHECK(load_camera_calib_data(output_dir, aprilgrids) == 0);
  // MU_CHECK(aprilgrids.size() > 0);
  // MU_CHECK(aprilgrids[0].nb_detections > 0);

  return 0;
}

// int test_load_multicam_calib_data() {
//   // Setup calibration target
//   calib_target_t target;
//   if (calib_target_load(target, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
//     return -1;
//   }
//
//   // Test preprocess data
//   LOG_INFO("Preprocessing images ...");
//   const std::string cam0_output_dir = STEREO_OUTPUT_DIR "/cam0";
//   const std::string cam1_output_dir = STEREO_OUTPUT_DIR "/cam1";
//   preprocess_camera_data(target, CAM0_IMAGE_DIR, cam0_output_dir);
//   preprocess_camera_data(target, CAM1_IMAGE_DIR, cam1_output_dir);
//
//   // Test load
//   LOG_INFO("Loading calib data ...");
//   std::map<int, aprilgrids_t> calib_data;
//   std::vector<std::string> output_dirs = {cam0_output_dir, cam1_output_dir};
//   MU_CHECK(load_multicam_calib_data(2, output_dirs, calib_data) == 0);
//
//   // Assert
//   const auto grids0 = calib_data[0];
//   const auto grids1 = calib_data[1];
//   // std::cout << "grids0 size: " << grids0.size() << std::endl;
//   // std::cout << "grids1 size: " << grids1.size() << std::endl;
//   MU_CHECK(grids0.size() == grids1.size());
//
//   const size_t nb_grids = grids0.size();
//   for (size_t i = 0; i < nb_grids; i++) {
//     const auto grid0 = grids0[i];
//     const auto grid1 = grids1[i];
//     MU_CHECK(grid0.ids == grid1.ids);
//   }
//
//   return 0;
// }

// int test_draw_calib_validation() {
//   // Setup camera geometry
//   // -- Camera model
//   const double fx = 458.654;
//   const double fy = 457.296;
//   const double cx = 367.215;
//   const double cy = 248.375;
//   const pinhole_t camera_model{fx, fy, cx, cy};
//   const mat3_t cam_K = camera_model.K();
//   // -- Distortion model
//   const double k1 = -0.28340811;
//   const double k2 = 0.07395907;
//   const double p1 = 0.00019359;
//   const double p2 = 1.76187114e-05;
//   const radtan4_t distortion_model{k1, k2, p1, p2};
//   const vec4_t cam_D{k1, k2, p1, p2};
//   // -- Camera Geometry
//   pinhole_radtan4_t camera{camera_model, distortion_model};
//
//   // Load aprilgrid
//   aprilgrid_t aprilgrid;
//   if (aprilgrid_configure(aprilgrid, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to configure aprilgrid!");
//     return -1;
//   }
//
//   // Detect AprilGrid
//   const auto detector = aprilgrid_detector_t();
//   cv::Mat image = cv::imread(APRILGRID_IMAGE);
//   aprilgrid_detect(aprilgrid, detector, image, cam_K, cam_D);
//
//   // Get measured keypoints and project points
//   vec2s_t measured;
//   vec2s_t projected;
//
//   for (size_t i = 0; i < aprilgrid.keypoints.size(); i++) {
//     const auto &kp = aprilgrid.keypoints[i];
//     measured.emplace_back(kp);
//
//     const auto &point = aprilgrid.points_CF[i];
//     const auto p = camera_geometry_project(camera, point);
//     projected.emplace_back(p);
//   }
//
//   // Draw validation
//   const cv::Scalar measured_color{0, 0, 255};  // Red
//   const cv::Scalar projected_color{0, 255, 0}; // Green
//   auto validation = draw_calib_validation(image,
//                                           measured,
//                                           projected,
//                                           measured_color,
//                                           projected_color);
//
//   // Visualize
//   const bool debug = false;
//   if (debug) {
//     cv::imshow("Validation:", validation);
//     cv::waitKey(0);
//   }
//
//   // Assert
//   validation = rgb2gray(validation);
//   image = rgb2gray(image);
//   MU_CHECK(is_equal(validation, image) == false);
//
//   return 0;
// }

// int test_validate_intrinsics() {
//   // Setup camera geometry
//   // -- Camera model
//   const double fx = 458.654;
//   const double fy = 457.296;
//   const double cx = 367.215;
//   const double cy = 248.375;
//   const pinhole_t camera_model{fx, fy, cx, cy};
//   const mat3_t cam_K = camera_model.K();
//   // -- Distortion model
//   const double k1 = -0.28340811;
//   const double k2 = 0.07395907;
//   const double p1 = 0.00019359;
//   const double p2 = 1.76187114e-05;
//   const radtan4_t distortion_model{k1, k2, p1, p2};
//   const vec4_t cam_D{k1, k2, p1, p2};
//   // -- Camera Geometry
//   pinhole_radtan4_t camera{camera_model, distortion_model};
//
//   // Load aprilgrid
//   aprilgrid_t aprilgrid;
//   if (aprilgrid_configure(aprilgrid, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to configure aprilgrid!");
//     return -1;
//   }
//
//   // Detect AprilGrid
//   const auto detector = aprilgrid_detector_t();
//   cv::Mat image = cv::imread(APRILGRID_IMAGE);
//   aprilgrid_detect(aprilgrid, detector, image, cam_K, cam_D);
//
//   // Validate intrinsics
//   auto validation = validate_intrinsics(image,
//                                         aprilgrid.keypoints,
//                                         aprilgrid.points_CF,
//                                         camera);
//
//   // Visualize
//   // const bool debug = true;
//   const bool debug = false;
//   if (debug) {
//     cv::imshow("Validation:", validation);
//     cv::waitKey(0);
//   }
//
//   // Assert
//   validation = rgb2gray(validation);
//   image = rgb2gray(image);
//   MU_CHECK(is_equal(validation, image) == false);
//
//   return 0;
// }

// int test_validate_stereo() {
//   // Setup cam0 geometry
//   // -- Camera model
//   const double cam0_fx = 458.654;
//   const double cam0_fy = 457.296;
//   const double cam0_cx = 367.215;
//   const double cam0_cy = 248.375;
//   const pinhole_t cam0_model{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
//   const mat3_t cam0_K = cam0_model.K();
//   // -- Distortion model
//   const double cam0_k1 = -0.28340811;
//   const double cam0_k2 = 0.07395907;
//   const double cam0_p1 = 0.00019359;
//   const double cam0_p2 = 1.76187114e-05;
//   const radtan4_t radtan0_model{cam0_k1, cam0_k2, cam0_p1, cam0_p2};
//   const vec4_t cam0_D = radtan0_model.vec();
//   // -- Camera Geometry
//   const pinhole_radtan4_t cam0{cam0_model, radtan0_model};
//
//   // Setup cam1 geometry
//   // -- Camera model
//   const double cam1_fx = 457.587;
//   const double cam1_fy = 456.134;
//   const double cam1_cx = 379.999;
//   const double cam1_cy = 255.238;
//   const pinhole_t cam1_model{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
//   const mat3_t cam1_K = cam1_model.K();
//   // -- Distortion model
//   const double cam1_k1 = -0.28368365;
//   const double cam1_k2 = 0.07451284;
//   const double cam1_p1 = -0.00010473;
//   const double cam1_p2 = -3.55590700e-05;
//   const radtan4_t radtan1_model{cam1_k1, cam1_k2, cam1_p1, cam1_p2};
//   const vec4_t cam1_D = radtan1_model.vec();
//   // -- Camera Geometry
//   const pinhole_radtan4_t cam1{cam1_model, radtan1_model};
//
//   // Camera to camera extrinsics
//   // clang-format off
//   mat4_t T_BC0;
//   T_BC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
//            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
//            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
//            0.0, 0.0, 0.0, 1.0;
//   mat4_t T_BC1;
//   T_BC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
//            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
//            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
//            0.0, 0.0, 0.0, 1.0;
//   const mat4_t T_C0C1 = T_BC0.inverse() * T_BC1;
//   // clang-format on
//   std::cout << T_C0C1 << std::endl;
//
//   // Load aprilgrid
//   aprilgrid_t aprilgrid0;
//   if (aprilgrid_configure(aprilgrid0, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to configure aprilgrid!");
//     return -1;
//   }
//
//   aprilgrid_t aprilgrid1;
//   if (aprilgrid_configure(aprilgrid1, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to configure aprilgrid!");
//     return -1;
//   }
//
//   // Detect AprilGrid
//   const auto detector = aprilgrid_detector_t();
//   cv::Mat image0 = cv::imread(CAM0_IMAGE);
//   if (aprilgrid_detect(aprilgrid0, detector, image0, cam0_K, cam0_D) == 0) {
//     LOG_ERROR("Failed to detect aprilgrid!");
//     return -1;
//   }
//
//   cv::Mat image1 = cv::imread(CAM1_IMAGE);
//   if (aprilgrid_detect(aprilgrid1, detector, image1, cam1_K, cam1_D) == 0) {
//     LOG_ERROR("Failed to detect aprilgrid!");
//     return -1;
//   }
//
//   // Validate stereo
//   const auto result = validate_stereo(image0,
//                                       image1,
//                                       aprilgrid0.keypoints,
//                                       aprilgrid0.points_CF,
//                                       aprilgrid1.keypoints,
//                                       aprilgrid1.points_CF,
//                                       cam0,
//                                       cam1,
//                                       T_C0C1);
//
//   // Visualize
//   // const bool debug = true;
//   const bool debug = false;
//   if (debug) {
//     cv::imshow("Validation:", result);
//     cv::waitKey(0);
//   }
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_preprocess_and_load_camera_data);
  // MU_ADD_TEST(test_load_multicam_calib_data);
  // MU_ADD_TEST(test_draw_calib_validation);
  // MU_ADD_TEST(test_validate_intrinsics);
  // MU_ADD_TEST(test_validate_stereo);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
