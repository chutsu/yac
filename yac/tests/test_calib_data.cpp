#include "munit.hpp"
#include "euroc_test_data.hpp"
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

int test_blur_measure() {
  // Get camera image paths
  const std::string image_dir = CAM0_IMAGE_DIR;
  std::vector<std::string> image_paths;
  if (list_dir(image_dir, image_paths) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", image_dir.c_str());
    return -1;
  }
  std::sort(image_paths.begin(), image_paths.end());

  for (const auto &image_path : image_paths) {
    printf("%s\n", (image_dir + "/" + image_path).c_str());
    auto image = cv::imread(image_dir + "/" + image_path);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
    const auto grid = detector.detect(0, image);

    std::vector<double> patch_vars;
    for (auto &kp : grid.keypoints()) {
      const double patch_width = 9;
      const double patch_height = 9;
      const double roi_x = round(kp(0)) - (patch_width / 2.0);
      const double roi_y = round(kp(1)) - (patch_height / 2.0);
      const double roi_width = patch_width;
      const double roi_height = patch_height;

      const cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
      const cv::Mat patch = image(roi);

      cv::Mat patch_laplacian;
      cv::Laplacian(patch, patch_laplacian, CV_64F);

      cv::Scalar mean;
      cv::Scalar stddev;
      cv::meanStdDev(patch_laplacian, mean, stddev, cv::Mat());
      const double var = stddev.val[0] * stddev.val[0];
      patch_vars.push_back(var);

      printf("var: %f\n", var);
      cv::imshow("Patch", patch);
      cv::imshow("Patch Laplacian", patch_laplacian);
      cv::waitKey(0);

      // std::string patch_output = "patch-" + std::to_string(i++) + ".png";
      // std::string patch_laplacian_output = "patch_laplacian-" + std::to_string(i++) + ".png";
      // cv::imwrite("/tmp/patches/" + patch_output, patch);
      // cv::imwrite("/tmp/patches/" + patch_laplacian_output, patch_laplacian);

    }
    if (patch_vars.size()) {
      printf("min var: %f, ", *std::min_element(patch_vars.begin(), patch_vars.end()));
      printf("max var: %f\n", *std::max_element(patch_vars.begin(), patch_vars.end()));
      cv::imshow("Image", image);
      cv::waitKey(0);
    }
  }

  return 0;
}

int test_calib_data() {
  test_data_t test_data = setup_test_data();

  calib_data_t calib_data;
  calib_data.add_calib_target(test_data.target);
  calib_data.add_camera(test_data.cam0);
  calib_data.add_camera(test_data.cam1);
  calib_data.add_camera_extrinsics(0);
  calib_data.add_camera_extrinsics(1);
  calib_data.add_grids(0, test_data.grids0);
  calib_data.add_grids(1, test_data.grids1);
  // calib_data.preprocess_data();
  // calib_data.check_data();
  return 0;
}

int test_focal_init() {
  const auto image = cv::imread(CAM0_IMAGE);

  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  const auto grid = detector.detect(0, image);

  double focal = 0;
  int retval = focal_init(grid, 0, focal);
  printf("focal: %f\n", focal);
  MU_CHECK(retval == 0);

  // cv::imshow("Image", image);
  // cv::waitKey(0);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_preprocess_and_load_camera_data);
  // MU_ADD_TEST(test_load_multicam_calib_data);
  // MU_ADD_TEST(test_blur_measure);
  MU_ADD_TEST(test_calib_data);
  MU_ADD_TEST(test_focal_init);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
