#include <gtest/gtest.h>

#include "AprilGrid.hpp"

// #include <limits>
// #include <filesystem>

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_OUTPUT "/tmp/aprilgrid.csv"
#define TEST_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define TEST_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"

namespace yac {

TEST(AprilGrid, construct) {
  const timestamp_t ts = 0;
  const int tag_rows = 6;
  const int tag_cols = 7;
  const double tag_size = 0.088;
  const double tag_spacing = 0.03;
  const AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

  ASSERT_EQ(grid.getNumRows(), tag_rows * 2);
  ASSERT_EQ(grid.getNumCols(), tag_cols * 2);
  ASSERT_EQ(grid.getTagRows(), tag_rows);
  ASSERT_EQ(grid.getTagCols(), tag_cols);
  ASSERT_EQ(grid.getTagSize(), tag_size);
  ASSERT_EQ(grid.getTagSpacing(), tag_spacing);
}

TEST(AprilGrid, add) {
  AprilGrid grid(0, 6, 6, 0.088, 0.3);

  // Keypoints
  const vec2_t kp1_gnd{1.0, 2.0};
  const vec2_t kp2_gnd{3.0, 4.0};
  const vec2_t kp3_gnd{5.0, 6.0};
  const vec2_t kp4_gnd{7.0, 8.0};

  // Test add
  const int tag_id = 0;
  grid.add(tag_id, 0, kp1_gnd);
  grid.add(tag_id, 1, kp2_gnd);
  grid.add(tag_id, 2, kp3_gnd);
  grid.add(tag_id, 3, kp4_gnd);
  // std::cout << grid.data << std::endl;

  // const vec2_t kp1 = grid.keypoint(tag_id, 0);
  // const vec2_t kp2 = grid.keypoint(tag_id, 1);
  // const vec2_t kp3 = grid.keypoint(tag_id, 2);
  // const vec2_t kp4 = grid.keypoint(tag_id, 3);
  //
  // // Assert
  // ASSERT_TRUE_FLOAT(0.0, (kp1_gnd - kp1).norm());
  // ASSERT_TRUE_FLOAT(0.0, (kp2_gnd - kp2).norm());
  // ASSERT_TRUE_FLOAT(0.0, (kp3_gnd - kp3).norm());
  // ASSERT_TRUE_FLOAT(0.0, (kp4_gnd - kp4).norm());
}

// TEST(AprilGrid, remove) {
//   aprilgrid_t grid(0, 6, 6, 0.088, 0.3);
//
//   // Keypoints
//   const vec2_t kp1_gnd{1.0, 2.0};
//   const vec2_t kp2_gnd{3.0, 4.0};
//   const vec2_t kp3_gnd{5.0, 6.0};
//   const vec2_t kp4_gnd{7.0, 8.0};
//
//   // Test add
//   const int tag_id = 0;
//   grid.add(tag_id, 0, kp1_gnd);
//   grid.add(tag_id, 1, kp2_gnd);
//   grid.add(tag_id, 2, kp3_gnd);
//   grid.add(tag_id, 3, kp4_gnd);
//
//   grid.remove(tag_id, 0);
//   grid.remove(tag_id, 1);
//   grid.remove(tag_id, 2);
//   grid.remove(tag_id, 3);
//
//   auto keypoints = grid.keypoints();
//   ASSERT_TRUE(keypoints.size() == 0);
//
//
// }

// TEST(AprilGrid, save_and_load) {
//   aprilgrid_t grid(0, 6, 6, 0.088, 0.3);
//
//   // Test save
//   grid.timestamp = 1544020482626424074;
//   grid.tag_rows = 6;
//   grid.tag_cols = 6;
//   grid.tag_size = 0.088;
//   grid.tag_spacing = 0.3;
//
//   const vec2_t kp0{1.0, 2.0};
//   const vec2_t kp1{3.0, 4.0};
//   const vec2_t kp2{5.0, 6.0};
//   const vec2_t kp3{7.0, 8.0};
//   grid.detected = true;
//   grid.add(0, 0, kp0);
//   grid.add(0, 1, kp1);
//   grid.add(0, 2, kp2);
//   grid.add(0, 3, kp3);
//
//   ASSERT_TRUE(grid.save(TEST_OUTPUT) == 0);
//
//   // Test load
//   aprilgrid_t grid2(TEST_OUTPUT);
//   std::cout << "timestamp: " << grid2.timestamp << std::endl;
//   std::cout << "tag_rows: " << grid2.tag_rows << std::endl;
//   std::cout << "tag_cols: " << grid2.tag_cols << std::endl;
//   std::cout << "tag_size: " << grid2.tag_size << std::endl;
//   std::cout << "tag_spacing: " << grid2.tag_spacing << std::endl;
//   std::cout << "detected: " << grid2.detected << std::endl;
//   std::cout << "nb_detections: " << grid2.nb_detections << std::endl;
//   std::cout << "data:\n" << grid2.data << std::endl;
//
//   ASSERT_TRUE(grid2.timestamp == grid.timestamp);
//   ASSERT_TRUE(grid2.detected == true);
//   ASSERT_TRUE(grid2.nb_detections == 4);
//   ASSERT_TRUE(grid2.tag_ids().size() == 1);
//   ASSERT_TRUE(grid2.keypoints().size() == 4);
//   ASSERT_TRUE_FLOAT(0.0, (grid2.keypoints()[0] - kp0).norm());
//   ASSERT_TRUE_FLOAT(0.0, (grid2.keypoints()[1] - kp1).norm());
//   ASSERT_TRUE_FLOAT(0.0, (grid2.keypoints()[2] - kp2).norm());
//   ASSERT_TRUE_FLOAT(0.0, (grid2.keypoints()[3] - kp3).norm());
//
//
// }

// TEST(AprilGrid, detect) {
//   auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
//   const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
//
//   // Detect
//   profiler_t prof;
//   prof.start("detect");
//   aprilgrid_t grid = detector.detect(0, image);
//   prof.stop("detect");
//   prof.print("detect");
//
//   cv::imwrite("/tmp/grid.png", grid.draw(image));
//   ASSERT_TRUE(grid.nb_detections > 0);
//   // grid.imshow("viz", image);
//   // cv::waitKey(0);
//
//   // const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
//   // const std::string img_dir = "/data/euroc/cam_april/mav0/cam0/data";
//   // std::vector<std::string> img_paths;
//   // list_files(img_dir, img_paths);
//   //
//   // for (const auto img_fname : img_paths) {
//   //   const std::string img_path = img_dir + "/" + img_fname;
//   //   const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
//   //   aprilgrid_t grid = detector.detect(0, img);
//   //   grid.imshow("viz", img);
//   //   cv::waitKey(1);
//   // }
//
//
// }
//
// TEST(AprilGrid, profile) {
//   auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
//
//   const std::string dataset_dir = "/data/euroc/imu_april/";
//   const std::string cam0_dir = dataset_dir + "mav0/cam0/data";
//   const std::string cam1_dir = dataset_dir + "mav0/cam1/data";
//   std::vector<std::string> cam0_files;
//   std::vector<std::string> cam1_files;
//   list_files(cam0_dir, cam0_files);
//   list_files(cam1_dir, cam1_files);
//
//   profiler_t prof;
//   const size_t nb_images = std::min(cam0_files.size(), cam1_files.size());
//
//   std::vector<timestamp_t> timestamps;
//   std::vector<real_t> timings;
//   std::vector<int> detections;
//
//   for (size_t k = 0; k < nb_images; k++) {
//     // Load image
//     const auto img0_path = cam0_dir + "/" + cam0_files[k];
//     const auto img1_path = cam1_dir + "/" + cam1_files[k];
//     cv::Mat img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
//     cv::Mat img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
//
//     // Detect
//     std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
//     cv::resize(img0, img0, cv::Size(), 0.75, 0.75);
//     cv::resize(img1, img1, cv::Size(), 0.75, 0.75);
//     img_buffer[0] = {k, img0};
//     img_buffer[1] = {k, img1};
//
//     prof.start("detect");
//     auto grids = detector.detect(img_buffer);
//     real_t det_elasped = prof.stop("detect");
//
//     // Track timestamps -- Parse timestamp from image filename
//     const std::string img0_fname =
//     std::filesystem::path(img0_path).filename(); const std::string ts_str =
//     img0_fname.substr(0, 19); const timestamp_t ts = std::stoull(ts_str);
//     timestamps.push_back(ts);
//
//     // Track timings
//     timings.push_back(det_elasped);
//
//     // Track number of detections
//     int num_dets = 0;
//     for (const auto &[ts, grid] : grids) {
//       num_dets += grid.nb_detections;
//     }
//     detections.push_back(num_dets);
//
//     // Imshow
//     printf("detect_time: %f [s], nb_detected: %d\n", det_elasped, num_dets);
//     // if (grids.count(0)) {
//     //   grids[0].imshow("viz", img0);
//     // } else {
//     //   cv::imshow("viz", img0);
//     // }
//     // cv::waitKey(1);
//   }
//
//   // Print stats
//   printf("max(timings): %f\n", max(timings));
//   printf("mean(timings): %f\n", mean(timings));
//   printf("median(timings): %f\n", median(timings));
//
//   // Save data
//   FILE *csv = fopen("/tmp/aprilgrid_detect.csv", "w");
//   fprintf(csv, "timestamps,timings,detections\n");
//   for (size_t i = 0; i < timestamps.size(); i++) {
//     fprintf(csv, "%ld,%f,%d\n", timestamps[i], timings[i], detections[i]);
//   }
//   fclose(csv);
//
//
// }

} // namespace yac
