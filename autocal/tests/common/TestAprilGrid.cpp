#include <limits>
#include <gtest/gtest.h>

#include "autocal/common/aprilgrid.hpp"

#include "../ceres/EuRoCTestSetup.hpp"
#define TEST_DATA "/data/euroc_mav/imu_april"

namespace autocal {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TEST_OUTPUT "/tmp/aprilgrid.csv"
#define TEST_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define TEST_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"

static void visualize_grid(const cv::Mat &image,
                           const mat3_t &K,
                           const vec4_t &D,
                           const vec3s_t &landmarks,
                           const vec2s_t &keypoints,
                           const bool show) {
  // Undistort image
  cv::Mat image_undistorted;
  cv::Mat intrinsics = convert(K);
  cv::Mat distortions = convert(D);
  cv::undistort(image, image_undistorted, intrinsics, distortions);

  // Project landmarks in 3D to image plane
  for (size_t i = 0; i < landmarks.size(); i++) {
    // Project then scale to image plane
    vec3_t x = landmarks[i];
    x(0) = x(0) / x(2);
    x(1) = x(1) / x(2);
    x(2) = x(2) / x(2);
    x = K * x;

    // Draw corner, Set color to yellow on first corner (origin), else blue
    auto color = (i == 0) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
    cv::circle(image_undistorted, cv::Point(x(0), x(1)), 1.0, color, 2, 8);

    // Label corner
    cv::Point2f cxy(keypoints[i](0), keypoints[i](1));
    cv::Scalar text_color(0, 255, 0);
    std::string text = std::to_string(i);
    int font = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1.0;
    int thickness = 2;
    cv::putText(image_undistorted,
                text,
                cxy,
                font,
                font_scale,
                text_color,
                thickness);
  }

  if (show) {
    cv::imshow("Visualize grid", image_undistorted);
    cv::waitKey(0);
  }
}

TEST(AprilGrid, aprilgrid_constructor) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  EXPECT_TRUE(grid.detected == false);
  EXPECT_TRUE(grid.ids.size() == 0);
  EXPECT_TRUE(grid.keypoints.size() == 0);

  EXPECT_TRUE(grid.estimated == false);
  EXPECT_TRUE(grid.points_CF.size() == 0);
  EXPECT_TRUE((I(4) - grid.T_CF).norm() < 1e-5);
}

TEST(AprilGrid, aprilgrid_add) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Keypoints
  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  std::vector<cv::Point2f> keypoints;
  keypoints.emplace_back(kp1(0), kp1(1));
  keypoints.emplace_back(kp2(0), kp2(1));
  keypoints.emplace_back(kp3(0), kp3(1));
  keypoints.emplace_back(kp4(0), kp4(1));

  // Test add
  const int tag_id = 1;
  aprilgrid_add(grid, tag_id, keypoints);
  ASSERT_FLOAT_EQ(0.0, (grid.keypoints[0] - kp1).norm());
  ASSERT_FLOAT_EQ(0.0, (grid.keypoints[1] - kp2).norm());
  ASSERT_FLOAT_EQ(0.0, (grid.keypoints[2] - kp3).norm());
  ASSERT_FLOAT_EQ(0.0, (grid.keypoints[3] - kp4).norm());
  ASSERT_TRUE(grid.points_CF.size() == 0);
}

TEST(AprilGrid, aprilgrid_remove) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Add keypoints and points_CF
  for (int i = 0; i < 5; i++) {
    std::vector<cv::Point2f> keypoints;
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);

    vec3s_t points_CF;
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);

    aprilgrid_add(grid, i, keypoints);
    extend(grid.points_CF, points_CF);
  }

  // Test remove
  aprilgrid_remove(grid, 2);
  aprilgrid_remove(grid, 4);
  aprilgrid_remove(grid, 5); // Remove non-existant id

  for (size_t i = 0; i < grid.ids.size(); i++) {
    const int id = grid.ids[i];

    EXPECT_TRUE((vec2_t(id, id) - grid.keypoints[i * 4]).norm() < 1e-8);
    EXPECT_TRUE((vec2_t(id, id) - grid.keypoints[i * 4 + 1]).norm() < 1e-8);
    EXPECT_TRUE((vec2_t(id, id) - grid.keypoints[i * 4 + 2]).norm() < 1e-8);
    EXPECT_TRUE((vec2_t(id, id) - grid.keypoints[i * 4 + 3]).norm() < 1e-8);

    EXPECT_TRUE((vec3_t(id, id, id) - grid.points_CF[i * 4]).norm() < 1e-8);
    EXPECT_TRUE((vec3_t(id, id, id) - grid.points_CF[i * 4 + 1]).norm() < 1e-8);
    EXPECT_TRUE((vec3_t(id, id, id) - grid.points_CF[i * 4 + 2]).norm() < 1e-8);
    EXPECT_TRUE((vec3_t(id, id, id) - grid.points_CF[i * 4 + 3]).norm() < 1e-8);
  }

  // Test remove everything
  for (int i = 0; i < 5; i++) {
    aprilgrid_remove(grid, i);
  }
  EXPECT_TRUE(grid.nb_detections == 0);
  EXPECT_TRUE(grid.ids.size() == 0);
  EXPECT_TRUE(grid.keypoints.size() == 0);
  EXPECT_TRUE(grid.points_CF.size() == 0);
  // std::cout << grid << std::endl;
}

TEST(AprilGrid, aprilgrid_get) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  for (int i = 0; i < 10; i++) {
    // Keypoints
    const vec2_t kp1(i, i);
    const vec2_t kp2(i, i);
    const vec2_t kp3(i, i);
    const vec2_t kp4(i, i);
    std::vector<cv::Point2f> keypoints;
    keypoints.emplace_back(kp1(0), kp1(1));
    keypoints.emplace_back(kp2(0), kp2(1));
    keypoints.emplace_back(kp3(0), kp3(1));
    keypoints.emplace_back(kp4(0), kp4(1));

    // Positions
    const vec3_t pos1(i, i, i);
    const vec3_t pos2(i, i, i);
    const vec3_t pos3(i, i, i);
    const vec3_t pos4(i, i, i);

    // Add measurment
    aprilgrid_add(grid, i, keypoints);
    grid.estimated = true;
    grid.points_CF.emplace_back(pos1);
    grid.points_CF.emplace_back(pos2);
    grid.points_CF.emplace_back(pos3);
    grid.points_CF.emplace_back(pos4);

    // Test get tag
    vec2s_t keypoints_result;
    vec3s_t positions_result;
    aprilgrid_get(grid, i, keypoints_result, positions_result);

    EXPECT_TRUE((vec2_t(i, i) - keypoints_result[0]).norm() < 1e-4);
    EXPECT_TRUE((vec2_t(i, i) - keypoints_result[1]).norm() < 1e-4);
    EXPECT_TRUE((vec2_t(i, i) - keypoints_result[2]).norm() < 1e-4);
    EXPECT_TRUE((vec2_t(i, i) - keypoints_result[3]).norm() < 1e-4);

    EXPECT_TRUE((vec3_t(i, i, i) - positions_result[0]).norm() < 1e-4);
    EXPECT_TRUE((vec3_t(i, i, i) - positions_result[1]).norm() < 1e-4);
    EXPECT_TRUE((vec3_t(i, i, i) - positions_result[2]).norm() < 1e-4);
    EXPECT_TRUE((vec3_t(i, i, i) - positions_result[3]).norm() < 1e-4);
  }
}

TEST(AprilGrid, aprilgrid_grid_index) {
  int i = 0;
  int j = 0;
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  EXPECT_TRUE(aprilgrid_grid_index(grid, 0, i, j) == 0);
  EXPECT_TRUE(i == 0);
  EXPECT_TRUE(j == 0);

  EXPECT_TRUE(aprilgrid_grid_index(grid, 1, i, j) == 0);
  EXPECT_TRUE(i == 0);
  EXPECT_TRUE(j == 1);

  EXPECT_TRUE(aprilgrid_grid_index(grid, 5, i, j) == 0);
  EXPECT_TRUE(i == 0);
  EXPECT_TRUE(j == 5);

  EXPECT_TRUE(aprilgrid_grid_index(grid, 7, i, j) == 0);
  EXPECT_TRUE(i == 1);
  EXPECT_TRUE(j == 1);

  EXPECT_TRUE(aprilgrid_grid_index(grid, 17, i, j) == 0);
  EXPECT_TRUE(i == 2);
  EXPECT_TRUE(j == 5);
}

// TEST(AprilGrid, aprilgrid_calc_relative_pose) {
//   // Detect tags
//   const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
//   const auto detector = aprilgrid_detector_t();
//   auto tags = detector.det.extractTags(image);
//
//   // Extract relative pose
//   const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
//   const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
//   const int tag_rows = 6;
//   const int tag_cols = 6;
//   const double tag_size = 0.088;
//   const double tag_spacing = 0.3;
//   aprilgrid_t grid(0, tag_rows, tag_cols, tag_size, tag_spacing);
//
//   for (const auto &tag : tags) {
//     // Image points (counter-clockwise, from bottom left)
//     std::vector<cv::Point2f> img_pts;
//     img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
//     img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
//     img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
//     img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Top left
//
//     aprilgrid_add(grid, tag.id, img_pts);
//   }
//
//   {
//     auto t = tic();
//     aprilgrid_calc_relative_pose(grid, K, D);
//     printf("OpenCV solvePnP time elasped: %fs\n", toc(&t));
//     print_matrix("T_CF", grid.T_CF);
//   }
// }

TEST(AprilGrid, aprilgrid_save_and_load) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  const vec3_t pos1{1.0, 2.0, 3.0};
  const vec3_t pos2{4.0, 5.0, 6.0};
  const vec3_t pos3{7.0, 8.0, 9.0};
  const vec3_t pos4{10.0, 11.0, 12.0};

  // Test save
  grid.configured = true;
  grid.tag_rows = 6;
  grid.tag_cols = 6;
  grid.tag_size = 0.088;
  grid.tag_spacing = 0.3;

  grid.detected = true;
  grid.timestamp = 1544020482626424074;
  grid.ids.push_back(1);
  grid.keypoints = {kp1, kp2, kp3, kp4};

  grid.estimated = true;
  grid.points_CF = {pos1, pos2, pos3, pos4};
  grid.T_CF = tf(I(3), vec3_t{1.0, 2.0, 3.0});
  EXPECT_TRUE(aprilgrid_save(grid, TEST_OUTPUT) == 0);

  // Test load
  aprilgrid_t grid2(0, 6, 6, 0.088, 0.3);
  EXPECT_TRUE(aprilgrid_load(grid2, TEST_OUTPUT) == 0);
  // std::cout << grid2 << std::endl;

  EXPECT_TRUE(grid2.ids.size() == 1);
  EXPECT_TRUE(grid2.keypoints.size() == 4);
  EXPECT_TRUE(grid2.points_CF.size() == 4);
  EXPECT_TRUE(grid.timestamp == grid2.timestamp);
  EXPECT_FLOAT_EQ(0.0, (grid2.keypoints[0] - kp1).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.keypoints[1] - kp2).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.keypoints[2] - kp3).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.keypoints[3] - kp4).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.points_CF[0] - pos1).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.points_CF[1] - pos2).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.points_CF[2] - pos3).norm());
  EXPECT_FLOAT_EQ(0.0, (grid2.points_CF[3] - pos4).norm());

  // std::cout << grid2.T_CF.block(0, 3, 3, 1) << std::endl;
  const vec3_t r_CF = tf_trans(grid2.T_CF);
  const auto diff = r_CF - vec3_t{1.0, 2.0, 3.0};
  EXPECT_TRUE((diff).norm() < 1e-3);
}

TEST(AprilGrid, aprilgrid_print) {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  const vec3_t pos1{1.0, 2.0, 3.0};
  const vec3_t pos2{4.0, 5.0, 6.0};
  const vec3_t pos3{7.0, 8.0, 9.0};
  const vec3_t pos4{10.0, 11.0, 12.0};

  grid.ids.push_back(1);
  grid.keypoints.push_back(kp1);
  grid.keypoints.push_back(kp2);
  grid.keypoints.push_back(kp3);
  grid.keypoints.push_back(kp4);
  grid.points_CF.push_back(pos1);
  grid.points_CF.push_back(pos2);
  grid.points_CF.push_back(pos3);
  grid.points_CF.push_back(pos4);

  // std::cout << grid << std::endl;
}

TEST(AprilGrid, aprilgrid_detect) {
  aprilgrid_t grid;
  EXPECT_TRUE(aprilgrid_configure(grid, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid, detector, image, K, D);

  aprilgrid_imshow(grid, "AprilGrid", image);
  cv::waitKey(0);

  for (const auto &corner : grid.points_CF) {
    EXPECT_TRUE(corner(0) < 1.0);
    EXPECT_TRUE(corner(0) > -1.0);
    EXPECT_TRUE(corner(1) < 1.0);
    EXPECT_TRUE(corner(1) > -1.0);
    EXPECT_TRUE(corner(2) < 2.0);
    EXPECT_TRUE(corner(2) > 0.5);
  }
}

TEST(AprilGrid, aprilgrid_detect_2) {
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

  create_dir("/tmp/cam0");
  long int nb_detections = 0;
  for (const auto &ts : test_data.timeline.timestamps) {
    const auto result = test_data.timeline.data.equal_range(ts);
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;

      if (event.type == CAMERA_EVENT && event.camera_index == 0) {
        const auto ts = event.ts;
        const auto image = cv::imread(event.image_path);

        aprilgrid_t grid(ts, 6, 6, 0.088, 0.3);
        aprilgrid_detect(grid, detector, image, K, D);
        grid.timestamp = ts;

        aprilgrid_imshow(grid, "aprilgrid detection", image);
        // cv::waitKey(0);

        std::string save_path = "/tmp/cam0/" + std::to_string(ts) + ".csv";

        nb_detections += grid.nb_detections;
        // aprilgrid_save(grid, save_path);
        // printf("saving grid to [%s]\n", save_path.c_str());
      }
    }
  }

  printf("total nb detections: %ld\n", nb_detections);
}

// TEST(AprilGrid, aprilgrid_intersection) {
//   aprilgrid_t grid0;
//   aprilgrid_t grid1;
//   EXPECT_TRUE(aprilgrid_configure(grid0, TEST_CONF) == 0);
//   EXPECT_TRUE(aprilgrid_configure(grid1, TEST_CONF) == 0);
//
//   const auto detector = aprilgrid_detector_t();
//   const cv::Mat image = cv::imread(TEST_IMAGE);
//   const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
//   const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
//   aprilgrid_detect(grid0, detector, image, K, D);
//   aprilgrid_detect(grid1, detector, image, K, D);
//
//   // Remove tag id 2 from grid1
//   aprilgrid_remove(grid1, 2);
//   // std::cout << grid0 << std::endl;
//   // std::cout << grid1 << std::endl;
//   EXPECT_TRUE(grid0.nb_detections == 36);
//   EXPECT_TRUE(grid1.nb_detections == 35);
//   EXPECT_TRUE(grid0.ids.size() == 36);
//   EXPECT_TRUE(grid1.ids.size() == 35);
//
//   // Test intersection
//   // aprilgrid_intersection(grid0, grid1);
//   // EXPECT_TRUE(grid0.nb_detections == 35);
//   // EXPECT_TRUE(grid1.nb_detections == 35);
//   // EXPECT_TRUE(grid0.ids.size() == 35);
//   // EXPECT_TRUE(grid1.ids.size() == 35);
// }

TEST(AprilGrid, aprilgrid_intersection2) {
  aprilgrid_t grid0;
  aprilgrid_t grid1;
  aprilgrid_t grid2;
  EXPECT_TRUE(aprilgrid_configure(grid0, TEST_CONF) == 0);
  EXPECT_TRUE(aprilgrid_configure(grid1, TEST_CONF) == 0);
  EXPECT_TRUE(aprilgrid_configure(grid2, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid0, detector, image, K, D);
  aprilgrid_detect(grid1, detector, image, K, D);
  aprilgrid_detect(grid2, detector, image, K, D);

  // Randomly remove between 2 to 10 tags from grid0, grid1 and grid2
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid0, randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid1, randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid2, randi(0, 35));
  }
  EXPECT_TRUE(grid0.nb_detections < 36);
  EXPECT_TRUE(grid1.nb_detections < 36);
  EXPECT_TRUE(grid2.nb_detections < 36);

  // Test intersection
  std::vector<aprilgrid_t *> data = {&grid0, &grid1, &grid2};
  aprilgrid_intersection(data);
  const int nb_detections = grid0.nb_detections;
  EXPECT_TRUE(grid0.nb_detections == nb_detections);
  EXPECT_TRUE(grid1.nb_detections == nb_detections);
  EXPECT_TRUE(grid2.nb_detections == nb_detections);
}

TEST(AprilGrid, aprilgrid_random_sample) {
  aprilgrid_t grid;
  EXPECT_TRUE(aprilgrid_configure(grid, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid, detector, image, K, D);

  const size_t n = 5;
  std::vector<int> tag_ids;
  std::vector<vec2s_t> keypoints;
  std::vector<vec3s_t> object_points;
  aprilgrid_random_sample(grid, n, tag_ids, keypoints, object_points);

  EXPECT_TRUE(tag_ids.size() == n);
  for (size_t i = 0; i < n; i++) {
    EXPECT_TRUE(std::count(tag_ids.begin(), tag_ids.end(), tag_ids[i]) == 1);
  }
  EXPECT_TRUE(keypoints.size() == n);
  for (size_t i = 0; i < n; i++) {
    EXPECT_TRUE(keypoints[i].size() == 4);
  }
  EXPECT_TRUE(object_points.size() == n);
  for (size_t i = 0; i < n; i++) {
    EXPECT_TRUE(object_points[i].size() == 4);
  }
}

} // namespace autocal
