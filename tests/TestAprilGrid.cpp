#include <gtest/gtest.h>

#include "AprilGrid.hpp"

#define TEST_OUTPUT "/tmp/aprilgrid.csv"
#define TEST_IMAGE TEST_DATA "/aprilgrid/aprilgrid.png"

namespace yac {

TEST(AprilGrid, construct) {
  const timestamp_t ts = 0;
  const int tag_rows = 6;
  const int tag_cols = 7;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  const AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

  ASSERT_EQ(grid.getTagRows(), tag_rows);
  ASSERT_EQ(grid.getTagCols(), tag_cols);
  ASSERT_EQ(grid.getTagSize(), tag_size);
  ASSERT_EQ(grid.getTagSpacing(), tag_spacing);
}

TEST(AprilGrid, addAndRemove) {
  const timestamp_t ts = 0;
  const int tag_rows = 6;
  const int tag_cols = 7;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

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
  {
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.getMeasurements(tag_ids, corner_indicies, keypoints, object_points);
    ASSERT_EQ(tag_ids.size(), 4);
    ASSERT_EQ(corner_indicies.size(), 4);
    ASSERT_EQ(keypoints.size(), 4);
    ASSERT_EQ(object_points.size(), 4);

    ASSERT_NEAR(0.0, (kp1_gnd - keypoints[0]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp2_gnd - keypoints[1]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp3_gnd - keypoints[2]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp4_gnd - keypoints[3]).norm(), 1e-8);
  }

  // Test remove
  grid.remove(tag_id, 0);
  grid.remove(tag_id, 2);
  {
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.getMeasurements(tag_ids, corner_indicies, keypoints, object_points);
    ASSERT_EQ(tag_ids.size(), 2);
    ASSERT_EQ(corner_indicies.size(), 2);
    ASSERT_EQ(keypoints.size(), 2);
    ASSERT_EQ(object_points.size(), 2);

    ASSERT_NEAR(0.0, (kp2_gnd - keypoints[0]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp4_gnd - keypoints[1]).norm(), 1e-8);
  }
}

TEST(AprilGrid, saveAndLoad) {
  const timestamp_t ts = 0;
  const int tag_rows = 6;
  const int tag_cols = 7;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

  // Test save
  const int tag_id = 0;
  const vec2_t kp0{1.0, 2.0};
  const vec2_t kp1{3.0, 4.0};
  const vec2_t kp2{5.0, 6.0};
  const vec2_t kp3{7.0, 8.0};
  grid.add(tag_id, 0, kp0);
  grid.add(tag_id, 1, kp1);
  grid.add(tag_id, 2, kp2);
  grid.add(tag_id, 3, kp3);
  ASSERT_TRUE(grid.save(TEST_OUTPUT) == 0);

  // Test load
  auto grid2 = AprilGrid::load(TEST_OUTPUT);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.getMeasurements(tag_ids, corner_indicies, keypoints, object_points);

  ASSERT_EQ(grid.getTimestamp(), grid2->getTimestamp());
  ASSERT_FLOAT_EQ(0.0, (keypoints[0] - kp0).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[1] - kp1).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[2] - kp2).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[3] - kp3).norm());
}

TEST(AprilGrid, detect) {
  const int tag_rows = 6;
  const int tag_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  AprilGridDetector detector(tag_rows, tag_cols, tag_size, tag_spacing);

  const auto img = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  auto grid = detector.detect(0, img);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid->getMeasurements(tag_ids, corner_indicies, keypoints, object_points);

  ASSERT_EQ(tag_ids.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(corner_indicies.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(keypoints.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(object_points.size(), tag_rows * tag_cols * 4);
}

} // namespace yac
