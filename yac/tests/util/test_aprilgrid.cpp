#include <limits>
#include "../munit.hpp"
#include "util/aprilgrid.hpp"
// #include "util/euroc.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_OUTPUT "/tmp/aprilgrid.csv"
#define TEST_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define TEST_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"

int test_aprilgrid_constructor() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  MU_CHECK(grid.timestamp == 0);
  MU_CHECK(grid.tag_rows == 6);
  MU_CHECK(grid.tag_cols == 6);
  MU_CHECK(fltcmp(grid.tag_size, 0.088) == 0);
  MU_CHECK(fltcmp(grid.tag_spacing, 0.3) == 0);

  MU_CHECK(grid.detected == false);
  MU_CHECK(grid.nb_detections == 0);

  // MU_CHECK(grid.keypoints.size() == 0);
  // MU_CHECK(grid.object_points.size() == 0);

  return 0;
}

int test_aprilgrid_grid_index() {
  int i = 0;
  int j = 0;
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  grid.grid_index(0, i, j);
  MU_CHECK(i == 0);
  MU_CHECK(j == 0);

  grid.grid_index(1, i, j);
  MU_CHECK(i == 0);
  MU_CHECK(j == 1);

  grid.grid_index(5, i, j);
  MU_CHECK(i == 0);
  MU_CHECK(j == 5);

  grid.grid_index(7, i, j);
  MU_CHECK(i == 1);
  MU_CHECK(j == 1);

  grid.grid_index(17, i, j);
  MU_CHECK(i == 2);
  MU_CHECK(j == 5);

  return 0;
}

int test_aprilgrid_object_point() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  vec3_t pt0 = grid.object_point(0, 0);
  vec3_t pt1 = grid.object_point(0, 1);
  vec3_t pt2 = grid.object_point(0, 2);
  vec3_t pt3 = grid.object_point(0, 3);

  print_vector("pt0", pt0);
  print_vector("pt1", pt1);
  print_vector("pt2", pt2);
  print_vector("pt3", pt3);

  return 0;
}

int test_aprilgrid_keypoint() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Keypoints
  const vec2_t kp0_gnd{1.0, 2.0};
  const vec2_t kp1_gnd{3.0, 4.0};
  const vec2_t kp2_gnd{5.0, 6.0};
  const vec2_t kp3_gnd{7.0, 8.0};

  // Test add
  const int tag_id = 0;
  grid.add(tag_id, 0, kp0_gnd);
  grid.add(tag_id, 1, kp1_gnd);
  grid.add(tag_id, 2, kp2_gnd);
  grid.add(tag_id, 3, kp3_gnd);

  vec2_t kp0 = grid.keypoint(0, 0);
  vec2_t kp1 = grid.keypoint(0, 1);
  vec2_t kp2 = grid.keypoint(0, 2);
  vec2_t kp3 = grid.keypoint(0, 3);

  print_vector("kp0", kp0);
  print_vector("kp1", kp1);
  print_vector("kp2", kp2);
  print_vector("kp3", kp3);

  MU_CHECK_FLOAT((kp0 - kp0_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kp1 - kp1_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kp2 - kp2_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kp3 - kp3_gnd).norm(), 0.0);

  return 0;
}

int test_aprilgrid_keypoints() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Keypoints
  const vec2_t kp0_gnd{1.0, 2.0};
  const vec2_t kp1_gnd{3.0, 4.0};
  const vec2_t kp2_gnd{5.0, 6.0};
  const vec2_t kp3_gnd{7.0, 8.0};

  // Test add
  const int tag_id = 0;
  grid.add(tag_id, 0, kp0_gnd);
  grid.add(tag_id, 1, kp1_gnd);
  grid.add(tag_id, 2, kp2_gnd);
  grid.add(tag_id, 3, kp3_gnd);

  // Test keypoints
  auto kps = grid.keypoints();

  MU_CHECK_FLOAT((kps[0] - kp0_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kps[1] - kp1_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kps[2] - kp2_gnd).norm(), 0.0);
  MU_CHECK_FLOAT((kps[3] - kp3_gnd).norm(), 0.0);

  return 0;
}

int test_aprilgrid_add() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

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

  const vec2_t kp1 = grid.keypoint(tag_id, 0);
  const vec2_t kp2 = grid.keypoint(tag_id, 1);
  const vec2_t kp3 = grid.keypoint(tag_id, 2);
  const vec2_t kp4 = grid.keypoint(tag_id, 3);

  // Assert
  MU_CHECK_FLOAT(0.0, (kp1_gnd - kp1).norm());
  MU_CHECK_FLOAT(0.0, (kp2_gnd - kp2).norm());
  MU_CHECK_FLOAT(0.0, (kp3_gnd - kp3).norm());
  MU_CHECK_FLOAT(0.0, (kp4_gnd - kp4).norm());

  return 0;
}

int test_aprilgrid_remove() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

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

  grid.remove(tag_id, 0);
  grid.remove(tag_id, 1);
  grid.remove(tag_id, 2);
  grid.remove(tag_id, 3);

  auto keypoints = grid.keypoints();
  MU_CHECK(keypoints.size() == 0);

  return 0;
}

int test_aprilgrid_estimate() {
  // Detect tags
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const int rows = 6;
  const int cols = 6;
  const double size = 0.088;
  const double spacing = 0.3;
  const auto detector = aprilgrid_detector_t(rows, cols, size, spacing);
  auto tags = detector.det.extractTags(image);

  // Extract relative pose
  int cam_res[2] = {752, 480};

  const real_t fx = 458.654;
  const real_t fy = 457.296;
  const real_t cx = 367.215;
  const real_t cy = 248.375;

  const real_t k1 = -0.28340811;
  const real_t k2 = 0.07395907;
  const real_t p1 = 0.00019359;
  const real_t p2 = 1.76187114e-05;

  vecx_t cam_params;
  cam_params.resize(8);
  cam_params << fx, fy, cx, cy, k1, k2, p1, p2;

  pinhole_radtan4_t cam_geom;
  aprilgrid_t grid(0, rows, cols, size, spacing);

  for (const auto &tag : tags) {
    // Image points (counter-clockwise, from bottom left)
    grid.add(tag.id, 0, vec2_t(tag.p[0].first, tag.p[0].second)); // Bottom left
    grid.add(tag.id,
             1,
             vec2_t(tag.p[1].first, tag.p[1].second)); // Bottom right
    grid.add(tag.id, 2, vec2_t(tag.p[2].first, tag.p[2].second)); // Top right
    grid.add(tag.id, 3, vec2_t(tag.p[3].first, tag.p[3].second)); // Top left
  }

  {
    auto t = yac::tic();
    mat4_t T_CF;
    grid.estimate(&cam_geom, cam_res, cam_params, T_CF);
    printf("OpenCV solvePnP time elasped: %fs\n", yac::toc(&t));
    print_matrix("T_CF", T_CF);
  }

  return 0;
}

int test_aprilgrid_intersect() {
  auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const vec4_t K{458.654, 457.296, 367.215, 248.375};
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  mat4_t T_C0F;
  mat4_t T_C1F;
  auto grid0 = detector.detect(0, image);
  auto grid1 = detector.detect(0, image);

  // Remove tag id 2 from grid1
  grid1.remove(2);
  MU_CHECK(grid0.nb_detections == 36 * 4);
  MU_CHECK(grid1.nb_detections == 35 * 4);
  MU_CHECK(grid0.tag_ids().size() == 36);
  MU_CHECK(grid1.tag_ids().size() == 35);

  // Test intersect
  grid0.intersect(grid1);
  MU_CHECK(grid0.nb_detections == 35 * 4);
  MU_CHECK(grid1.nb_detections == 35 * 4);
  MU_CHECK(grid0.tag_ids().size() == 35);
  MU_CHECK(grid1.tag_ids().size() == 35);

  return 0;
}

int test_aprilgrid_intersect2() {
  auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const vec4_t K{458.654, 457.296, 367.215, 248.375};
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  auto grid0 = detector.detect(0, image);
  auto grid1 = detector.detect(0, image);
  auto grid2 = detector.detect(0, image);

  // Randomly remove between 2 to 10 tags from grid0, grid1 and grid2
  for (int i = 0; i < randi(2, 10); i++) {
    grid0.remove(randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    grid1.remove(randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    grid2.remove(randi(0, 35));
  }
  MU_CHECK(grid0.nb_detections < (36 * 4));
  MU_CHECK(grid1.nb_detections < (36 * 4));
  MU_CHECK(grid2.nb_detections < (36 * 4));

  // Test intersection
  std::vector<aprilgrid_t *> data = {&grid0, &grid1, &grid2};
  aprilgrid_t::intersect(data);
  const int nb_detections = grid0.nb_detections;
  MU_CHECK(grid0.nb_detections == nb_detections);
  MU_CHECK(grid1.nb_detections == nb_detections);
  MU_CHECK(grid2.nb_detections == nb_detections);

  return 0;
}

int test_aprilgrid_sample() {
  auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const vec4_t K{458.654, 457.296, 367.215, 248.375};
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  mat4_t T_CF;
  auto grid = detector.detect(0, image);

  const size_t n = 144;
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.sample(n, tag_ids, corner_indicies, keypoints, object_points);

  MU_CHECK(tag_ids.size() == n);
  for (size_t i = 0; i < 36; i++) {
    MU_CHECK(std::count(tag_ids.begin(), tag_ids.end(), tag_ids[i]) == 4);
  }
  MU_CHECK(keypoints.size() == n);
  MU_CHECK(object_points.size() == n);

  return 0;
}

int test_aprilgrid_save_and_load() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Test save
  grid.timestamp = 1544020482626424074;
  grid.tag_rows = 6;
  grid.tag_cols = 6;
  grid.tag_size = 0.088;
  grid.tag_spacing = 0.3;

  const vec2_t kp0{1.0, 2.0};
  const vec2_t kp1{3.0, 4.0};
  const vec2_t kp2{5.0, 6.0};
  const vec2_t kp3{7.0, 8.0};
  grid.detected = true;
  grid.add(0, 0, kp0);
  grid.add(0, 1, kp1);
  grid.add(0, 2, kp2);
  grid.add(0, 3, kp3);

  MU_CHECK(grid.save(TEST_OUTPUT) == 0);

  // Test load
  aprilgrid_t grid2(TEST_OUTPUT);
  std::cout << "timestamp: " << grid2.timestamp << std::endl;
  std::cout << "tag_rows: " << grid2.tag_rows << std::endl;
  std::cout << "tag_cols: " << grid2.tag_cols << std::endl;
  std::cout << "tag_size: " << grid2.tag_size << std::endl;
  std::cout << "tag_spacing: " << grid2.tag_spacing << std::endl;
  std::cout << "detected: " << grid2.detected << std::endl;
  std::cout << "nb_detections: " << grid2.nb_detections << std::endl;
  std::cout << "data:\n" << grid2.data << std::endl;

  MU_CHECK(grid2.timestamp == grid.timestamp);
  MU_CHECK(grid2.detected == true);
  MU_CHECK(grid2.nb_detections == 4);
  MU_CHECK(grid2.tag_ids().size() == 1);
  MU_CHECK(grid2.keypoints().size() == 4);
  MU_CHECK_FLOAT(0.0, (grid2.keypoints()[0] - kp0).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints()[1] - kp1).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints()[2] - kp2).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints()[3] - kp3).norm());

  return 0;
}

int test_aprilgrid_print() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

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

  std::cout << grid << std::endl;

  return 0;
}

int test_aprilgrid_detect() {
  auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const vec4_t K{458.654, 457.296, 367.215, 248.375};
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_t grid = detector.detect(0, image);

  cv::imwrite("/tmp/grid.png", grid.draw(image));
  MU_CHECK(grid.nb_detections > 0);
  // grid.imshow("viz", image);
  // cv::waitKey(0);

  // const auto detector = aprilgrid_detector_t(6, 6, 0.088, 0.3);
  // const std::string img_dir = "/data/euroc/cam_april/mav0/cam0/data";
  // std::vector<std::string> img_paths;
  // list_files(img_dir, img_paths);
  //
  // for (const auto img_fname : img_paths) {
  //   const std::string img_path = img_dir + "/" + img_fname;
  //   const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
  //   aprilgrid_t grid = detector.detect(0, img);
  //   grid.imshow("viz", img);
  //   cv::waitKey(1);
  // }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_aprilgrid_constructor);
  MU_ADD_TEST(test_aprilgrid_grid_index);
  MU_ADD_TEST(test_aprilgrid_object_point);
  MU_ADD_TEST(test_aprilgrid_keypoint);
  MU_ADD_TEST(test_aprilgrid_keypoints);
  MU_ADD_TEST(test_aprilgrid_add);
  MU_ADD_TEST(test_aprilgrid_remove);
  MU_ADD_TEST(test_aprilgrid_estimate);
  MU_ADD_TEST(test_aprilgrid_intersect);
  MU_ADD_TEST(test_aprilgrid_intersect2);
  MU_ADD_TEST(test_aprilgrid_sample);
  MU_ADD_TEST(test_aprilgrid_save_and_load);
  MU_ADD_TEST(test_aprilgrid_print);
  MU_ADD_TEST(test_aprilgrid_detect);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
