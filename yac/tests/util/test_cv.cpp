#include "../munit.hpp"

#include "util/data.hpp"
#include "util/cv.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define CV_TEST_IMAGE TEST_PATH "/test_data/core/test_image.jpg"

int test_feature_mask() {
  const auto image = cv::imread(CV_TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Cannot load image [%s]!", CV_TEST_IMAGE);
    return -1;
  }

  const int image_width = image.cols;
  const int image_height = image.rows;
  auto keypoints = grid_fast(image, 100, 5, 5, 30);
  std::vector<cv::Point2f> points;
  for (auto kp : keypoints) {
    points.emplace_back(kp);
  }
  points.emplace_back(0, 0);
  points.emplace_back(image_height, image_width);
  points.emplace_back(0, image_width);
  points.emplace_back(image_height, 0);

  auto mask = feature_mask(image_width, image_height, points, 4);
  const bool debug = false;
  if (debug) {
    cv::imshow("Mask", convert(mask));
    cv::waitKey(0);
  }

  return 0;
}

int test_grid_fast() {
  const cv::Mat image = cv::imread(CV_TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Cannot load image [%s]!", CV_TEST_IMAGE);
    return -1;
  }

  auto features = grid_fast(image, // Input image
                            1000,  // Max number of corners
                            5,     // Grid rows
                            5,     // Grid columns
                            10.0,  // Threshold
                            true); // Nonmax suppression

  bool debug = false;
  if (debug) {
    auto out_image = draw_grid_features(image, 5, 5, features);
    cv::imshow("Grid Features", out_image);
    cv::waitKey(0);
  }

  return 0;
}

int test_grid_good() {
  const cv::Mat image = cv::imread(CV_TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Cannot load image [%s]!", CV_TEST_IMAGE);
    return -1;
  }

  auto features = grid_good(image, // Input image
                            1000,  // Max number of corners
                            5,     // Grid rows
                            5);    // Grid columns

  bool debug = false;
  if (debug) {
    auto out_image = draw_grid_features(image, 5, 5, features);
    cv::imshow("Grid Features", out_image);
    cv::waitKey(0);
  }

  return 0;
}

int benchmark_grid_fast() {
  // Grid-FAST corner detector
  {
    const cv::Mat image = cv::imread(CV_TEST_IMAGE);
    auto keypoints = grid_fast(image, // Input image
                               1000,  // Max number of corners
                               10,    // Grid rows
                               10,    // Grid columns
                               10.0,  // Threshold
                               true); // Nonmax suppression

    // Save keypoints to file
    matx_t data;
    data.resize(keypoints.size(), 2);
    int row_index = 0;
    for (auto kp : keypoints) {
      data(row_index, 0) = kp.x;
      data(row_index, 1) = kp.y;
      row_index++;
    }
    mat2csv("/tmp/grid_fast.csv", data);
    cv::imwrite("/tmp/grid_fast.png", image);
  }

  // Standard FAST corner detector
  {
    // Prepare input image - make sure it is grayscale
    const cv::Mat image = cv::imread(CV_TEST_IMAGE);
    cv::Mat image_gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    } else {
      image_gray = image.clone();
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(image_gray, // Input image
             keypoints,  // Keypoints
             10.0,       // Threshold
             true);      // Nonmax suppression

    // Sort by keypoint response
    keypoints = sort_keypoints(keypoints, 1000);

    // Draw corners
    for (auto kp : keypoints) {
      cv::circle(image, kp.pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // Draw
    cv::imshow("FAST", image);
    cv::waitKey(1);

    // Save image and keypoints to file
    matx_t data;
    data.resize(keypoints.size(), 2);
    int row_index = 0;
    for (auto kp : keypoints) {
      data(row_index, 0) = kp.pt.x;
      data(row_index, 1) = kp.pt.y;
      row_index++;
    }
    mat2csv("/tmp/fast.csv", data);
    cv::imwrite("/tmp/fast.png", image);
  }

  // Visualize results
  cv::waitKey(0);

  return 0;
}

struct vision_test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = pinhole_focal(image_width, fov);
  const double fy = pinhole_focal(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
};

int test_radtan4_distort() {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.01, 0.01, 0.01};
  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t x{p(0) / p(2), p(1) / p(2)};
    const vec2_t pixel = radtan4_distort(dist_params, x);

    // Use opencv to use radtan distortion to distort point
    const std::vector<cv::Point3f> points{cv::Point3f(p(0), p(1), p(2))};
    const cv::Vec3f rvec;
    const cv::Vec3f tvec;
    const cv::Mat K = convert(I(3));
    const cv::Vec4f D(k1, k2, p1, p2);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(points, rvec, tvec, K, D, image_points);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((pixel - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_radtan4_undistort() {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.01, 0.01, 0.01};

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = radtan4_distort(dist_params, p);
    const vec2_t p_ud = radtan4_undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p - p_ud).norm() < 1.0e-5);
  }

  return 0;
}

int test_equi_distort() {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.01, 0.01, 0.01};
  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t k3 = dist_params(2);
  const real_t k4 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = equi4_distort(dist_params, p);

    // Use opencv to use equi distortion to distort point
    const std::vector<cv::Point2f> points{cv::Point2f(p(0), p(1))};
    const cv::Mat K = convert(I(3));
    const cv::Vec4f D(k1, k2, k3, k4);
    std::vector<cv::Point2f> image_points;
    cv::fisheye::distortPoints(points, image_points, K, D);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p_d - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_equi_undistort() {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.2, 0.3, 0.4};

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = equi4_distort(dist_params, p);
    const vec2_t p_ud = equi4_undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p - p_ud).norm() < 1.0e-5);
  }

  return 0;
}

int test_pinhole_focal() {
  const real_t focal = pinhole_focal(640, 90.0);
  MU_CHECK(fltcmp(focal, 320) == 0);

  return 0;
}

int test_pinhole_K() {
  const int res[2] = {640, 480};
  const real_t fx = pinhole_focal(res[0], 90.0);
  const real_t fy = pinhole_focal(res[0], 90.0);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const mat3_t K = pinhole_K(fx, fy, cx, cy);
  MU_CHECK(fltcmp(K(0, 0), fx) == 0);
  MU_CHECK(fltcmp(K(1, 1), fy) == 0);
  MU_CHECK(fltcmp(K(0, 2), cx) == 0);
  MU_CHECK(fltcmp(K(1, 2), cy) == 0);
  MU_CHECK(fltcmp(K(2, 2), 1.0) == 0);

  return 0;
}

int test_pinhole_project() {
  const int res[2] = {640, 480};
  const real_t fx = pinhole_focal(res[0], 90.0);
  const real_t fy = pinhole_focal(res[0], 90.0);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};

  const vec3_t p{0.0, 0.0, 1.0};
  vec2_t z_hat;
  int retval = pinhole_project(res, proj_params, p, z_hat);
  MU_CHECK(retval == 0);
  MU_CHECK(fltcmp(z_hat(0), 320) == 0);
  MU_CHECK(fltcmp(z_hat(1), 240) == 0);

  return 0;
}

void test_suite() {
  // Vision
  MU_ADD_TEST(test_feature_mask);
  MU_ADD_TEST(test_grid_fast);
  MU_ADD_TEST(test_grid_good);
  // MU_ADD_TEST(benchmark_grid_fast);

  MU_ADD_TEST(test_radtan4_distort);
  MU_ADD_TEST(test_radtan4_undistort);

  MU_ADD_TEST(test_equi_distort);
  MU_ADD_TEST(test_equi_undistort);

  MU_ADD_TEST(test_pinhole_focal);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_project);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
