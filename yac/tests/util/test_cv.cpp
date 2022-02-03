#include "../munit.hpp"

#include "util/data.hpp"
#include "util/cv.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define CV_TEST_IMAGE TEST_PATH "/test_data/core/test_image.jpg"

void setup_pinhole_radtan4_test(int res[2], vecx_t &params) {
  res[0] = 640;
  res[1] = 480;

  const real_t fx = pinhole_focal(res[0], 90.0);
  const real_t fy = pinhole_focal(res[0], 90.0);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t p1 = 0.001;
  const real_t p2 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;
}

void setup_pinhole_equi4_test(int res[2], vecx_t &params) {
  res[0] = 640;
  res[1] = 480;

  const real_t fx = pinhole_focal(res[0], 90.0);
  const real_t fy = pinhole_focal(res[0], 90.0);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.001;
  const real_t k4 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, k3, k4;
}

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
      cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
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

  const vec3_t p_C{0.0, 0.0, 1.0};
  vec2_t z_hat;
  int retval = pinhole_project(res, proj_params, p_C, z_hat);
  MU_CHECK(retval == 0);
  MU_CHECK(fltcmp(z_hat(0), 320) == 0);
  MU_CHECK(fltcmp(z_hat(1), 240) == 0);

  return 0;
}

int test_pinhole_radtan4_project() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_radtan4_test(res, params);

  // Test pinhole radtan4 project
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat;
  int retval = pinhole_radtan4_project(res, params, p_C, z_hat);
  MU_CHECK(retval == 0);
  // print_vector("z_hat", z_hat);

  // Test OpenCV's version
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double k1 = params(4);
  const double k2 = params(5);
  const double p1 = params(6);
  const double p2 = params(7);
  const cv::Point3f obj_pt(p_C.x(), p_C.y(), p_C.z());
  const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  const cv::Mat D = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const std::vector<cv::Point3f> object_points = {obj_pt};
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, rvec, tvec, K, D, image_points);

  const auto z_hat_gnd = vec2_t{image_points[0].x, image_points[0].y};
  // print_vector("z_hat    ", z_hat);
  // print_vector("z_hat_gnd", z_hat_gnd);
  MU_CHECK(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  MU_CHECK(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);

  return 0;
}

int test_pinhole_radtan4_project_jacobian() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_radtan4_test(res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_radtan4_project_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 3);

  vec2_t z_hat;
  pinhole_radtan4_project(res, params, p_C, z_hat);

  for (int i = 0; i < 3; i++) {
    vec3_t p_C_diff = p_C;
    p_C_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_radtan4_project(res, params, p_C_diff, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  // print_matrix("J", J);
  // print_matrix("fdiff", fdiff);
  MU_CHECK((J - fdiff).norm() < 1e-4);

  return 0;
}

int test_pinhole_radtan4_params_jacobian() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_radtan4_test(res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_radtan4_params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 8);

  vec2_t z_hat;
  pinhole_radtan4_project(res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    vecx_t params_diff = params;
    params_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_radtan4_project(res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  // print_matrix("J", J);
  // print_matrix("fdiff", fdiff);
  MU_CHECK((J - fdiff).norm() < 1e-4);

  return 0;
}

int test_pinhole_radtan4_undistort() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_radtan4_test(res, params);

  // Project without distortion
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat_gnd;
  pinhole_project(res, params.head(4), p_C, z_hat_gnd);

  // Project with distortion and undistort it
  vec2_t z_hat;
  pinhole_radtan4_project(res, params, p_C, z_hat);
  z_hat = pinhole_radtan4_undistort(params, z_hat);

  // print_vector("z_hat_gnd", z_hat_gnd);
  // print_vector("z_hat    ", z_hat);
  MU_CHECK(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  MU_CHECK(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);

  return 0;
}

int test_pinhole_equi4_project() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_equi4_test(res, params);

  // Test pinhole equi4 project
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat;
  int retval = pinhole_equi4_project(res, params, p_C, z_hat);
  MU_CHECK(retval == 0);

  // Test OpenCV's version
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double k1 = params(4);
  const double k2 = params(5);
  const double k3 = params(6);
  const double k4 = params(7);
  const cv::Point3f obj_pt(p_C.x(), p_C.y(), p_C.z());
  const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  const cv::Mat D = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const std::vector<cv::Point3f> object_points = {obj_pt};
  std::vector<cv::Point2f> image_points;
  cv::fisheye::projectPoints(object_points, image_points, rvec, tvec, K, D);

  const auto z_hat_gnd = vec2_t{image_points[0].x, image_points[0].y};
  // print_vector("z_hat    ", z_hat);
  // print_vector("z_hat_gnd", z_hat_gnd);
  MU_CHECK(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  MU_CHECK(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);

  return 0;
}

int test_pinhole_equi4_project_jacobian() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_equi4_test(res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_equi4_project_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 3);

  vec2_t z_hat;
  pinhole_equi4_project(res, params, p_C, z_hat);

  for (int i = 0; i < 3; i++) {
    vec3_t p_C_diff = p_C;
    p_C_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_equi4_project(res, params, p_C_diff, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  // print_matrix("J", J);
  // print_matrix("fdiff", fdiff);
  MU_CHECK((J - fdiff).norm() < 1e-4);

  return 0;
}

int test_pinhole_equi4_params_jacobian() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_equi4_test(res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_equi4_params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 8);

  vec2_t z_hat;
  pinhole_equi4_project(res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    vecx_t params_diff = params;
    params_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_equi4_project(res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  // print_matrix("J", J);
  // print_matrix("fdiff", fdiff);
  MU_CHECK((J - fdiff).norm() < 1e-4);

  return 0;
}

int test_pinhole_equi4_undistort() {
  // Setup
  int res[2];
  vecx_t params;
  setup_pinhole_equi4_test(res, params);

  // Project without distortion
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat_gnd;
  pinhole_project(res, params.head(4), p_C, z_hat_gnd);

  // Project with distortion and undistort
  vec2_t z_hat;
  pinhole_equi4_project(res, params, p_C, z_hat);
  z_hat = pinhole_equi4_undistort(params, z_hat);

  // print_vector("z_hat_gnd", z_hat_gnd);
  // print_vector("z_hat    ", z_hat);
  MU_CHECK(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  MU_CHECK(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);

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

  MU_ADD_TEST(test_pinhole_radtan4_project);
  MU_ADD_TEST(test_pinhole_radtan4_project_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_params_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_undistort);

  MU_ADD_TEST(test_pinhole_equi4_project);
  MU_ADD_TEST(test_pinhole_equi4_project_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_params_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_undistort);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
