#include <gtest/gtest.h>

#include "Core.hpp"
#include "CameraModel.hpp"

namespace yac {

void setup_pinhole_radtan4_test(vec2i_t &cam_res, vecx_t &params) {
  cam_res = vec2i_t{640, 480};

  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double p1 = 0.001;
  const double p2 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;
}

void setup_pinhole_equi4_test(vec2i_t &cam_res, vecx_t &params) {
  cam_res = vec2i_t{640, 480};

  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double k3 = 0.001;
  const double k4 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, k3, k4;
}

TEST(CameraModel, radtan4_distort) {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t x{p(0) / p(2), p(1) / p(2)};
    const vec2_t pixel = radtan4_distort(dist_params, x);

    // Use opencv to use radtan distortion to distort point
    const std::vector<cv::Point3f> points{cv::Point3f(p(0), p(1), p(2))};
    const cv::Vec3f rvec;
    const cv::Vec3f tvec;
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, p1, p2);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(points, rvec, tvec, K, D, image_points);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((pixel - expected).norm() < 1.0e-5);
  }
}

TEST(CameraModel, radtan4_undistort) {
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

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

TEST(CameraModel, equi_distort) {
  const int nb_points = 100;
  const vec4_t dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = equi4_distort(dist_params, p);

    // Use opencv to use equi distortion to distort point
    const std::vector<cv::Point2f> points{cv::Point2f(p(0), p(1))};
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, k3, k4);
    std::vector<cv::Point2f> image_points;
    cv::fisheye::distortPoints(points, image_points, K, D);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p_d - expected).norm() < 1.0e-5);
  }
}

TEST(CameraModel, equi_undistort) {
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

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

TEST(CameraModel, pinhole_focal) {
  const double focal = pinhole_focal(640, 90.0);
  ASSERT_TRUE(fltcmp(focal, 320) == 0);
}

TEST(CameraModel, pinhole_K) {
  const vec2i_t cam_res = {640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const mat3_t K = pinhole_K(fx, fy, cx, cy);
  ASSERT_FLOAT_EQ(K(0, 0), fx);
  ASSERT_FLOAT_EQ(K(1, 1), fy);
  ASSERT_FLOAT_EQ(K(0, 2), cx);
  ASSERT_FLOAT_EQ(K(1, 2), cy);
  ASSERT_FLOAT_EQ(K(2, 2), 1.0);
}

TEST(CameraModel, pinhole_project) {
  const vec2i_t cam_res = {640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};

  const vec3_t p_C{0.0, 0.0, 1.0};
  vec2_t z_hat;
  int retval = pinhole_project(cam_res, proj_params, p_C, z_hat);
  ASSERT_EQ(retval, 0);
  ASSERT_FLOAT_EQ(z_hat(0), 320.0);
  ASSERT_FLOAT_EQ(z_hat(1), 240.0);
}

TEST(CameraModel, pinhole_radtan4_project) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_radtan4_test(cam_res, params);

  // Test pinhole radtan4 project
  for (int i = 0; i < 1000; i++) {
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const vec3_t p_C{px, py, pz};
    vec2_t z_hat;
    int retval = pinhole_radtan4_project(cam_res, params, p_C, z_hat);
    if (retval != 0) {
      continue;
    }

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
    ASSERT_NEAR(z_hat_gnd.x(), z_hat.x(), 1e-4);
    ASSERT_NEAR(z_hat_gnd.y(), z_hat.y(), 1e-4);
  }
}

TEST(CameraModel, pinhole_radtan4_project_jacobian) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_radtan4_test(cam_res, params);

  // Analytical jacobian
  for (int i = 0; i < 100; i++) {
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const vec3_t p_C{px, py, pz};

    vec2_t z_hat;
    if (pinhole_radtan4_project(cam_res, params, p_C, z_hat) != 0) {
      continue;
    }
    const matx_t J = pinhole_radtan4_project_jacobian(params, p_C);

    // Numerical diff
    const double step = 1e-8;
    matx_t fdiff = zeros(2, 3);

    for (int i = 0; i < 3; i++) {
      vec3_t p_C_diff = p_C;
      p_C_diff(i) += step;

      vec2_t z_hat_prime;
      pinhole_radtan4_project(cam_res, params, p_C_diff, z_hat_prime);
      fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
    }
    ASSERT_TRUE((J - fdiff).norm() < 1e-4);
  }
}

TEST(CameraModel, pinhole_radtan4_params_jacobian) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_radtan4_test(cam_res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_radtan4_params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 8);

  vec2_t z_hat;
  pinhole_radtan4_project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    vecx_t params_diff = params;
    params_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_radtan4_project(cam_res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

static vec2_t opencv_undistort_point(const vecx_t &cam_params,
                                     const vec2_t &kp) {
  std::vector<cv::Point2f> pts_in;
  pts_in.emplace_back(kp.x(), kp.y());

  std::vector<cv::Point2f> pts_out;

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  std::vector<double> D;
  D.push_back(cam_params(4));
  D.push_back(cam_params(5));
  D.push_back(cam_params(6));
  D.push_back(cam_params(7));

  cv::undistortPoints(pts_in, pts_out, K, D, cv::noArray(), K);
  const vec2_t kp_opencv{pts_out[0].x, pts_out[0].y};
  return kp_opencv;
}

TEST(CameraModel, pinhole_radtan4_undistort) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_radtan4_test(cam_res, params);

  for (int i = 0; i < 100; i++) {
    // Project without distortion
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const vec3_t p_C{px, py, pz};
    vec2_t kp_gnd;
    if (pinhole_project(cam_res, params.head(4), p_C, kp_gnd) != 0) {
      continue;
    }

    // Project with distortion
    vec2_t kp_dist;
    pinhole_radtan4_project(cam_res, params, p_C, kp_dist);

    // OpenCV undistort
    const vec2_t kp_opencv = opencv_undistort_point(params, kp_dist);

    // Project with distortion and undistort it
    const vec2_t kp_yac = pinhole_radtan4_undistort(params, kp_dist);

    ASSERT_FLOAT_EQ(kp_gnd.x(), kp_yac.x());
    ASSERT_FLOAT_EQ(kp_gnd.y(), kp_yac.y());
    ASSERT_NEAR(kp_opencv.x(), kp_yac.x(), 1.0);
    ASSERT_NEAR(kp_opencv.y(), kp_yac.y(), 1.0);
  }
}

TEST(CameraModel, pinhole_equi4_project) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_equi4_test(cam_res, params);

  // Test pinhole equi4 project
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat;
  int retval = pinhole_equi4_project(cam_res, params, p_C, z_hat);
  ASSERT_TRUE(retval == 0);

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
  ASSERT_TRUE(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  ASSERT_TRUE(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);
}

TEST(CameraModel, pinhole_equi4_project_jacobian) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_equi4_test(cam_res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 1.0};
  const matx_t J = pinhole_equi4_project_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 3);

  vec2_t z_hat;
  pinhole_equi4_project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 3; i++) {
    vec3_t p_C_diff = p_C;
    p_C_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_equi4_project(cam_res, params, p_C_diff, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(CameraModel, pinhole_equi4_params_jacobian) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_equi4_test(cam_res, params);

  // Analytical jacobian
  const vec3_t p_C{0.1, 0.2, 5.0};
  const matx_t J = pinhole_equi4_params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  matx_t fdiff = zeros(2, 8);

  vec2_t z_hat;
  pinhole_equi4_project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    vecx_t params_diff = params;
    params_diff(i) += step;

    vec2_t z_hat_prime;
    pinhole_equi4_project(cam_res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(CameraModel, pinhole_equi4_undistort) {
  // Setup
  vec2i_t cam_res;
  vecx_t params;
  setup_pinhole_equi4_test(cam_res, params);

  // Project without distortion
  const vec3_t p_C{0.1, 0.2, 1.0};
  vec2_t z_hat_gnd;
  pinhole_project(cam_res, params.head(4), p_C, z_hat_gnd);

  // Project with distortion and undistort
  vec2_t z_hat;
  pinhole_equi4_project(cam_res, params, p_C, z_hat);
  z_hat = pinhole_equi4_undistort(params, z_hat);

  ASSERT_FLOAT_EQ(z_hat_gnd.x(), z_hat.x());
  ASSERT_FLOAT_EQ(z_hat_gnd.y(), z_hat.y());
}

} // namespace yac
