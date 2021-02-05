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

void fit_circle(const vec2s_t &points, double &cx, double &cy, double &radius) {
  assert(points.size() > 3);

  // Parametric circle equation
  // (x - cx)^2 + (y - cy)^2 = r^2

  // Expand and rewrite the circle equation
  // (x^2 - 2x * cx + cx^2) + (y^2 - 2y * cy + cy^2) = r^2
  // -2x * cx + cx^2 - 2y * cy + cy^2 = r^2 - x^2 - y^2
  // (-2x * cx + cx^2) - (2y * cy + cy^2) - r^2 = -(x^2 + y^2)
  // (-2x * cx) + (-2y * cy) + (-r^2  + cx^2 + cy^2) = -(x^2 + y^2)

  // Matrix form: Ax = b
  // Let
  //   A = [-2x -2y 1]
  //   x = [cx, cy, -r^2 + cx^2 + cy^2]'
  //   b = [-(x^2 + y^2)]'
  // [-2x -2y 1] [cx cy -r^2+cx^2+cy^2]' = [-(x^2 + y^2)]'

  // Form A matrix and vector b
  int nb_points = points.size();
  matx_t A;
  vecx_t b;
  A.resize(nb_points, 3);
  b.resize(nb_points, 1);

  for (int i = 0; i < nb_points; i++) {
    const vec2_t p = points[i];
    A(i, 0) = -2.0 * p.x();
    A(i, 1) = -2.0 * p.y();
    A(i, 2) = 1.0;
    b(i) = -(p.x() * p.x() + p.y() * p.y());
  }

  // Solve Ax = b
  Eigen::JacobiSVD<matx_t> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  vecx_t x = svd.solve(b);

  // Results
  cx = x(0);
  cy = x(1);
  radius= sqrt((cx * cx) + (cy * cy) - x(2));
}

int test_fit_circle() {
  int nb_points = 1000;
  int nb_tests = 10;

  for (int i = 0; i < nb_tests; i++) {
    vec2s_t points;
    double gnd_cx = randf(-5.0, 5.0);
    double gnd_cy = randf(-5.0, 5.0);
    double gnd_radius = randf(0.1, 5.0);

    double theta = 0.0;
    double dtheta = 2.0 * M_PI / (double) nb_points;
    while (theta <= 2.0 * M_PI) {
      const double x = gnd_radius * cos(theta) + gnd_cx;
      const double y = gnd_radius * sin(theta) + gnd_cy;
      points.emplace_back(x, y);
      theta += dtheta;
    }

    double est_cx = 0.0;
    double est_cy = 0.0;
    double est_radius = 0.0;
    fit_circle(points, est_cx, est_cy, est_radius);

    // printf("true_cx: %f, est_cx: %f\n", gnd_cx, est_cx);
    // printf("true_cy: %f, est_cy: %f\n", gnd_cy, est_cy);
    // printf("true_radius: %f, est_radius: %f\n", gnd_radius, est_radius);

    MU_CHECK(fabs(est_cx - gnd_cx) < 1e-4);
    MU_CHECK(fabs(est_cy - gnd_cy) < 1e-4);
    MU_CHECK(fabs(est_radius - gnd_radius) < 1e-4);
  }

  return 0;
}

vec2s_t intersect_circles(const double cx0, const double cy0, const double r0,
                          const double cx1, const double cy1, const double r1) {
  vec2s_t ipts;

  // Check if circles are separate
  double d = sqrt(pow(cx0 - cx1, 2) + pow(cy0 - cy1, 2));
  if (d > r0 + r1) {
    return ipts;
  }

  // Check if one circle is contained within the other
  if (d < fabs(r0 - r1)) {
    return ipts;
  }

  // Check if circles intersect only at a single point
  double a = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2.0 * d);
  double h = sqrt(pow(r0, 2) - pow(a, 2));
  double x3 = cx0 + a * (cx1 - cx0) / d;
  double y3 = cy0 + a * (cy1 - cy0) / d;
  if (h < 1e-10) {
    ipts.emplace_back(x3, y3);
    return ipts;
  }

  // Circles interset at two points
  ipts.emplace_back(x3 + h * (cy1 - cy0) / d, y3 - h * (cx1 - cx0) / d);
  ipts.emplace_back(x3 - h * (cy1 - cy0) / d, y3 + h * (cx1 - cx0) / d);
}

int focal_init(const aprilgrid_t &grid, const int axis, double &focal) {
  // Using the method in:
  // C. Hughes, P. Denny, M. Glavin, and E. Jones,
  // Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
  // Extraction, PAMI 2010

  // Method summary:
  // 1. Form vanishing point lines from aprilgrid
  // 2. For each of these lines estimate a circle to obtain cx, cy, radius
  // 3. For each pair of circles find the intersection (i.e. vanishing points)
  // 4. From each pair of vanishing points the focal length can be estimated

  // Form vanishing point lines from aprilgrid and estimate circle params
  // -------------------------------------------------------------------------
  // Form vanishing point lines horizontally or vertically using the aprilgrid
  // detected points.  Fit a circle for each of these lines. By fitting a
  // circle to these lines we obtain circle cx, cy and radius. And from that we
  // can find the intersection of these circles / lines to obtain vanishing
  // points.
  std::vector<vec2_t> centers;
  std::vector<double> radiuses;

  if (axis == 0) {
    int tag_id = 0;
    for (int i = 0; i < grid.tag_rows; i++) {
      vec2s_t points;
      for (int j = 0; j < grid.tag_cols; j++) {
        points.push_back(grid.keypoint(tag_id, 0)); // Bottom left
        points.push_back(grid.keypoint(tag_id, 1)); // Bottom right
        tag_id++;
      }

      double cx = 0.0;
      double cy = 0.0;
      double r = 0.0;
      fit_circle(points, cx, cy, r);
      centers.emplace_back(cx, cy);
      radiuses.push_back(r);
    }
  } else if (axis == 1) {
    int tag_id = 0;
    for (int j = 0; j < grid.tag_cols; j++) {
      vec2s_t points;
      for (int i = 0; i < grid.tag_rows; i++) {
        points.push_back(grid.keypoint(tag_id, 0)); // Bottom left
        points.push_back(grid.keypoint(tag_id, 3)); // Top left
        tag_id++;
      }

      double cx = 0.0;
      double cy = 0.0;
      double r = 0.0;
      fit_circle(points, cx, cy, r);
      centers.emplace_back(cx, cy);
      radiuses.push_back(r);
    }
  } else {
    FATAL("Invalid axis[%d], expecting 0 or 1!", axis);
  }

  // Estimate focal lengths between all pairs of vanishing points
  std::vector<double> focal_guesses;
  int nb_circles = centers.size();
  for (int i = 0; i < nb_circles; i++) {
    for (int j = i + 1; j < nb_circles; j++) {
      // Find vanishing points between two circles
      const auto cx0 = centers[i].x();
      const auto cy0 = centers[i].y();
      const auto r0 = radiuses[i];
      const auto cx1 = centers[j].x();
      const auto cy1 = centers[j].y();
      const auto r1 = radiuses[j];
      const auto ipts = intersect_circles(cx0, cy0, r0, cx1, cy1, r1);
      if (ipts.size() < 2) {
        continue;
      }

      // Estimate focal length. Equation (8) in the paper.
      const vec2_t vp0 = ipts[0];
      const vec2_t vp1 = ipts[1];
      double f_guess = (vp0 - vp1).norm() / M_PI;
      focal_guesses.push_back(f_guess);
    }
  }

  // From all the focal length guesses return the median
  focal = median(focal_guesses);

  return 0;
}

int test_focal_init() {
  const auto image = cv::imread(CAM0_IMAGE);

  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  const auto grid = detector.detect(0, image);

  double focal = 0;
  focal_init(grid, 0, focal);
  printf("focal: %f\n", focal);

  // cv::imshow("Image", image);
  // cv::waitKey(0);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_preprocess_and_load_camera_data);
  // MU_ADD_TEST(test_load_multicam_calib_data);
  MU_ADD_TEST(test_blur_measure);
  MU_ADD_TEST(test_fit_circle);
  MU_ADD_TEST(test_focal_init);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
