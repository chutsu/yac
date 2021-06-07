#ifndef YAC_CV_HPP
#define YAC_CV_HPP

#include "core.hpp"

namespace yac {

/*****************************************************************************
 *                                  CV
 ****************************************************************************/

/**
 * Compare `cv::Mat` whether they are equal
 *
 * @param m1 First matrix
 * @param m2 Second matrix
 * @returns true or false
 */
bool is_equal(const cv::Mat &m1, const cv::Mat &m2);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const cv::Mat &x, matx_t &y);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const matx_t &x, cv::Mat &y);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @returns Matrix as Eigen::Matrix
 */
matx_t convert(const cv::Mat &x);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @returns Matrix as cv::Mat
 */
cv::Mat convert(const matx_t &x);

/**
 * Sort Keypoints
 *
 * @param keypoints
 * @param limit
 * @returns Sorted keypoints by response
 */
std::vector<cv::KeyPoint> sort_keypoints(
    const std::vector<cv::KeyPoint> keypoints, const size_t limit = 0);

/**
 * Convert gray-scale image to rgb image
 *
 * @param image
 *
 * @returns RGB image
 */
cv::Mat gray2rgb(const cv::Mat &image);

/**
 * Convert rgb image to gray-scale image
 *
 * @param image
 *
 * @returns Gray-scale image
 */
cv::Mat rgb2gray(const cv::Mat &image);

/**
 * Create ROI from an image
 *
 * @param[in] image Input image
 * @param[in] width ROI width
 * @param[in] height ROI height
 * @param[in] cx ROI center x-axis
 * @param[in] cy ROI center y-axis
 *
 * @returns ROI
 */
cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const real_t cx,
            const real_t cy);

/**
 * Compare two keypoints based on the response.
 *
 * @param[in] kp1 First keypoint
 * @param[in] kp2 Second keypoint
 * @returns Boolean to denote if first keypoint repose is larger than second
 */
bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 *
 * @returns Undistorted image using radial-tangential distortion
 */
cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 * @param[in] balance Balance
 * @param[in,out] Knew New camera matrix K
 *
 * @returns Undistorted image using equidistant distortion
 */
cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew);
/**
 * Illumination invariant transform.
 *
 * @param[in] image Image
 * @param[in] lambda_1 Lambad 1
 * @param[in] lambda_2 Lambad 2
 * @param[in] lambda_3 Lambad 3
 */
void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3);

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Image frame 0
 * @param[in] img1 Image frame 1
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] status Inlier vector
 *
 * @returns Image with feature matches between frame 0 and 1
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Previous image frame
 * @param[in] img1 Current image frame
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] matches Feature matches
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features);

/**
 * Grid fast
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] threshold Fast threshold
 * @param[in] nonmax_suppression Nonmax Suppression
 *
 * @returns List of keypoints
 */
std::vector<cv::Point2f> grid_fast(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const real_t threshold = 10.0,
                                   const bool nonmax_suppression = true);

/**
 * Grid good
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] quality_level Quality level
 * @param[in] min_distance Min distance
 * @param[in] mask Mask
 * @param[in] block_size Block size
 * @param[in] use_harris_detector Use Harris detector
 * @param[in] k Free parameter for Harris detector
 *
 * @returns List of points
 */
std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const real_t quality_level = 0.01,
                                   const real_t min_distance = 10,
                                   const cv::Mat mask = cv::Mat(),
                                   const int block_size = 3,
                                   const bool use_harris_detector = false,
                                   const real_t k = 0.04);

/**
 * Distortion model
 */
struct distortion_t {
  vecx_t params;

  distortion_t() {}

  distortion_t(const vecx_t &params_)
    : params{params_} {}

  distortion_t(const real_t *params_, const size_t params_size_) {
    params.resize(params_size_);
    for (size_t i = 0; i < params_size_; i++) {
      params(i) = params_[i];
    }
  }

  virtual ~distortion_t() {}

  virtual vec2_t distort(const vec2_t &p) const = 0;
  virtual vec2_t undistort(const vec2_t &p) const = 0;
  virtual mat2_t J_point(const vec2_t &p) const = 0;
  virtual matx_t J_dist(const vec2_t &p) const = 0;
};

/**
 * No distortion
 */
struct nodist_t : distortion_t {
  static const size_t params_size = 0;

  nodist_t() {}
  nodist_t(const vecx_t &) {}
  nodist_t(const real_t *) {}
  ~nodist_t() {}

  vec2_t distort(const vec2_t &p) const {
    return p;
  }

  vec2_t undistort(const vec2_t &p) const {
    return p;
  }

  mat2_t J_point(const vec2_t &p) const {
    UNUSED(p);
    return I(2);
  }

  matx_t J_dist(const vec2_t &p) const {
    UNUSED(p);
    matx_t J;
    J.resize(2, 0);
    return J;
  }
};

/**
 * Radial-tangential distortion
 */
struct radtan4_t : distortion_t {
  static const size_t params_size = 4;

  radtan4_t() {}

  radtan4_t(const vecx_t &params_)
    : distortion_t{params_} {}

  radtan4_t(const real_t *dist_params)
    : distortion_t{dist_params, params_size} {}

  radtan4_t(const real_t k1,
            const real_t k2,
            const real_t p1,
            const real_t p2)
    : distortion_t{vec4_t{k1, k2, p1, p2}} {}

  virtual ~radtan4_t() {}

  real_t k1() const { return params(0); }
  real_t k2() const { return params(1); }
  real_t p1() const { return params(2); }
  real_t p2() const { return params(3); }

  vec2_t distort(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    // Apply radial distortion
    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;
    const real_t radial_factor = 1 + (k1() * r2) + (k2() * r4);
    const real_t x_dash = x * radial_factor;
    const real_t y_dash = y * radial_factor;

    // Apply tangential distortion
    const real_t xy = x * y;
    const real_t x_ddash = x_dash + (2 * p1() * xy + p2() * (r2 + 2 * x2));
    const real_t y_ddash = y_dash + (p1() * (r2 + 2 * y2) + 2 * p2() * xy);

    return vec2_t{x_ddash, y_ddash};
  }

  vec2_t undistort(const vec2_t &p0) const {
    vec2_t p = p0;
    int max_iter = 5;

    for (int i = 0; i < max_iter; i++) {
      // Error
      const vec2_t p_distorted = distort(p);
      const vec2_t err = (p0 - p_distorted);

      // Jacobian
      mat2_t J = J_point(p);
      const mat2_t pinv = (J.transpose() * J).inverse() * J.transpose();
      const vec2_t dp = pinv * err;
      p = p + dp;

      if ((err.transpose() * err) < 1.0e-15) {
        break;
      }
    }

    return p;
  }

  mat2_t J_point(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;

    // Let p = [x; y] normalized point
    // Let p' be the distorted p
    // The jacobian of p' w.r.t. p (or dp'/dp) is:
    mat2_t J_point;
    J_point(0, 0) = 1 + k1() * r2 + k2() * r4;
    J_point(0, 0) += 2 * p1() * y + 6 * p2() * x;
    J_point(0, 0) += x * (2 * k1() * x + 4 * k2() * x * r2);
    J_point(1, 0) = 2 * p1() * x + 2 * p2() * y;
    J_point(1, 0) += y * (2 * k1() * x + 4 * k2() * x * r2);
    J_point(0, 1) = J_point(1, 0);
    J_point(1, 1) = 1 + k1() * r2 + k2() * r4;
    J_point(1, 1) += 6 * p1() * y + 2 * p2() * x;
    J_point(1, 1) += y * (2 * k1() * y + 4 * k2() * y * r2);
    // Above is generated using sympy

    return J_point;
  }

  matx_t J_dist(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    const real_t xy = x * y;
    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;

    mat_t<2, 4> J_dist = zeros(2, 4);
    J_dist(0, 0) = x * r2;
    J_dist(0, 1) = x * r4;
    J_dist(0, 2) = 2 * xy;
    J_dist(0, 3) = 3 * x2 + y2;

    J_dist(1, 0) = y * r2;
    J_dist(1, 1) = y * r4;
    J_dist(1, 2) = x2 + 3 * y2;
    J_dist(1, 3) = 2 * xy;

    return J_dist;
  }
};

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4);

/**
 * Equi-distant distortion
 */
struct equi4_t : distortion_t {
  static const size_t params_size = 4;

  equi4_t() {}

  equi4_t(const vecx_t &dist_params)
    : distortion_t{dist_params} {}

  equi4_t(const real_t *dist_params)
    : distortion_t{dist_params, params_size} {}

  equi4_t(const real_t k1,
          const real_t k2,
          const real_t k3,
          const real_t k4) {
    params.resize(4);
    params << k1, k2, k3, k4;
  }

  ~equi4_t() {}

  real_t k1() const { return this->params(0); }
  real_t k2() const { return this->params(1); }
  real_t k3() const { return this->params(2); }
  real_t k4() const { return this->params(3); }

  vec2_t distort(const vec2_t &p) const {
    const real_t r = p.norm();
    if (r < 1e-8) {
      return p;
    }

    // Apply equi distortion
    const real_t th = atan(r);
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    const real_t thd = th * (1 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    const real_t x_dash = (thd / r) * p(0);
    const real_t y_dash = (thd / r) * p(1);

    return vec2_t{x_dash, y_dash};
  }

  vec2_t undistort(const vec2_t &p) const {
    const real_t thd = sqrt(p(0) * p(0) + p(1) * p(1));

    real_t th = thd; // Initial guess
    for (int i = 20; i > 0; i--) {
      const real_t th2 = th * th;
      const real_t th4 = th2 * th2;
      const real_t th6 = th4 * th2;
      const real_t th8 = th4 * th4;
      th = thd / (1 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    }

    const real_t scaling = tan(th) / thd;
    return vec2_t{p(0) * scaling, p(1) * scaling};
  }

  mat2_t J_point(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);
    const real_t r = p.norm();
    const real_t th = atan(r);
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    const real_t thd = th * (1.0 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    const real_t s = thd / r;

    // Form jacobian
    const real_t th_r = 1.0 / (r * r + 1.0);
    real_t thd_th = 1.0 + 3.0 * k1() * th2;
    thd_th += 5.0 * k2() * th4;
    thd_th += 7.0 * k3() * th6;
    thd_th += 9.0 * k4() * th8;
    const real_t s_r = thd_th * th_r / r - thd / (r * r);
    const real_t r_x = 1.0 / r * x;
    const real_t r_y = 1.0 / r * y;

    mat2_t J_point = I(2);
    J_point(0, 0) = s + x * s_r * r_x;
    J_point(0, 1) = x * s_r * r_y;
    J_point(1, 0) = y * s_r * r_x;
    J_point(1, 1) = s + y * s_r * r_y;

    return J_point;
  }

  matx_t J_dist(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);
    const real_t r = p.norm();
    const real_t th = atan(r);

    const real_t th3 = th * th * th;
    const real_t th5 = th3 * th * th;
    const real_t th7 = th5 * th * th;
    const real_t th9 = th7 * th * th;

    matx_t J_dist = zeros(2, 4);
    J_dist(0, 0) = x * th3 / r;
    J_dist(0, 1) = x * th5 / r;
    J_dist(0, 2) = x * th7 / r;
    J_dist(0, 3) = x * th9 / r;

    J_dist(1, 0) = y * th3 / r;
    J_dist(1, 1) = y * th5 / r;
    J_dist(1, 2) = y * th7 / r;
    J_dist(1, 3) = y * th9 / r;

    return J_dist;
  }
};

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4);

/* Projection model */
template <typename DM = nodist_t>
struct projection_t {
  int resolution[2] = {0, 0};
  vecx_t params;
  DM distortion;

  projection_t() {}

  projection_t(const int resolution_[2],
               const vecx_t &proj_params_,
               const vecx_t &dist_params_)
    : resolution{resolution_[0], resolution_[1]},
      params{proj_params_},
      distortion{dist_params_} {}

  projection_t(const int resolution_[2],
               const vecx_t &params_,
               const size_t proj_params_size_,
               const size_t dist_params_size_)
    : projection_t{resolution_,
                   params_.head(proj_params_size_),
                   params_.tail(dist_params_size_)} {}

  ~projection_t() {}

  virtual mat2_t J_point() const = 0;
  virtual matx_t J_proj(const vec2_t &p) const = 0;
  virtual matx_t J_dist(const vec2_t &p) const = 0;
};

/**
 * Pinhole projection model
 */
template <typename DM = nodist_t>
struct pinhole_t : projection_t<DM> {
  static const size_t proj_params_size = 4;
  static const size_t dist_params_size = DM::params_size;
  static const size_t params_size = proj_params_size + dist_params_size;

  pinhole_t() {}

  pinhole_t(const int resolution[2],
            const vecx_t &proj_params,
            const vecx_t &dist_params)
    : projection_t<DM>{resolution, proj_params, dist_params} {}

  pinhole_t(const int resolution[2],
            const vecx_t &params)
    : projection_t<DM>{resolution, params, proj_params_size, DM::params_size} {}

  pinhole_t(const int resolution[2],
            const real_t fx,
            const real_t fy,
            const real_t cx,
            const real_t cy)
      : projection_t<DM>{resolution, vec4_t{fx, fy, cx, cy}, zeros(0)} {}

  virtual ~pinhole_t() {}

  real_t fx() const { return this->params(0); }
  real_t fy() const { return this->params(1); }
  real_t cx() const { return this->params(2); }
  real_t cy() const { return this->params(3); }

  vecx_t proj_params() const {
    return this->params;
  }

  vecx_t dist_params() const {
    return this->distortion.params;
  }

  mat3_t K() const {
    mat3_t K = zeros(3, 3);
    K(0, 0) = fx();
    K(1, 1) = fy();
    K(0, 2) = cx();
    K(1, 2) = cy();
    K(2, 2) = 1.0;
    return K;
  }

  int project(const vec3_t &p_C, vec2_t &z_hat) const {
    // Check validity of the point, simple depth test.
    const real_t x = p_C(0);
    const real_t y = p_C(1);
    const real_t z = p_C(2);
    if (z < 0.0) {
      return -1;
    }

    // Project, distort and then scale and center
    const vec2_t p{x / z, y / z};
    const vec2_t p_dist = this->distortion.distort(p);
    z_hat(0) = fx() * p_dist(0) + cx();
    z_hat(1) = fy() * p_dist(1) + cy();

    // Check projection
    const bool x_ok = (z_hat(0) >= 0 && z_hat(0) <= this->resolution[0]);
    const bool y_ok = (z_hat(1) >= 0 && z_hat(1) <= this->resolution[1]);
    if (x_ok == false || y_ok == false) {
      return -2;
    }

    return 0;
  }

  int project(const vec3_t &p_C, vec2_t &z_hat, mat_t<2, 3> &J_h) const {
    int retval = project(p_C, z_hat);

    // Projection Jacobian
    const real_t x = p_C(0);
    const real_t y = p_C(1);
    const real_t z = p_C(2);
    mat_t<2, 3> J_proj;
    J_proj.setZero();
    J_proj(0, 0) = 1.0 / z;
    J_proj(1, 1) = 1.0 / z;
    J_proj(0, 2) = -x / (z * z);
    J_proj(1, 2) = -y / (z * z);

    // Measurement Jacobian
    const vec2_t p{x / z, y / z};
    J_h = J_point() * this->distortion.J_point(p) * J_proj;

    return retval;
  }

  int back_project(const vec2_t &kp, vec3_t &ray) const {
    const real_t px = (kp(0) - cx()) / fx();
    const real_t py = (kp(1) - cy()) / fy();
    const vec2_t p{px, py};

    const vec2_t p_undist = this->distortion.undistort(p);
    ray(0) = p_undist(0);
    ray(1) = p_undist(1);
    ray(2) = 1.0;

    return 0;
  }

  vec2_t undistort_keypoint(const vec2_t &z) const {
    // Back-project and undistort
    const real_t px = (z(0) - cx()) / fx();
    const real_t py = (z(1) - cy()) / fy();
    const vec2_t p{px, py};
    const vec2_t p_undist = this->distortion.undistort(p);

    // Project undistorted point to image plane
    const real_t x = p_undist(0) * fx() + cx();
    const real_t y = p_undist(1) * fy() + cy();
    const vec2_t z_undist = {x, y};

    return z_undist;
  }

  mat2_t J_point() const {
    mat2_t J_K = zeros(2, 2);
    J_K(0, 0) = fx();
    J_K(1, 1) = fy();
    return J_K;
  }

  matx_t J_proj(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    mat_t<2, 4> J_proj = zeros(2, 4);
    J_proj(0, 0) = x;
    J_proj(1, 1) = y;
    J_proj(0, 2) = 1;
    J_proj(1, 3) = 1;

    return J_proj;
  }

  matx_t J_dist(const vec2_t &p) const {
    return J_point() * this->distortion.J_dist(p);
  }

  matx_t J_params(const vec2_t &p) const {
    const vec2_t p_dist = this->distortion.distort(p);

    matx_t J = zeros(2, params_size);
    J.block(0, 0, 2, proj_params_size) = J_proj(p_dist);
    J.block(0, dist_params_size, 2, proj_params_size) = J_dist(p);
    return J;
  }
};

typedef pinhole_t<radtan4_t> pinhole_radtan4_t;
typedef pinhole_t<equi4_t> pinhole_equi4_t;
typedef pinhole_t<nodist_t> pinhole_ideal_t;

template <typename DM>
std::ostream &operator<<(std::ostream &os, const pinhole_t<DM> &pinhole) {
  os << "fx: " << pinhole.fx() << std::endl;
  os << "fy: " << pinhole.fy() << std::endl;
  os << "cx: " << pinhole.cx() << std::endl;
  os << "cy: " << pinhole.cy() << std::endl;

  os << std::endl;
  os << pinhole.distortion << std::endl;
  return os;
}

real_t pinhole_focal(const int image_size, const real_t fov);

mat3_t pinhole_K(const real_t fx,
                 const real_t fy,
                 const real_t cx,
                 const real_t cy);

mat3_t pinhole_K(const vec4_t &params);

mat3_t pinhole_K(const int img_w,
                 const int img_h,
                 const real_t lens_hfov,
                 const real_t lens_vfov);

template <typename CAMERA_TYPE>
int solvepnp(const CAMERA_TYPE &cam,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_CF) {
  assert(keypoints.size() == object_points.size());

  // Create object points (counter-clockwise, from bottom left)
  size_t nb_points = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  for (size_t i = 0; i < nb_points; i++) {
    const vec2_t kp = cam.undistort_keypoint(keypoints[i]);
    const vec3_t pt = object_points[i];
    img_pts.emplace_back(kp(0), kp(1));
    obj_pts.emplace_back(pt(0), pt(1), pt(2));
  }

  // Extract out camera intrinsics
  const vecx_t proj_params = cam.proj_params();
  const double fx = proj_params(0);
  const double fy = proj_params(1);
  const double cx = proj_params(2);
  const double cy = proj_params(3);

  // Solve pnp
  cv::Mat camera_matrix(3, 3, CV_32FC1, 0.0f);
  camera_matrix.at<float>(0, 0) = fx;
  camera_matrix.at<float>(1, 1) = fy;
  camera_matrix.at<float>(0, 2) = cx;
  camera_matrix.at<float>(1, 2) = cy;
  camera_matrix.at<float>(2, 2) = 1.0;
  cv::Vec4f distortion_params(0, 0, 0, 0); // SolvPnP assumes radtan

  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(obj_pts,
              img_pts,
              camera_matrix,
              distortion_params,
              rvec,
              tvec,
              false,
              CV_ITERATIVE);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF = tf(convert(R), convert(tvec));

  return 0;
}

} // namespace yac
#endif // YAC_CV_HPP
