#ifndef YAC_CV_HPP
#define YAC_CV_HPP

#include "core.hpp"

namespace yac {

/*****************************************************************************
 *                                  CV
 ****************************************************************************/

/****************************** OPENCV UTILS ********************************/

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

// /**
//  * Create ROI from an image
//  *
//  * @param[in] image Input image
//  * @param[in] width ROI width
//  * @param[in] height ROI height
//  * @param[in] cx ROI center x-axis
//  * @param[in] cy ROI center y-axis
//  *
//  * @returns ROI
//  */
// cv::Mat roi(const cv::Mat &image,
//             const int width,
//             const int height,
//             const real_t cx,
//             const real_t cy);
//
// /**
//  * Compare two keypoints based on the response.
//  *
//  * @param[in] kp1 First keypoint
//  * @param[in] kp2 Second keypoint
//  * @returns Boolean to denote if first keypoint repose is larger than second
//  */
// bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
//                                   const cv::KeyPoint &kp2);
//
// /**
//  * Calculate reprojection error
//  *
//  * @param[in] measured Measured image pixels
//  * @param[in] projected Projected image pixels
//  * @returns Reprojection error
//  */
// real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected);
//
// /**
//  * Calculate reprojection error
//  *
//  * @param[in] measured Measured image pixels
//  * @param[in] projected Projected image pixels
//  * @returns Reprojection error
//  */
// real_t reprojection_error(const std::vector<cv::Point2f> &measured,
//                           const std::vector<cv::Point2f> &projected);
//
// /**
//  * Create feature mask
//  *
//  * @param[in] image_width Image width
//  * @param[in] image_height Image height
//  * @param[in] points Points
//  * @param[in] patch_width Patch width
//  *
//  * @returns Feature mask
//  */
// matx_t feature_mask(const int image_width,
//                     const int image_height,
//                     const std::vector<cv::Point2f> points,
//                     const int patch_width);
//
// /**
//  * Create feature mask
//  *
//  * @param[in] image_width Image width
//  * @param[in] image_height Image height
//  * @param[in] keypoints Keypoints
//  * @param[in] patch_width Patch width
//  *
//  * @returns Feature mask
//  */
// matx_t feature_mask(const int image_width,
//                     const int image_height,
//                     const std::vector<cv::KeyPoint> keypoints,
//                     const int patch_width);
//
// /**
//  * Create feature mask
//  *
//  * @param[in] image_width Image width
//  * @param[in] image_height Image height
//  * @param[in] points Points
//  * @param[in] patch_width Patch width
//  *
//  * @returns Feature mask
//  */
// cv::Mat feature_mask_opencv(const int image_width,
//                             const int image_height,
//                             const std::vector<cv::Point2f> points,
//                             const int patch_width);
//
// /**
//  * Create feature mask
//  *
//  * @param[in] image_width Image width
//  * @param[in] image_height Image height
//  * @param[in] keypoints Keypoints
//  * @param[in] patch_width Patch width
//  *
//  * @returns Feature mask
//  */
// cv::Mat feature_mask_opencv(const int image_width,
//                             const int image_height,
//                             const std::vector<cv::KeyPoint> keypoints,
//                             const int patch_width);

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

/********************************* RADTAN4 ************************************/

vec2_t radtan4_distort(const vec4_t &dist_params, const vec2_t &p);
vec2_t radtan4_undistort(const vec4_t &dist_params, const vec2_t &p0);
mat2_t radtan4_point_jacobian(const vec4_t &dist_params, const vec2_t &p);
matx_t radtan4_params_jacobian(const vec4_t &dist_params, const vec2_t &p);

/********************************** EQUI4 *************************************/

vec2_t equi4_distort(const vec4_t &dist_params, const vec2_t &p);
vec2_t equi4_undistort(const vec4_t &dist_params, const vec2_t &p0);
mat2_t equi4_point_jacobian(const vec4_t &dist_params, const vec2_t &p);
matx_t equi4_params_jacobian(const vec4_t &dist_params, const vec2_t &p);

/********************************* PROJECT ************************************/

vec2_t project_point(const vec3_t &p_C);
mat_t<2, 3> project_jacobian(const vec3_t &p_C);

/********************************* PINHOLE ************************************/

real_t pinhole_focal(const int image_size, const real_t fov);
mat3_t
pinhole_K(const real_t fx, const real_t fy, const real_t cx, const real_t cy);
int pinhole_project(const int res[2],
                    const vec4_t &proj_params,
                    const vec3_t &p,
                    vec2_t &z_hat);
mat2_t pinhole_point_jacobian(const vec4_t &proj_params);
mat_t<2, 4> pinhole_params_jacobian(const vec4_t &proj_params, const vec2_t &p);

/****************************** PINHOLE-RADTAN4 *******************************/

int pinhole_radtan4_project(const int res[2],
                            const vecx_t &params,
                            const vec3_t &p_C,
                            vec2_t &z_hat);
matx_t pinhole_radtan4_project_jacobian(const vecx_t &params,
                                        const vec3_t &p_C);
matx_t pinhole_radtan4_params_jacobian(const vecx_t &params, const vec3_t &p_C);
int pinhole_radtan4_back_project(const vecx_t &params,
                                 const vec2_t &x,
                                 vec3_t &ray);
vec2_t pinhole_radtan4_undistort(const vecx_t &params, const vec2_t &z);

/******************************* PINHOLE-EQUI4 ********************************/

int pinhole_equi4_project(const int res[2],
                          const vecx_t &params,
                          const vec3_t &p_C,
                          vec2_t &z_hat);
matx_t pinhole_equi4_project_jacobian(const vecx_t &params, const vec3_t &p_C);
matx_t pinhole_equi4_params_jacobian(const vecx_t &params, const vec3_t &p_C);
int pinhole_equi4_back_project(const vecx_t &params,
                               const vec2_t &x,
                               vec3_t &ray);
vec2_t pinhole_equi4_undistort(const vecx_t &params, const vec2_t &z);

/****************************** CAMERA GEOMETRY *******************************/

struct camera_geometry_t {
  std::string type;

  camera_geometry_t(const std::string &type_) : type{type_} {}
  virtual ~camera_geometry_t() = default;

  virtual int project(const int res[2],
                      const vecx_t &params,
                      const vec3_t &p_C,
                      vec2_t &z_hat) const = 0;

  virtual matx_t project_jacobian(const vecx_t &params,
                                  const vec3_t &p_C) const = 0;

  virtual matx_t params_jacobian(const vecx_t &params,
                                 const vec3_t &p_C) const = 0;

  virtual int back_project(const vecx_t &params,
                           const vec2_t &x,
                           vec3_t &ray) const = 0;

  virtual vec2_t undistort(const vecx_t &params, const vec2_t &z) const = 0;
};

struct pinhole_radtan4_t : camera_geometry_t {
  pinhole_radtan4_t() : camera_geometry_t{"PINHOLE-RADTAN4"} {}

  int project(const int res[2],
              const vecx_t &params,
              const vec3_t &p_C,
              vec2_t &z_hat) const override {
    return pinhole_radtan4_project(res, params, p_C, z_hat);
  }

  matx_t project_jacobian(const vecx_t &params,
                          const vec3_t &p_C) const override {
    return pinhole_radtan4_project_jacobian(params, p_C);
  }

  matx_t params_jacobian(const vecx_t &params,
                         const vec3_t &p_C) const override {
    return pinhole_radtan4_params_jacobian(params, p_C);
  }

  int back_project(const vecx_t &params,
                   const vec2_t &x,
                   vec3_t &ray) const override {
    return pinhole_radtan4_back_project(params, x, ray);
  }

  vec2_t undistort(const vecx_t &params, const vec2_t &z) const override {
    return pinhole_radtan4_undistort(params, z);
  }
};

struct pinhole_equi4_t : camera_geometry_t {
  pinhole_equi4_t() : camera_geometry_t{"PINHOLE-EQUI4"} {}

  int project(const int res[2],
              const vecx_t &params,
              const vec3_t &p_C,
              vec2_t &z_hat) const override {
    return pinhole_equi4_project(res, params, p_C, z_hat);
  }

  matx_t project_jacobian(const vecx_t &params,
                          const vec3_t &p_C) const override {
    return pinhole_equi4_project_jacobian(params, p_C);
  }

  matx_t params_jacobian(const vecx_t &params,
                         const vec3_t &p_C) const override {
    return pinhole_equi4_params_jacobian(params, p_C);
  }

  int back_project(const vecx_t &params,
                   const vec2_t &x,
                   vec3_t &ray) const override {
    return pinhole_equi4_back_project(params, x, ray);
  }

  vec2_t undistort(const vecx_t &params, const vec2_t &z) const override {
    return pinhole_equi4_undistort(params, z);
  }
};

/*********************************** MISC *************************************/

int solvepnp(const camera_geometry_t *cam,
             const int cam_res[2],
             const vecx_t &cam_params,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_CF);

} // namespace yac
#endif // YAC_CV_HPP
