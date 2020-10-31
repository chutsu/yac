#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include <string>

#include <opencv2/calib3d/calib3d.hpp>

#include "core.hpp"
#include "aprilgrid.hpp"

namespace yac {

template <typename T>
int pinhole_radtan4_project(const Eigen::Matrix<T, 8, 1> &params,
                            const Eigen::Matrix<T, 3, 1> &point,
                            Eigen::Matrix<T, 2, 1> &image_point) {
  // Check for singularity
  const T z_norm = sqrt(point(2) * point(2)); // std::abs doesn't work for all T
  if ((T) z_norm < (T) 1.0e-12) {
    return -1;
  }

  // Extract intrinsics params
  const T fx = params(0);
  const T fy = params(1);
  const T cx = params(2);
  const T cy = params(3);
  const T k1 = params(4);
  const T k2 = params(5);
  const T p1 = params(6);
  const T p2 = params(7);

  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

  // Apply Radial distortion factor
  const T x2 = x * x;
  const T y2 = y * y;
  const T r2 = x2 + y2;
  const T r4 = r2 * r2;
  const T radial_factor = T(1) + (k1 * r2) + (k2 * r4);
  const T x_dash = x * radial_factor;
  const T y_dash = y * radial_factor;

  // Apply Tangential distortion factor
  const T xy = x * y;
  const T x_ddash = x_dash + (T(2) * p1 * xy + p2 * (r2 + T(2) * x2));
  const T y_ddash = y_dash + (p1 * (r2 + T(2) * y2) + T(2) * p2 * xy);

  // Scale and center
  image_point(0) = fx * x_ddash + cx;
  image_point(1) = fy * y_ddash + cy;

  if (point(2) > T(0.0)) {
    return 0; // Point is infront of camera
  } else {
    return 1; // Point is behind camera
  }
}

template <typename T>
int pinhole_equi4_project(const Eigen::Matrix<T, 8, 1> &params,
                          const Eigen::Matrix<T, 3, 1> &point,
                          Eigen::Matrix<T, 2, 1> &image_point) {
  // Check for singularity
  const T z_norm = sqrt(point(2) * point(2)); // std::abs doesn't work for all T
  if ((T) z_norm < (T) 1.0e-12) {
    return -1;
  }

  // Extract intrinsics params
  const T fx = params(0);
  const T fy = params(1);
  const T cx = params(2);
  const T cy = params(3);
  const T k1 = params(4);
  const T k2 = params(5);
  const T k3 = params(6);
  const T k4 = params(7);

  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

	// Apply equi distortion
	const T r = sqrt(x * x + y * y);
	if ((T) r < (T) 1e-8) {
		return -1;
	}
	const T th = atan(r);
	const T th2 = th * th;
	const T th4 = th2 * th2;
	const T th6 = th4 * th2;
	const T th8 = th4 * th4;
	const T thd = th * (T(1.0) + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
	const T x_dash = (thd / r) * x;
	const T y_dash = (thd / r) * y;

  // Scale and center
  image_point(0) = fx * x_dash + cx;
  image_point(1) = fy * y_dash + cy;

  if (point(2) > T(0.0)) {
    return 0; // Point is infront of camera
  } else {
    return 1; // Point is behind camera
  }
}

/**
 * Pose parameter block
 */
struct calib_pose_t {
  double q[4] = {0.0, 0.0, 0.0, 1.0}; // x, y, z, w
  double r[3] = {0.0, 0.0, 0.0};      // x, y, z

  calib_pose_t(const mat4_t &T) {
    quat_t q_{tf_rot(T)};
    q[0] = q_.x();
    q[1] = q_.y();
    q[2] = q_.z();
    q[3] = q_.w();

    r[0] = T(0, 3);
    r[1] = T(1, 3);
    r[2] = T(2, 3);
  }

  calib_pose_t(const mat3_t &rot, const vec3_t &trans) {
    quat_t q_{rot};
    q[0] = q_.x();
    q[1] = q_.y();
    q[2] = q_.z();
    q[3] = q_.w();

    r[0] = trans(0);
    r[1] = trans(1);
    r[2] = trans(2);
  }

  calib_pose_t(const quat_t &rot, const vec3_t &trans) {
    q[0] = rot.x();
    q[1] = rot.y();
    q[2] = rot.z();
    q[3] = rot.w();

    r[0] = trans(0);
    r[1] = trans(1);
    r[2] = trans(2);
  }

  ~calib_pose_t() {}

  mat4_t T() {
    quat_t rot(q[3], q[0], q[1], q[2]);
    vec3_t trans(r[0], r[1], r[2]);
    return tf(rot, trans);
  }
};

/**
 * Calibration parameters
 */
struct calib_params_t {
  int img_w;
  int img_h;
  std::string proj_model;
  std::string dist_model;
  vecx_t proj_params;
  vecx_t dist_params;

  calib_params_t() {}

  calib_params_t(const std::string &proj_model_,
                 const std::string &dist_model_,
                 const int img_w_,
                 const int img_h_,
                 const vecx_t &proj_params_,
                 const vecx_t &dist_params_)
    : img_w{img_w_}, img_h{img_h_},
      proj_model{proj_model_}, dist_model{dist_model_},
      proj_params{proj_params_}, dist_params{dist_params_} {}

  calib_params_t(const std::string &proj_model_,
                 const std::string &dist_model_,
                 const int img_w_, const int img_h_,
                 const int lens_hfov_, const int lens_vfov_) {
    img_w = img_w_;
    img_h = img_h_;
    proj_model = proj_model_;
    dist_model = dist_model_;

    const double fx = pinhole_focal(img_w_, lens_hfov_);
    const double fy = pinhole_focal(img_h_, lens_vfov_);
    const double cx = img_w_ / 2.0;
    const double cy = img_h_ / 2.0;
    proj_params.resize(4);
    dist_params.resize(4);
    proj_params << fx, fy, cx, cy;
    dist_params << 0.01, 0.0001, 0.0001, 0.0001;
  }

	std::vector<int> resolution() {
		std::vector<int> res{img_w, img_h};
		return res;
	}

	std::vector<int> resolution() const {
		std::vector<int> res{img_w, img_h};
		return res;
	}

  std::string toString(const int cam_index=-1) const {
    const std::string indent = (cam_index != -1) ? "  " : "";
    const std::string str_img_w = std::to_string(img_w);
    const std::string str_img_h = std::to_string(img_h);

    std::string s;
    s += (cam_index != -1) ? "cam" + std::to_string(cam_index) + ":\n" : "";
    s += indent + "resolution: [" + str_img_w + ", " + str_img_h + "]\n";
    s += indent + "proj_model: " + proj_model + "\n";
    s += indent + "dist_model: " + dist_model + "\n";
    s += indent + "proj_params: " + vec2str(proj_params) + "\n";
    s += indent + "dist_params: " + vec2str(dist_params) + "\n";
    return s;
  }

  std::string toString(const int cam_index=-1) {
    return static_cast<const calib_params_t &>(*this).toString(cam_index);
  }
};

/**
 * Calibration target.
 */
struct calib_target_t {
  std::string target_type;
  int tag_rows = 0;
  int tag_cols = 0;
  real_t tag_size = 0.0;
  real_t tag_spacing = 0.0;

  calib_target_t() {}
  ~calib_target_t() {}
};

/**
 * Load calibration target.
 * @returns 0 or -1 for success or failure
 */
int calib_target_load(calib_target_t &ct,
                      const std::string &target_file,
                      const std::string &prefix = "");

/**
 * Preprocess camera image data and output AprilGrid detection data as
 * csv. The AprilGrid tag corners are estimated using the camera intrinsics
 * matrix `cam_K` and distortion vector `cam_D`. Once the data is preprocessed
 * the data is saved to `output_dir`.
 *
 * @returns 0 for Success, -1 for failure, and 1 where the output directory
 * contains data.
 */
int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const mat3_t &cam_K,
                           const vec4_t &cam_D,
                           const std::string &output_dir,
                           const bool imshow = false,
                           const bool show_progress = true);

/**
 * Preprocess camera image data and output AprilGrid detection data as
 * csv. The data is initialized with `image_size` in pixels, the horizontal
 * lens fov `lens_hfov` and vertical lens fov `lens_vfov` in degrees. Once the
 * data is preprocessed the data is saved to `output_dir`.
 *
 * @returns 0 for Success, -1 for failure, and 1 where the output directory
 * contains data.
 */
int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const vec2_t &image_size,
                           const real_t lens_hfov,
                           const real_t lens_vfov,
                           const std::string &output_dir,
                           const bool imshow = false,
                           const bool show_progress = true);

/**
 * Load preprocess-ed camera calibration data located in `data_dir` where the
 * data will be loaded in `aprilgrids`. By default, this function will only
 * return aprilgrids that are detected. To return all calibration data
 * including camera frames where aprilgrids were not detected, change
 * `detected_only` to false.
 *
 * @returns 0 or -1 for success or failure
 */
int load_camera_calib_data(const std::string &data_dir,
                           aprilgrids_t &aprilgrids,
                           timestamps_t &timestamps,
                           bool detected_only = true);

/**
 * Preprocess stereo image data and output AprilGrid detection data as
 * csv. The data is initialized with `image_size` in pixels, the horizontal
 * lens fov `lens_hfov` and vertical lens fov `lens_vfov` in degrees.
 *
 * This function assumes:
 *
 * - Stereo camera images are synchronized
 * - Number of images observed by both cameras are the same
 *
 * @returns 0 for Success, -1 for failure, and 1 where the output directory
 * contains data.
 */
int preprocess_stereo_data(const calib_target_t &target,
                           const std::string &cam0_image_dir,
                           const std::string &cam1_image_dir,
                           const vec2_t &cam0_image_size,
                           const vec2_t &cam1_image_size,
                           const real_t cam0_lens_hfov,
                           const real_t cam0_lens_vfov,
                           const real_t cam1_lens_hfov,
                           const real_t cam1_lens_vfov,
                           const std::string &cam0_output_dir,
                           const std::string &cam1_output_dir,
                           const bool imshow = false);

/**
 * Extract and only keep common aprilgrid corners between `grids0` and `grids1`.
 */
void extract_common_calib_data(aprilgrids_t &grids0, aprilgrids_t &grids1);

/**
 * Load preprocessed stereo calibration data, where `cam0_data_dir` and
 * `cam1_data_dir` are preprocessed calibration data observed from cam0 and
 * cam1. The preprocessed calibration data will be loaded into
 * `cam0_aprilgrids` and `cam1_aprilgrids` respectively, where the data
 * contains AprilGrids observed by both cameras at the same timestamp.**
 *
 * This function assumes:
 *
 * - Stereo camera images are synchronized
 * - Images that are synchronized are expected to have the **same exact
 *   timestamp**
 *
 * @returns 0 or -1 for success or failure
 */
int load_stereo_calib_data(const std::string &cam0_data_dir,
                           const std::string &cam1_data_dir,
                           aprilgrids_t &cam0_aprilgrids,
                           aprilgrids_t &cam1_aprilgrids);

/**
 * Load preprocessed multi-camera calibration data, where each data path in
 * `data_dirs` are the preprocessed calibration data observed by each camera,
 * and the preprocessed calibration data will be loaded into `calib_data` where
 * the key is the camera index and the value is the detected aprilgrids. The
 * data in `calib_data` contains AprilGrids observed by all cameras at the same
 * timestamp.
 *
 * This function assumes:
 *
 * - Camera images are synchronized
 * - Images that are synchronized are expected to have the **same exact
 *   timestamp**
 *
 * @returns 0 or -1 for success or failure
 */
int load_multicam_calib_data(const int nb_cams,
                             const std::vector<std::string> &data_dirs,
                             std::map<int, aprilgrids_t> &calib_data);

/**
 * Draw measured and projected pixel points.
 * @returns Image
 */
cv::Mat draw_calib_validation(const cv::Mat &image,
                              const vec2s_t &measured,
                              const vec2s_t &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color);

// /**
//  * Validate calibration.
//  * @returns Validation image for visual inspection
//  */
// template <typename CM, typename DM>
// cv::Mat validate_intrinsics(const cv::Mat &image,
//                             const vec2s_t &keypoints,
//                             const vec3s_t &points,
//                             const camera_geometry_t<CM, DM> &camera_geometry) {
//   assert(image.empty() == false);
//   assert(keypoints.size() != 0);
//   assert(points.size() != 0);
//   assert(keypoints.size() == points.size());
//
//   // Project points to image plane
//   vec2s_t projected;
//   for (const auto &point : points) {
//     const auto p = camera_geometry_project(camera_geometry, point);
//     projected.emplace_back(p);
//   }
//
//   // Colors
//   const cv::Scalar red{0, 0, 255};
//   const cv::Scalar green{0, 255, 0};
//
//   // Draw detected chessboard corners
//   const auto keypoints_color = red;
//   const auto projected_color = green;
//   cv::Mat result = draw_calib_validation(image,
//                                          keypoints,
//                                          projected,
//                                          keypoints_color,
//                                          projected_color);
//
//   return result;
// }

// /**
//  * Validate stereo extrinsics.
//  * @returns Validation image for visual inspection
//  */
// template <typename CM, typename DM>
// cv::Mat validate_stereo(const cv::Mat &image0,
//                         const cv::Mat &image1,
//                         const vec2s_t &kps0,
//                         const vec3s_t &points0,
//                         const vec2s_t &kps1,
//                         const vec3s_t &points1,
//                         const camera_geometry_t<CM, DM> &cam0,
//                         const camera_geometry_t<CM, DM> &cam1,
//                         const mat4_t &T_C0C1) {
//   assert(image0.empty() == false);
//   assert(image1.empty() == false);
//   assert(kps0.size() == kps1.size());
//   assert(points0.size() == points1.size());
//
//   if (kps0.size() == 0 || kps1.size() == 0) {
//     cv::Mat result;
//     cv::vconcat(image0, image1, result);
//     return result;
//   }
//
//   // Project points observed in cam1 to cam0 image plane
//   vec2s_t projected0;
//   for (const auto &point_C1F : points1) {
//     const auto point_C0F = (T_C0C1 * point_C1F.homogeneous()).head(3);
//     const auto p = camera_geometry_project(cam0, point_C0F);
//     projected0.emplace_back(p);
//   }
//
//   // Project points observed in cam0 to cam1 image plane
//   vec2s_t projected1;
//   const mat4_t T_C1C0 = T_C0C1.inverse();
//   for (const auto &point_C0F : points0) {
//     const auto point_C1F = (T_C1C0 * point_C0F.homogeneous()).head(3);
//     const auto p = camera_geometry_project(cam1, point_C1F);
//     projected1.emplace_back(p);
//   }
//
//   // Draw
//   const cv::Scalar red{0, 0, 255};
//   const cv::Scalar green{0, 255, 0};
//   auto result0 = draw_calib_validation(image0, kps0, projected0, red, green);
//   auto result1 = draw_calib_validation(image1, kps1, projected1, red, green);
//
//   // Combine cam0 and cam1 images
//   cv::Mat result;
//   cv::vconcat(result0, result1, result);
//   return result;
// }

/**
 * `calib_target_t` to output stream.
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
