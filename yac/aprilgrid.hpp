#ifndef YAC_APRILGRID_HPP
#define YAC_APRILGRID_HPP

#include <algorithm>

/// Order matters with the AprilTags lib. The detector has to be first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

#include "yac/core.hpp"

namespace yac {

/**
 * AprilGrid Detector
 */
struct aprilgrid_detector_t {
  AprilTags::AprilGridDetector det;

  aprilgrid_detector_t();
  ~aprilgrid_detector_t();
};

/**
 * AprilGrid detection
 */
struct aprilgrid_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /// Grid properties
  bool configured = false;
  int tag_rows = 0;
  int tag_cols = 0;
  real_t tag_size = 0.0;
  real_t tag_spacing = 0.0;

  /// Detections
  bool detected = false;
  int nb_detections = 0;
  timestamp_t timestamp = 0;
  std::vector<int> ids;
  vec2s_t keypoints;

  /// Estimation
  bool estimated = false;
  vec3s_t points_CF;
  mat4_t T_CF = I(4);

  aprilgrid_t();
  aprilgrid_t(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const real_t tag_size,
              const real_t tag_spacing);
  ~aprilgrid_t();
};
typedef AprilTags::TagDetection apriltag_t;
typedef std::vector<aprilgrid_t> aprilgrids_t;

/** Add AprilTag measurement to AprilGrid. */
void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints);

/** Remove AprilTag measurement from AprilGrid based on id. */
void aprilgrid_remove(aprilgrid_t &grid, const int id);

/** Clear AprilGrid **/
void aprilgrid_clear(aprilgrid_t &grid);

/**
 * Get AprilTag measurements based on tag id.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_get(const aprilgrid_t &grid, const int id, vec2s_t &keypoints);

/**
 * Get AprilTag measurements based on tag id.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  vec2s_t &keypoints,
                  vec3s_t &points_CF);

/** Set AprilGrid properties */
void aprilgrid_set_properties(aprilgrid_t &grid,
                              const int tag_rows,
                              const int tag_cols,
                              const real_t tag_size,
                              const real_t tag_spacing);

/**
 * Get the tag's grid index using the tag id
 * Note: The origin of the grid is bottom left corner rather than top left.
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_grid_index(const aprilgrid_t &grid, const int id, int &i, int &j);

/**
 * Get the object point for a specific `tag_id` and `corner_id` in the
 * AprilGrid `grid`.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_object_point(const aprilgrid_t &grid,
                           const int tag_id,
                           const int corner_id,
                           vec3_t &object_point);

/**
 * Get the object point for a specific `tag_id` in the AprilGrid `grid`.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_object_points(const aprilgrid_t &grid,
                            const int tag_id,
                            vec3s_t &object_points);

/**
 * Get all object points in the AprilGrid `grid`.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_object_points(const aprilgrid_t &grid, vec3s_t &object_points);

/**
 * Calculate relative position between AprilGrid and camera using solvepnp.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_calc_relative_pose(aprilgrid_t &grid,
                                 const mat3_t &cam_K,
                                 const vec4_t &cam_D);

/** Show detection. */
void aprilgrid_imshow(const aprilgrid_t &grid,
                      const std::string &title,
                      const cv::Mat &image);

/**
 * Save AprilGrid detection.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_save(const aprilgrid_t &grid, const std::string &save_path);

/**
 * Load AprilGrid detection.
 * @returns 0 or -1 for success or failure.
 */
int aprilgrid_load(aprilgrid_t &grid, const std::string &data_path);

/**
 * Configure AprilGrid detector.
 * @returns 0 or 1 for success or failure.
 */
int aprilgrid_configure(aprilgrid_t &grid, const std::string &config_file);

/** Filter tags detected */
void aprilgrid_filter_tags(const cv::Mat &image,
                           std::vector<AprilTags::TagDetection> &tags);

/**
 * Detect AprilGrid.
 * @returns number of AprilTags detected
 */
int aprilgrid_detect(aprilgrid_t &grid,
                     const aprilgrid_detector_t &detector,
                     const cv::Mat &image);

/**
 * Detect AprilGrid.
 * @returns number of AprilTags detected.
 */
int aprilgrid_detect(aprilgrid_t &grid,
                     const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     const mat3_t &cam_K,
                     const vec4_t &cam_D);

/**
 * Find the intersection of two aprilgrids
 */
void aprilgrid_intersection(aprilgrid_t &grid0, aprilgrid_t &grid1);

/**
 * Find the intersection of aprilgrids
 */
void aprilgrid_intersection(std::vector<aprilgrid_t *> &grids);

/**
 * Random sample `n` aprilgrid measurements.
 */
void aprilgrid_random_sample(const aprilgrid_t &grid, const size_t n,
                             std::vector<int> &sample_tag_ids,
                             std::vector<vec2s_t> &sample_keypoints,
                             std::vector<vec3s_t> &sample_object_points);

/** Comparator to sort detected AprilTags by id */
bool sort_apriltag_by_id(const AprilTags::TagDetection &a,
                         const AprilTags::TagDetection &b);

/** aprilgrid_t to output stream */
std::ostream &operator<<(std::ostream &os, const aprilgrid_t &grid);

} // namespace yac
#endif // YAC_APRILGRID_HPP
