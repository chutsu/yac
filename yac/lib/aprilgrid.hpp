#ifndef YAC_APRILGRID_HPP
#define YAC_APRILGRID_HPP

#include <algorithm>

/// Order matters with the AprilTags lib by Michael Kaess. The detector first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

/// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag3/apriltag.h"
#include "apriltag3/tag36h11.h"
#include "apriltag3/tag25h9.h"
#include "apriltag3/tag16h5.h"
#include "apriltag3/tagCircle21h7.h"
#include "apriltag3/tagCircle49h12.h"
#include "apriltag3/tagCustom48h12.h"
#include "apriltag3/tagStandard41h12.h"
#include "apriltag3/tagStandard52h13.h"
#include "apriltag3/common/getopt.h"
}

#include "core.hpp"

namespace yac {

/**
 * AprilGrid Detector
 */
struct aprilgrid_detector_t {
  /// Grid properties
  bool configured = true;
  int tag_rows = 0;
  int tag_cols = 0;
  real_t tag_size = 0.0;
  real_t tag_spacing = 0.0;

  // AprilTags by Michael Kaess
  AprilTags::AprilGridDetector det;

  // AprilTag3 by Ed Olsen
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *det_v3 = apriltag_detector_create();

  aprilgrid_detector_t(const int tag_rows_,
                       const int tag_cols_,
                       const real_t tag_size_,
                       const real_t tag_spacing_)
      : tag_rows{tag_rows_},
        tag_cols{tag_cols_},
        tag_size{tag_size_},
        tag_spacing{tag_spacing_} {
    apriltag_detector_add_family(det_v3, tf);
    det_v3->quad_decimate = 1.0;
    det_v3->quad_sigma = 0.0;  // Blur
    det_v3->nthreads = 2;
    det_v3->debug = 0;
    det_v3->refine_edges = 1;
  }

  ~aprilgrid_detector_t() {
    apriltag_detector_destroy(det_v3);
  }
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

  aprilgrid_t() {}
  aprilgrid_t(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const real_t tag_size,
              const real_t tag_spacing)
      : configured{true},
        tag_rows{tag_rows}, tag_cols{tag_cols},
        tag_size{tag_size}, tag_spacing{tag_spacing},
        timestamp{timestamp} {}
  ~aprilgrid_t() {}
};
typedef AprilTags::TagDetection apriltag_t;
typedef std::vector<aprilgrid_t> aprilgrids_t;

/** Add AprilTag measurement to AprilGrid. */
void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints);

/** Add AprilTag measurement to AprilGrid. */
void aprilgrid_add(aprilgrid_t &grid, const int id, const vec2s_t &keypoints);

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
 * Return AprilGrid center based on number of rows and cols, as well as the tag
 * size and tag spacing.
 */
vec2_t aprilgrid_center(const int rows, const int cols,
                        const double tag_size, const double tag_spacing);

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
 * Load AprilGrids.
 */
aprilgrids_t load_aprilgrids(const std::string &dir_path);

/**
 * Configure AprilGrid detector.
 * @returns 0 or 1 for success or failure.
 */
int aprilgrid_configure(aprilgrid_t &grid, const std::string &config_file);

/** Filter tags detected */
void aprilgrid_filter_tags(const cv::Mat &image,
                           std::vector<AprilTags::TagDetection> &tags);

/**
 * Compare two aprilgrids to see if they're equal.
 * @returns true or false.
 */
int aprilgrid_equal(const aprilgrid_t &grid0, const aprilgrid_t &grid1);

/**
 * Detect AprilGrid.
 * @returns number of AprilTags detected
 */
int aprilgrid_detect(const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     aprilgrid_t &grid,
                     const bool use_v3=false);

/**
 * Detect AprilGrid.
 * @returns number of AprilTags detected.
 */
int aprilgrid_detect(const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     const mat3_t &cam_K,
                     const vec4_t &cam_D,
                     aprilgrid_t &grid,
                     const bool use_v3=false);

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
