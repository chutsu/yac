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
#include "cv.hpp"
#include "data.hpp"

namespace yac {

/* AprilGrid */
struct aprilgrid_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Grid properties
  bool init = false;
  timestamp_t timestamp = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  // Grid data
  bool detected = false;
  int nb_detections = 0; // Number of corners detected
  matx_t data;

  aprilgrid_t() = default;
  aprilgrid_t(const std::string &csv_path);
  aprilgrid_t(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing);
  ~aprilgrid_t() = default;

  void clear();

  bool fully_observable() const;
  bool has(const int tag_id, const int corner_idx) const;
  vec2_t center() const;
  void grid_index(const int tag_id, int &i, int &j) const;
  vec3_t object_point(const int tag_id, const int corner_idx) const;
  vec3s_t object_points() const;
  vec2_t keypoint(const int tag_id, const int corner_idx) const;
  vec2s_t keypoints() const;

  void get_measurements(std::vector<int> &tag_ids,
                        std::vector<int> &corner_indicies,
                        vec2s_t &keypoints,
                        vec3s_t &object_points) const;

  std::vector<int> tag_ids() const;

  void add(const int tag_id, const int corner_idx, const vec2_t &kp);
  void remove(const int tag_id, const int corner_idx);
  void remove(const int tag_id);

  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  void imshow(const std::string &title, const cv::Mat &image) const;

  int estimate(const camera_geometry_t *cam,
               const int cam_res[2],
               const vecx_t &cam_params,
               mat4_t &T_CF) const;

  int save(const std::string &save_path) const;
  int load(const std::string &data_path);
  int equal(const aprilgrid_t &grid1) const;

  void intersect(aprilgrid_t &grid1);
  static void intersect(std::vector<aprilgrid_t *> &grids);

  void sample(const size_t n,
              std::vector<int> &sample_tag_ids,
              std::vector<int> &sample_corner_indicies,
              vec2s_t &sample_keypoints,
              vec3s_t &sample_object_points);

  static void common_measurements(const aprilgrid_t &grid_i,
                                  const aprilgrid_t &grid_j,
                                  std::vector<int> &tag_ids,
                                  std::vector<int> &corner_indicies,
                                  vec2s_t &grid_i_keypoints,
                                  vec2s_t &grid_j_keypoints,
                                  vec3s_t &object_points);

  friend std::ostream &operator<<(std::ostream &os, const aprilgrid_t &grid);
};

/* AprilGrids */
// clang-format off
using aprilgrids_t = std::vector<aprilgrid_t, Eigen::aligned_allocator<aprilgrid_t>>;
// clang-format on

/* Load AprilGrids */
aprilgrids_t load_aprilgrids(const std::string &dir_path);

/* AprilGrid Detector */
struct aprilgrid_detector_t {
  // Grid properties
  bool configured = true;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  // AprilTags by Michael Kaess
  AprilTags::AprilGridDetector det;

  // AprilTag3 by Ed Olsen
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *det_v3 = apriltag_detector_create();

  aprilgrid_detector_t(const int tag_rows_,
                       const int tag_cols_,
                       const double tag_size_,
                       const double tag_spacing_);
  ~aprilgrid_detector_t();

  void filter_tags(const cv::Mat &image,
                   std::vector<AprilTags::TagDetection> &tags,
                   const bool verbose = false) const;
  void filter_measurements(const cv::Mat &image,
                           std::vector<int> &tag_ids,
                           std::vector<int> &corner_indicies,
                           vec2s_t &keypoints) const;

  aprilgrid_t detect(const timestamp_t ts,
                     const cv::Mat &image,
                     const bool use_v3 = false) const;
};

} // namespace yac
#endif // YAC_APRILGRID_HPP
