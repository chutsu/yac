#ifndef YAC_APRILGRID_HPP
#define YAC_APRILGRID_HPP

#include <algorithm>

/// Order matters with the AprilTags lib by Michael Kaess. The detector first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

/// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
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
  aprilgrid_t(const std::string &csv_path, const bool format_v2 = false);
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
  int loadv2(const std::string &data_path);
  int equal(const aprilgrid_t &grid1) const;

  void intersect(aprilgrid_t &grid1);
  static void intersect(std::vector<aprilgrid_t *> &grids);

  void sample(const size_t n,
              std::vector<int> &sample_tag_ids,
              std::vector<int> &sample_corner_indicies,
              vec2s_t &sample_keypoints,
              vec3s_t &sample_object_points) const;

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
using aprilgrids_t = std::vector<aprilgrid_t>;

/* Load AprilGrids */
aprilgrids_t load_aprilgrids(const std::string &dir_path,
                             const bool format_v2 = false);

struct aprilgrid_detect_data_t {
  const AprilTags::AprilGridDetector &det;
  const int tag_rows;
  const int tag_cols;
  const real_t tag_size;
  const real_t tag_spacing;

  const int cam_idx;
  const timestamp_t ts;
  const cv::Mat image;

  aprilgrid_t grid;

  aprilgrid_detect_data_t() = delete;
  aprilgrid_detect_data_t(const AprilTags::AprilGridDetector &det_,
                          const int tag_rows_,
                          const int tag_cols_,
                          const real_t tag_size_,
                          const real_t tag_spacing_,
                          const int cam_idx_,
                          const timestamp_t ts_,
                          const cv::Mat image_)
      : det{det_}, tag_rows{tag_rows_}, tag_cols{tag_cols_},
        tag_size{tag_size_},
        tag_spacing{tag_spacing_}, cam_idx{cam_idx_}, ts{ts_}, image{image_} {}
};
void *aprilgrid_detect_thread(void *data);

/* AprilGrid Detector */
struct aprilgrid_detector_t {
  // Grid properties
  bool configured = true;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  int min_tags_threshold = 0;

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

  aprilgrid_t detect(const timestamp_t ts,
                     const cv::Mat &image,
                     const bool use_v3 = false) const;

  std::map<int, aprilgrid_t>
  detect(const std::map<int, std::pair<timestamp_t, cv::Mat>> &img_buffer,
         const bool use_v3 = false) const;
};

} // namespace yac
#endif // YAC_APRILGRID_HPP
