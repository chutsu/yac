#pragma once

#include <string>
#include <algorithm>

#include "Core.hpp"
#include "CameraModel.hpp"

// /// AprilTags3 by Ed Olsen
// extern "C" {
// #include "apriltag/apriltag.h"
// #include "apriltag/tag36h11.h"
// #include "apriltag/tag25h9.h"
// #include "apriltag/tag16h5.h"
// #include "apriltag/tagCircle21h7.h"
// #include "apriltag/tagCircle49h12.h"
// #include "apriltag/tagCustom48h12.h"
// #include "apriltag/tagStandard41h12.h"
// #include "apriltag/tagStandard52h13.h"
// #include "apriltag/common/getopt.h"
// }

namespace yac {

/* AprilGrid */
struct AprilGrid {
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

  AprilGrid() = default;
  AprilGrid(const std::string &csv_path);
  AprilGrid(const timestamp_t &timestamp,
            const int tag_rows,
            const int tag_cols,
            const double tag_size,
            const double tag_spacing);
  ~AprilGrid() = default;

  void clear();

  bool fully_observable() const;
  bool has(const int tag_id, const int corner_idx) const;
  vec2_t center() const;
  void gridIndex(const int tag_id, int &i, int &j) const;
  vec3_t objectPoint(const int tag_id, const int corner_idx) const;
  vec3s_t objectPoints() const;
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

  int estimate(const CameraModel *cam,
               const int cam_res[2],
               const vecx_t &cam_params,
               mat4_t &T_CF) const;

  int save(const std::string &save_path) const;
  int load(const std::string &data_path);
};

} // namespace yac
