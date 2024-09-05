#pragma once

#include <string>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "Core.hpp"
#include "CameraModel.hpp"
#include "CalibTarget.hpp"

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
class AprilGrid : public CalibTarget {
private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Grid properties
  int tag_rows_ = 0;
  int tag_cols_ = 0;
  double tag_size_ = 0.0;
  double tag_spacing_ = 0.0;

  // Data
  struct TagDetection {
    std::unordered_set<int> corner_indicies;
    std::unordered_map<int, vec2_t> keypoints;
    std::unordered_map<int, vec3_t> object_points;
  };
  std::unordered_map<int, TagDetection> data_;

  /** Calculate AprilGrid tag index based on tag ID */
  void gridIndex(const int tag_id, int &i, int &j) const;

  /** Calculate object point based on tag ID and corner index */
  vec3_t objectPoint(const int tag_id, const int corner_index) const;

  /** Calculate all object points */
  vec3s_t objectPoints() const;

public:
  AprilGrid() = delete;
  AprilGrid(const timestamp_t &timestamp,
            const int tag_rows,
            const int tag_cols,
            const double tag_size,
            const double tag_spacing);
  ~AprilGrid() = default;

  /** Return number of tag rows */
  int getTagRows() const;

  /** Return number of tag cols */
  int getTagCols() const;

  /** Return number of tag spacing */
  double getTagSpacing() const;

  /** Return number of tag size */
  double getTagSize() const;

  /** Get the center of the AprilGrid */
  vec2_t center() const;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Get all measurements */
  void getMeasurements(std::vector<int> &tag_ids,
                       std::vector<int> &corner_indicies,
                       vec2s_t &keypoints,
                       vec3s_t &object_points) const;

  /** Get all measurements */
  void getMeasurements(std::vector<int> &corner_ids,
                       vec2s_t &keypoints,
                       vec3s_t &object_points) const override;

  /** Add measurmeent */
  void add(const int tag_id, const int corner_index, const vec2_t &kp);

  /** Remove measurmeent */
  void remove(const int tag_id, const int corner_index);

  /** Remove measurmeent */
  void remove(const int tag_id);

  /** Save AprilGrid **/
  int save(const std::string &save_path) const;

  /** Load AprilGrid **/
  static AprilGrid load(const std::string &data_path);

  /** Draw AprilGrid */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  /** Imshow AprilGrid */
  void imshow(const std::string &title, const cv::Mat &image) const;
};

} // namespace yac
