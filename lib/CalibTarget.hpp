#pragma once

#include "Core.hpp"

namespace yac {

/** Calibration Target */
class CalibTarget {
protected:
  timestamp_t ts_ = 0;
  int tag_rows_ = 0;
  int tag_cols_ = 0;
  double tag_size_ = 0.0;
  double tag_spacing_ = 0.0;

  // Data
  struct TagDetection {
    std::set<int> corner_indicies;
    std::unordered_map<int, vec2_t> keypoints;
  };
  std::unordered_map<int, TagDetection> data_;

public:
  CalibTarget(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing);
  virtual ~CalibTarget() = default;

  /** Check if detected */
  bool detected() const;

  /** Get Timestamp **/
  timestamp_t getTimestamp() const;

  /** Return number of tag rows */
  int getTagRows() const;

  /** Return number of tag cols */
  int getTagCols() const;

  /** Return number of tag size */
  double getTagSize() const;

  /** Return number of tag spacing */
  double getTagSpacing() const;

  /** Get the center of the AprilGrid */
  vec2_t getCenter() const;

  /** Get number detected */
  int getNumDetected() const;

  /** Get measurements **/
  virtual void getMeasurements(std::vector<int> &tag_ids,
                               std::vector<int> &corner_indicies,
                               vec2s_t &keypoints,
                               vec3s_t &object_points) const = 0;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Add measurmeent */
  void add(const int tag_id, const int corner_index, const vec2_t &kp);

  /** Remove measurmeent */
  void remove(const int tag_id, const int corner_index);

  /** Remove measurmeent */
  void remove(const int tag_id);

  /** Draw AprilGrid */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  /** Imshow AprilGrid */
  void imshow(const std::string &title, const cv::Mat &image) const;
};

} // namespace yac
