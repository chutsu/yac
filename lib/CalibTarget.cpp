#include "CalibTarget.hpp"

namespace yac {

CalibTarget::CalibTarget(const timestamp_t &ts,
                         const int tag_rows,
                         const int tag_cols,
                         const double tag_size,
                         const double tag_spacing)
    : ts_{ts}, tag_rows_{tag_rows}, tag_cols_{tag_cols}, tag_size_{tag_size},
      tag_spacing_{tag_spacing} {}

timestamp_t CalibTarget::getTimestamp() const { return ts_; }

int CalibTarget::getTagRows() const { return tag_rows_; }

int CalibTarget::getTagCols() const { return tag_cols_; }

double CalibTarget::getTagSize() const { return tag_size_; }

double CalibTarget::getTagSpacing() const { return tag_spacing_; }

vec2_t CalibTarget::getCenter() const {
  double x = ((tag_cols_ / 2.0) * tag_size_);
  x += (((tag_cols_ / 2.0) - 1) * tag_spacing_ * tag_size_);
  x += (0.5 * tag_spacing_ * tag_size_);

  double y = ((tag_rows_ / 2.0) * tag_size_);
  y += (((tag_rows_ / 2.0) - 1) * tag_spacing_ * tag_size_);
  y += (0.5 * tag_spacing_ * tag_size_);

  return vec2_t{x, y};
}

int CalibTarget::getNumDetected() const {
  int num_detected = 0;
  for (const auto &[tag_id, tag_det] : data_) {
    num_detected += tag_det.keypoints.size();
  }

  return num_detected;
}

void CalibTarget::add(const int tag_id,
                      const int corner_index,
                      const vec2_t &kp) {
  if (tag_id < 0 || tag_id >= (tag_rows_ * tag_cols_)) {
    return;
  }

  if (data_.count(tag_id) == 0) {
    data_[tag_id] = TagDetection();
  }

  data_[tag_id].corner_indicies.insert(corner_index);
  data_[tag_id].keypoints[corner_index] = kp;
}

bool CalibTarget::has(const int tag_id, const int corner_index) const {
  if (data_.count(tag_id) == 0) {
    return false;
  }

  if (data_.at(tag_id).corner_indicies.count(corner_index) == 0) {
    return false;
  }

  return true;
}

void CalibTarget::remove(const int tag_id, const int corner_index) {
  if (data_.count(tag_id) == 0) {
    return;
  }

  data_[tag_id].corner_indicies.erase(corner_index);
  data_[tag_id].keypoints.erase(corner_index);
}

void CalibTarget::remove(const int tag_id) {
  remove(tag_id, 0);
  remove(tag_id, 1);
  remove(tag_id, 2);
  remove(tag_id, 3);
}

cv::Mat CalibTarget::draw(const cv::Mat &image,
                          const int marker_size,
                          const cv::Scalar &color) const {
  const cv::Scalar text_color(0, 255, 0);
  const int font = cv::FONT_HERSHEY_PLAIN;
  const double font_scale = 1.0;
  const int thickness = 2;
  cv::Mat image_rgb = gray2rgb(image);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  getMeasurements(tag_ids, corner_indicies, keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    // Setup
    const auto tag_id = tag_ids[i];
    const auto kp = keypoints[i];

    // Draw corners
    cv::Point2f p(kp(0), kp(1));
    cv::circle(image_rgb, p, marker_size, color, -1);

    // Label corner
    cv::Point2f cxy(kp.x(), kp.y());
    std::string text = std::to_string(tag_id);
    cv::putText(image_rgb, text, cxy, font, font_scale, text_color, thickness);
  }

  return image_rgb;
}

void CalibTarget::imshow(const std::string &title, const cv::Mat &image) const {
  cv::imshow(title, draw(image));
  cv::waitKey(1);
}

// int CalibTarget::estimate(const CameraModel *cam,
//                         const int cam_res[2],
//                         const vecx_t &cam_params,
//                         mat4_t &T_CF) const {
//
//
//   // Check if we actually have data to work with
//   if (nb_detections == 0) {
//     return -1;
//   }
//
//   // Create object points (counter-clockwise, from bottom left)
//   vec2s_t img_pts;
//   vec3s_t obj_pts;
//   for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
//     if (data(i, 0) > 0) {
//       img_pts.emplace_back(data(i, 1), data(i, 2));
//       obj_pts.emplace_back(data(i, 3), data(i, 4), data(i, 5));
//     }
//   }
//
//   return solvepnp(cam, cam_res, cam_params, img_pts, obj_pts, T_CF);
// }

} // namespace yac
