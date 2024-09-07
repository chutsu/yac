#include "AprilGrid.hpp"

namespace yac {

AprilGrid::AprilGrid(const timestamp_t &timestamp,
                     const int tag_rows,
                     const int tag_cols,
                     const double tag_size,
                     const double tag_spacing)
    : CalibTarget{timestamp, tag_rows * 2, tag_cols * 2}, tag_rows_{tag_rows},
      tag_cols_{tag_cols}, tag_size_{tag_size}, tag_spacing_{tag_spacing} {}

void AprilGrid::gridIndex(const int tag_id, int &i, int &j) const {
  if (tag_id > (tag_rows_ * tag_cols_)) {
    FATAL("tag_id > (tag_rows * tag_cols)!");
  } else if (tag_id < 0) {
    FATAL("tag_id < 0!");
  }

  i = int(tag_id / tag_cols_);
  j = int(tag_id % tag_cols_);
}

vec3_t AprilGrid::objectPoint(const int tag_id, const int corner_index) const {

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  gridIndex(tag_id, i, j);

  // Caculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const double x = j * (tag_size_ + tag_size_ * tag_spacing_);
  const double y = i * (tag_size_ + tag_size_ * tag_spacing_);

  // Calculate the x and y of each corner
  vec3_t object_point;
  switch (corner_index) {
    case 0: // Bottom left
      object_point = vec3_t(x, y, 0);
      break;
    case 1: // Bottom right
      object_point = vec3_t(x + tag_size_, y, 0);
      break;
    case 2: // Top right
      object_point = vec3_t(x + tag_size_, y + tag_size_, 0);
      break;
    case 3: // Top left
      object_point = vec3_t(x, y + tag_size_, 0);
      break;
    default:
      FATAL("Incorrect corner id [%d]!", corner_index);
      break;
  }

  return object_point;
}

vec3s_t AprilGrid::objectPoints() const {
  vec3s_t object_points_;

  // for (int i = 0; i < (tag_rows_ * tag_cols_ * 4); i++) {
  //   if (data(i, 0) > 0) {
  //     object_points_.push_back(data.block(i, 3, 1, 3).transpose());
  //   }
  // }

  return object_points_;
}

int AprilGrid::getTagRows() const { return tag_rows_; }

int AprilGrid::getTagCols() const { return tag_cols_; }

double AprilGrid::getTagSize() const { return tag_size_; }

double AprilGrid::getTagSpacing() const { return tag_spacing_; }

vec2_t AprilGrid::getCenter() const {
  double x = ((tag_cols_ / 2.0) * tag_size_);
  x += (((tag_cols_ / 2.0) - 1) * tag_spacing_ * tag_size_);
  x += (0.5 * tag_spacing_ * tag_size_);

  double y = ((tag_rows_ / 2.0) * tag_size_);
  y += (((tag_rows_ / 2.0) - 1) * tag_spacing_ * tag_size_);
  y += (0.5 * tag_spacing_ * tag_size_);

  return vec2_t{x, y};
}

void AprilGrid::getMeasurements(std::vector<int> &tag_ids,
                                std::vector<int> &corner_indicies,
                                vec2s_t &keypoints,
                                vec3s_t &object_points) const {
  for (const auto &[tag_id, tag_det] : data_) {
    for (const auto corner_index : tag_det.corner_indicies) {
      tag_ids.push_back(tag_id);
      corner_indicies.push_back(corner_index);
      keypoints.push_back(tag_det.keypoints.at(corner_index));
      object_points.push_back(tag_det.object_points.at(corner_index));
    }
  }
}

void AprilGrid::getMeasurements(std::vector<int> &corner_ids,
                                vec2s_t &keypoints,
                                vec3s_t &object_points) const {
  for (const auto &[tag_id, tag_det] : data_) {
    for (const auto corner_index : tag_det.corner_indicies) {
      const int corner_id = tag_id * 4 + corner_index;
      corner_ids.push_back(corner_id);
      keypoints.push_back(tag_det.keypoints.at(corner_index));
      object_points.push_back(tag_det.object_points.at(corner_index));
    }
  }
}

bool AprilGrid::has(const int tag_id, const int corner_index) const {
  if (data_.count(tag_id) == 0) {
    return false;
  }

  if (data_.at(tag_id).corner_indicies.count(corner_index) == 0) {
    return false;
  }

  return true;
}

void AprilGrid::add(const int tag_id,
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
  data_[tag_id].object_points[corner_index] = objectPoint(tag_id, corner_index);
}

void AprilGrid::remove(const int tag_id, const int corner_index) {
  if (data_.count(tag_id) == 0) {
    return;
  }

  data_[tag_id].corner_indicies.erase(corner_index);
  data_[tag_id].keypoints.erase(corner_index);
  data_[tag_id].object_points.erase(corner_index);
}

void AprilGrid::remove(const int tag_id) {
  remove(tag_id, 0);
  remove(tag_id, 1);
  remove(tag_id, 2);
  remove(tag_id, 3);
}

cv::Mat AprilGrid::draw(const cv::Mat &image,
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

void AprilGrid::imshow(const std::string &title, const cv::Mat &image) const {
  cv::imshow(title, draw(image));
  cv::waitKey(1);
}

int AprilGrid::save(const std::string &save_path) const {
  // Check save dir
  const std::string dir_path = dir_name(save_path);
  if (dir_create(dir_path) != 0) {
    LOG_ERROR("Could not create dir [%s]!", dir_path.c_str());
    return -1;
  }

  // Open file for saving
  const auto fp = fopen(save_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", save_path.c_str());
    return -1;
  }

  // Get measurement data
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  getMeasurements(tag_ids, corner_indicies, keypoints, object_points);

  // Output header
  fprintf(fp, "timestamp %ld\n", ts_);
  fprintf(fp, "num_rows %d\n", tag_rows_);
  fprintf(fp, "num_cols %d\n", tag_cols_);
  fprintf(fp, "tag_size %f\n", tag_size_);
  fprintf(fp, "tag_spacing %f\n", tag_spacing_);
  fprintf(fp, "corners_detected %ld\n", corner_indicies.size());
  fprintf(fp, "\n");
  fprintf(fp, "# tag_id corner_index kp_x kp_y pt_x pt_y pt_z\n");
  for (size_t i = 0; i < corner_indicies.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_index = corner_indicies[i];
    const vec2_t kp = keypoints[i];
    const vec3_t pt = object_points[i];

    fprintf(fp, "%d ", tag_id);
    fprintf(fp, "%d ", corner_index);
    fprintf(fp, "%lf ", kp.x());
    fprintf(fp, "%lf ", kp.y());
    fprintf(fp, "%lf ", pt.x());
    fprintf(fp, "%lf ", pt.y());
    fprintf(fp, "%lf\n", pt.z());
  }

  // Close up
  fclose(fp);
  return 0;
}

static void aprilgrid_parse_line(FILE *fp,
                                 const char *key,
                                 const char *value_type,
                                 void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = " ";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *)value = atol(value_str);
  } else if (strcmp(value_type, "int") == 0) {
    *(int *)value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *)value = atof(value_str);
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

static void aprilgrid_parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  char *retval = fgets(buf, buf_len, fp);
  UNUSED(retval);
}

AprilGrid AprilGrid::load(const std::string &data_path) {
  // Open file for loading
  FILE *fp = fopen(data_path.c_str(), "r");
  if (fp == NULL) {
    FATAL("Failed to open [%s]!\n", data_path.c_str());
  }

  // Parse configuration
  timestamp_t ts = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0;
  double tag_spacing = 0;
  int corners_detected = 0;
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &ts);
  aprilgrid_parse_line(fp, "num_rows", "int", &tag_rows);
  aprilgrid_parse_line(fp, "num_cols", "int", &tag_cols);
  aprilgrid_parse_line(fp, "tag_size", "double", &tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "double", &tag_spacing);
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);
  aprilgrid_parse_skip_line(fp);
  AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

  // Parse data
  const char *scan_format = "%d %d %lf %lf %lf %lf %lf";
  for (int i = 0; i < corners_detected; i++) {
    // Parse data line
    int tag_id = 0;
    int corner_index = 0;
    double kp[2] = {0};
    double p[3] = {0};
    const int retval = fscanf(fp,
                              scan_format,
                              &tag_id,
                              &corner_index,
                              &kp[0],
                              &kp[1],
                              &p[0],
                              &p[1],
                              &p[2]);
    if (retval != 7) {
      FATAL("Failed to parse data line %d in [%s]\n", i, data_path.c_str());
    }
    grid.add(tag_id, corner_index, {kp[0], kp[1]});
  }

  // Clean up
  fclose(fp);

  return grid;
}

std::vector<AprilGrid> AprilGrid::loadDirectory(const std::string &dir_path) {
  std::vector<std::string> csv_files;
  if (list_dir(dir_path, csv_files) != 0) {
    FATAL("Failed to list dir [%s]!", dir_path.c_str());
  }
  sort(csv_files.begin(), csv_files.end());

  std::vector<AprilGrid> grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path + "/" + grid_csv;
    grids.push_back(AprilGrid::load(grid_csv));
  }

  return grids;
}

AprilGridDetector::AprilGridDetector(const int tag_rows,
                                     const int tag_cols,
                                     const double tag_size,
                                     const double tag_spacing)
    : tag_rows_{tag_rows}, tag_cols_{tag_cols}, tag_size_{tag_size},
      tag_spacing_{tag_spacing} {
  det_->quad_decimate = 1.0;
  det_->quad_sigma = 0.0; // Blur
  det_->nthreads = 2;
  det_->debug = 0;
  det_->refine_edges = 1;
}

AprilGridDetector::~AprilGridDetector() {
  apriltag_detector_destroy(det_);
  tag36h11_destroy(tf_);
}

void AprilGridDetector::olsenDetect(const cv::Mat &image,
                                    std::vector<int> &tag_ids,
                                    std::vector<int> &corner_indicies,
                                    std::vector<vec2_t> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  // Make an image_u8_t header for the Mat data
  image_u8_t im = {.width = image.cols,
                   .height = image.rows,
                   .stride = image.cols,
                   .buf = image.data};

  // Detector tags
  zarray_t *detections = apriltag_detector_detect(det_, &im);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    // Check tag id
    if (det->id < 0 || det->id >= (tag_rows_ * tag_cols_)) {
      continue;
    }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= det->p[i][0] < min_border_dist_;
      bad_tag |= det->p[i][0] > image_width - min_border_dist_;
      bad_tag |= det->p[i][1] < min_border_dist_;
      bad_tag |= det->p[i][1] > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(det->id); // Bottom left
    tag_ids.push_back(det->id); // Bottom right
    tag_ids.push_back(det->id); // Top right
    tag_ids.push_back(det->id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    keypoints.emplace_back(det->p[0][0], det->p[0][1]); // Bottom left
    keypoints.emplace_back(det->p[1][0], det->p[1][1]); // Bottom right
    keypoints.emplace_back(det->p[2][0], det->p[2][1]); // Top right
    keypoints.emplace_back(det->p[3][0], det->p[3][1]); // Top left
  }
  apriltag_detections_destroy(detections);
}

void AprilGridDetector::kaessDetect(const cv::Mat &image,
                                    std::vector<int> &tag_ids,
                                    std::vector<int> &corner_indicies,
                                    std::vector<vec2_t> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  const auto detections = detector.extractTags(image);
  for (const auto &tag : detections) {
    // Check detection
    if (tag.good == false) {
      continue;
    }

    // Check tag id
    if (tag.id < 0 || tag.id >= (tag_rows_ * tag_cols_)) {
      continue;
    }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= tag.p[i].first < min_border_dist_;
      bad_tag |= tag.p[i].first > image_width - min_border_dist_;
      bad_tag |= tag.p[i].second < min_border_dist_;
      bad_tag |= tag.p[i].second > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(tag.id); // Bottom left
    tag_ids.push_back(tag.id); // Bottom right
    tag_ids.push_back(tag.id); // Top right
    tag_ids.push_back(tag.id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    keypoints.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    keypoints.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
    keypoints.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    keypoints.emplace_back(tag.p[3].first, tag.p[3].second); // Top left
  }
}

AprilGrid AprilGridDetector::detect(const timestamp_t ts,
                                    const cv::Mat &image) {
  AprilGrid grid{ts, tag_rows_, tag_cols_, tag_size_, tag_spacing_};

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  std::vector<vec2_t> keypoints;
  kaessDetect(image, tag_ids, corner_indicies, keypoints);
  for (size_t i = 0; i < tag_ids.size(); i++) {
    grid.add(tag_ids[i], corner_indicies[i], keypoints[i]);
  }

  return grid;
}

} // namespace yac
