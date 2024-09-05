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

double AprilGrid::getTagSpacing() const { return tag_spacing_; }

double AprilGrid::getTagSize() const { return tag_size_; }

bool AprilGrid::has(const int tag_id, const int corner_index) const {
  if (data_.count(tag_id) == 0) {
    return false;
  }

  if (data_.at(tag_id).corner_indicies.count(corner_index) == 0) {
    return false;
  }

  return true;
}

vec2_t AprilGrid::center() const {

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

void AprilGrid::add(const int tag_id,
                    const int corner_index,
                    const vec2_t &kp) {
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

// void AprilGrid::imshow(const std::string &title, const cv::Mat &image) const
// {
//
//   cv::imshow(title, draw(image));
//   cv::waitKey(1);
// }
//
// int AprilGrid::estimate(const CameraModel *cam,
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

  // Output header
  // -- Configuration
  fprintf(fp, "#");
  fprintf(fp, "ts,");
  fprintf(fp, "tag_rows,");
  fprintf(fp, "tag_cols,");
  fprintf(fp, "tag_size,");
  fprintf(fp, "tag_spacing,");
  // -- Keypoint
  fprintf(fp, "kp_x,kp_y,");
  // -- Object point
  fprintf(fp, "p_x,p_y,p_z,");
  // -- tag_id, corner_index
  fprintf(fp, "tag_id,corner_index");
  fprintf(fp, "\n");

  // Output data
  // if (detected) {
  //   vec2s_t kps = keypoints();
  //   for (long i = 0; i < data.rows(); i++) {
  //     const int tag_id = int(i / 4);
  //     const int corner_index = (i % 4);
  //
  //     int corner_detected = data(i, 0);
  //     if (corner_detected > 0) {
  //       const vec2_t kp = data.block(i, 1, 1, 2).transpose();
  //       const vec3_t p = data.block(i, 3, 1, 3).transpose();
  //       fprintf(fp, "%ld,", timestamp);
  //       fprintf(fp, "%d,", tag_rows);
  //       fprintf(fp, "%d,", tag_cols);
  //       fprintf(fp, "%f,", tag_size);
  //       fprintf(fp, "%f,", tag_spacing);
  //       fprintf(fp, "%.10f,", kp.x());
  //       fprintf(fp, "%.10f,", kp.y());
  //       fprintf(fp, "%f,", p(0));
  //       fprintf(fp, "%f,", p(1));
  //       fprintf(fp, "%f,", p(2));
  //       fprintf(fp, "%d,", tag_id);
  //       fprintf(fp, "%d", corner_index);
  //       fprintf(fp, "\n");
  //     }
  //   }
  // }

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
  char delim[2] = ":";
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
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &ts);
  aprilgrid_parse_line(fp, "num_rows", "int", &tag_rows);
  aprilgrid_parse_line(fp, "num_cols", "int", &tag_cols);
  aprilgrid_parse_line(fp, "tag_size", "double", &tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "double", &tag_spacing);
  aprilgrid_parse_skip_line(fp);

  // Parse data
  AprilGrid grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};
  int corners_detected = 0;
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);

  const char *scan_format = "%d,%d,%lf,%lf,%lf,%lf,%lf";
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
      FATAL("Failed to parse data line in [%s]\n", data_path.c_str());
    }

    // // Resize data if not already
    // if (data.rows() == 0 && data.cols() == 0) {
    //   data.resize(tag_rows * tag_cols * 4, 6);
    //   data.setZero();
    // }

    // // Map variables back to data
    // const int data_row = (tag_id * 4) + corner_index;
    // data(data_row, 0) = 1.0;
    // data(data_row, 1) = kp[0];
    // data(data_row, 2) = kp[1];
    // data(data_row, 3) = p[0];
    // data(data_row, 4) = p[1];
    // data(data_row, 5) = p[2];
    // detected = true;
    // nb_detections++;
  }

  // Clean up
  fclose(fp);

  return grid;
}

// // aprilgrids_t load_aprilgrids(const std::string &dir_path,
// //                              const bool format_v2) {
// //   std::vector<std::string> csv_files;
// //   if (list_dir(dir_path, csv_files) != 0) {
// //     FATAL("Failed to list dir [%s]!", dir_path.c_str());
// //   }
// //   sort(csv_files.begin(), csv_files.end());
// //
// //   aprilgrids_t grids;
// //   for (const auto &grid_csv : csv_files) {
// //     const auto csv_path = dir_path + "/" + grid_csv;
// //     AprilGrid grid{csv_path, format_v2};
// //     if (grid.detected) {
// //       grids.push_back(grid);
// //     }
// //   }
// //
// //   return grids;
// // }

} // namespace yac
