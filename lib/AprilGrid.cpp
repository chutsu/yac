#include "AprilGrid.hpp"

namespace yac {

AprilGrid::AprilGrid(const std::string &csv_path) : init{true} {
  load(csv_path);
}

AprilGrid::AprilGrid(const timestamp_t &timestamp,
                     const int tag_rows,
                     const int tag_cols,
                     const double tag_size,
                     const double tag_spacing)
    : init{true}, timestamp{timestamp}, tag_rows{tag_rows}, tag_cols{tag_cols},
      tag_size{tag_size},
      tag_spacing{tag_spacing}, data{zeros(tag_rows * tag_cols * 4, 6)} {}

void AprilGrid::clear() {
  detected = false;
  nb_detections = 0;
  data.setZero();
}

bool AprilGrid::has(const int tag_id, const int corner_idx) const {
  assert(init == true);
  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) <= 0) {
    return false;
  }

  return true;
}

vec2_t AprilGrid::center() const {
  assert(init == true);

  double x = ((tag_cols / 2.0) * tag_size);
  x += (((tag_cols / 2.0) - 1) * tag_spacing * tag_size);
  x += (0.5 * tag_spacing * tag_size);

  double y = ((tag_rows / 2.0) * tag_size);
  y += (((tag_rows / 2.0) - 1) * tag_spacing * tag_size);
  y += (0.5 * tag_spacing * tag_size);

  return vec2_t{x, y};
}

void AprilGrid::gridIndex(const int tag_id, int &i, int &j) const {
  assert(init == true);

  if (tag_id > (tag_rows * tag_cols)) {
    FATAL("tag_id > (tag_rows * tag_cols)!");
  } else if (tag_id < 0) {
    FATAL("tag_id < 0!");
  }

  i = int(tag_id / tag_cols);
  j = int(tag_id % tag_cols);
}

vec3_t AprilGrid::objectPoint(const int tag_id, const int corner_idx) const {
  assert(init == true);

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  gridIndex(tag_id, i, j);

  // Caculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const double x = j * (tag_size + tag_size * tag_spacing);
  const double y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner
  vec3_t object_point;
  switch (corner_idx) {
    case 0: // Bottom left
      object_point = vec3_t(x, y, 0);
      break;
    case 1: // Bottom right
      object_point = vec3_t(x + tag_size, y, 0);
      break;
    case 2: // Top right
      object_point = vec3_t(x + tag_size, y + tag_size, 0);
      break;
    case 3: // Top left
      object_point = vec3_t(x, y + tag_size, 0);
      break;
    default:
      FATAL("Incorrect corner id [%d]!", corner_idx);
      break;
  }

  return object_point;
}

vec3s_t AprilGrid::objectPoints() const {
  assert(init == true);
  vec3s_t object_points_;

  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      object_points_.push_back(data.block(i, 3, 1, 3).transpose());
    }
  }

  return object_points_;
}

vec2_t AprilGrid::keypoint(const int tag_id, const int corner_idx) const {
  assert(init == true);
  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) > 0) {
    return data.block(data_row, 1, 1, 2).transpose();
  }

  FATAL("Keypoint [tag_id: %d, corner: %d] does not exist!",
        tag_id,
        corner_idx);
}

vec2s_t AprilGrid::keypoints() const {
  assert(init == true);
  vec2s_t keypoints_;

  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      keypoints_.push_back(data.block(i, 1, 1, 2).transpose());
    }
  }

  return keypoints_;
}

void AprilGrid::get_measurements(std::vector<int> &tag_ids,
                                 std::vector<int> &corner_indicies,
                                 vec2s_t &keypoints,
                                 vec3s_t &object_points) const {
  assert(init == true);

  if (detected == false) {
    return;
  }

  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      tag_ids.push_back(int(i / 4));
      corner_indicies.push_back(i % 4);
      keypoints.emplace_back(data(i, 1), data(i, 2));
      object_points.emplace_back(data(i, 3), data(i, 4), data(i, 5));
    }
  }
}

std::vector<int> AprilGrid::tag_ids() const {
  assert(init == true);
  std::set<int> tag_ids_;
  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0)) {
      tag_ids_.insert(int(i / 4));
    }
  }

  return std::vector<int>{tag_ids_.begin(), tag_ids_.end()};
}

void AprilGrid::add(const int tag_id, const int corner_idx, const vec2_t &kp) {
  assert(init == true);

  // Set AprilGrid as detected
  detected = true;
  nb_detections++;

  // Push tag_id and keypoints
  const int data_row = (tag_id * 4) + corner_idx;
  const vec3_t obj_pt = objectPoint(tag_id, corner_idx).transpose();
  data(data_row, 0) = 1;
  data.block(data_row, 1, 1, 2) = kp.transpose();
  data.block(data_row, 3, 1, 3) = obj_pt.transpose();
}

void AprilGrid::remove(const int tag_id, const int corner_idx) {
  assert(init == true);

  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) > 0) {
    data.block(data_row, 0, 1, 6).setZero();
    nb_detections = std::max(nb_detections - 1, 0);
    detected = (nb_detections == 0) ? false : true;
  } else {
    // FATAL("tag_id: %d, corner_idx: %d does not exist", tag_id, corner_idx);
  }
}

void AprilGrid::remove(const int tag_id) {
  assert(init == true);

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
  get_measurements(tag_ids, corner_indicies, keypoints, object_points);

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
  assert(init == true);
  cv::imshow(title, draw(image));
  cv::waitKey(1);
}

int AprilGrid::estimate(const CameraModel *cam,
                        const int cam_res[2],
                        const vecx_t &cam_params,
                        mat4_t &T_CF) const {
  assert(init == true);

  // Check if we actually have data to work with
  if (nb_detections == 0) {
    return -1;
  }

  // Create object points (counter-clockwise, from bottom left)
  vec2s_t img_pts;
  vec3s_t obj_pts;
  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      img_pts.emplace_back(data(i, 1), data(i, 2));
      obj_pts.emplace_back(data(i, 3), data(i, 4), data(i, 5));
    }
  }

  return solvepnp(cam, cam_res, cam_params, img_pts, obj_pts, T_CF);
}

int AprilGrid::save(const std::string &save_path) const {
  assert(init == true);

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
  // -- tag_id, corner_idx
  fprintf(fp, "tag_id,corner_idx");
  fprintf(fp, "\n");

  // Output data
  if (detected) {
    vec2s_t kps = keypoints();
    for (long i = 0; i < data.rows(); i++) {
      const int tag_id = int(i / 4);
      const int corner_idx = (i % 4);

      int corner_detected = data(i, 0);
      if (corner_detected > 0) {
        const vec2_t kp = data.block(i, 1, 1, 2).transpose();
        const vec3_t p = data.block(i, 3, 1, 3).transpose();
        fprintf(fp, "%ld,", timestamp);
        fprintf(fp, "%d,", tag_rows);
        fprintf(fp, "%d,", tag_cols);
        fprintf(fp, "%f,", tag_size);
        fprintf(fp, "%f,", tag_spacing);
        fprintf(fp, "%.10f,", kp.x());
        fprintf(fp, "%.10f,", kp.y());
        fprintf(fp, "%f,", p(0));
        fprintf(fp, "%f,", p(1));
        fprintf(fp, "%f,", p(2));
        fprintf(fp, "%d,", tag_id);
        fprintf(fp, "%d", corner_idx);
        fprintf(fp, "\n");
      }
    }
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

int AprilGrid::load(const std::string &data_path) {
  // Open file for loading
  FILE *fp = fopen(data_path.c_str(), "r");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s]!\n", data_path.c_str());
    return -1;
  }

  // Parse configuration
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &timestamp);
  aprilgrid_parse_line(fp, "num_rows", "int", &tag_rows);
  aprilgrid_parse_line(fp, "num_cols", "int", &tag_cols);
  aprilgrid_parse_line(fp, "tag_size", "double", &tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "double", &tag_spacing);
  aprilgrid_parse_skip_line(fp);

  // Parse data
  int corners_detected = 0;
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);

#if PRECISION == 1
  const char *scan_format = "%d,%d,%f,%f,%f,%f,%f";
#else
  const char *scan_format = "%d,%d,%lf,%lf,%lf,%lf,%lf";
#endif
  for (int i = 0; i < corners_detected; i++) {
    // Parse data line
    int tag_id = 0;
    int corner_idx = 0;
    double kp[2] = {0};
    double p[3] = {0};
    const int retval = fscanf(fp,
                              scan_format,
                              &tag_id,
                              &corner_idx,
                              &kp[0],
                              &kp[1],
                              &p[0],
                              &p[1],
                              &p[2]);
    if (retval != 7) {
      FATAL("Failed to parse data line in [%s]\n", data_path.c_str());
    }

    // Resize data if not already
    if (data.rows() == 0 && data.cols() == 0) {
      data.resize(tag_rows * tag_cols * 4, 6);
      data.setZero();
    }

    // Map variables back to data
    const int data_row = (tag_id * 4) + corner_idx;
    data(data_row, 0) = 1.0;
    data(data_row, 1) = kp[0];
    data(data_row, 2) = kp[1];
    data(data_row, 3) = p[0];
    data(data_row, 4) = p[1];
    data(data_row, 5) = p[2];
    detected = true;
    nb_detections++;
  }

  // Clean up
  fclose(fp);

  return 0;
}

// aprilgrids_t load_aprilgrids(const std::string &dir_path,
//                              const bool format_v2) {
//   std::vector<std::string> csv_files;
//   if (list_dir(dir_path, csv_files) != 0) {
//     FATAL("Failed to list dir [%s]!", dir_path.c_str());
//   }
//   sort(csv_files.begin(), csv_files.end());
//
//   aprilgrids_t grids;
//   for (const auto &grid_csv : csv_files) {
//     const auto csv_path = dir_path + "/" + grid_csv;
//     AprilGrid grid{csv_path, format_v2};
//     if (grid.detected) {
//       grids.push_back(grid);
//     }
//   }
//
//   return grids;
// }

} // namespace yac
