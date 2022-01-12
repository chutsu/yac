#include "aprilgrid.hpp"

namespace yac {

aprilgrid_t::aprilgrid_t() {}

aprilgrid_t::aprilgrid_t(const std::string &csv_path) : init{true} {
  load(csv_path);
}

aprilgrid_t::aprilgrid_t(const timestamp_t &timestamp,
                         const int tag_rows,
                         const int tag_cols,
                         const double tag_size,
                         const double tag_spacing)
    : init{true}, timestamp{timestamp}, tag_rows{tag_rows}, tag_cols{tag_cols},
      tag_size{tag_size},
      tag_spacing{tag_spacing}, data{zeros(tag_rows * tag_cols * 4, 6)} {}

aprilgrid_t::~aprilgrid_t() {}

void aprilgrid_t::clear() {
  assert(init == true);

  detected = false;
  nb_detections = 0;
  data.setZero();
}

bool aprilgrid_t::fully_observable() const {
  assert(init == true);
  return (nb_detections == (tag_rows * tag_cols * 4));
}

bool aprilgrid_t::has(const int tag_id, const int corner_idx) const {
  assert(init == true);
  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) <= 0) {
    return false;
  }

  return true;
}

vec2_t aprilgrid_t::center() const {
  assert(init == true);

  double x = ((tag_cols / 2.0) * tag_size);
  x += (((tag_cols / 2.0) - 1) * tag_spacing * tag_size);
  x += (0.5 * tag_spacing * tag_size);

  double y = ((tag_rows / 2.0) * tag_size);
  y += (((tag_rows / 2.0) - 1) * tag_spacing * tag_size);
  y += (0.5 * tag_spacing * tag_size);

  return vec2_t{x, y};
}

void aprilgrid_t::grid_index(const int tag_id, int &i, int &j) const {
  assert(init == true);

  if (tag_id > (tag_rows * tag_cols)) {
    FATAL("tag_id > (tag_rows * tag_cols)!");
  } else if (tag_id < 0) {
    FATAL("tag_id < 0!");
  }

  i = int(tag_id / tag_cols);
  j = int(tag_id % tag_cols);
}

vec3_t aprilgrid_t::object_point(const int tag_id, const int corner_idx) const {
  assert(init == true);

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  grid_index(tag_id, i, j);

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

vec3s_t aprilgrid_t::object_points() const {
  assert(init == true);
  vec3s_t object_points_;

  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      object_points_.push_back(data.block(i, 3, 1, 3).transpose());
    }
  }

  return object_points_;
}

vec2_t aprilgrid_t::keypoint(const int tag_id, const int corner_idx) const {
  assert(init == true);
  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) > 0) {
    return data.block(data_row, 1, 1, 2).transpose();
  }

  FATAL("Keypoint [tag_id: %d, corner: %d] does not exist!",
        tag_id,
        corner_idx);
}

vec2s_t aprilgrid_t::keypoints() const {
  assert(init == true);
  vec2s_t keypoints_;

  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0) > 0) {
      keypoints_.push_back(data.block(i, 1, 1, 2).transpose());
    }
  }

  return keypoints_;
}

void aprilgrid_t::get_measurements(std::vector<int> &tag_ids,
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

std::vector<int> aprilgrid_t::tag_ids() const {
  assert(init == true);
  std::set<int> tag_ids_;
  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0)) {
      tag_ids_.insert(int(i / 4));
    }
  }

  return std::vector<int>{tag_ids_.begin(), tag_ids_.end()};
}

void aprilgrid_t::add(const int tag_id,
                      const int corner_idx,
                      const vec2_t &kp) {
  assert(init == true);

  // Set AprilGrid as detected
  detected = true;
  nb_detections++;

  // Push tag_id and keypoints
  const int data_row = (tag_id * 4) + corner_idx;
  const vec3_t obj_pt = object_point(tag_id, corner_idx).transpose();
  data(data_row, 0) = 1;
  data.block(data_row, 1, 1, 2) = kp.transpose();
  data.block(data_row, 3, 1, 3) = obj_pt.transpose();
}

void aprilgrid_t::remove(const int tag_id, const int corner_idx) {
  assert(init == true);

  const int data_row = (tag_id * 4) + corner_idx;
  if (data(data_row, 0) > 0) {
    data.block(data_row, 0, 1, 6).setZero();
    nb_detections = (nb_detections > 0) ? (nb_detections - 1) : 0;
    detected = (nb_detections == 0) ? false : true;
  }
}

void aprilgrid_t::remove(const int tag_id) {
  assert(init == true);

  remove(tag_id, 0);
  remove(tag_id, 1);
  remove(tag_id, 2);
  remove(tag_id, 3);
}

cv::Mat aprilgrid_t::draw(const cv::Mat &image,
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

void aprilgrid_t::imshow(const std::string &title, const cv::Mat &image) const {
  assert(init == true);
  cv::imshow(title, draw(image));
  cv::waitKey(1);
}

int aprilgrid_t::estimate(const camera_geometry_t *cam,
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

int aprilgrid_t::save(const std::string &save_path) const {
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
        fprintf(fp, "%f,", kp(0));
        fprintf(fp, "%f,", kp(1));
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

int aprilgrid_t::load(const std::string &data_path) {
  assert(init == true);
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", data_path.c_str());
    return -1;
  }

  // Create format string
  std::string str_format;
  // -- Timestamp, tag_rows, tag_cols, tag_size, tag_spacing
  str_format += "%ld,%d,%d,%lf,%lf,";
  // -- Keypoint
  str_format += "%lf,%lf,";
  // -- Object point
  str_format += "%lf,%lf,%lf,";
  // -- tag_id, corner_idx
  str_format += "%d,%d";

  // Parse file
  clear();

  // Parse data
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    double kp_x, kp_y = 0.0;
    double p_x, p_y, p_z = 0.0;
    int tag_id = 0;
    int corner_idx = 0;
    int retval = fscanf(
        // File pointer
        fp,
        // String format
        str_format.c_str(),
        &timestamp,
        &tag_rows,
        &tag_cols,
        &tag_size,
        &tag_spacing,
        &kp_x,
        &kp_y,
        &p_x,
        &p_y,
        &p_z,
        &tag_id,
        &corner_idx);
    if (retval != 12) {
      LOG_INFO("Failed to parse line in [%s:%d]", data_path.c_str(), i);
      return -1;
    }

    // Resize data if not already
    if (data.rows() == 0 && data.cols() == 0) {
      data.resize(tag_rows * tag_cols * 4, 6);
      data.setZero();
    }

    // Map variables back to data
    const int data_row = (tag_id * 4) + corner_idx;
    data(data_row, 0) = 1.0;
    data(data_row, 1) = kp_x;
    data(data_row, 2) = kp_y;
    data(data_row, 3) = p_x;
    data(data_row, 4) = p_y;
    data(data_row, 5) = p_z;
    detected = true;
    nb_detections++;
  }

  // Clean up
  fclose(fp);

  return 0;
}

int aprilgrid_t::equal(const aprilgrid_t &grid1) const {
  assert(init == true);
  bool timestamp_ok = (this->timestamp == grid1.timestamp);
  bool tag_rows_ok = (this->tag_rows == grid1.tag_rows);
  bool tag_cols_ok = (this->tag_cols == grid1.tag_cols);
  bool tag_size_ok = (this->tag_size == grid1.tag_size);
  bool tag_spacing_ok = (this->tag_spacing == grid1.tag_spacing);

  bool detected_ok = (this->detected == grid1.detected);
  bool nb_detections_ok = (this->nb_detections == grid1.nb_detections);

  std::vector<bool> checklist = {timestamp_ok,
                                 tag_rows_ok,
                                 tag_cols_ok,
                                 tag_size_ok,
                                 tag_spacing_ok,
                                 detected_ok,
                                 nb_detections_ok};

  std::vector<std::string> checklist_str = {"timestamp",
                                            "tag_rows",
                                            "tag_cols",
                                            "tag_size",
                                            "tag_spacing",
                                            "detected",
                                            "nb_detections"};

  // Check
  for (size_t i = 0; i < checklist.size(); i++) {
    const auto &item = checklist[i];
    if (item == false) {
      printf("[%s] not the same!\n", checklist_str[i].c_str());
      return 0;
    }
  }

  // Check data
  bool data_ok = ((this->data - grid1.data).norm() < 1e-6);
  if (data_ok == false) {
    return 0;
  }

  return 1;
}

void aprilgrid_t::intersect(aprilgrid_t &grid1) {
  assert(init == true);
  // Get rows in data that are detected
  std::vector<int> grid0_rows;
  std::vector<int> grid1_rows;
  for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
    if (data(i, 0))
      grid0_rows.push_back(i);
    if (grid1.data(i, 0))
      grid1_rows.push_back(i);
  }

  // For rows that are different set to zero (remove)
  for (auto i : set_symmetric_diff(grid0_rows, grid1_rows)) {
    if (data(i, 0) > 0) {
      data.block(i, 0, 1, 6).setZero();
      nb_detections--;
    }

    if (grid1.data(i, 0) > 0) {
      grid1.data.block(i, 0, 1, 6).setZero();
      grid1.nb_detections--;
    }
  }
}

void aprilgrid_t::intersect(std::vector<aprilgrid_t *> &grids) {
  const int nb_grids = grids.size();
  std::map<int, int> row_counter;

  // Initialize row_counter
  for (int i = 0; i < (grids[0]->tag_rows * grids[0]->tag_cols * 4); i++) {
    row_counter[i] = 0;
  }

  // For all AprilGrids, book keep rows that are non-zero
  for (aprilgrid_t *grid : grids) {
    for (int i = 0; i < (grid->tag_rows * grid->tag_cols * 4); i++) {
      if (grid->data(i, 0)) {
        row_counter[i] = row_counter[i] + 1;
      }
    }
  }

  // Find which rows are not common amonst all AprilGrids
  std::vector<int> rows_remove;
  for (auto &kv : row_counter) {
    const int row_index = kv.first;
    const int count = kv.second;

    if (count != nb_grids) {
      rows_remove.push_back(row_index);
    }
  }

  // Remove
  for (aprilgrid_t *grid : grids) {
    for (const int row_index : rows_remove) {
      if (grid->data(row_index, 0)) {
        grid->data.block(row_index, 0, 1, 6).setZero();
        grid->nb_detections--;
      }
    }
  }
}

void aprilgrid_t::sample(const size_t n,
                         std::vector<int> &sample_tag_ids,
                         std::vector<int> &sample_corner_indicies,
                         vec2s_t &sample_keypoints,
                         vec3s_t &sample_object_points) {
  assert(init == true);
  if (nb_detections == 0) {
    return;
  }

  std::set<std::string> sample_list;
  auto tag_ids_ = tag_ids();
  while (sample_tag_ids.size() < std::min((size_t)nb_detections, n)) {
    const auto tag_id = tag_ids_[randi(0, tag_ids_.size())];
    const auto corner_idx = randi(0, 4);

    // Skip this tag_id if we've already sampled it
    const auto tag_id_str = std::to_string(tag_id);
    const auto corner_idx_str = std::to_string(corner_idx);
    const auto sample_id = tag_id_str + "-" + corner_idx_str;
    if (std::count(sample_list.begin(), sample_list.end(), sample_id)) {
      continue;
    }
    sample_list.insert(sample_id);

    // Add to samples if detected
    const int i = (tag_id * 4) + corner_idx;

    if (data(i, 0)) {
      sample_tag_ids.push_back(tag_id);
      sample_corner_indicies.push_back(corner_idx);
      sample_keypoints.emplace_back(data(i, 1), data(i, 2));
      sample_object_points.emplace_back(data(i, 3), data(i, 4), data(i, 5));
    }
  }
}

void aprilgrid_t::common_measurements(const aprilgrid_t &grid_i,
                                      const aprilgrid_t &grid_j,
                                      std::vector<int> &tag_ids,
                                      std::vector<int> &corner_indicies,
                                      vec2s_t &grid_i_keypoints,
                                      vec2s_t &grid_j_keypoints,
                                      vec3s_t &object_points) {
  if (grid_i.detected == false || grid_j.detected == false) {
    return;
  }

  assert(grid_i.data.rows() == grid_j.data.rows());
  assert(grid_i.data.cols() == grid_j.data.cols());
  for (long i = 0; i < grid_i.data.rows(); i++) {
    if (grid_i.data(i, 0) && grid_j.data(i, 0)) {
      tag_ids.push_back(int(i / 4));
      corner_indicies.push_back(i % 4);
      grid_i_keypoints.emplace_back(grid_i.data(i, 1), grid_i.data(i, 2));
      grid_j_keypoints.emplace_back(grid_j.data(i, 1), grid_j.data(i, 2));
      object_points.emplace_back(grid_i.data(i, 3),
                                 grid_i.data(i, 4),
                                 grid_i.data(i, 5));
    }
  }
}

} // namespace yac
