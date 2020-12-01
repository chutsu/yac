#include "aprilgrid.hpp"

namespace yac {

void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints) {
  assert(keypoints.size() == 4);

  // Set AprilGrid as detected
  grid.detected = true;
  grid.nb_detections++;

  // Push id and keypoints
  grid.ids.push_back(id);
  for (const auto &keypoint : keypoints) {
    grid.keypoints.emplace_back(keypoint.x, keypoint.y);
  }
}

void aprilgrid_add(aprilgrid_t &grid, const int id, const vec2s_t &keypoints) {
  assert(keypoints.size() == 4);

  // Set AprilGrid as detected
  grid.detected = true;
  grid.nb_detections++;

  // Push id and keypoints
  grid.ids.push_back(id);
  for (const auto &keypoint : keypoints) {
    grid.keypoints.push_back(keypoint);
  }
}

void aprilgrid_remove(aprilgrid_t &grid, const int id) {
  // Get index where id is stored
  size_t index = 0;
  bool found = false;
  for (index = 0; index < grid.ids.size(); index++) {
    if (grid.ids.at(index) == id) {
      found = true;
      break;
    }
  }
  if (found == false) {
    return;
  }

  // Remove id
  auto &ids = grid.ids;
  ids.erase(ids.begin() + index);
  ids.shrink_to_fit();

  // Remove keypoints
  auto &kps = grid.keypoints;
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.shrink_to_fit();

  // Update nb_detections
  grid.nb_detections--;
}

void aprilgrid_clear(aprilgrid_t &grid) {
  grid.configured = false;

  grid.detected = false;
  grid.nb_detections = 0;
  grid.timestamp = 0;
  grid.ids.clear();
  grid.keypoints.clear();
}

int aprilgrid_keypoints(const aprilgrid_t &grid, const int id, vec2s_t &keypoints) {
  // Check if tag id was actually detected
  int index = -1;
  for (size_t i = 0; i < grid.ids.size(); i++) {
    if (grid.ids[i] == id) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    LOG_ERROR("Failed to find tag id [%d] in AprilTagDetection!", id);
    return -1;
  }

  // Set keypoints
  keypoints.emplace_back(grid.keypoints[(index * 4)]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 1]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 2]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 3]);

  return 0;
}

void aprilgrid_set_properties(aprilgrid_t &grid,
                              const int tag_rows,
                              const int tag_cols,
                              const real_t tag_size,
                              const real_t tag_spacing) {
  grid.configured = true;
  grid.tag_rows = tag_rows;
  grid.tag_cols = tag_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;
}

int aprilgrid_grid_index(const aprilgrid_t &grid,
                         const int id,
                         int &i,
                         int &j) {
  if (id > (grid.tag_rows * grid.tag_cols) || id < 0) {
    return -1;
  }

  i = int(id / grid.tag_cols);
  j = int(id % grid.tag_cols);
  return 0;
}

int aprilgrid_object_point(const aprilgrid_t &grid,
                           const int tag_id,
                           const int corner_id,
                           vec3_t &object_point) {
  const real_t tag_size = grid.tag_size;
  const real_t tag_spacing = grid.tag_spacing;

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  if (aprilgrid_grid_index(grid, tag_id, i, j) != 0) {
    LOG_ERROR("Incorrect tag id [%d]!", tag_id);
    return -1;
  }

  // Caculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const real_t x = j * (tag_size + tag_size * tag_spacing);
  const real_t y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner
  switch (corner_id) {
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
  default: FATAL("Incorrect corner id [%d]!", corner_id); break;
  }

  return 0;
}

int aprilgrid_object_points(const aprilgrid_t &grid,
                            const int tag_id,
                            vec3s_t &object_points) {
  const real_t tag_size = grid.tag_size;
  const real_t tag_spacing = grid.tag_spacing;

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  if (aprilgrid_grid_index(grid, tag_id, i, j) != 0) {
    LOG_ERROR("Incorrect tag id [%d]!", tag_id);
    return -1;
  }

  // Calculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const real_t x = j * (tag_size + tag_size * tag_spacing);
  const real_t y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner (from the bottom left corner in a
  // anti-clockwise fashion)
  object_points.emplace_back(x, y, 0);
  object_points.emplace_back(x + tag_size, y, 0);
  object_points.emplace_back(x + tag_size, y + tag_size, 0);
  object_points.emplace_back(x, y + tag_size, 0);

  return 0;
}

int aprilgrid_object_points(const aprilgrid_t &grid, vec3s_t &object_points) {
  for (int i = 0; i < (grid.tag_rows * grid.tag_cols); i++) {
    if (aprilgrid_object_points(grid, i, object_points) != 0) {
      return -1;
    }
  }

  return 0;
}

vec2_t aprilgrid_center(const int rows,
                        const int cols,
                        const double tag_size,
                        const double tag_spacing) {
  double x = ((cols / 2.0) * tag_size);
  x +=  (((cols / 2.0) - 1) * tag_spacing * tag_size);
  x +=  (0.5 * tag_spacing * tag_size);

  double y = ((rows / 2.0) * tag_size);
  y +=  (((rows / 2.0) - 1) * tag_spacing * tag_size);
  y +=  (0.5 * tag_spacing * tag_size);

  return vec2_t{x, y};
}

int aprilgrid_calc_relative_pose(const aprilgrid_t &grid,
                                 const vec4_t &proj_params,
                                 const vec4_t &dist_params,
                                 mat4_t &T_CF) {
  // Check if we actually have data to work with
  if (grid.ids.size() == 0) {
    return 0;
  }

  // Create object points (counter-clockwise, from bottom left)
  std::vector<cv::Point3f> obj_pts;
  for (const auto id : grid.ids) {
    int i = 0;
    int j = 0;
    if (aprilgrid_grid_index(grid, id, i, j) == -1) {
      LOG_ERROR("Invalid id [%d]!", id);
      return -1;
    }

    // Caculate the x and y of the tag origin (bottom left corner of tag)
    // relative to grid origin (bottom left corner of entire grid)
    const real_t x = j * (grid.tag_size + grid.tag_size * grid.tag_spacing);
    const real_t y = i * (grid.tag_size + grid.tag_size * grid.tag_spacing);

    // Calculate the x and y of each corner
    const cv::Point3f bottom_left(x, y, 0);
    const cv::Point3f bottom_right(x + grid.tag_size, y, 0);
    const cv::Point3f top_right(x + grid.tag_size, y + grid.tag_size, 0);
    const cv::Point3f top_left(x, y + grid.tag_size, 0);

    // Add object points
    obj_pts.emplace_back(bottom_left);
    obj_pts.emplace_back(bottom_right);
    obj_pts.emplace_back(top_right);
    obj_pts.emplace_back(top_left);
  }

  // Create image points
  std::vector<cv::Point2f> img_pts;
  for (const auto &kp : grid.keypoints) {
    img_pts.emplace_back(kp(0), kp(1));
  }

  // Extract out camera intrinsics
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);

  // Extract out camera distortion
  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  // Solve pnp
  cv::Vec4f distortion_params(k1, k2, p1, p2); // SolvPnP assumes radtan
  cv::Mat camera_matrix(3, 3, CV_32FC1, 0.0f);
  camera_matrix.at<float>(0, 0) = fx;
  camera_matrix.at<float>(1, 1) = fy;
  camera_matrix.at<float>(0, 2) = cx;
  camera_matrix.at<float>(1, 2) = cy;
  camera_matrix.at<float>(2, 2) = 1.0;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(obj_pts,
               img_pts,
               camera_matrix,
               distortion_params,
               rvec,
               tvec,
               false,
               CV_ITERATIVE);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF = tf(convert(R), convert(tvec));

  return 0;
}

void aprilgrid_imshow(const aprilgrid_t &grid,
                      const std::string &title,
                      const cv::Mat &image) {
  // Draw corners
  cv::Mat image_rgb = gray2rgb(image);

  for (size_t i = 0; i < grid.ids.size(); i++) {
    const size_t id = grid.ids[i];
    vec2s_t kps;
    if (aprilgrid_keypoints(grid, id, kps) != 0) {
      return;
    }

    // Form corner points
    cv::Point2f p1(kps[0](0), kps[0](1)); // Bottom left
    cv::Point2f p2(kps[1](0), kps[1](1)); // Top left
    cv::Point2f p3(kps[2](0), kps[2](1)); // Top right
    cv::Point2f p4(kps[3](0), kps[3](1)); // Bottom right

    // Draw corners
    cv::circle(image_rgb, p1, 2, cv::Scalar(0, 0, 255), -1); // Bottom left
    cv::circle(image_rgb, p2, 2, cv::Scalar(0, 0, 255), -1); // Top left
    cv::circle(image_rgb, p3, 2, cv::Scalar(0, 0, 255), -1); // Top right
    cv::circle(image_rgb, p4, 2, cv::Scalar(0, 0, 255), -1); // Bottom right

    // // Draw Tag ID
    // cv::Point2f cxy(tags[i].cxy.first - 10, tags[i].cxy.second + 5);
    // cv::Scalar text_color(0, 0, 255);
    // std::string text = std::to_string(tags[i].id);
    // int font = cv::FONT_HERSHEY_PLAIN;
    // real_t font_scale = 1.0;
    // int thickness = 2;
    // cv::putText(image_rgb, text, cxy, font, font_scale, text_color,
    // thickness);
  }

  // Show
  cv::imshow(title, image_rgb);
  cv::waitKey(1);
}

int aprilgrid_save(const aprilgrid_t &grid, const std::string &save_path) {
  assert((grid.keypoints.size() % 4) == 0);
  // assert((grid.points_CF.size() % 4) == 0);

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
  fprintf(fp, "# configured,");
  fprintf(fp, "tag_rows,");
  fprintf(fp, "tag_cols,");
  fprintf(fp, "tag_size,");
  fprintf(fp, "tag_spacing,");
  // -- Keypoints
  fprintf(fp, "ts,id,kp_x,kp_y");
  fprintf(fp, "\n");

  // Output data
  for (size_t i = 0; i < grid.ids.size(); i++) {
    const int tag_id = grid.ids[i];

    for (int j = 0; j < 4; j++) {
      fprintf(fp, "%d,", grid.configured);
      fprintf(fp, "%d,", grid.tag_rows);
      fprintf(fp, "%d,", grid.tag_cols);
      fprintf(fp, "%f,", grid.tag_size);
      fprintf(fp, "%f,", grid.tag_spacing);

      const vec2_t keypoint = grid.keypoints[(i * 4) + j];
      fprintf(fp, "%ld,", grid.timestamp);
      fprintf(fp, "%d,", tag_id);
      fprintf(fp, "%f,", keypoint(0));
      fprintf(fp, "%f", keypoint(1));
      fprintf(fp, "\n");
    }
  }

  // Close up
  fclose(fp);
  return 0;
}

int aprilgrid_load(aprilgrid_t &grid, const std::string &data_path) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", data_path.c_str());
    return -1;
  }

  // Create format string
  std::string str_format;
  // -- AprilGrid properties
  str_format += "%d,%d,%d,%lf,%lf,";
  // -- Timestamp, tag id and keypoint
  str_format += "%ld,%d,%lf,%lf,";
  // -- Corner point
  str_format += "%d,%lf,%lf,%lf";

  // Parse file
  grid.ids.clear();
  grid.keypoints.clear();

  // Parse data
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    int configured = 0;
    int tag_id = 0;
    real_t kp_x, kp_y = 0.0;
    int retval = fscanf(
      // File pointer
      fp,
      // String format
      str_format.c_str(),
      // Configuration
      &configured,
      &grid.tag_rows,
      &grid.tag_cols,
      &grid.tag_size,
      &grid.tag_spacing,
      // Timestamp, tag id and keypoint
      &grid.timestamp,
      &tag_id,
      &kp_x,
      &kp_y
    );
    if (retval != 9) {
      LOG_INFO("Failed to parse line in [%s:%d]", data_path.c_str(), i);
      return -1;
    }

    // Map variables back to AprilGrid
    // -- Grid configured, estimated
    grid.configured = configured;
    // -- AprilTag id and keypoint
    if (std::count(grid.ids.begin(), grid.ids.end(), tag_id) == 0) {
      grid.ids.emplace_back(tag_id);
    }
    grid.keypoints.emplace_back(kp_x, kp_y);
  }

  // Grid detected
  if (grid.ids.size()) {
    grid.detected = true;
    grid.nb_detections = grid.ids.size();
  }

  // Clean up
  fclose(fp);

  return 0;
}

aprilgrids_t load_aprilgrids(const std::string &dir_path) {
  std::vector<std::string> csv_files;
  if (list_dir(dir_path, csv_files) != 0) {
    FATAL("Failed to list dir [%s]!", dir_path.c_str());
  }
  sort(csv_files.begin(), csv_files.end());

  aprilgrids_t grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path + "/" + grid_csv;
    aprilgrid_t grid;
    if (aprilgrid_load(grid, csv_path) != 0) {
      FATAL("Failed to load AprilGrid [%s]!", grid_csv.c_str());
    }

    if (grid.detected) {
      grids.push_back(grid);
    }
  }

  return grids;
}


int aprilgrid_configure(aprilgrid_t &grid, const std::string &target_file) {
  // Load target file
  config_t config{target_file};
  if (config.ok == false) {
    return -1;
  }
  std::string target_type;
  parse(config, "target_type", target_type);
  parse(config, "tag_rows", grid.tag_rows);
  parse(config, "tag_cols", grid.tag_cols);
  parse(config, "tag_size", grid.tag_size);
  parse(config, "tag_spacing", grid.tag_spacing);

  // real_t check tag type
  if (target_type != "aprilgrid") {
    FATAL("Invalid target_type [%s], expecting 'aprilgrid'!",
          target_type.c_str());
  }

  grid.configured = true;
  return 0;
}

void aprilgrid_filter_tags(const cv::Mat &image,
                           std::vector<apriltag_t> &tags,
                           const bool verbose) {
  const real_t min_border_dist = 4.0;
  const real_t max_subpix_disp = 1.0;

  const size_t nb_tags_before = tags.size();
  int removed = 0;
  std::vector<apriltag_t>::iterator iter = tags.begin();
  for (iter = tags.begin(); iter != tags.end();) {
    bool remove = false;

    for (int j = 0; j < 4; j++) {
      remove |= iter->p[j].first < min_border_dist;
      remove |= iter->p[j].first > (float) (image.cols) - min_border_dist;
      remove |= iter->p[j].second < min_border_dist;
      remove |= iter->p[j].second > (float) (image.rows) - min_border_dist;
    }

    // Remove tags that are flagged as bad
    if (iter->good != 1) {
      remove |= true;
    }

    if (remove == false) {
      std::vector<cv::Point2f> corners_before;
      std::vector<cv::Point2f> corners_after;

      std::vector<cv::Point2f> corners;
      corners.emplace_back(iter->p[0].first, iter->p[0].second); // Bottom left
      corners.emplace_back(iter->p[1].first, iter->p[1].second); // Bottom right
      corners.emplace_back(iter->p[2].first, iter->p[2].second); // Top right
      corners.emplace_back(iter->p[3].first, iter->p[3].second); // Top left

      corners_before.push_back(corners[0]);
      corners_before.push_back(corners[1]);
      corners_before.push_back(corners[2]);
      corners_before.push_back(corners[3]);

      const cv::Size win_size(2, 2);
      const cv::Size zero_zone(-1, -1);
      const cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
      cv::cornerSubPix(image, corners, win_size, zero_zone, criteria);

      corners_after.push_back(corners[0]);
      corners_after.push_back(corners[1]);
      corners_after.push_back(corners[2]);
      corners_after.push_back(corners[3]);

      for (int i = 0; i < 4; i++) {
        const auto &p_before = corners_before[i];
        const auto &p_after = corners_after[i];

        const auto dx = p_before.x - p_after.x;
        const auto dy = p_before.y - p_after.y;
        const auto dist = sqrt(dx * dx + dy * dy);
        // printf("dist: %f\n", dist);
        if (dist >= max_subpix_disp) {
          remove = true;
        }
      }

      iter->p[0].first = corners[0].x; iter->p[0].second = corners[0].y;
      iter->p[1].first = corners[1].x; iter->p[1].second = corners[1].y;
      iter->p[2].first = corners[2].x; iter->p[2].second = corners[2].y;
      iter->p[3].first = corners[3].x; iter->p[3].second = corners[3].y;

      // cv::Mat image_rgb = gray2rgb(image);
      // for (const auto &corner : corners_before) {
      //   cv::circle(image_rgb, corner, 2, cv::Scalar(0, 0, 255), -1);
      // }
      // for (const auto &corner : corners_after) {
      //   cv::circle(image_rgb, corner, 2, cv::Scalar(0, 255, 0), -1);
      // }
    }

    // Delete flagged tags
    if (remove) {
      iter = tags.erase(iter);
      removed++;
    } else {
      ++iter;
    }
  }

  // Clear tags if less than 4 tags are observed
  if (tags.size() < 4) {
    tags.clear();
  }

  if (verbose) {
    printf("tags [before]: %ld\t", nb_tags_before);
    printf("tags [after]: %ld\t", tags.size());
    printf("removed: %d\n", removed);
  }
}

int aprilgrid_equal(const aprilgrid_t &grid0, const aprilgrid_t &grid1) {
  bool configured_ok = (grid0.configured == grid1.configured);
  bool tag_rows_ok = (grid0.tag_rows == grid1.tag_rows);
  bool tag_cols_ok = (grid0.tag_cols == grid1.tag_cols);
  bool tag_size_ok = (grid0.tag_size == grid1.tag_size);
  bool tag_spacing_ok = (grid0.tag_spacing == grid1.tag_spacing);

  bool detected_ok = (grid0.detected == grid1.detected);
  bool nb_detections_ok = (grid0.nb_detections == grid1.nb_detections);
  bool timestamp_ok = (grid0.timestamp == grid1.timestamp);
  bool ids_ok = (grid0.ids == grid1.ids);
  bool keypoints_ok = (grid0.keypoints.size() == grid1.keypoints.size());

  std::vector<bool> checklist = {
    // Grid properties
    configured_ok,
    tag_rows_ok,
    tag_cols_ok,
    tag_size_ok,
    tag_spacing_ok,
    // Detections
    detected_ok,
    nb_detections_ok,
    timestamp_ok,
    ids_ok,
    keypoints_ok
  };

  std::vector<std::string> checklist_str = {
    // Grid properties
    "configured",
    "tag_rows",
    "tag_cols",
    "tag_size",
    "tag_spacing",
    // Detections
    "detected",
    "nb_detections",
    "timestamp",
    "ids",
    "keypoints"
  };

  // Check
  for (size_t i = 0; i < checklist.size(); i++) {
    const auto &item = checklist[i];
    if (item == false) {
      printf("[%s] not the same!\n", checklist_str[i].c_str());
      return false;
    }
  }

  // Double check ids
  if (grid0.ids.size() != grid1.ids.size()) {
    printf("grid0.ids.size() != grid1.ids.size()\n");
    return 0;
  }
  for (size_t i = 0; i < grid0.ids.size(); i++) {
    if (grid0.ids[i] != grid1.ids[i]) {
      printf("grid0.ids[%ld] != grid1.ids[%ld]\n", i, i);
      return 0;
    }
  }

  // Double check keypoints
  if (grid0.keypoints.size() != grid1.keypoints.size()) {
    printf("grid0.keypoints.size() != grid1.keypoints.size()\n");
    return 0;
  }
  for (size_t i = 0; i < grid0.keypoints.size(); i++) {
    const vec2_t kp0 = grid0.keypoints[i];
    const vec2_t kp1 = grid1.keypoints[i];

    const bool x_ok = fabs(kp0(0) - kp1(0)) < 1e-4;
    const bool y_ok = fabs(kp0(1) - kp1(1)) < 1e-4;
    if (x_ok == false || y_ok == false) {
      printf("grid0.keypoints[%ld] != grid1.keypoints[%ld]\n", i, i);
      printf("kp0: (%f, %f)\n", kp0(0), kp0(1));
      printf("kp1: (%f, %f)\n", kp1(0), kp1(1));
      return 0;
    }
  }

  return 1;
}

int aprilgrid_detect(const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     aprilgrid_t &grid,
                     const bool use_v3) {
  assert(detector.configured);
  aprilgrid_clear(grid);

  grid.configured = true;
  grid.tag_rows = detector.tag_rows;
  grid.tag_cols = detector.tag_cols;
  grid.tag_size = detector.tag_size;
  grid.tag_spacing = detector.tag_spacing;

  // Convert image to gray-scale
  const cv::Mat image_gray = rgb2gray(image);

  if (use_v3) {
    // Use AprilTags3
    // -- Make an image_u8_t header for the Mat data
    image_u8_t im = {.width = image_gray.cols,
                    .height = image_gray.rows,
                    .stride = image_gray.cols,
                    .buf = image_gray.data};

    // -- Detector tags
    zarray_t *detections = apriltag_detector_detect(detector.det_v3, &im);
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      // if (det->decision_margin < 180.0) {
      //   continue;
      // }

      // printf("apriltag hamming: %d\n", det->hamming);
      // printf("apriltag decision margin: %f\n", det->decision_margin);
      // printf("\n");

      std::vector<cv::Point2f> img_pts;
      img_pts.emplace_back(det->p[0][0], det->p[0][1]); // Bottom left
      img_pts.emplace_back(det->p[1][0], det->p[1][1]); // Bottom right
      img_pts.emplace_back(det->p[2][0], det->p[2][1]); // Top right
      img_pts.emplace_back(det->p[3][0], det->p[3][1]); // Top left
      aprilgrid_add(grid, det->id, img_pts);
    }
    apriltag_detections_destroy(detections);

  } else {
    // Use AprilTags by Michael Kaess
    // Extract tags
    std::vector<apriltag_t> tags = detector.det.extractTags(image_gray);
    aprilgrid_filter_tags(image_gray, tags);
    std::sort(tags.begin(), tags.end(), sort_apriltag_by_id);

    // Form results
    for (const auto &tag : tags) {
      if (tag.good == false) {
        continue;
      }

      std::vector<cv::Point2f> img_pts;
      img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
      img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Top left
      img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
      img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Bottom right
      aprilgrid_add(grid, tag.id, img_pts);
    }
  }

  return grid.nb_detections;
}

int aprilgrid_detect(const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     const vec4_t &proj_params,
                     const vec4_t &dist_params,
                     aprilgrid_t &grid,
                     mat4_t &T_CF,
                     const bool use_v3) {
  assert(detector.configured);
  if (aprilgrid_detect(detector, image, grid, use_v3) == 0) {
    return 0;
  }
  aprilgrid_calc_relative_pose(grid, proj_params, dist_params, T_CF);

  return grid.nb_detections;
}

void aprilgrid_intersection(aprilgrid_t &grid0, aprilgrid_t &grid1) {
  // Find the symmetric difference of AprilTag ids
  std::vector<int> unique_ids = set_symmetric_diff(grid0.ids, grid1.ids);

  // Remove AprilTag based on unique ids, since we want common ids not unique
  for (const auto &id : unique_ids) {
    aprilgrid_remove(grid0, id);
    aprilgrid_remove(grid1, id);
  }
  assert(grid0.ids.size() == grid1.ids.size());
}

void aprilgrid_intersection(std::vector<aprilgrid_t *> &grids) {
  // Find the symmetric difference of AprilTag ids
  std::list<std::vector<int>> grid_ids;
  for (const aprilgrid_t *grid : grids) {
    grid_ids.push_back(grid->ids);
  }
  const std::set<int> common_ids = intersection(grid_ids);

  // Remove AprilTag based on unique ids since we want common ids
  for (size_t i = 0; i < grids.size(); i++) {
    // Make a copy of ids while deleting them in-place, else std::vector would
    // behave very weirdly
    const auto ids = grids[i]->ids;

    // Remove ids that are not common across all AprilGrids
    for (auto &id : ids) {
      if (common_ids.find(id) == common_ids.end()) {
        aprilgrid_remove(*grids[i], id);
      }
    }
  }
}

void aprilgrid_random_sample(const aprilgrid_t &grid, const size_t n,
                             std::vector<int> &sample_tag_ids,
                             std::vector<vec2s_t> &sample_keypoints,
                             std::vector<vec3s_t> &sample_object_points) {
  if (grid.ids.size() == 0) {
    return;
  }

  while (sample_tag_ids.size() < std::min(grid.ids.size(), n)) {
    const int index = randi(0, grid.ids.size());
    const auto tag_id = grid.ids[index];

    // Skip this tag_id if we've already sampled it
    if (std::count(sample_tag_ids.begin(), sample_tag_ids.end(), tag_id)) {
      continue;
    }

    vec2s_t keypoints;
    vec3s_t object_points;
    aprilgrid_keypoints(grid, tag_id, keypoints);
    aprilgrid_object_points(grid, tag_id, object_points);
    sample_tag_ids.push_back(tag_id);
    sample_keypoints.push_back(keypoints);
    sample_object_points.push_back(object_points);
  }
}

bool sort_apriltag_by_id(const AprilTags::TagDetection &a,
                         const AprilTags::TagDetection &b) {
  return (a.id < b.id);
}

std::ostream &operator<<(std::ostream &os, const aprilgrid_t &grid) {
  os << "AprilGrid: " << std::endl;
  os << "ts: " << grid.timestamp << std::endl;
  os << "configured: " << grid.configured << std::endl;
  if (grid.configured) {
    os << "tag_rows: " << grid.tag_rows << std::endl;
    os << "tag_cols: " << grid.tag_cols << std::endl;
    os << "tag_size: " << grid.tag_size << std::endl;
    os << "tag_spacing: " << grid.tag_spacing << std::endl;
    os << std::endl;
  }

  os << "tags: " << std::endl;
  for (size_t i = 0; i < grid.ids.size(); i++) {
    // Tag id
    os << "tag_id: " << grid.ids[i] << std::endl;
    os << "----------" << std::endl;

    // Keypoint and relative position
    for (size_t j = 0; j < 4; j++) {
      os << "keypoint: " << grid.keypoints[(i * 4) + j].transpose();
      os << std::endl;
    }
    os << std::endl;
  }

  return os;
}

} // namespace yac
