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

  // Remove points
  auto &points_CF = grid.points_CF;
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.shrink_to_fit();

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

  grid.estimated = false;
  grid.points_CF.clear();
  grid.T_CF = I(4);
}

int aprilgrid_get(const aprilgrid_t &grid, const int id, vec2s_t &keypoints) {
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

int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  vec2s_t &keypoints,
                  vec3s_t &points_CF) {
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

  // Set points
  if (grid.estimated) {
    points_CF.emplace_back(grid.points_CF[(index * 4)]);
    points_CF.emplace_back(grid.points_CF[(index * 4) + 1]);
    points_CF.emplace_back(grid.points_CF[(index * 4) + 2]);
    points_CF.emplace_back(grid.points_CF[(index * 4) + 3]);
  } else {
    aprilgrid_object_points(grid, id, points_CF);
  }

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

int aprilgrid_calc_relative_pose(aprilgrid_t &grid,
                                 const mat3_t &cam_K,
                                 const vec4_t &cam_D) {
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
  const real_t fx = cam_K(0, 0);
  const real_t fy = cam_K(1, 1);
  const real_t cx = cam_K(0, 2);
  const real_t cy = cam_K(1, 2);

  // Extract out camera distortion
  const real_t k1 = cam_D(0);
  const real_t k2 = cam_D(1);
  const real_t p1 = cam_D(2);
  const real_t p2 = cam_D(3);

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
  grid.T_CF = tf(convert(R), convert(tvec));

  // Calculate corner points
  for (size_t idx = 0; idx < grid.ids.size(); idx++) {
    // Get tag grid index
    int i = 0;
    int j = 0;
    const auto id = grid.ids[idx];
    if (aprilgrid_grid_index(grid, id, i, j) == -1) {
      LOG_ERROR("Invalid id [%d]!", id);
      return -1;
    }

    // Calculate the x and y of the tag origin (bottom left corner of tag)
    // relative to grid origin (bottom left corner of entire grid)
    const real_t x = j * (grid.tag_size + grid.tag_size * grid.tag_spacing);
    const real_t y = i * (grid.tag_size + grid.tag_size * grid.tag_spacing);

    // Calculate the x and y of each corner
    const vec4_t bottom_left(x, y, 0, 1);
    const vec4_t bottom_right(x + grid.tag_size, y, 0, 1);
    const vec4_t top_right(x + grid.tag_size, y + grid.tag_size, 0, 1);
    const vec4_t top_left(x, y + grid.tag_size, 0, 1);

    // Transform object points to corner points expressed in camera frame
    grid.points_CF.emplace_back((grid.T_CF * bottom_left).head(3));
    grid.points_CF.emplace_back((grid.T_CF * bottom_right).head(3));
    grid.points_CF.emplace_back((grid.T_CF * top_right).head(3));
    grid.points_CF.emplace_back((grid.T_CF * top_left).head(3));
  }
  grid.estimated = true;

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
    if (aprilgrid_get(grid, id, kps) != 0) {
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
  assert((grid.points_CF.size() % 4) == 0);

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
  fprintf(fp, "configured,");
  fprintf(fp, "tag_rows,");
  fprintf(fp, "tag_cols,");
  fprintf(fp, "tag_size,");
  fprintf(fp, "tag_spacing,");
  // -- Keypoints
  fprintf(fp, "ts,id,kp_x,kp_y,");
  // -- Estimation
  fprintf(fp, "estimated,");
  fprintf(fp, "p_x,p_y,p_z,");
  fprintf(fp, "q_w,q_x,q_y,q_z,");
  fprintf(fp, "t_x,t_y,t_z\n");

  // Decompose relative pose into rotation (quaternion) and translation
  const mat3_t R_CF = grid.T_CF.block(0, 0, 3, 3);
  const quat_t q_CF{R_CF};
  const vec3_t t_CF{grid.T_CF.block(0, 3, 3, 1)};

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
      fprintf(fp, "%f,", keypoint(1));

      fprintf(fp, "%d,", grid.estimated);
      if (grid.estimated) {
        const vec3_t point_CF = grid.points_CF[(i * 4) + j];
        fprintf(fp, "%f,", point_CF(0));
        fprintf(fp, "%f,", point_CF(1));
        fprintf(fp, "%f,", point_CF(2));
        fprintf(fp, "%f,", q_CF.w());
        fprintf(fp, "%f,", q_CF.x());
        fprintf(fp, "%f,", q_CF.y());
        fprintf(fp, "%f,", q_CF.z());
        fprintf(fp, "%f,", t_CF(0));
        fprintf(fp, "%f,", t_CF(1));
        fprintf(fp, "%f\n", t_CF(2));
      } else {
        fprintf(fp, "0,0,0,0,0,0,0,0,0,0\n");
      }
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
  str_format += "%d,%lf,%lf,%lf,";
  // -- AprilGrid pose
  str_format += "%lf,%lf,%lf,%lf,%lf,%lf,%lf";

  // Parse file
  grid.ids.clear();
  grid.keypoints.clear();
  grid.points_CF.clear();

  // Parse data
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    int configured = 0;
    int estimated = 0;
    int tag_id = 0;
    real_t kp_x, kp_y = 0.0;
    real_t p_x, p_y, p_z = 0.0;
    real_t q_w, q_x, q_y, q_z = 0.0;
    real_t r_x, r_y, r_z = 0.0;
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
        &kp_y,
        // Corner point
        &estimated,
        &p_x,
        &p_y,
        &p_z,
        // AprilGrid pose
        &q_w,
        &q_x,
        &q_y,
        &q_z,
        &r_x,
        &r_y,
        &r_z);
    if (retval != 20) {
      LOG_INFO("Failed to parse line in [%s:%d]", data_path.c_str(), i);
      return -1;
    }

    // Map variables back to AprilGrid
    // -- Grid configured, estimated
    grid.configured = configured;
    grid.estimated = estimated;
    // -- AprilTag id and keypoint
    if (std::count(grid.ids.begin(), grid.ids.end(), tag_id) == 0) {
      grid.ids.emplace_back(tag_id);
    }
    grid.keypoints.emplace_back(kp_x, kp_y);
    // -- Point
    grid.points_CF.emplace_back(p_x, p_y, p_z);
    // -- AprilGrid pose
    const quat_t q_CF{q_w, q_x, q_y, q_z};
    const vec3_t t_CF{r_x, r_y, r_z};
    grid.T_CF = tf(q_CF.toRotationMatrix(), t_CF);
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
                           std::vector<apriltag_t> &tags) {
  std::vector<apriltag_t>::iterator iter = tags.begin();

  const real_t min_border_dist = 4.0;
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

    // Delete flagged tags
    if (remove) {
      iter = tags.erase(iter);
    } else {
      ++iter;
    }
  }
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
		aprilgrid_filter_tags(image, tags);
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
                     const mat3_t &cam_K,
                     const vec4_t &cam_D,
                     aprilgrid_t &grid,
					 	 	 		 	 const bool use_v3) {
  assert(detector.configured);
  if (aprilgrid_detect(detector, image, grid, use_v3) == 0) {
    return 0;
  }
  aprilgrid_calc_relative_pose(grid, cam_K, cam_D);

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
    aprilgrid_get(grid, tag_id, keypoints, object_points);
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

  os << "estimated: " << grid.estimated << std::endl;
  if (grid.estimated) {
    os << "T_CF: " << grid.T_CF << std::endl;
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
      if (grid.estimated) {
        os << "p_CF: " << grid.points_CF[(i * 4) + j].transpose();
        os << std::endl;
      }
    }
    os << std::endl;
  }

  return os;
}

} // namespace yac
