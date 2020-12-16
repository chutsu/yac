#ifndef YAC_APRILGRID_HPP
#define YAC_APRILGRID_HPP

#include <algorithm>

/// Order matters with the AprilTags lib by Michael Kaess. The detector first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

/// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag3/apriltag.h"
#include "apriltag3/tag36h11.h"
#include "apriltag3/tag25h9.h"
#include "apriltag3/tag16h5.h"
#include "apriltag3/tagCircle21h7.h"
#include "apriltag3/tagCircle49h12.h"
#include "apriltag3/tagCustom48h12.h"
#include "apriltag3/tagStandard41h12.h"
#include "apriltag3/tagStandard52h13.h"
#include "apriltag3/common/getopt.h"
}

#include "core.hpp"

namespace yac {

/* AprilGrid */
struct aprilgrid_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Grid properties
  timestamp_t timestamp = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  // Grid data
  bool detected = false;
  int nb_detections = 0; // Number of corners detected
  matx_t data;

  aprilgrid_t() {}
  aprilgrid_t(const std::string &csv_path) { load(csv_path); }
  aprilgrid_t(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing)
      : timestamp{timestamp},
        tag_rows{tag_rows}, tag_cols{tag_cols},
        tag_size{tag_size}, tag_spacing{tag_spacing},
        data{zeros(tag_rows * tag_cols * 4, 6)} {}
  ~aprilgrid_t() {}

  void clear() {
    detected = false;
    nb_detections = 0;
    data.setZero();
  }

  vec2_t center() const {
    double x = ((tag_cols / 2.0) * tag_size);
    x +=  (((tag_cols / 2.0) - 1) * tag_spacing * tag_size);
    x +=  (0.5 * tag_spacing * tag_size);

    double y = ((tag_rows / 2.0) * tag_size);
    y +=  (((tag_rows / 2.0) - 1) * tag_spacing * tag_size);
    y +=  (0.5 * tag_spacing * tag_size);

    return vec2_t{x, y};
  }

  vec2_t center() {
    return static_cast<const aprilgrid_t>(*this).center();
  }

  void grid_index(const int tag_id, int &i, int &j) const {
    if (tag_id > (tag_rows * tag_cols)) {
      FATAL("tag_id > (tag_rows * tag_cols)!");
    } else if (tag_id < 0) {
      FATAL("tag_id < 0!");
    }

    i = int(tag_id / tag_cols);
    j = int(tag_id % tag_cols);
  }

  void grid_index(const int tag_id, int &i, int &j) {
    static_cast<const aprilgrid_t>(*this).grid_index(tag_id, i, j);
  }

  vec3_t object_point(const int tag_id, const int corner_id) const {
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

    return object_point;
  }

  vec3_t object_point(const int tag_id, const int corner_idx) {
    return static_cast<const aprilgrid_t>(*this).object_point(tag_id, corner_idx);
  }

  vec2_t keypoint(const int tag_id, const int corner_idx) const {
    const int data_row = (tag_id * 4) + corner_idx;
    if (data(data_row, 0) > 0) {
      return data.block(data_row, 1, 1, 2).transpose();
    }

    FATAL("Keypoint [tag_id: %d, corner: %d] does not exist!",
          tag_id, corner_idx);
  }

  vec2_t keypoint(const int tag_id, const int corner_idx) {
    return static_cast<const aprilgrid_t>(*this).keypoint(tag_id, corner_idx);
  }

  vec2s_t keypoints() const {
    vec2s_t keypoints_;

    for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
      if (data(i, 0) > 0) {
        keypoints_.push_back(data.block(i, 1, 1, 2).transpose());
      }
    }

    return keypoints_;
  }

  vec2s_t keypoints() {
    return static_cast<const aprilgrid_t>(*this).keypoints();
  }

  void get_measurements(std::vector<int> &tag_ids,
                        std::vector<int> &corner_indicies,
                        vec2s_t &keypoints,
                        vec3s_t &object_points) const {
    for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
      if (data(i, 0) > 0) {
        tag_ids.push_back(int(i / 4));
        corner_indicies.push_back(i % 4);
        keypoints.emplace_back(data(i, 1), data(i, 2));
        object_points.emplace_back(data(i, 3), data(i, 4), data(i, 5));
      }
    }
  }

  std::vector<int> tag_ids() const {
    std::set<int> tag_ids_;
    for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
      if (data(i, 0)) {
        tag_ids_.insert(int(i / 4));
      }
    }

    return std::vector<int>{tag_ids_.begin(), tag_ids_.end()};
  }

  std::vector<int> tag_ids() {
    return static_cast<const aprilgrid_t>(*this).tag_ids();
  }

  void add(const int tag_id, const int corner_idx, const vec2_t &kp) {
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

  void remove(const int tag_id, const int corner_idx) {
    const int data_row = (tag_id * 4) + corner_idx;
    if (data(data_row, 0) > 0) {
      data.block(data_row, 0, 1, 6).setZero();
      nb_detections = (nb_detections > 0) ? (nb_detections - 1) : 0;
      detected = (nb_detections == 0) ? false : true;
    }
  }

  void remove(const int tag_id) {
    remove(tag_id, 0);
    remove(tag_id, 1);
    remove(tag_id, 2);
    remove(tag_id, 3);
  }

  void imshow(const std::string &title, const cv::Mat &image) const {
    cv::Mat image_rgb = gray2rgb(image);
    const cv::Scalar red{0, 0, 255};

    for (const vec2_t &kp : keypoints()) {
      cv::Point2f p(kp(0), kp(1));
      cv::circle(image_rgb, p, 2, red, -1);
    }

    cv::imshow(title, image_rgb);
    cv::waitKey(1);
  }

  void imshow(const std::string &title, const cv::Mat &image) {
    static_cast<const aprilgrid_t>(*this).imshow(title, image);
  }

  int estimate(const vec4_t &proj_params, const vec4_t &dist_params, mat4_t &T_CF) const {
    // Check if we actually have data to work with
    if (nb_detections == 0) {
      return -1;
    }

    // Create object points (counter-clockwise, from bottom left)
    std::vector<cv::Point3f> obj_pts;
    std::vector<cv::Point2f> img_pts;
    for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
      if (data(i, 0) > 0) {
        img_pts.emplace_back(data(i, 1), data(i, 2));
        obj_pts.emplace_back(data(i, 3), data(i, 4), data(i, 5));
      }
    }

    // Extract out camera intrinsics
    const double fx = proj_params(0);
    const double fy = proj_params(1);
    const double cx = proj_params(2);
    const double cy = proj_params(3);

    // Extract out camera distortion
    const double k1 = dist_params(0);
    const double k2 = dist_params(1);
    const double p1 = dist_params(2);
    const double p2 = dist_params(3);

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

  int estimate(const vec4_t &proj_params, const vec4_t &dist_params, mat4_t &T_CF) {
    return static_cast<const aprilgrid_t>(*this).estimate(proj_params, dist_params, T_CF);
  }

  int save(const std::string &save_path) const {
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
    vec2s_t kps = keypoints();
    for (long i = 0; i < data.rows(); i++) {
      const int tag_id = int(i / 4);
      const int corner_idx = (i % 4);

      int detected = data(i, 0);
      if (detected > 0) {
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

    // Close up
    fclose(fp);
    return 0;
  }

  int save(const std::string &save_path) {
    return static_cast<const aprilgrid_t>(*this).save(save_path);
  }

  int load(const std::string &data_path) {
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
        &corner_idx
      );
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

  int equal(const aprilgrid_t &grid1) const {
    bool timestamp_ok = (this->timestamp == grid1.timestamp);
    bool tag_rows_ok = (this->tag_rows == grid1.tag_rows);
    bool tag_cols_ok = (this->tag_cols == grid1.tag_cols);
    bool tag_size_ok = (this->tag_size == grid1.tag_size);
    bool tag_spacing_ok = (this->tag_spacing == grid1.tag_spacing);

    bool detected_ok = (this->detected == grid1.detected);
    bool nb_detections_ok = (this->nb_detections == grid1.nb_detections);

    std::vector<bool> checklist = {
      timestamp_ok,
      tag_rows_ok,
      tag_cols_ok,
      tag_size_ok,
      tag_spacing_ok,
      detected_ok,
      nb_detections_ok
    };

    std::vector<std::string> checklist_str = {
      "timestamp",
      "tag_rows",
      "tag_cols",
      "tag_size",
      "tag_spacing",
      "detected",
      "nb_detections"
    };

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

  int equal(const aprilgrid_t &grid1) {
    return static_cast<const aprilgrid_t>(*this).equal(grid1);
  }

  void intersect(aprilgrid_t &grid1) {
		// Get rows in data that are detected
		std::vector<int> grid0_rows;
		std::vector<int> grid1_rows;
		for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
			if (data(i, 0)) grid0_rows.push_back(i);
			if (grid1.data(i, 0)) grid1_rows.push_back(i);
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

  static void intersect(std::vector<aprilgrid_t *> &grids) {
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

  void sample(const size_t n,
              std::vector<int> &sample_tag_ids,
              std::vector<int> &sample_corner_indicies,
              vec2s_t &sample_keypoints,
              vec3s_t &sample_object_points) {
    if (nb_detections == 0) {
      return;
    }

    std::set<std::string> sample_list;
    auto tag_ids_ = tag_ids();
    while (sample_tag_ids.size() < std::min((size_t) nb_detections, n)) {
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

  friend std::ostream &operator<<(std::ostream &os, const aprilgrid_t &grid) {
    os << "ts: " << grid.timestamp << std::endl;
    os << "tag_rows: " << grid.tag_rows << std::endl;
    os << "tag_cols: " << grid.tag_cols << std::endl;
    os << "tag_size: " << grid.tag_size << std::endl;
    os << "tag_spacing: " << grid.tag_spacing << std::endl;
    os << std::endl;

    os << "data: " << std::endl;
    os << grid.data << std::endl;

    return os;
  }
};

/* AprilGrids */
typedef std::vector<aprilgrid_t> aprilgrids_t;

/* AprilGrid Detector */
struct aprilgrid_detector_t {
  /// Grid properties
  bool configured = true;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  // AprilTags by Michael Kaess
  AprilTags::AprilGridDetector det;

  // AprilTag3 by Ed Olsen
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *det_v3 = apriltag_detector_create();

  aprilgrid_detector_t(const int tag_rows_,
                       const int tag_cols_,
                       const double tag_size_,
                       const double tag_spacing_)
      : tag_rows{tag_rows_},
        tag_cols{tag_cols_},
        tag_size{tag_size_},
        tag_spacing{tag_spacing_} {
    apriltag_detector_add_family(det_v3, tf);
    det_v3->quad_decimate = 1.0;
    det_v3->quad_sigma = 0.0;  // Blur
    det_v3->nthreads = 2;
    det_v3->debug = 0;
    det_v3->refine_edges = 1;
  }

  ~aprilgrid_detector_t() {
    apriltag_detector_destroy(det_v3);
  }

  void filter_tags(const cv::Mat &image,
                   std::vector<AprilTags::TagDetection> &tags,
                   const bool verbose=false) {
    const double min_border_dist = 4.0;
    const double max_subpix_disp = sqrt(1.5);

    const size_t nb_tags_before = tags.size();
    int removed = 0;
    auto iter = tags.begin();

    while (iter != tags.end()) {
      bool remove = false;

      // Remove if too close to boundaries of image
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

      // Remove if corner subpix failed
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

  void filter_measurements(const cv::Mat &image,
                           std::vector<int> &tag_ids,
                           std::vector<int> &corner_indicies,
                           vec2s_t &keypoints) {
    const double max_subpix_disp = sqrt(1.5);
    const size_t nb_measurements = tag_ids.size();

    std::vector<int> filtered_tag_ids;
    std::vector<int> filtered_corner_indicies;
    vec2s_t filtered_keypoints;

    std::vector<cv::Point2f> corners_before;
    std::vector<cv::Point2f> corners_after;
    for (size_t i = 0; i < nb_measurements; i++) {
      const vec2_t kp = keypoints[i];
      corners_before.emplace_back(kp(0), kp(1));
      corners_after.emplace_back(kp(0), kp(1));
    }

    const cv::Size win_size(2, 2);
    const cv::Size zero_zone(-1, -1);
    const cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
    cv::cornerSubPix(image, corners_after, win_size, zero_zone, criteria);

    for (size_t i = 0; i < nb_measurements; i++) {
      const auto &p_before = corners_before[i];
      const auto &p_after = corners_after[i];
      const auto dx = p_before.x - p_after.x;
      const auto dy = p_before.y - p_after.y;
      const auto dist = sqrt(dx * dx + dy * dy);
      if (dist < max_subpix_disp) {
        filtered_tag_ids.push_back(tag_ids[i]);
        filtered_corner_indicies.push_back(corner_indicies[i]);
        filtered_keypoints.emplace_back(p_after.x, p_after.y);
      }
    }

    tag_ids = filtered_tag_ids;
    corner_indicies = filtered_corner_indicies;
    keypoints = filtered_keypoints;
  }

  aprilgrid_t detect(const timestamp_t ts,
                     const cv::Mat &image,
                     const bool use_v3=false) {
    aprilgrid_t grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

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
      zarray_t *detections = apriltag_detector_detect(det_v3, &im);
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // if (det->decision_margin < 180.0) {
        //   continue;
        // }

        // printf("apriltag hamming: %d\n", det->hamming);
        // printf("apriltag decision margin: %f\n", det->decision_margin);
        // printf("\n");

        const auto tag_id = det->id;
        const auto kps = det->p;
        grid.add(tag_id, 0, vec2_t(kps[0][0], kps[0][1])); // Bottom left
        grid.add(tag_id, 1, vec2_t(kps[1][0], kps[1][1])); // Bottom right
        grid.add(tag_id, 2, vec2_t(kps[2][0], kps[2][1])); // Top right
        grid.add(tag_id, 3, vec2_t(kps[3][0], kps[3][1])); // Top left
      }
      apriltag_detections_destroy(detections);

    } else {
      // Use AprilTags by Michael Kaess
      // -- Extract tags
      std::vector<AprilTags::TagDetection> tags = det.extractTags(image_gray);
      // -- Sort by tag_id (inorder)
      std::sort(tags.begin(), tags.end(),
                [](const AprilTags::TagDetection &a,
                   const AprilTags::TagDetection &b) {
        return (a.id < b.id);
      });
      // -- Setup data
			const double min_border_dist = 4.0;
			const float img_rows = image_gray.rows;
			const float img_cols = image_gray.cols;

      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;

      for (const auto &tag : tags) {
        if (tag.good) {
          for (int i = 0; i < 4; i++) {
						bool remove = false;
						remove |= tag.p[i].first < min_border_dist;
						remove |= tag.p[i].first > img_cols - min_border_dist;
						remove |= tag.p[i].second < min_border_dist;
						remove |= tag.p[i].second > img_rows - min_border_dist;
						if (remove) {
							continue;
						}

            tag_ids.push_back(tag.id);
            corner_indicies.push_back(i);
            keypoints.emplace_back(tag.p[i].first, tag.p[i].second);
          }
        }
      }
      // -- Check if too few measurements
      int nb_measurements = tag_ids.size();
      if (nb_measurements < (4 * 4)) {
        return grid;
      }
      // -- Filter tags
      filter_measurements(image_gray, tag_ids, corner_indicies, keypoints);
      // -- Add filtered tags to grid
      for (size_t i = 0; i < tag_ids.size(); i++) {
        grid.add(tag_ids[i], corner_indicies[i], keypoints[i]);
      }
		}

    return grid;
  }
};

} // namespace yac
#endif // YAC_APRILGRID_HPP
