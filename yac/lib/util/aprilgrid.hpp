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
#include "cv.hpp"
#include "data.hpp"

namespace yac {

/* AprilGrid */
struct aprilgrid_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Grid properties
  bool init = false;
  timestamp_t timestamp = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  // Grid data
  bool detected = false;
  int nb_detections = 0; // Number of corners detected
  matx_t data;

  aprilgrid_t();
  aprilgrid_t(const std::string &csv_path);
  aprilgrid_t(const timestamp_t &timestamp,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing);

  ~aprilgrid_t();

  void clear();

  bool fully_observable() const;
  bool has(const int tag_id, const int corner_idx) const;
  vec2_t center() const;
  void grid_index(const int tag_id, int &i, int &j) const;
  vec3_t object_point(const int tag_id, const int corner_idx) const;
  vec3s_t object_points() const;
  vec2_t keypoint(const int tag_id, const int corner_idx) const;
  vec2s_t keypoints() const;

  void get_measurements(std::vector<int> &tag_ids,
                        std::vector<int> &corner_indicies,
                        vec2s_t &keypoints,
                        vec3s_t &object_points) const;

  std::vector<int> tag_ids() const;

  void add(const int tag_id, const int corner_idx, const vec2_t &kp);
  void remove(const int tag_id, const int corner_idx);
  void remove(const int tag_id);

  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  void imshow(const std::string &title, const cv::Mat &image) const;

  int estimate(const camera_geometry_t *cam,
               const int cam_res[2],
               const vecx_t &cam_params,
               mat4_t &T_CF) const;

  int save(const std::string &save_path) const;
  int load(const std::string &data_path);
  int equal(const aprilgrid_t &grid1) const;

  void intersect(aprilgrid_t &grid1);
  static void intersect(std::vector<aprilgrid_t *> &grids);

  void sample(const size_t n,
              std::vector<int> &sample_tag_ids,
              std::vector<int> &sample_corner_indicies,
              vec2s_t &sample_keypoints,
              vec3s_t &sample_object_points);

  static void common_measurements(const aprilgrid_t &grid_i,
                                  const aprilgrid_t &grid_j,
                                  std::vector<int> &tag_ids,
                                  std::vector<int> &corner_indicies,
                                  vec2s_t &grid_i_keypoints,
                                  vec2s_t &grid_j_keypoints,
                                  vec3s_t &object_points);

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

/* Load AprilGrids */
inline aprilgrids_t load_aprilgrids(const std::string &dir_path) {
  std::vector<std::string> csv_files;
  if (list_dir(dir_path, csv_files) != 0) {
    FATAL("Failed to list dir [%s]!", dir_path.c_str());
  }
  sort(csv_files.begin(), csv_files.end());

  aprilgrids_t grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path + "/" + grid_csv;
    aprilgrid_t grid{csv_path};
    if (grid.detected) {
      grids.push_back(grid);
    }
  }

  return grids;
}

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

  // Blur threshold
  double blur_threshold = 0.0;
  // double blur_threshold = 1200.0;

  aprilgrid_detector_t(const int tag_rows_,
                       const int tag_cols_,
                       const double tag_size_,
                       const double tag_spacing_)
      : tag_rows{tag_rows_}, tag_cols{tag_cols_}, tag_size{tag_size_},
        tag_spacing{tag_spacing_} {
    apriltag_detector_add_family(det_v3, tf);
    det_v3->quad_decimate = 1.0;
    det_v3->quad_sigma = 0.0; // Blur
    det_v3->nthreads = 2;
    det_v3->debug = 0;
    det_v3->refine_edges = 1;
  }

  ~aprilgrid_detector_t() {
    apriltag_detector_destroy(det_v3);
  }

  void filter_tags(const cv::Mat &image,
                   std::vector<AprilTags::TagDetection> &tags,
                   const bool verbose = false) {
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
        remove |= iter->p[j].first > (float)(image.cols) - min_border_dist;
        remove |= iter->p[j].second < min_border_dist;
        remove |= iter->p[j].second > (float)(image.rows) - min_border_dist;
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
        corners.emplace_back(iter->p[0].first,
                             iter->p[0].second); // Bottom left
        corners.emplace_back(iter->p[1].first,
                             iter->p[1].second); // Bottom right
        corners.emplace_back(iter->p[2].first, iter->p[2].second); // Top right
        corners.emplace_back(iter->p[3].first, iter->p[3].second); // Top left

        corners_before.push_back(corners[0]);
        corners_before.push_back(corners[1]);
        corners_before.push_back(corners[2]);
        corners_before.push_back(corners[3]);

        const cv::Size win_size(2, 2);
        const cv::Size zero_zone(-1, -1);
        const cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                                        30,
                                        0.1);
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

        iter->p[0].first = corners[0].x;
        iter->p[0].second = corners[0].y;
        iter->p[1].first = corners[1].x;
        iter->p[1].second = corners[1].y;
        iter->p[2].first = corners[2].x;
        iter->p[2].second = corners[2].y;
        iter->p[3].first = corners[3].x;
        iter->p[3].second = corners[3].y;

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

  static double blur_score(const cv::Mat &image, const vec2_t &kp) {
    // Laplacian blurr score
    const double x = kp(0);
    const double y = kp(1);
    const double patch_width = 5.0;
    const double patch_height = 5.0;
    const double roi_x = round(x) - (patch_width / 2.0);
    const double roi_y = round(y) - (patch_height / 2.0);
    const double roi_width = patch_width;
    const double roi_height = patch_height;
    // -- Patch around the corner
    const cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
    const cv::Mat patch = image(roi);
    // -- Form laplacian image patch
    cv::Mat patch_laplacian;
    cv::Laplacian(patch, patch_laplacian, CV_64F);
    // -- Calculate variance
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(patch_laplacian, mean, stddev, cv::Mat());
    const double var = stddev.val[0] * stddev.val[0];

    return var;
  }

  static std::vector<double> blur_scores(const cv::Mat &image,
                                         const aprilgrid_t &grid) {
    std::vector<double> patch_vars;
    for (auto &kp : grid.keypoints()) {
      patch_vars.push_back(blur_score(image, kp));
    }
    return patch_vars;
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
    const cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                                    30,
                                    0.1);
    cv::cornerSubPix(image, corners_after, win_size, zero_zone, criteria);

    for (size_t i = 0; i < nb_measurements; i++) {
      // Subpixel distance
      const auto &p_before = corners_before[i];
      const auto &p_after = corners_after[i];
      const auto dx = p_before.x - p_after.x;
      const auto dy = p_before.y - p_after.y;
      const auto dist = sqrt(dx * dx + dy * dy);

      // Laplacian blurr score
      const vec2_t kp{p_after.x, p_after.y};
      // const double var = blur_score(image, kp);

      if (dist < max_subpix_disp /*  && var > blur_threshold */) {
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
                     const bool use_v3 = false) {
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
      std::sort(tags.begin(),
                tags.end(),
                [](const AprilTags::TagDetection &a,
                   const AprilTags::TagDetection &b) { return (a.id < b.id); });
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
