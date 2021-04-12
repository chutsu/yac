#include "cv.hpp"

namespace yac {

/*****************************************************************************
 *                                  CV
 ****************************************************************************/

bool is_equal(const cv::Mat &m1, const cv::Mat &m2) {
  // Pre-check
  if (m1.empty() && m2.empty()) {
    return true;
  }

  // Check dimensions
  if (m1.cols != m2.cols) {
    return false;
  } else if (m1.rows != m2.rows) {
    return false;
  } else if (m1.dims != m2.dims) {
    return false;
  }

  // Check matrix elements
  cv::Mat diff;
  cv::compare(m1, m2, diff, cv::CMP_NE);

  return cv::countNonZero(diff) ? false : true;
}

void convert(const cv::Mat &x, matx_t &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<real_t>(i, j);
    }
  }
}

void convert(const matx_t &x, cv::Mat &y) {
  y = cv::Mat(x.rows(), x.cols(), cv::DataType<real_t>::type);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      y.at<real_t>(i, j) = x(i, j);
    }
  }
}

matx_t convert(const cv::Mat &x) {
  matx_t y;
  convert(x, y);
  return y;
}

cv::Mat convert(const matx_t &x) {
  cv::Mat y;
  convert(x, y);
  return y;
}

std::vector<cv::KeyPoint>
sort_keypoints(const std::vector<cv::KeyPoint> keypoints, const size_t limit) {
  if (keypoints.size() == 0) {
    return std::vector<cv::KeyPoint>();
  }

  // Obtain vector responses
  std::vector<int> responses;
  for (size_t i = 0; i < keypoints.size(); i++) {
    responses.push_back(keypoints[i].response);
  }

  // Sort responses
  std::vector<int> index(responses.size());
  std::iota(std::begin(index), std::end(index), 0);
  cv::sortIdx(responses, index, CV_SORT_DESCENDING);

  // Form sorted keypoints
  std::vector<cv::KeyPoint> keypoints_sorted;
  for (size_t i = 0; i < keypoints.size(); i++) {
    keypoints_sorted.push_back(keypoints[index[i]]);
    if (keypoints_sorted.size() == limit) {
      break;
    }
  }

  return keypoints_sorted;
}

cv::Mat gray2rgb(const cv::Mat &image) {
  const int image_height = image.rows;
  const int image_width = image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);

  if (image.channels() == 1) {
    cv::cvtColor(image, out_image, CV_GRAY2RGB);
  } else {
    return image.clone();
  }

  return out_image;
}

cv::Mat rgb2gray(const cv::Mat &image) {
  cv::Mat image_gray;

  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  } else {
    return image.clone();
  }

  return image_gray;
}

cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const real_t cx,
            const real_t cy) {
  const real_t x = cx - width / 2.0;
  const real_t y = cy - height / 2.0;
  cv::Rect roi(x, y, width, height);
  return image(roi);
}

bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2) {
  // Keypoint with higher response will be at the beginning of the vector
  return kp1.response > kp2.response;
}

real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += pow((measured[i] - projected[i]).norm(), 2);
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

real_t reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += pow(cv::norm(measured[i] - projected[i]), 2);
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

real_t reprojection_error(const std::vector<vec2_t> &errors) {
  double sse = 0.0;
  for (const auto &e : errors) {
    sse += pow(e.norm(), 2);
  }
  const double mse = sse / errors.size();

  return sqrt(mse);
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width) {
  matx_t mask = ones(image_height, image_width);

  // Create a mask around each point
  for (const auto &p : points) {
    // Skip if pixel is out of image bounds
    const real_t px = static_cast<int>(p.x);
    const real_t py = static_cast<int>(p.y);
    if (px >= image_width || px <= 0) {
      continue;
    } else if (py >= image_height || py <= 0) {
      continue;
    }

    // Calculate patch top left corner, patch width and height
    vec2_t top_left{px - patch_width, py - patch_width};
    vec2_t top_right{px + patch_width, py - patch_width};
    vec2_t btm_left{px - patch_width, py + patch_width};
    vec2_t btm_right{px + patch_width, py + patch_width};
    std::vector<vec2_t *> corners{&top_left, &top_right, &btm_left, &btm_right};
    for (auto corner : corners) {
      // Check corner in x-axis
      if ((*corner)(0) < 0) {
        (*corner)(0) = 0;
      } else if ((*corner)(0) > image_width) {
        (*corner)(0) = image_width;
      }

      // Check corner in y-axis
      if ((*corner)(1) < 0) {
        (*corner)(1) = 0;
      } else if ((*corner)(1) > image_height) {
        (*corner)(1) = image_height;
      }
    }

    // Create mask around pixel
    const int row = top_left(1);
    const int col = top_left(0);
    int width = top_right(0) - top_left(0) + 1;
    int height = btm_left(1) - top_left(1) + 1;
    width = (col + width) > image_width ? width - 1 : width;
    height = (row + height) > image_height ? height - 1 : height;

    // std::cout << "---" << std::endl;
    // std::cout << image_width << std::endl;
    // std::cout << image_height << std::endl;
    // std::cout << row << std::endl;
    // std::cout << col << std::endl;
    // std::cout << width << std::endl;
    // std::cout << height << std::endl;
    // std::cout << "---" << std::endl;

    mask.block(row, col, height, width) = zeros(height, width);
  }

  return mask;
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width) {
  std::vector<cv::Point2f> points;
  for (const auto &kp : keypoints) {
    points.emplace_back(kp.pt);
  }

  return feature_mask(image_width, image_height, points, patch_width);
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, points, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, keypoints, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image) {
  cv::Mat image_ud;
  cv::Mat K_ud = convert(K).clone();
  cv::undistort(image, image_ud, convert(K), convert(D), K_ud);
  return image_ud;
}

cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew) {
  // Estimate new camera matrix first
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(convert(K),
                                                          convert(D),
                                                          image.size(),
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, convert(K), convert(D), Knew);

  return image_ud;
}

void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3) {
  // The following method is adapted from:
  // Illumination Invariant Imaging: Applications in Robust Vision-based
  // Localisation, Mapping and Classification for Autonomous Vehicles
  // Maddern et al (2014)

  // clang-format off
  real_t alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
                 (lambda_2 * lambda_3 - lambda_1 * lambda_2);
  // clang-format on

  std::vector<cv::Mat> channels(3);
  split(image, channels);
  channels[0].convertTo(channels[0], CV_32F);
  channels[1].convertTo(channels[1], CV_32F);
  channels[2].convertTo(channels[2], CV_32F);

  channels[0].row(0).setTo(cv::Scalar(1));
  channels[1].row(0).setTo(cv::Scalar(1));
  channels[2].row(0).setTo(cv::Scalar(1));

  cv::Mat log_ch_1, log_ch_2, log_ch_3;
  cv::log(channels[0] / 255.0, log_ch_1);
  cv::log(channels[1] / 255.0, log_ch_2);
  cv::log(channels[2] / 255.0, log_ch_3);

  image = 0.5 + log_ch_2 - alpha * log_ch_3 - (1 - alpha) * log_ch_1;
  image.setTo(0, image < 0);
  image.setTo(1, image > 1);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}

cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status) {
  // Draw tracks
  for (size_t i = 0; i < status.size(); i++) {
    // Check if point was lost
    if (status[i] == 0) {
      continue;
    }

    // Draw circle and line
    cv::circle(img_cur, p0[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img_cur, p1[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::line(img_cur, p0[i], p1[i], cv::Scalar(0, 255, 0));
  }

  return img_cur;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i]) {
      cv::Point2f p0 = k0[i];
      cv::Point2f p1 = k1[i];

      // Point 1
      p1.y += img0.rows;

      // Draw circle and line
      cv::circle(match_img, p0, 2, cv::Scalar(0, 255, 0), -1);
      cv::circle(match_img, p1, 2, cv::Scalar(0, 255, 0), -1);
      cv::line(match_img, p0, p1, cv::Scalar(0, 255, 0));
    }
  }

  return match_img;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < matches.size(); i++) {
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    cv::KeyPoint p0 = k0[k0_idx];
    cv::KeyPoint p1 = k1[k1_idx];

    // Point 1
    p1.pt.y += img0.rows;

    // Draw circle and line
    cv::circle(match_img, p0.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0.pt, p1.pt, cv::Scalar(0, 255, 0));
  }

  return match_img;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features) {
  cv::Mat out_image = image.clone();

  // Draw corners
  for (auto p : features) {
    cv::circle(out_image, p, 2, cv::Scalar(0, 255, 0), -1);
  }

  // Draw vertical lines
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;

  for (int x = dx; x < image_width; x += dx) {
    const cv::Point start(x, 0);
    const cv::Point end(x, image_height);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  // Draw horizontal lines
  for (int y = dy; y < image_height; y += dy) {
    const cv::Point start(0, y);
    const cv::Point end(image_width, y);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  return out_image;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features) {
  std::vector<cv::Point2f> points;
  for (const auto &f : features) {
    points.emplace_back(f.pt);
  }

  return draw_grid_features(image, grid_rows, grid_cols, points);
}

std::vector<cv::Point2f> grid_fast(const cv::Mat &image,
                                   const int max_keypoints,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const real_t threshold,
                                   const bool nonmax_suppression) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_per_cell = std::ceil((float) max_keypoints / (float) nb_cells);

  // Detect corners in each grid cell
  std::vector<cv::Point2f> keypoints_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      cv::Rect roi = cv::Rect(x, y, w, h);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image_gray(roi), keypoints, threshold, nonmax_suppression);

      // Sort by keypoint response
      keypoints = sort_keypoints(keypoints);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      size_t cell_counter = 0;
      for (auto &kp : keypoints) {
        kp.pt.x += x;
        kp.pt.y += y;

        keypoints_all.push_back(kp.pt);
        cell_counter++;
        if (cell_counter >= max_per_cell) {
          break;
        }
      }
    }
  }

  return keypoints_all;
}

std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_keypoints,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const real_t quality_level,
                                   const real_t min_distance,
                                   const cv::Mat mask,
                                   const int block_size,
                                   const bool use_harris_detector,
                                   const real_t k) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_per_cell = std::ceil((float) max_keypoints / (float) nb_cells);

  // Detect corners in each grid cell
  std::vector<cv::Point2f> keypoints_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect keypoints in grid cell
      const cv::Rect roi = cv::Rect(x, y, w, h);
      const cv::Mat sub_mask = (mask.rows == 0) ? cv::Mat() : mask(roi);
      std::vector<cv::Point2f> keypoints;
      cv::goodFeaturesToTrack(image_gray(roi),
                              keypoints,
                              max_keypoints,
                              quality_level,
                              min_distance,
                              sub_mask,
                              block_size,
                              use_harris_detector,
                              k);

      // Adjust keypoint's position according to the offset limit to max
      // keypoints per cell
      size_t cell_counter = 0;
      for (auto &kp : keypoints) {
        kp.x += x;
        kp.y += y;

        keypoints_all.push_back(kp);
        cell_counter++;
        if (cell_counter >= max_per_cell) {
          break;
        }
      }
    }
  }

  return keypoints_all;
}

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4) {
  os << "k1: " << radtan4.k1() << std::endl;
  os << "k2: " << radtan4.k2() << std::endl;
  os << "p1: " << radtan4.p1() << std::endl;
  os << "p2: " << radtan4.p2() << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4) {
  os << "k1: " << equi4.k1() << std::endl;
  os << "k2: " << equi4.k2() << std::endl;
  os << "k3: " << equi4.k3() << std::endl;
  os << "k4: " << equi4.k4() << std::endl;
  return os;
}

real_t pinhole_focal(const int image_size, const real_t fov) {
  return ((image_size / 2.0) / tan(deg2rad(fov) / 2.0));
}

mat3_t pinhole_K(const real_t fx,
                 const real_t fy,
                 const real_t cx,
                 const real_t cy) {
  mat3_t K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

mat3_t pinhole_K(const vec4_t &params) {
  return pinhole_K(params(0), params(1), params(2), params(3));
}

mat3_t pinhole_K(const int img_w,
                 const int img_h,
                 const real_t lens_hfov,
                 const real_t lens_vfov) {
  const real_t fx = pinhole_focal(img_w, lens_hfov);
  const real_t fy = pinhole_focal(img_h, lens_vfov);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;

  mat3_t K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

} // namespace yac
