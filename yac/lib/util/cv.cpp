#include "cv.hpp"

namespace yac {

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
  cv::sortIdx(responses, index, cv::SORT_DESCENDING);

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
    cv::cvtColor(image, out_image, cv::COLOR_GRAY2RGB);
  } else {
    return image.clone();
  }

  return out_image;
}

cv::Mat rgb2gray(const cv::Mat &image) {
  cv::Mat image_gray;

  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    return image.clone();
  }

  return image_gray;
}

// cv::Mat roi(const cv::Mat &image,
//             const int width,
//             const int height,
//             const real_t cx,
//             const real_t cy) {
//   const real_t x = cx - width / 2.0;
//   const real_t y = cy - height / 2.0;
//   cv::Rect roi(x, y, width, height);
//   return image(roi);
// }

// bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
//                                   const cv::KeyPoint &kp2) {
//   // Keypoint with higher response will be at the beginning of the vector
//   return kp1.response > kp2.response;
// }
//
// real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected)
// {
//   assert(measured.size() == projected.size());
//
//   real_t sse = 0.0;
//   const size_t nb_keypoints = measured.size();
//   for (size_t i = 0; i < nb_keypoints; i++) {
//     sse += pow((measured[i] - projected[i]).norm(), 2);
//   }
//   const real_t rmse = sqrt(sse / nb_keypoints);
//
//   return rmse;
// }
//
// real_t reprojection_error(const std::vector<cv::Point2f> &measured,
//                           const std::vector<cv::Point2f> &projected) {
//   assert(measured.size() == projected.size());
//
//   real_t sse = 0.0;
//   const size_t nb_keypoints = measured.size();
//   for (size_t i = 0; i < nb_keypoints; i++) {
//     sse += pow(cv::norm(measured[i] - projected[i]), 2);
//   }
//   const real_t rmse = sqrt(sse / nb_keypoints);
//
//   return rmse;
// }
//
// real_t reprojection_error(const std::vector<vec2_t> &errors) {
//   double sse = 0.0;
//   for (const auto &e : errors) {
//     sse += pow(e.norm(), 2);
//   }
//   const double mse = sse / errors.size();
//
//   return sqrt(mse);
// }
//
// matx_t feature_mask(const int image_width,
//                     const int image_height,
//                     const std::vector<cv::Point2f> points,
//                     const int patch_width) {
//   matx_t mask = ones(image_height, image_width);
//
//   // Create a mask around each point
//   for (const auto &p : points) {
//     // Skip if pixel is out of image bounds
//     const real_t px = static_cast<int>(p.x);
//     const real_t py = static_cast<int>(p.y);
//     if (px >= image_width || px <= 0) {
//       continue;
//     } else if (py >= image_height || py <= 0) {
//       continue;
//     }
//
//     // Calculate patch top left corner, patch width and height
//     vec2_t top_left{px - patch_width, py - patch_width};
//     vec2_t top_right{px + patch_width, py - patch_width};
//     vec2_t btm_left{px - patch_width, py + patch_width};
//     vec2_t btm_right{px + patch_width, py + patch_width};
//     std::vector<vec2_t *> corners{&top_left, &top_right, &btm_left,
//     &btm_right}; for (auto corner : corners) {
//       // Check corner in x-axis
//       if ((*corner)(0) < 0) {
//         (*corner)(0) = 0;
//       } else if ((*corner)(0) > image_width) {
//         (*corner)(0) = image_width;
//       }
//
//       // Check corner in y-axis
//       if ((*corner)(1) < 0) {
//         (*corner)(1) = 0;
//       } else if ((*corner)(1) > image_height) {
//         (*corner)(1) = image_height;
//       }
//     }
//
//     // Create mask around pixel
//     const int row = top_left(1);
//     const int col = top_left(0);
//     int width = top_right(0) - top_left(0) + 1;
//     int height = btm_left(1) - top_left(1) + 1;
//     width = (col + width) > image_width ? width - 1 : width;
//     height = (row + height) > image_height ? height - 1 : height;
//
//     // std::cout << "---" << std::endl;
//     // std::cout << image_width << std::endl;
//     // std::cout << image_height << std::endl;
//     // std::cout << row << std::endl;
//     // std::cout << col << std::endl;
//     // std::cout << width << std::endl;
//     // std::cout << height << std::endl;
//     // std::cout << "---" << std::endl;
//
//     mask.block(row, col, height, width) = zeros(height, width);
//   }
//
//   return mask;
// }
//
// matx_t feature_mask(const int image_width,
//                     const int image_height,
//                     const std::vector<cv::KeyPoint> keypoints,
//                     const int patch_width) {
//   std::vector<cv::Point2f> points;
//   for (const auto &kp : keypoints) {
//     points.emplace_back(kp.pt);
//   }
//
//   return feature_mask(image_width, image_height, points, patch_width);
// }
//
// cv::Mat feature_mask_opencv(const int image_width,
//                             const int image_height,
//                             const std::vector<cv::Point2f> points,
//                             const int patch_width) {
//   auto mask = feature_mask(image_width, image_height, points, patch_width);
//
//   cv::Mat mask_cv;
//   cv::eigen2cv(mask, mask_cv);
//   mask_cv.convertTo(mask_cv, CV_8UC1);
//
//   return mask_cv;
// }
//
// cv::Mat feature_mask_opencv(const int image_width,
//                             const int image_height,
//                             const std::vector<cv::KeyPoint> keypoints,
//                             const int patch_width) {
//   auto mask = feature_mask(image_width, image_height, keypoints,
//   patch_width);
//
//   cv::Mat mask_cv;
//   cv::eigen2cv(mask, mask_cv);
//   mask_cv.convertTo(mask_cv, CV_8UC1);
//
//   return mask_cv;
// }

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
  const size_t max_per_cell = std::ceil((float)max_keypoints / (float)nb_cells);

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
  const size_t max_per_cell = std::ceil((float)max_keypoints / (float)nb_cells);

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

// std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4) {
//   os << "k1: " << radtan4.k1() << std::endl;
//   os << "k2: " << radtan4.k2() << std::endl;
//   os << "p1: " << radtan4.p1() << std::endl;
//   os << "p2: " << radtan4.p2() << std::endl;
//   return os;
// }

// std::ostream &operator<<(std::ostream &os, const equi4_t &equi4) {
//   os << "k1: " << equi4.k1() << std::endl;
//   os << "k2: " << equi4.k2() << std::endl;
//   os << "k3: " << equi4.k3() << std::endl;
//   os << "k4: " << equi4.k4() << std::endl;
//   return os;
// }

/********************************* RADTAN4 ************************************/

vec2_t radtan4_distort(const vec4_t &dist_params, const vec2_t &p) {
  const real_t x = p.x();
  const real_t y = p.y();

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  return vec2_t{x_ddash, y_ddash};
}

vec2_t radtan4_undistort(const vec4_t &dist_params, const vec2_t &p0) {
  int max_iter = 5;
  vec2_t p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const vec2_t p_distorted = radtan4_distort(dist_params, p);
    const vec2_t err = (p0 - p_distorted);

    // Jacobian
    const mat2_t J = radtan4_point_jacobian(dist_params, p);
    const vec2_t dp = (J.transpose() * J).inverse() * J.transpose() * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

mat2_t radtan4_point_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  const real_t x = p(0);
  const real_t y = p(1);

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  mat2_t J_point;
  J_point(0, 0) = 1.0 + k1 * r2 + k2 * r4;
  J_point(0, 0) += 2.0 * p1 * y + 6.0 * p2 * x;
  J_point(0, 0) += x * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(1, 0) = 2.0 * p1 * x + 2.0 * p2 * y;
  J_point(1, 0) += y * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1.0 + k1 * r2 + k2 * r4;
  J_point(1, 1) += 6.0 * p1 * y + 2.0 * p2 * x;
  J_point(1, 1) += y * (2.0 * k1 * y + 4.0 * k2 * y * r2);
  // Above is generated using sympy

  // const auto radtan = k1 * r2 + k2 * r2 * r2;
  // J_point(0, 0) = 1 + radtan + k1 * 2.0 * x2 + k2 * r2 * 4 * x2 +
  //                 2.0 * p1 * p.y() + 6 * p2 * p.x();
  // J_point(1, 0) = k1 * 2.0 * p.x() * p.y() + k2 * 4 * r2 * p.x() * p.y() +
  //                 p1 * 2.0 * p.x() + 2.0 * p2 * p.y();
  // J_point(0, 1) = J_point(1, 0);
  // J_point(1, 1) = 1 + radtan + k1 * 2.0 * y2 + k2 * r2 * 4 * y2 +
  //                 6 * p1 * p.y() + 2.0 * p2 * p.x();

  return J_point;
}

matx_t radtan4_params_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  UNUSED(dist_params);

  const real_t x = p.x();
  const real_t y = p.y();

  const real_t xy = x * y;
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  mat_t<2, 4> J_dist = zeros(2, 4);
  J_dist(0, 0) = x * r2;
  J_dist(0, 1) = x * r4;
  J_dist(0, 2) = 2.0 * xy;
  J_dist(0, 3) = r2 + 2.0 * x2;

  J_dist(1, 0) = y * r2;
  J_dist(1, 1) = y * r4;
  J_dist(1, 2) = r2 + 2.0 * y2;
  J_dist(1, 3) = 2 * xy;

  return J_dist;
}

/********************************** EQUI4 *************************************/

vec2_t equi4_distort(const vec4_t &dist_params, const vec2_t &p) {
  const real_t r = p.norm();
  if (r < 1e-8) {
    return p;
  }

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t k3 = dist_params(2);
  const real_t k4 = dist_params(3);

  // Apply equi distortion
  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t x_dash = (thd / r) * p(0);
  const real_t y_dash = (thd / r) * p(1);

  return vec2_t{x_dash, y_dash};
}

vec2_t equi4_undistort(const vec4_t &dist_params, const vec2_t &p) {
  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t k3 = dist_params(2);
  const real_t k4 = dist_params(3);

  const real_t thd = sqrt(p(0) * p(0) + p(1) * p(1));
  real_t th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const real_t scaling = tan(th) / thd;
  return vec2_t{p(0) * scaling, p(1) * scaling};
}

mat2_t equi4_point_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t k3 = dist_params(2);
  const real_t k4 = dist_params(3);

  const real_t x = p(0);
  const real_t y = p(1);
  const real_t r = p.norm();
  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  // Form jacobian
  const real_t th_r = 1.0 / (r * r + 1.0);
  real_t thd_th = 1.0 + 3.0 * k1 * th2;
  thd_th += 5.0 * k2 * th4;
  thd_th += 7.0 * k3 * th6;
  thd_th += 9.0 * k4 * th8;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  mat2_t J_point = I(2);
  J_point(0, 0) = s + x * s_r * r_x;
  J_point(0, 1) = x * s_r * r_y;
  J_point(1, 0) = y * s_r * r_x;
  J_point(1, 1) = s + y * s_r * r_y;

  return J_point;
}

matx_t equi4_params_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  UNUSED(dist_params);

  const real_t x = p(0);
  const real_t y = p(1);
  const real_t r = p.norm();
  const real_t th = atan(r);

  const real_t th3 = th * th * th;
  const real_t th5 = th3 * th * th;
  const real_t th7 = th5 * th * th;
  const real_t th9 = th7 * th * th;

  matx_t J_dist = zeros(2, 4);
  J_dist(0, 0) = x * th3 / r;
  J_dist(0, 1) = x * th5 / r;
  J_dist(0, 2) = x * th7 / r;
  J_dist(0, 3) = x * th9 / r;

  J_dist(1, 0) = y * th3 / r;
  J_dist(1, 1) = y * th5 / r;
  J_dist(1, 2) = y * th7 / r;
  J_dist(1, 3) = y * th9 / r;

  return J_dist;
}

/********************************* PROJECT ************************************/

vec2_t project_point(const vec3_t &p_C) {
  return vec2_t{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
}

mat_t<2, 3> project_jacobian(const vec3_t &p_C) {
  const real_t x = p_C.x();
  const real_t y = p_C.y();
  const real_t z = p_C.z();

  mat_t<2, 3> J;

  J(0, 0) = 1.0 / z;
  J(0, 1) = 0.0;
  J(0, 2) = -x / (z * z);

  J(1, 0) = 0.0;
  J(1, 1) = 1.0 / z;
  J(1, 2) = -y / (z * z);

  return J;
}

/********************************* PINHOLE ************************************/

real_t pinhole_focal(const int image_size, const real_t fov) {
  return ((image_size / 2.0) / tan(deg2rad(fov) / 2.0));
}

mat3_t
pinhole_K(const real_t fx, const real_t fy, const real_t cx, const real_t cy) {
  mat3_t K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

int pinhole_project(const int res[2],
                    const vec4_t &proj_params,
                    const vec3_t &p_C,
                    vec2_t &z_hat) {
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);

  // Project, distort and then scale and center
  const vec2_t x = project_point(p_C);
  z_hat(0) = fx * x(0) + cx;
  z_hat(1) = fy * x(1) + cy;

  // Check projection
  const bool x_ok = (z_hat(0) >= 0 && z_hat(0) <= res[0]);
  const bool y_ok = (z_hat(1) >= 0 && z_hat(1) <= res[1]);
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

mat2_t pinhole_point_jacobian(const vec4_t &proj_params) {
  UNUSED(proj_params);
  mat2_t J = zeros(2, 2);
  J(0, 0) = proj_params(0); // fx
  J(1, 1) = proj_params(1); // fy
  return J;
}

mat_t<2, 4> pinhole_params_jacobian(const vec4_t &proj_params,
                                    const vec2_t &x) {
  UNUSED(proj_params);
  mat_t<2, 4> J = zeros(2, 4);
  J(0, 0) = x(0); // x
  J(1, 1) = x(1); // y
  J(0, 2) = 1;
  J(1, 3) = 1;
  return J;
}

/****************************** PINHOLE-RADTAN4 *******************************/

int pinhole_radtan4_project(const int res[2],
                            const vecx_t &params,
                            const vec3_t &p_C,
                            vec2_t &z_hat) {
  // Setup
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);

  // Project, distort and then scale and center
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);
  const vec2_t p = project_point(p_C);
  const vec2_t p_d = radtan4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res[0]);
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res[1]);
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

matx_t pinhole_radtan4_project_jacobian(const vecx_t &params,
                                        const vec3_t &p_C) {
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);
  const vec2_t p = project_point(p_C);

  const mat_t<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const mat_t<2, 2> J_d = radtan4_point_jacobian(dist_params, p);
  const matx_t J_p = project_jacobian(p_C);
  matx_t J_proj = J_k * J_d * J_p;

  return J_proj;
}

matx_t pinhole_radtan4_params_jacobian(const vecx_t &params,
                                       const vec3_t &p_C) {
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);

  const vec2_t p = project_point(p_C);
  const vec2_t p_d = radtan4_distort(dist_params, p);

  const mat_t<2, 4> J_proj_params = pinhole_params_jacobian(proj_params, p_d);
  const mat_t<2, 2> J_proj_point = pinhole_point_jacobian(proj_params);
  const mat_t<2, 4> J_dist_params = radtan4_params_jacobian(dist_params, p);

  matx_t J_params;
  J_params.resize(2, 8);
  J_params.block(0, 0, 2, 4) = J_proj_params;
  J_params.block(0, 4, 2, 4) = J_proj_point * J_dist_params;

  return J_params;
}

int pinhole_radtan4_back_project(const vecx_t &params,
                                 const vec2_t &x,
                                 vec3_t &ray) {
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (x.x() - cx) / fx;
  const real_t py = (x.y() - cy) / fy;
  const vec2_t p{px, py};

  const vec2_t kp = radtan4_undistort(params.tail(4), p);
  ray.x() = kp.x();
  ray.y() = kp.y();
  ray.z() = 1.0;

  return 0;
}

vec2_t pinhole_radtan4_undistort(const vecx_t &params, const vec2_t &z) {
  // Back-project and undistort
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (z.x() - cx) / fx;
  const real_t py = (z.y() - cy) / fy;
  const vec2_t p{px, py};
  const vec2_t p_undist = radtan4_undistort(params.tail(4), p);

  // Project undistorted point to image plane
  const real_t x = p_undist.x() * fx + cx;
  const real_t y = p_undist.y() * fy + cy;
  const vec2_t z_undist = {x, y};

  return z_undist;
}

/******************************* PINHOLE-EQUI4 ********************************/

int pinhole_equi4_project(const int res[2],
                          const vecx_t &params,
                          const vec3_t &p_C,
                          vec2_t &z_hat) {
  // Setup
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);

  // Check validity of the point, simple depth test.
  if (p_C.z() < 0.0) {
    return -1;
  }

  // Project, distort and then scale and center
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);
  const vec2_t p = project_point(p_C);
  const vec2_t p_d = equi4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() <= res[0]);
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() <= res[1]);
  const bool valid = (x_ok && y_ok) ? true : false;

  return (valid) ? 0 : -2;
}

matx_t pinhole_equi4_project_jacobian(const vecx_t &params, const vec3_t &p_C) {
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);
  const vec2_t p = project_point(p_C);

  const mat_t<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const mat_t<2, 2> J_d = equi4_point_jacobian(dist_params, p);
  const matx_t J_p = project_jacobian(p_C);
  matx_t J_proj = J_k * J_d * J_p;

  return J_proj;
}

matx_t pinhole_equi4_params_jacobian(const vecx_t &params, const vec3_t &p_C) {
  const vec4_t proj_params = params.head(4);
  const vec4_t dist_params = params.tail(4);

  const vec2_t p = project_point(p_C);
  const vec2_t p_d = equi4_distort(dist_params, p);

  const mat_t<2, 4> J_proj_params = pinhole_params_jacobian(proj_params, p_d);
  const mat_t<2, 2> J_proj_point = pinhole_point_jacobian(proj_params);
  const mat_t<2, 4> J_dist_params = equi4_params_jacobian(dist_params, p);

  matx_t J_params;
  J_params.resize(2, 8);
  J_params.block(0, 0, 2, 4) = J_proj_params;
  J_params.block(0, 4, 2, 4) = J_proj_point * J_dist_params;

  return J_params;
}

int pinhole_equi4_back_project(const vecx_t &params,
                               const vec2_t &x,
                               vec3_t &ray) {
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (x.x() - cx) / fx;
  const real_t py = (x.y() - cy) / fy;
  const vec2_t p{px, py};

  const vec2_t p_undist = equi4_undistort(params.tail(4), p);
  ray(0) = p_undist(0);
  ray(1) = p_undist(1);
  ray(2) = 1.0;

  return 0;
}

vec2_t pinhole_equi4_undistort(const vecx_t &params, const vec2_t &z) {
  // Back-project and undistort
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (z.x() - cx) / fx;
  const real_t py = (z.y() - cy) / fy;
  const vec2_t p{px, py};
  const vec2_t p_undist = equi4_undistort(params.tail(4), p);

  // Project undistorted point to image plane
  const real_t x = p_undist.x() * fx + cx;
  const real_t y = p_undist.y() * fy + cy;
  const vec2_t z_undist = {x, y};

  return z_undist;
}

static vec2_t opencv_undistort_point(const vecx_t &cam_params,
                                     const vec2_t &kp) {
  std::vector<cv::Point2f> pts_in;
  pts_in.emplace_back(kp.x(), kp.y());

  std::vector<cv::Point2f> pts_out;

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  std::vector<double> D;
  D.push_back(cam_params(4));
  D.push_back(cam_params(5));
  D.push_back(cam_params(6));
  D.push_back(cam_params(7));

  cv::undistortPoints(pts_in, pts_out, K, D, cv::noArray(), K);
  const vec2_t kp_opencv{pts_out[0].x, pts_out[0].y};

  return kp_opencv;
}

void kalibr_distort(const real_t k1,
                    const real_t k2,
                    const real_t p1,
                    const real_t p2,
                    vec2_t &y,
                    mat2_t &J) {
  double mx2_u = y[0] * y[0];
  double my2_u = y[1] * y[1];
  double mxy_u = y[0] * y[1];
  double rho2_u = mx2_u + my2_u;
  double rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

  J.setZero();
  J(0, 0) = 1 + rad_dist_u + k1 * 2.0 * mx2_u + k2 * rho2_u * 4 * mx2_u +
            2.0 * p1 * y[1] + 6 * p2 * y[0];
  J(1, 0) = k1 * 2.0 * y[0] * y[1] + k2 * 4 * rho2_u * y[0] * y[1] +
            p1 * 2.0 * y[0] + 2.0 * p2 * y[1];
  J(0, 1) = J(1, 0);
  J(1, 1) = 1 + rad_dist_u + k1 * 2.0 * my2_u + k2 * rho2_u * 4 * my2_u +
            6 * p1 * y[1] + 2.0 * p2 * y[0];

  y[0] += y[0] * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
  y[1] += y[1] * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

bool kalibr_back_project(const vecx_t &params, const vec2_t &kp, vec3_t &ray) {
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);

  const real_t k1 = params(4);
  const real_t k2 = params(5);
  const real_t p1 = params(6);
  const real_t p2 = params(7);

  // back-project
  const real_t px = (kp.x() - cx) / fx;
  const real_t py = (kp.y() - cy) / fy;
  vec2_t p{px, py};

  // undistort
  vec2_t pbar = p;
  mat2_t F;
  vec2_t p_tmp;
  for (int i = 0; i < 5; i++) {
    p_tmp = pbar;
    kalibr_distort(k1, k2, p1, p2, p_tmp, F);
    vec2_t e(p - p_tmp);
    vec2_t du = (F.transpose() * F).inverse() * F.transpose() * e;
    pbar += du;
    if (e.dot(e) < 1e-15) {
      break;
    }
  }
  p = pbar;

  ray.x() = p.x();
  ray.y() = p.y();
  ray.z() = 1.0;

  return true;
}

int solvepnp(const camera_geometry_t *cam,
             const int cam_res[2],
             const vecx_t &cam_params,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_CF) {
  UNUSED(cam_res);
  assert(keypoints.size() == object_points.size());

  // Create object points (counter-clockwise, from bottom left)
  // Note: SolvPnP assumes radtan which may not be true, therefore we
  // have to manually undistort the keypoints ourselves
  size_t nb_points = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  for (size_t i = 0; i < nb_points; i++) {
    // Check keypoint is valid
    const vec2_t z = keypoints[i];
    const bool x_ok = (z.x() >= 0 && z.x() <= cam_res[0]);
    const bool y_ok = (z.y() >= 0 && z.y() <= cam_res[1]);
    const bool valid = (x_ok && y_ok) ? true : false;
    if (valid == false) {
      printf("INVALID!\n");
      continue;
    }

    // Keypoint
    const vec2_t &kp = cam->undistort(cam_params, z);
    // const vec2_t &kp = opencv_undistort_point(cam_params, keypoints[i]);
    //   vec3_t p;
    //   if (kalibr_back_project(cam_params, img_pt, p)) {
    //     img_pts.emplace_back(p.x() / p.z(), p.y() / p.z());
    //   }
    img_pts.emplace_back(kp.x(), kp.y());

    // Object point
    const vec3_t &pt = object_points[i];
    obj_pts.emplace_back(pt.x(), pt.y(), pt.z());
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  // D.at<double>(0) = cam_params(4);
  // D.at<double>(1) = cam_params(5);
  // D.at<double>(2) = cam_params(6);
  // D.at<double>(3) = cam_params(7);

  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);
  cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF = tf(convert(R), convert(tvec));

  return 0;
}

} // namespace yac
