#include "SolvePnp.hpp"

namespace yac {

SolvePnp::SolvePnp(const std::shared_ptr<CameraGeometry> &camera_geometry)
    : camera_geometry_{camera_geometry} {}

int SolvePnp::estimate(const vec2s_t &keypoints,
                       const vec3s_t &object_points,
                       mat4_t &T_camera_object) {
  assert(keypoints.size() == object_points.size());

  // Form object and image points
  const vec2i_t resolution = camera_geometry_->getResolution();
  const auto camera_model = camera_geometry_->getCameraModel();
  const vecx_t &intrinsic = camera_geometry_->getIntrinsic();
  const size_t N = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;

  for (size_t i = 0; i < N; i++) {
    // Check keypoint is valid
    const vec2_t z = keypoints[i];
    const bool x_ok = (z.x() >= 0 && z.x() <= resolution.x());
    const bool y_ok = (z.y() >= 0 && z.y() <= resolution.y());
    const bool valid = (x_ok && y_ok) ? true : false;
    if (valid == false) {
      continue;
    }

    // Keypoint
    // Note: SolvPnP assumes radtan which may not be true, therefore we have to
    // manually undistort the keypoints ourselves
    const vec2_t &kp = camera_model->undistort(intrinsic, z);
    img_pts.emplace_back(kp.x(), kp.y());

    // Object point
    const vec3_t &pt = object_points[i];
    obj_pts.emplace_back(pt.x(), pt.y(), pt.z());
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = intrinsic(0);
  K.at<double>(1, 1) = intrinsic(1);
  K.at<double>(0, 2) = intrinsic(2);
  K.at<double>(1, 2) = intrinsic(3);

  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat cv_rvec(3, 1, CV_64F);
  cv::Mat cv_tvec(3, 1, CV_64F);
  cv::solvePnP(obj_pts, img_pts, K, D, cv_rvec, cv_tvec);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat cv_R;
  cv::Rodrigues(cv_rvec, cv_R);
  // -- Form full transformation matrix
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(cv_R, R);
  cv::cv2eigen(cv_tvec, t);
  T_camera_object = tf(R, t);

  return 0;
}

} // namespace yac
