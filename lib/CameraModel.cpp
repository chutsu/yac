#include "CameraModel.hpp"

namespace yac {

/********************************* RADTAN4 ************************************/

vec2_t radtan4_distort(const vec4_t &dist_params, const vec2_t &p) {
  const double x = p.x();
  const double y = p.y();

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  // Apply radial distortion
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  // Apply tangential distortion
  const double xy = x * y;
  const double x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const double y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

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
  const double x = p(0);
  const double y = p(1);

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

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

  const double x = p.x();
  const double y = p.y();

  const double xy = x * y;
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

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
  const double r = p.norm();
  if (r < 1e-8) {
    return p;
  }

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  // Apply equi distortion
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (thd / r) * p(0);
  const double y_dash = (thd / r) * p(1);

  return vec2_t{x_dash, y_dash};
}

vec2_t equi4_undistort(const vec4_t &dist_params, const vec2_t &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double thd = sqrt(p(0) * p(0) + p(1) * p(1));
  double th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const double th2 = th * th;
    const double th4 = th2 * th2;
    const double th6 = th4 * th2;
    const double th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const double scaling = tan(th) / thd;
  return vec2_t{p(0) * scaling, p(1) * scaling};
}

mat2_t equi4_point_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;

  // Form jacobian
  const double th_r = 1.0 / (r * r + 1.0);
  double thd_th = 1.0 + 3.0 * k1 * th2;
  thd_th += 5.0 * k2 * th4;
  thd_th += 7.0 * k3 * th6;
  thd_th += 9.0 * k4 * th8;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;

  mat2_t J_point = I(2);
  J_point(0, 0) = s + x * s_r * r_x;
  J_point(0, 1) = x * s_r * r_y;
  J_point(1, 0) = y * s_r * r_x;
  J_point(1, 1) = s + y * s_r * r_y;

  return J_point;
}

matx_t equi4_params_jacobian(const vec4_t &dist_params, const vec2_t &p) {
  UNUSED(dist_params);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);

  const double th3 = th * th * th;
  const double th5 = th3 * th * th;
  const double th7 = th5 * th * th;
  const double th9 = th7 * th * th;

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
  const double x = p_C.x();
  const double y = p_C.y();
  const double z = p_C.z();

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

double pinhole_focal(const int image_size, const double fov) {
  return ((image_size / 2.0) / tan(deg2rad(fov) / 2.0));
}

mat3_t
pinhole_K(const double fx, const double fy, const double cx, const double cy) {
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
  const double fx = proj_params(0);
  const double fy = proj_params(1);
  const double cx = proj_params(2);
  const double cy = proj_params(3);

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

PINHOLE_PROJECT(pinhole_radtan4_project, radtan4_distort);
PINHOLE_PROJECT_J(pinhole_radtan4_project_jacobian, radtan4_point_jacobian);
PINHOLE_PARAMS_J(pinhole_radtan4_params_jacobian, radtan4_params_jacobian);
PINHOLE_BACK_PROJECT(pinhole_radtan4_back_project, radtan4_undistort);
PINHOLE_UNDISTORT(pinhole_radtan4_undistort, radtan4_undistort);

PINHOLE_PROJECT(pinhole_equi4_project, equi4_distort);
PINHOLE_PROJECT_J(pinhole_equi4_project_jacobian, equi4_point_jacobian);
PINHOLE_PARAMS_J(pinhole_equi4_params_jacobian, equi4_params_jacobian);
PINHOLE_BACK_PROJECT(pinhole_equi4_back_project, equi4_undistort);
PINHOLE_UNDISTORT(pinhole_equi4_undistort, equi4_undistort);

/*********************************** MISC *************************************/

int solvepnp(const CameraModel *cam,
             const int cam_res[2],
             const vecx_t &cam_params,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_camera_object) {
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
