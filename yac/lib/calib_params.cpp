#include "calib_params.hpp"

namespace yac {

/******************************** param_t *************************************/

param_t::param_t(const std::string &type_,
                 const timestamp_t &ts_,
                 const long local_size_,
                 const long global_size_,
                 const bool fixed_)
    : fixed{fixed_}, type{type_}, ts{ts_}, local_size{local_size_},
      global_size{global_size_}, param{zeros(global_size_, 1)} {}

param_t::param_t(const std::string &type_,
                 const long local_size_,
                 const long global_size_,
                 const bool fixed_)
    : param_t{type_, 0, local_size_, global_size_, fixed_} {}

double *param_t::data() { return param.data(); }

void param_t::mark_marginalize() {
  marginalize = true;
  type = "marg_" + type;
}

void param_t::plus(const vecx_t &dx) { param += dx; }

void param_t::minus(const vecx_t &dx) { param -= dx; }

void param_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

/********************************* pose_t *************************************/

pose_t::pose_t(const timestamp_t &ts_, const vec_t<7> &pose, const bool fixed_)
    : param_t{"pose_t", ts_, 6, 7, fixed_} {
  param = pose;
}

pose_t::pose_t(const timestamp_t &ts_, const mat4_t &T, const bool fixed_)
    : param_t{"pose_t", ts_, 6, 7, fixed_} {
  const quat_t q{tf_quat(T)};
  const vec3_t r{tf_trans(T)};

  param(0) = r(0);
  param(1) = r(1);
  param(2) = r(2);

  param(3) = q.x();
  param(4) = q.y();
  param(5) = q.z();
  param(6) = q.w();
}

quat_t pose_t::rot() const {
  return quat_t{param[6], param[3], param[4], param[5]};
}

vec3_t pose_t::trans() const { return vec3_t{param[0], param[1], param[2]}; }

mat4_t pose_t::tf() const { return yac::tf(rot(), trans()); }

quat_t pose_t::rot() { return static_cast<const pose_t &>(*this).rot(); }
vec3_t pose_t::trans() { return static_cast<const pose_t &>(*this).trans(); }
mat4_t pose_t::tf() { return static_cast<const pose_t &>(*this).tf(); }

void pose_t::set_trans(const vec3_t &r) {
  param(0) = r(0);
  param(1) = r(1);
  param(2) = r(2);
}

void pose_t::set_rot(const quat_t &q) {
  param(3) = q.x();
  param(4) = q.y();
  param(5) = q.z();
  param(6) = q.w();
}

void pose_t::set_rot(const mat3_t &C) {
  quat_t q{C};
  param(3) = q.x();
  param(4) = q.y();
  param(5) = q.z();
  param(6) = q.w();
}

void pose_t::set_tf(const mat3_t &C, const vec3_t &r) {
  set_rot(C);
  set_trans(r);
}

void pose_t::set_tf(const quat_t &q, const vec3_t &r) {
  set_rot(q);
  set_trans(r);
}

void pose_t::set_tf(const mat4_t &T) {
  set_rot(tf_rot(T));
  set_trans(tf_trans(T));
}

void pose_t::plus(const vecx_t &dx) {
  // Translation component
  param(0) += dx(0);
  param(1) += dx(1);
  param(2) += dx(2);

  // Rotation component
  const vec3_t dalpha{dx(3), dx(4), dx(5)};
  const quat_t dq = quat_delta(dalpha);
  const quat_t q{param[6], param[3], param[4], param[5]};
  const quat_t q_updated = dq * q;
  param(3) = q_updated.x();
  param(4) = q_updated.y();
  param(5) = q_updated.z();
  param(6) = q_updated.w();
}

void pose_t::minus(const vecx_t &dx) {
  plus(-dx);
  // const vec3_t dr = param.head<3>() - dx.head<3>();
  // const quat_t q_i(param(6), param(3), param(4), param(5));
  // const quat_t q_j(dx(6), dx(3), dx(4), dx(5));
  // const quat_t dq = q_j.inverse() * q_i;
  //
  // param.segment<3>(0) = dr;
  // param.segment<3>(3) = 2.0 * dq.vec();
  // if (!(dq.w() >= 0)) {
  //   param.segment<3>(3) = 2.0 * -dq.vec();
  // }
}

void pose_t::perturb(const int i, const real_t step_size) {
  if (i >= 0 && i < 3) {
    const auto T_WS_diff = tf_perturb_trans(this->tf(), step_size, i);
    this->set_rot(tf_rot(T_WS_diff));
    this->set_trans(tf_trans(T_WS_diff));
  } else if (i >= 3 && i <= 5) {
    const auto T_WS_diff = tf_perturb_rot(this->tf(), step_size, i - 3);
    this->set_rot(tf_rot(T_WS_diff));
    this->set_trans(tf_trans(T_WS_diff));
  } else {
    FATAL("Invalid perturbation index [%d]!", i);
  }
}

/****************************** fiducial_t **********************************/

#if FIDUCIAL_PARAMS_SIZE == 2
fiducial_t::fiducial_t(const mat4_t &T_WF_, const bool fixed_)
    : param_t{"fiducial_t", 0, 2, 2, fixed_} {
  const vec3_t rpy = quat2euler(tf_quat(T_WF_));
  param = vec2_t{rpy(0), rpy(1)};
  T_WF = T_WF_;
}
#elif FIDUCIAL_PARAMS_SIZE == 3
fiducial_t::fiducial_t(const mat4_t &T_WF_, const bool fixed_)
    : param_t{"fiducial_t", 0, 3, 3, fixed_} {
  param = quat2euler(tf_quat(T_WF_));
  T_WF = T_WF_;
}
#elif FIDUCIAL_PARAMS_SIZE == 7
fiducial_t::fiducial_t(const mat4_t &T_WF_, const bool fixed_)
    : pose_t{0, T_WF_, fixed_} {
  this->type = "fiducial_t";
}
#endif

#if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
void fiducial_t::plus(const vecx_t &dx) {
  param += dx;
  update();
}

void fiducial_t::minus(const vecx_t &dx) {
  param -= dx;
  update();
}

void fiducial_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
  update();
}

void fiducial_t::update() {
#if FIDUCIAL_PARAMS_SIZE == 2
  const double yaw = quat2euler(tf_quat(T_WF))(2);
  const vec3_t rpy{param(0), param(1), yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF);
  T_WF = tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 3
  const vec3_t rpy{param(0), param(1), param(2)};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF);
  T_WF = tf(C_WF, r_WF);
#endif
}
#endif

mat4_t fiducial_t::estimate() const {
#if FIDUCIAL_PARAMS_SIZE == 2
  update();
  const double yaw = quat2euler(tf_quat(T_WF))(2);
  const vec3_t rpy{param(0), param(1), yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF);
  return tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 3
  update();
  const vec3_t rpy{param(0), param(1), param(2)};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF);
  return tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 7
  return yac::tf(param);
#endif
}

/**************************** camera_params_t *********************************/

int focal_init(const aprilgrid_t &grid, const int axis, double &focal) {
  // Using the method in:
  // C. Hughes, P. Denny, M. Glavin, and E. Jones,
  // Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
  // Extraction, PAMI 2010

  // Method summary:
  // 1. Form vanishing point lines from aprilgrid
  // 2. For each of these lines estimate a circle to obtain cx, cy, radius
  // 3. For each pair of circles find the intersection (i.e. vanishing points)
  // 4. From each pair of vanishing points the focal length can be estimated

  // Check if aprilgrid is fully observed
  if (grid.nb_detections < (grid.tag_rows * grid.tag_cols * 4)) {
    return -1;
  }

  // Form vanishing point lines from aprilgrid and estimate circle params
  // -------------------------------------------------------------------------
  // Form vanishing point lines horizontally or vertically using the aprilgrid
  // detected points.  Fit a circle for each of these lines. By fitting a
  // circle to these lines we obtain circle cx, cy and radius. And from that we
  // can find the intersection of these circles / lines to obtain vanishing
  // points.
  std::vector<vec2_t> centers;
  std::vector<double> radiuses;

  if (axis == 0) {
    for (int i = 0; i < grid.tag_rows; i++) {
      int tag_id = i * grid.tag_cols;
      vec2s_t points;
      for (int j = 0; j < grid.tag_cols; j++) {
        points.push_back(grid.keypoint(tag_id, 0)); // Bottom left
        points.push_back(grid.keypoint(tag_id, 1)); // Bottom right
        tag_id++;
      }

      double cx = 0.0;
      double cy = 0.0;
      double r = 0.0;
      fit_circle(points, cx, cy, r);
      centers.emplace_back(cx, cy);
      radiuses.push_back(r);
    }
  } else if (axis == 1) {
    for (int j = 0; j < grid.tag_cols; j++) {
      int tag_id = j;
      vec2s_t points;
      for (int i = 0; i < grid.tag_rows; i++) {
        points.push_back(grid.keypoint(tag_id, 0)); // Bottom left
        points.push_back(grid.keypoint(tag_id, 3)); // Top left
        tag_id += grid.tag_cols;
      }

      double cx = 0.0;
      double cy = 0.0;
      double r = 0.0;
      fit_circle(points, cx, cy, r);
      centers.emplace_back(cx, cy);
      radiuses.push_back(r);
    }
  } else {
    FATAL("Invalid axis[%d], expecting 0 or 1!", axis);
  }

  // Estimate focal lengths between all pairs of vanishing points
  std::vector<double> focal_guesses;
  int nb_circles = centers.size();
  for (int i = 0; i < nb_circles; i++) {
    for (int j = i + 1; j < nb_circles; j++) {
      // Find vanishing points between two circles
      const auto cx0 = centers[i].x();
      const auto cy0 = centers[i].y();
      const auto r0 = radiuses[i];
      const auto cx1 = centers[j].x();
      const auto cy1 = centers[j].y();
      const auto r1 = radiuses[j];
      const auto ipts = intersect_circles(cx0, cy0, r0, cx1, cy1, r1);
      if (ipts.size() < 2) {
        continue;
      }

      // Estimate focal length. Equation (8) in the paper.
      const vec2_t vp0 = ipts[0];
      const vec2_t vp1 = ipts[1];
      double f_guess = (vp0 - vp1).norm() / M_PI;
      focal_guesses.push_back(f_guess);
    }
  }

  // From all the focal length guesses return the median
  if (focal_guesses.size() == 0) {
    LOG_ERROR("No focal guesses!");
    return -1;
  }
  focal = median(focal_guesses);

  return 0;
}

camera_params_t::camera_params_t(const int cam_index_,
                                 const int resolution_[2],
                                 const std::string proj_model_,
                                 const std::string dist_model_,
                                 const vecx_t &proj_params_,
                                 const vecx_t &dist_params_,
                                 const bool fixed_)
    : param_t{"camera_params_t",
              proj_params_.size() + dist_params_.size(),
              proj_params_.size() + dist_params_.size(),
              fixed_},
      cam_index{cam_index_}, resolution{resolution_[0], resolution_[1]},
      proj_model{proj_model_}, dist_model{dist_model_},
      proj_size{(int)proj_params_.size()}, dist_size{(int)dist_params_.size()} {
  param.resize(proj_size + dist_size);
  param.head(proj_size) = proj_params_;
  param.tail(dist_size) = dist_params_;
}

vecx_t camera_params_t::proj_params() const { return param.head(proj_size); }

vecx_t camera_params_t::dist_params() const { return param.tail(dist_size); }

camera_params_t camera_params_t::init(const int cam_index_,
                                      const int resolution_[2],
                                      const std::string proj_model_,
                                      const std::string dist_model_,
                                      const bool fixed_) {
  // Projection params
  vecx_t proj_params;
  if (proj_model_ == "pinhole") {
    const double fx = pinhole_focal(resolution_[0], 90);
    const double fy = pinhole_focal(resolution_[0], 90);
    const double cx = resolution_[0] / 2.0;
    const double cy = resolution_[1] / 2.0;
    proj_params.resize(4);
    proj_params << fx, fy, cx, cy;
  } else {
    FATAL("Unsupported [%s]!", proj_model_.c_str());
  }

  // Distortion params
  vecx_t dist_params;
  if (dist_model_ == "radtan4" || dist_model_ == "equi4") {
    dist_params = zeros(4, 1);
  } else {
    FATAL("Unsupported [%s]!", dist_model_.c_str());
  }

  return camera_params_t{cam_index_,
                         resolution_,
                         proj_model_,
                         dist_model_,
                         proj_params,
                         dist_params,
                         fixed_};
}

/****************************** sb_params_t ***********************************/

sb_params_t::sb_params_t(const timestamp_t &ts_,
                         const vec3_t &v_,
                         const vec3_t &ba_,
                         const vec3_t &bg_,
                         const bool fixed_)
    : param_t{"sb_params_t", ts_, 9, 9, fixed_} {
  param << v_, ba_, bg_;
}

sb_params_t::sb_params_t(const timestamp_t &ts_,
                         const vec_t<9> &sb_,
                         const bool fixed_)
    : param_t{"sb_params_t", ts_, 9, 9, fixed_} {
  param = sb_;
}

/***************************** time_delay_t ***********************************/

time_delay_t::time_delay_t(const double td_, const bool fixed_)
    : param_t{"time_delay_t", 1, 1, fixed_} {
  param(0) = td_;
}

} // namespace yac
