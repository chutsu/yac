#include "calib_params.hpp"

namespace yac {

/******************************** param_t *************************************/

param_t::param_t() {}

param_t::param_t(const std::string &type_,
        const id_t id_,
        const timestamp_t &ts_,
        const long local_size_,
        const long global_size_,
        const bool fixed_)
  : fixed{fixed_},
    type{type_},
    id{id_},
    ts{ts_},
    local_size{local_size_},
    global_size{global_size_},
    param{zeros(global_size_, 1)} {}

param_t::param_t(const std::string &type_,
        const id_t id_,
        const long local_size_,
        const long global_size_,
        const bool fixed_)
  : param_t{type_, id_, 0, local_size_, global_size_, fixed_} {}

param_t::~param_t() {}

void param_t::mark_marginalize() {
  marginalize = true;
  type = "marg_" + type;
}

void param_t::plus(const vecx_t &dx) {
  param += dx;
}

void param_t::minus(const vecx_t &dx) {
  param -= dx;
}

void param_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

/********************************* pose_t *************************************/

pose_t::pose_t() {}

pose_t::pose_t(const id_t id_,
        const timestamp_t &ts_,
        const vec_t<7> &pose,
        const bool fixed_)
    : param_t{"pose_t", id_, ts_, 6, 7, fixed_} {
  param = pose;
}

pose_t::pose_t(const id_t id_,
      const timestamp_t &ts_,
      const mat4_t &T,
      const bool fixed_)
  : param_t{"pose_t", id_, ts_, 6, 7, fixed_} {
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

vec3_t pose_t::trans() const {
  return vec3_t{param[0], param[1], param[2]};
}

mat4_t pose_t::tf() const {
  return yac::tf(rot(), trans());
}

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

fiducial_t::fiducial_t() {}

#if FIDUCIAL_PARAMS_SIZE == 2
fiducial_t::fiducial_t(const id_t id_,
                       const mat4_t &T_WF_,
                       const bool fixed_)
  : param_t{"fiducial_t", id_, 0, 2, 2, fixed_} {
  const vec3_t rpy = quat2euler(tf_quat(T_WF_));
  param = vec2_t{rpy(0), rpy(1)};
  T_WF = T_WF_;
}
#elif FIDUCIAL_PARAMS_SIZE == 3
fiducial_t::fiducial_t(const id_t id_,
                       const mat4_t &T_WF_,
                       const bool fixed_)
  : param_t{"fiducial_t", id_, 0, 3, 3, fixed_} {
  param = quat2euler(tf_quat(T_WF_));
  T_WF = T_WF_;
}
#elif FIDUCIAL_PARAMS_SIZE == 7
fiducial_t::fiducial_t(const id_t id_,
                       const mat4_t &T_WF_,
                       const bool fixed_)
  : pose_t{id_, 0, T_WF_, fixed_} {
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

mat4_t fiducial_t::estimate() {
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

camera_params_t::camera_params_t() {}

camera_params_t::camera_params_t(const id_t id_,
                                 const int cam_index_,
                                 const int resolution_[2],
                                 const std::string proj_model_,
                                 const std::string dist_model_,
                                 const vecx_t &proj_params_,
                                 const vecx_t &dist_params_,
                                 const bool fixed_)
    : param_t{"camera_params_t", id_,
              proj_params_.size() + dist_params_.size(),
              proj_params_.size() + dist_params_.size(),
              fixed_},
      cam_index{cam_index_},
      resolution{resolution_[0], resolution_[1]},
      proj_model{proj_model_},
      dist_model{dist_model_},
      proj_size{proj_params_.size()},
      dist_size{dist_params_.size()} {
  param.resize(proj_size + dist_size);
  param.head(proj_size) = proj_params_;
  param.tail(dist_size) = dist_params_;
}

vecx_t camera_params_t::proj_params() {
  return param.head(proj_size);
}

vecx_t camera_params_t::dist_params() {
  return param.tail(dist_size);
}

vecx_t camera_params_t::proj_params() const {
  return param.head(proj_size);
}

vecx_t camera_params_t::dist_params() const {
  return param.tail(dist_size);
}

void camera_params_t::plus(const vecx_t &dx) {
  param += dx;
}

void camera_params_t::minus(const vecx_t &dx) {
  param -= dx;
}

void camera_params_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

/****************************** sb_params_t ***********************************/

sb_params_t::sb_params_t() {}

sb_params_t::sb_params_t(const id_t id_,
                         const timestamp_t &ts_,
                         const vec3_t &v_,
                         const vec3_t &ba_,
                         const vec3_t &bg_,
                         const bool fixed_)
  : param_t{"sb_params_t", id_, ts_, 9, 9, fixed_} {
  param << v_, ba_, bg_;
}

sb_params_t::sb_params_t(const id_t id_,
                         const timestamp_t &ts_,
                         const vec_t<9> &sb_,
                         const bool fixed_)
  : param_t{"sb_params_t", id_, ts_, 9, 9, fixed_} {
  param = sb_;
}

void sb_params_t::plus(const vecx_t &dx) {
  param += dx;
}

void sb_params_t::minus(const vecx_t &dx) {
  param -= dx;
}

void sb_params_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

/***************************** time_delay_t ***********************************/

time_delay_t::time_delay_t() {}

time_delay_t::time_delay_t(const id_t id_,
                           const double td_,
                           const bool fixed_)
  : param_t{"time_delay_t", id_, 1, 1, fixed_} {
  param(0) = td_;
}

void time_delay_t::plus(const vecx_t &dx) {
  param += dx;
}

void time_delay_t::minus(const vecx_t &dx) {
  param -= dx;
}

void time_delay_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

} // namespace yac
