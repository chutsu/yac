#ifndef YAC_CALIB_PARAMS_HPP
#define YAC_CALIB_PARAMS_HPP

#include "core.hpp"

namespace yac {

typedef ssize_t id_t;

struct param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  bool fixed = false;
  bool marginalize = false;

  std::string type;
  id_t id = -1;
  timestamp_t ts = 0;
  long local_size = 0;
  long global_size = 0;
  vecx_t param;

  std::vector<id_t> factor_ids;

  param_t() {}

  param_t(const std::string &type_,
          const id_t id_,
          const timestamp_t &ts_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false)
    : fixed{fixed_},
      type{type_},
      id{id_},
      ts{ts_},
      local_size{local_size_},
      global_size{global_size_},
      param{zeros(global_size_, 1)} {}

  param_t(const std::string &type_,
          const id_t id_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false)
    : param_t{type_, id_, 0, local_size_, global_size_, fixed_} {}

  virtual ~param_t() {}

  void mark_marginalize() {
    marginalize = true;
    type = "marg_" + type;
  }

  virtual void plus(const vecx_t &) = 0;
  virtual void perturb(const int i, const real_t step_size) = 0;
};

struct pose_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  pose_t() {}

  pose_t(const id_t id_,
         const timestamp_t &ts_,
         const vec_t<7> &pose,
         const bool fixed_=false)
      : param_t{"pose_t", id_, ts_, 6, 7, fixed_} {
    param = pose;
  }

  pose_t(const id_t id_,
         const timestamp_t &ts_,
         const mat4_t &T,
         const bool fixed_=false)
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

  quat_t rot() const {
    return quat_t{param[6], param[3], param[4], param[5]};
  }

  vec3_t trans() const {
    return vec3_t{param[0], param[1], param[2]};
  }

  mat4_t tf() const {
    return yac::tf(rot(), trans());
  }

  quat_t rot() { return static_cast<const pose_t &>(*this).rot(); }
  vec3_t trans() { return static_cast<const pose_t &>(*this).trans(); }
  mat4_t tf() { return static_cast<const pose_t &>(*this).tf(); }

  void set_trans(const vec3_t &r) {
    param(0) = r(0);
    param(1) = r(1);
    param(2) = r(2);
  }

  void set_rot(const quat_t &q) {
    param(3) = q.x();
    param(4) = q.y();
    param(5) = q.z();
    param(6) = q.w();
  }

  void set_rot(const mat3_t &C) {
    quat_t q{C};
    param(3) = q.x();
    param(4) = q.y();
    param(5) = q.z();
    param(6) = q.w();
  }

  void plus(const vecx_t &dx) {
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

  void perturb(const int i, const real_t step_size) {
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
};

struct fiducial_pose_t : pose_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  fiducial_pose_t() {}

  fiducial_pose_t(const id_t id_, const mat4_t &T, const bool fixed_=false)
    : pose_t{id_, 0, T, fixed_} {
    this->type = "fiducial_pose_t";
  }
};

struct extrinsic_t : pose_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  extrinsic_t() {}

  extrinsic_t(const id_t id_, const mat4_t &T, const bool fixed_=false)
    : pose_t{id_, 0, T, fixed_} {
    this->type = "extrinsic_t";
  }
};

struct landmark_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  landmark_t() {}

  landmark_t(const id_t id_, const vec3_t &p_W_, const bool fixed_=false)
    : param_t{"landmark_t", id_, 3, 3, fixed_} {
    param = p_W_;
  }

  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param[i] += step_size; }
};

struct camera_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_index = 0;
  int resolution[2] = {0, 0};
  std::string proj_model;
  std::string dist_model;
  long proj_size = 0;
  long dist_size = 0;

  camera_params_t() {}

  camera_params_t(const id_t id_,
                  const int cam_index_,
                  const int resolution_[2],
                  const std::string proj_model_,
                  const std::string dist_model_,
                  const vecx_t &proj_params_,
                  const vecx_t &dist_params_,
                  const bool fixed_=false)
    : param_t{"camera_params_t", id_, proj_params_.size() + dist_params_.size(),
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

  vecx_t proj_params() { return param.head(proj_size); }
  vecx_t dist_params() { return param.tail(dist_size); }
  vecx_t proj_params() const { return param.head(proj_size); }
  vecx_t dist_params() const { return param.tail(dist_size); }
  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param(i) += step_size; }
};

struct sb_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  sb_params_t() {}

  sb_params_t(const id_t id_,
             const timestamp_t &ts_,
             const vec3_t &v_,
             const vec3_t &ba_,
             const vec3_t &bg_,
             const bool fixed_=false)
    : param_t{"sb_params_t", id_, ts_, 9, 9, fixed_} {
    param << v_, ba_, bg_;
  }

  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param(i) += step_size; }
};

} // namespace yac
#endif // YAC_CALIB_PARAMS_HPP
