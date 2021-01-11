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

  param_t();
  param_t(const std::string &type_,
          const id_t id_,
          const timestamp_t &ts_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false);
  param_t(const std::string &type_,
          const id_t id_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false);
  virtual ~param_t();

  void mark_marginalize();
  virtual void plus(const vecx_t &) = 0;
  virtual void perturb(const int i, const real_t step_size) = 0;
};

struct pose_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  pose_t();
  pose_t(const id_t id_,
         const timestamp_t &ts_,
         const vec_t<7> &pose,
         const bool fixed_=false);
  pose_t(const id_t id_,
         const timestamp_t &ts_,
         const mat4_t &T,
         const bool fixed_=false);

  quat_t rot() const;
  vec3_t trans() const;
  mat4_t tf() const;

  quat_t rot();
  vec3_t trans();
  mat4_t tf();

  void set_trans(const vec3_t &r);
  void set_rot(const quat_t &q);
  void set_rot(const mat3_t &C);
  void set_tf(const mat3_t &C, const vec3_t &r);
  void set_tf(const quat_t &q, const vec3_t &r);
  void set_tf(const mat4_t &T);
  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
};

struct fiducial_pose_t : pose_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  fiducial_pose_t() {}
  fiducial_pose_t(const id_t id_, const mat4_t &T, const bool fixed_=false)
    : pose_t{id_, 0, T, fixed_} {
    this->type = "fiducial_pose_t";
  }
};

struct fiducial_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
	mat4_t T_WF;

public:
  fiducial_params_t();
  fiducial_params_t(const id_t id_,
			 	 	 	 	 	 	 	const mat4_t &T_WF_,
                    const bool fixed_=false);

  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
	void update();
	mat4_t estimate() const;
	mat4_t estimate();
};

struct extrinsics_t : pose_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  extrinsics_t() {}
  extrinsics_t(const id_t id_, const mat4_t &T, const bool fixed_=false)
    : pose_t{id_, 0, T, fixed_} {
    this->type = "extrinsics_t";
  }
};

struct landmark_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  landmark_t() {}
  landmark_t(const id_t id_, const vec3_t &p_W_, const bool fixed_=false)
    : param_t{"landmark_t", id_, 3, 3, fixed_} {
    param = p_W_;
  }

  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
};

struct camera_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_index = 0;
  int resolution[2] = {0, 0};
  std::string proj_model;
  std::string dist_model;
  long proj_size = 0;
  long dist_size = 0;

  camera_params_t();
  camera_params_t(const id_t id_,
                  const int cam_index_,
                  const int resolution_[2],
                  const std::string proj_model_,
                  const std::string dist_model_,
                  const vecx_t &proj_params_,
                  const vecx_t &dist_params_,
                  const bool fixed_=false);

  vecx_t proj_params();
  vecx_t dist_params();
  vecx_t proj_params() const;
  vecx_t dist_params() const;

  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
};

struct sb_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  sb_params_t();
  sb_params_t(const id_t id_,
              const timestamp_t &ts_,
              const vec3_t &v_,
              const vec3_t &ba_,
              const vec3_t &bg_,
              const bool fixed_=false);
  sb_params_t(const id_t id_,
              const timestamp_t &ts_,
              const vec_t<9> &sb_,
              const bool fixed_=false);

  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
};

struct time_delay_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  time_delay_t();
  time_delay_t(const id_t id_,
              const double td_,
              const bool fixed_=false);

  void plus(const vecx_t &dx);
  void perturb(const int i, const real_t step_size);
};

// struct imu_params_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   mat4_t T_BS;         // Transformation from Body frame to IMU (sensor frame S).
//   double a_max;        // Accelerometer saturation. [m/s^2]
//   double g_max;        // Gyroscope saturation. [rad/s]
//   double sigma_g_c;    // Gyroscope noise density.
//   double sigma_bg;     // Initial gyroscope bias.
//   double sigma_a_c;    // Accelerometer noise density.
//   double sigma_ba;     // Initial accelerometer bias
//   double sigma_gw_c;   // Gyroscope drift noise density.
//   double sigma_aw_c;   // Accelerometer drift noise density.
//   double tau;          // Reversion time constant of accerometer bias. [s]
//   double g;            // Earth acceleration.
//   vec3_t a0;           // Mean of the prior accelerometer bias.
//   int rate;            // IMU rate in Hz.
// };

} // namespace yac
#endif // YAC_CALIB_PARAMS_HPP
