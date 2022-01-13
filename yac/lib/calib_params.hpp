#ifndef YAC_CALIB_PARAMS_HPP
#define YAC_CALIB_PARAMS_HPP

#include "util/util.hpp"

namespace yac {

typedef ssize_t id_t;

struct param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  bool fixed = false;
  bool marginalize = false;

  std::string type;
  timestamp_t ts = 0;
  long local_size = 0;
  long global_size = 0;
  vecx_t param;

  std::vector<id_t> factor_ids;

  param_t() = default;
  param_t(const std::string &type_,
          const timestamp_t &ts_,
          const long local_size_,
          const long global_size_,
          const bool fixed_ = false);
  param_t(const std::string &type_,
          const long local_size_,
          const long global_size_,
          const bool fixed_ = false);
  virtual ~param_t() = default;

  double *data();
  void mark_marginalize();
  virtual void plus(const vecx_t &);
  virtual void minus(const vecx_t &);
  virtual void perturb(const int i, const real_t step_size);
};

struct pose_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  int fix = 0;

  pose_t() = default;
  pose_t(const timestamp_t &ts_,
         const vec_t<7> &pose,
         const bool fixed_ = false);
  pose_t(const timestamp_t &ts_, const mat4_t &T, const bool fixed_ = false);
  ~pose_t() = default;

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
  virtual void plus(const vecx_t &dx);
  virtual void minus(const vecx_t &dx);
  virtual void perturb(const int i, const real_t step_size);
};

#define FIDUCIAL_PARAMS_SIZE 2

#if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
struct fiducial_t : param_t {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  fiducial_t(const mat4_t &T_WF_, const bool fixed_ = false);

  void plus(const vecx_t &dx) override;
  void minus(const vecx_t &dx) override;
  void perturb(const int i, const real_t step_size) override;
  void update();
  mat4_t estimate();

private:
  mat4_t T_WF;
};

#elif FIDUCIAL_PARAMS_SIZE == 7
struct fiducial_t : pose_t {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  fiducial_t();
  fiducial_t(const id_t id_, const mat4_t &T_WF_, const bool fixed_ = false);

  mat4_t estimate();
};
#endif

struct extrinsics_t : pose_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  extrinsics_t(const mat4_t &T = I(4), const bool fixed_ = false)
      : pose_t{0, T, fixed_} {
    this->type = "extrinsics_t";
  }

  extrinsics_t(const vec_t<7> &pose, const bool fixed_ = false)
      : pose_t{0, pose, fixed_} {
    this->type = "extrinsics_t";
  }

  mat4_t tf() { return pose_t::tf(); }
  mat4_t tf() const { return pose_t::tf(); }
  void plus(const vecx_t &dx) { pose_t::plus(dx); }
  void perturb(const int i, const real_t step_size) {
    pose_t::perturb(i, step_size);
  }
};

struct landmark_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  landmark_t() {}
  landmark_t(const vec3_t &p_W_, const bool fixed_ = false)
      : param_t{"landmark_t", 3, 3, fixed_} {
    param = p_W_;
  }
};

/**
 * Initialize focal lengths using aprilgrid. `axis` denotes the focal length
 * for x-axis or y-axis, 0 or 1 repsectively. The result is written to `focal`.
 * Returns 0 for success, -1 for failure.
 */
int focal_init(const aprilgrid_t &grid, const int axis, double &focal);

struct camera_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_index = 0;
  int resolution[2] = {0, 0};
  std::string proj_model;
  std::string dist_model;
  int proj_size = 0;
  int dist_size = 0;

  camera_params_t() = default;
  camera_params_t(const int cam_index_,
                  const int resolution_[2],
                  const std::string proj_model_,
                  const std::string dist_model_,
                  const vecx_t &proj_params_,
                  const vecx_t &dist_params_,
                  const bool fixed_ = false);

  vecx_t proj_params() const;
  vecx_t dist_params() const;

  static camera_params_t init(const int cam_index_,
                              const int resolution_[2],
                              const std::string proj_model_,
                              const std::string dist_model_,
                              const bool fixed_ = false);
};

struct sb_params_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  sb_params_t() = default;
  sb_params_t(const timestamp_t &ts_,
              const vec3_t &v_,
              const vec3_t &ba_,
              const vec3_t &bg_,
              const bool fixed_ = false);
  sb_params_t(const timestamp_t &ts_,
              const vec_t<9> &sb_,
              const bool fixed_ = false);
};

struct time_delay_t : param_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  time_delay_t(const double td_, const bool fixed_ = false);
};

// struct imu_params_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   mat4_t T_BS;         // Transformation from Body frame to IMU (sensor frame
//   S). double a_max;        // Accelerometer saturation. [m/s^2] double g_max;
//   // Gyroscope saturation. [rad/s] double sigma_g_c;    // Gyroscope noise
//   density. double sigma_bg;     // Initial gyroscope bias. double sigma_a_c;
//   // Accelerometer noise density. double sigma_ba;     // Initial
//   accelerometer bias double sigma_gw_c;   // Gyroscope drift noise density.
//   double sigma_aw_c;   // Accelerometer drift noise density.
//   double tau;          // Reversion time constant of accerometer bias. [s]
//   double g;            // Earth acceleration.
//   vec3_t a0;           // Mean of the prior accelerometer bias.
//   int rate;            // IMU rate in Hz.
// };

} // namespace yac
#endif // YAC_CALIB_PARAMS_HPP
