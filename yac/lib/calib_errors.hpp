#ifndef YAC_CALIB_ERRORS_HPP
#define YAC_CALIB_ERRORS_HPP

#include <mutex>
#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

// CALIB ERROR /////////////////////////////////////////////////////////////////

struct calib_error_t : public ceres::CostFunction {
  // Data
  std::vector<param_t *> param_blocks;
  ceres::LossFunction *loss_fn = nullptr;

  /* Constructor */
  calib_error_t() = default;

  /* Destructor */
  virtual ~calib_error_t() = default;

  /* Get parameter block pointers */
  std::vector<double *> param_block_ptrs();

  /* Evaluate with Minimal jacobians */
  virtual bool EvaluateWithMinimalJacobians(double const *const *params,
                                            double *res,
                                            double **jacs,
                                            double **min_jacs) const = 0;

  /* Evaluate with Minimal jacobians */
  bool Evaluate(double const *const *params, double *res, double **jacs) const;

  /* Evaluate */
  bool eval(double const *const *params,
            double *res,
            double **jacs,
            double **min_jacs) const;

  /* Check jacobians */
  bool check_jacs(const int param_idx,
                  const std::string &jac_name,
                  const double step = 1e-8,
                  const double tol = 1e-4);
};

// POSE ERROR //////////////////////////////////////////////////////////////////

struct pose_error_t : public calib_error_t {
  const pose_t *pose = nullptr;
  const mat4_t pose_meas;

  // Covariance
  mat_t<6, 6> covar;
  mat_t<6, 6> info;
  mat_t<6, 6> sqrt_info;

  /* Constructor */
  pose_error_t(pose_t *pose_, const mat_t<6, 6> &covar_);

  /* Destructor */
  ~pose_error_t() = default;

  /* Evaluate */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const;
};

// REPROJECTION ERROR //////////////////////////////////////////////////////////

/** Reprojection Error */
struct reproj_error_t : public calib_error_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Data
  // -- Parameters
  const camera_geometry_t *cam_geom = nullptr;
  const camera_params_t *cam_params = nullptr;
  const pose_t *T_BCi = nullptr;
  const pose_t *T_C0F = nullptr;
  const fiducial_corner_t *p_FFi = nullptr;
  // -- Measurement
  const vec2_t z;
  // -- Covariance, information and square-root information
  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  /** Constructor */
  reproj_error_t(camera_geometry_t *cam_geom_,
                 camera_params_t *cam_params_,
                 pose_t *T_BCi_,
                 pose_t *T_C0F_,
                 fiducial_corner_t *p_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_);

  /* Destructor */
  ~reproj_error_t() = default;

  /** Get residual */
  int get_residual(vec2_t &z_hat, vec2_t &r) const;

  /** Get residual */
  int get_residual(vec2_t &r) const;

  /** Get reprojection error */
  int get_reproj_error(real_t &error) const;

  /** Evaluate */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const;
};

// FIDUCIAL ERROR //////////////////////////////////////////////////////////////

struct fiducial_error_t : public calib_error_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Data
  const timestamp_t ts_ = 0;
  // -- Parameters
  camera_geometry_t *cam_geom_;
  camera_params_t *cam_params_;
  extrinsics_t *cam_exts_;
  extrinsics_t *imu_exts_;
  fiducial_t *fiducial_;
  pose_t *pose_;
  // -- Measurement
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_{0.0, 0.0};
  const mat4_t T_WF_ = I(4);
  // -- Covariance, information and square-root information
  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  /* Constructor */
  fiducial_error_t(const timestamp_t &ts,
                   camera_geometry_t *cam_geom,
                   camera_params_t *cam_params,
                   extrinsics_t *cam_exts,
                   extrinsics_t *imu_exts,
                   fiducial_t *fiducial,
                   pose_t *pose,
                   const int tag_id,
                   const int corner_idx,
                   const vec3_t &r_FFi,
                   const vec2_t &z,
                   const mat2_t &covar);

  /* Destructor */
  ~fiducial_error_t() = default;

  /** Get residual */
  int get_residual(vec2_t &r) const;

  /** Get reprojection error */
  int get_reproj_error(real_t &error) const;

  /** Evaluate */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const;
};

// INERTIAL ERROR //////////////////////////////////////////////////////////////

#define EST_TIMEDELAY 0

/**
 * Implements a nonlinear IMU factor.
 */
class imu_error_t : public calib_error_t {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  imu_params_t imu_params_;
  imu_data_t imu_data_;
  timestamp_t t0_;
  timestamp_t t1_;

  pose_t *pose_i_ = nullptr;
  sb_params_t *sb_i_ = nullptr;
  pose_t *pose_j_ = nullptr;
  sb_params_t *sb_j_ = nullptr;

  mutable mat4_t T_WS_0_last_;
  mutable mat4_t T_WS_1_last_;
  mutable vec_t<9> sb0_last_;
  mutable vec_t<9> sb1_last_;

  // Constructor
  imu_error_t() = delete;
  imu_error_t(const imu_params_t &imu_params,
              const imu_data_t &imu_data,
              pose_t *pose_i,
              sb_params_t *sb_i,
              pose_t *pose_j,
              sb_params_t *sb_j);

  // Destructor
  virtual ~imu_error_t() = default;

  /* Propagate IMU Measurements */
  static int propagation(const imu_data_t &imu_data,
                         const imu_params_t &imu_params,
                         mat4_t &T_WS,
                         vec_t<9> &sb,
                         const timestamp_t &t_start,
                         const timestamp_t &t_end,
                         mat_t<15, 15> *covariance = 0,
                         mat_t<15, 15> *jacobian = 0);

  /* Redo Preintegration */
  int redoPreintegration(const mat4_t & /*T_WS*/,
                         const vec_t<9> &sb,
                         timestamp_t time,
                         timestamp_t end) const;

  /* Evaluate */
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  /* Evaluate with Minimal Jacobians */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const;

protected:
  // Preintegration stuff. the mutable is a TERRIBLE HACK, but what can I do.
  ///< Protect access of intermediate results.
  mutable std::mutex preintegrationMutex_;

  // increments (initialise with identity)
  mutable quat_t Delta_q_ = quat_t(1, 0, 0, 0);
  mutable mat3_t C_integral_ = mat3_t::Zero();
  mutable mat3_t C_doubleintegral_ = mat3_t::Zero();
  mutable vec3_t acc_integral_ = vec3_t::Zero();
  mutable vec3_t acc_doubleintegral_ = vec3_t::Zero();

  // Cross matrix accumulatrion
  mutable mat3_t cross_ = mat3_t::Zero();

  // Sub-Jacobians
  mutable mat3_t dalpha_db_g_ = mat3_t::Zero();
  mutable mat3_t dv_db_g_ = mat3_t::Zero();
  mutable mat3_t dp_db_g_ = mat3_t::Zero();

  ///< The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double, 15, 15> P_delta_ =
      Eigen::Matrix<double, 15, 15>::Zero();

  ///< Reference biases that are updated when called redoPreintegration.
  mutable vec_t<9> sb_ref_;

  ///< Keeps track of whether redoPreintegration() needs to be called.
  mutable bool redo_ = true;

  ///< Counts the number of preintegrations for statistics.
  mutable int redoCounter_ = 0;

  // information matrix and its square root form
  mutable mat_t<15, 15> information_;
  mutable mat_t<15, 15> squareRootInformation_;
};

// MARGINALIZATION ERROR ///////////////////////////////////////////////////////

class marg_error_t : public calib_error_t {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool marginalized_ = false;

  // Residual blocks and parameters involved for marginalization
  size_t m_ = 0; // Size of params to marginalize
  size_t r_ = 0; // Size of params to remain
  std::vector<calib_error_t *> res_blocks_;
  std::map<param_t *, bool> params_seen_;
  std::vector<param_t *> marg_param_ptrs_;
  std::vector<param_t *> remain_param_ptrs_;
  std::vector<param_t *> remain_pose_param_ptrs_;
  std::vector<param_t *> remain_sb_param_ptrs_;
  std::vector<param_t *> remain_camera_param_ptrs_;
  std::vector<param_t *> remain_extrinsics_ptrs_;
  std::vector<param_t *> remain_fiducial_ptrs_;
  std::unordered_map<param_t *, int> param_blocks_;
  std::unordered_map<param_t *, int> param_index_;

  std::unordered_map<real_t *, vecx_t> x0_; // Linearization point x0
  vecx_t r0_;                               // Linearized residuals at x0
  matx_t J0_;                               // Linearized jacobians at x0

  /* Constructor */
  marg_error_t() = default;

  /* Destructor */
  ~marg_error_t();

  /* Get Residual Size */
  size_t get_residual_size() const;

  /* Get Parameters */
  std::vector<param_t *> get_params();

  /* Get Parameter Pointers */
  std::vector<double *> get_param_ptrs();

  /* Add Cost Function */
  void add(calib_error_t *error);

  /* Form Hessian */
  void form_hessian(matx_t &H, vecx_t &b);

  /* Schurs Complement */
  void schurs_complement(const matx_t &H,
                         const vecx_t &b,
                         const size_t m,
                         const size_t r,
                         matx_t &H_marg,
                         vecx_t &b_marg);

  /* Marginalize */
  ceres::ResidualBlockId marginalize(ceres::Problem *problem,
                                     bool debug = true);

  /* Compute Delta Chi */
  vecx_t compute_delta_chi(double const *const *params) const;

  /* Evaluate */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const;
};

// TYPEDEFS ///////////////////////////////////////////////////////////////////

// clang-format off
using CamIdx2Grids = std::map<int, aprilgrid_t>;
using CamIdx2Geometry = std::map<int, camera_geometry_t *>;
using CamIdx2Parameters = std::map<int, camera_params_t *>;
using CamIdx2Extrinsics = std::map<int, extrinsics_t *>;
using CamIdx2ReprojErrors = std::map<int, std::deque<reproj_error_t *>>;
using CamIdx2ReprojErrorIds = std::map<int, std::deque<ceres::ResidualBlockId>>;
using CamIdx2FiducialErrors = std::map<int, std::deque<fiducial_error_t *>>;
using CamIdx2FiducialErrorIds = std::map<int, std::deque<ceres::ResidualBlockId>>;
// clang-format on

} //  namespace yac
#endif // YAC_CALIB_ERRORS_HPP
