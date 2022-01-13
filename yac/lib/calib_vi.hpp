#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <mutex>

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_target.hpp"
#include "calib_params.hpp"

namespace yac {

using CameraGrids = std::map<int, aprilgrid_t>;

// POSE ERROR //////////////////////////////////////////////////////////////////

struct pose_error_t : public ceres::SizedCostFunction<6, 7> {
  pose_t pose_meas_;
  matx_t covar_;
  matx_t info_;
  matx_t sqrt_info_;

  pose_error_t(const pose_t &pose, const matx_t &covar);
  ~pose_error_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// FIDUCIAL ERROR //////////////////////////////////////////////////////////////

struct fiducial_error_t : public ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts_ = 0;
  const camera_geometry_t *cam_geom_;
  const int cam_idx_;
  const int cam_res_[2] = {0, 0};
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_{0.0, 0.0};
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  fiducial_error_t(const timestamp_t &ts,
                   const camera_geometry_t *cam_geom,
                   const int cam_idx,
                   const int cam_res[2],
                   const int tag_id,
                   const int corner_idx,
                   const vec3_t &r_FFi,
                   const vec2_t &z,
                   const mat4_t &T_WF,
                   const mat2_t &covar);
  ~fiducial_error_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// FIDUCIAL WITH TIME DELAY ERROR //////////////////////////////////////////////

#if FIDUCIAL_PARAMS_SIZE == 2
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 2, 7, 7, 7, 8, 1> {
#elif FIDUCIAL_PARAMS_SIZE == 3
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 3, 7, 7, 7, 8, 1> {
#elif FIDUCIAL_PARAMS_SIZE == 7
struct fiducial_td_error_t
    : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 8, 1> {
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts_i_ = 0;
  const timestamp_t ts_j_ = 0;
  const camera_geometry_t *cam_geom_;
  const int cam_idx_;
  const int cam_res_[2] = {0, 0};
  const int tag_id_ = -1;
  const int corner_idx_ = -1;
  const vec3_t r_FFi_{0.0, 0.0, 0.0};
  const vec2_t z_i_{0.0, 0.0};
  const vec2_t z_j_{0.0, 0.0};
  const vec2_t v_ij_{0.0, 0.0}; // pixel velocity: (z_j - z_i) / dt
  const mat4_t T_WF_ = I(4);

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  fiducial_td_error_t(const timestamp_t &ts_i,
                      const timestamp_t &ts_j,
                      const camera_geometry_t *cam_geom,
                      const int cam_idx,
                      const int cam_res[2],
                      const int tag_id,
                      const int corner_idx,
                      const vec3_t &r_FFi,
                      const vec2_t &z_i,
                      const vec2_t &z_j,
                      const mat4_t &T_WF,
                      const mat2_t &covar);
  ~fiducial_td_error_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// INERTIAL ERROR //////////////////////////////////////////////////////////////

#define EST_TIMEDELAY 0

/**
 * Implements a nonlinear IMU factor.
 */
class ImuError : public ::ceres::SizedCostFunction<15, 7, 9, 7, 9> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  size_t state_id = 0;
  timestamp_t imu_t0 = 0;
  timestamp_t imu_t1 = 0;
  size_t pose0_id = 0;
  size_t pose1_id = 0;
  size_t state0_id = 0;
  size_t state1_id = 0;
  size_t timedelay_id = 0;

  mutable mat4_t T_WS_0_last_;
  mutable mat4_t T_WS_1_last_;
  mutable vec_t<9> sb0_last_;
  mutable vec_t<9> sb1_last_;

  ///< The number of residuals
  typedef Eigen::Matrix<double, 15, 15> covariance_t;
  typedef covariance_t information_t;
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;
  typedef Eigen::Matrix<double, 15, 7> jacobian0_t;
  typedef Eigen::Matrix<double, 15, 9> jacobian1_t;

  ImuError() = default;
  ImuError(const imu_data_t &imu_data,
           const imu_params_t &imu_params,
           const timestamp_t &t0,
           const timestamp_t &t1);
  virtual ~ImuError() = default;

  static int propagation(const imu_data_t &imu_data,
                         const imu_params_t &imu_params,
                         mat4_t &T_WS,
                         vec_t<9> &sb,
                         const timestamp_t &t_start,
                         const timestamp_t &t_end,
                         covariance_t *covariance = 0,
                         jacobian_t *jacobian = 0);

  int redoPreintegration(const mat4_t & /*T_WS*/,
                         const vec_t<9> &sb,
                         timestamp_t time,
                         timestamp_t end) const;

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;

  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *residuals,
                                    double **jacobians,
                                    double **jacobiansMinimal) const;

  imu_params_t imu_params_;
  imu_data_t imu_data_;
  timestamp_t t0_;
  timestamp_t t1_;

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
  mutable information_t information_;
  mutable information_t squareRootInformation_;
};

// VISUAL INERTIAL CALIBRATOR //////////////////////////////////////////////////

struct calib_vi_view_t {
  // Data
  timestamp_t ts;
  CameraGrids grids;

  // Parameters (internal)
  pose_t pose;
  sb_params_t sb;

  // Parameters (external)
  std::map<int, camera_geometry_t *> &cam_geoms;
  std::map<int, camera_params_t *> &cam_params;
  std::map<int, extrinsics_t *> &cam_exts;
  extrinsics_t *imu_exts = nullptr;
  fiducial_t *fiducial = nullptr;

  // Problem
  ceres::Problem *problem = nullptr;
  std::map<int, std::deque<ceres::ResidualBlockId>> fiducial_error_ids;
  std::map<int, std::deque<fiducial_error_t>> fiducial_errors;
  ceres::ResidualBlockId imu_error_id;
  ImuError imu_errors;

  calib_vi_view_t() = default;

  calib_vi_view_t(const timestamp_t ts_,
                  ceres::Problem *problem_,
                  const CameraGrids &grids_,
                  const mat4_t &T_WS,
                  std::map<int, camera_geometry_t *> &cam_geoms_,
                  std::map<int, camera_params_t *> &cam_params_,
                  std::map<int, extrinsics_t *> &cam_exts_,
                  extrinsics_t *imu_exts_,
                  fiducial_t *fiducial_)
      : ts{ts_}, grids{grids_}, pose{ts_, T_WS}, cam_geoms{cam_geoms_},
        cam_params{cam_params_}, cam_exts{cam_exts_}, imu_exts{imu_exts_},
        fiducial{fiducial_}, problem{problem_} {

    // Add fiducial errors
    const mat4_t T_WF = fiducial->estimate();
    const mat2_t covar = I(2);

    for (const auto &[cam_idx, grid] : grids) {
      // Get AprilGrid measurements
      std::vector<int> tag_ids;
      std::vector<int> corner_idxs;
      vec2s_t kps;
      vec3s_t pts;
      grid.get_measurements(tag_ids, corner_idxs, kps, pts);

      // Add residuals to problem
      for (size_t i = 0; i < tag_ids.size(); i++) {
        const int tag_id = tag_ids[i];
        const int corner_idx = corner_idxs[i];
        const vec2_t z = kps[i];
        const vec3_t r_FFi = pts[i];

        // Form residual
        fiducial_errors[cam_idx].emplace_back(ts,
                                              cam_geoms[cam_idx],
                                              cam_idx,
                                              cam_params[cam_idx]->resolution,
                                              tag_id,
                                              corner_idx,
                                              r_FFi,
                                              z,
                                              T_WF,
                                              covar);

        // Add to problem
        auto error_id =
            problem->AddResidualBlock(&fiducial_errors[cam_idx].back(),
                                      NULL,
                                      fiducial->param.data(),
                                      pose.param.data(),
                                      imu_exts->param.data(),
                                      cam_exts[cam_idx]->param.data(),
                                      cam_params[cam_idx]->param.data());
        fiducial_error_ids[cam_idx].push_back(error_id);
      }
    }
  }

  ~calib_vi_view_t() = default;
};

struct calib_vi_t {
  // Settings
  double sigma_vision = 1.0;
  int batch_max_iter = 30;
  bool enable_outlier_rejection = true;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // State-Variables
  std::map<int, const camera_geometry_t *> cam_geoms;
  std::map<int, camera_params_t *> cam_params;
  std::map<int, extrinsics_t *> cam_exts;
  extrinsics_t *imu_exts;
  fiducial_t *fiducial;
  time_delay_t *time_delay;

  // Data
  std::map<int, std::deque<aprilgrid_t>> grid_buf;
  CameraGrids prev_grids;
  imu_params_t imu_params;
  imu_data_t imu_buf;
  bool initialized = false;

  // Optimization
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;
  std::deque<calib_vi_view_t> calib_views;

  // Constructor
  calib_vi_t();
  ~calib_vi_t();

  void add_imu(const imu_params_t &imu_params_,
               const mat4_t &T_BS,
               const double td = 0.0,
               const bool fix_extrinsics = false,
               const bool fix_time_delay = false);
  void add_camera(const int cam_idx,
                  const int resolution[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const vecx_t &proj_params,
                  const vecx_t &dist_params,
                  const mat4_t &T_BCi = I(4),
                  const bool fix_params = false,
                  const bool fix_extrinsics = false);

  int nb_cams();
  veci2_t get_cam_resolution(const int cam_idx);
  vecx_t get_cam_params(const int cam_idx);
  mat4_t get_cam_extrinsics(const int cam_idx);
  mat4_t get_imu_extrinsics();
  mat4_t get_sensor_pose(const int pose_index);
  mat4_t get_fiducial_pose();

  mat4_t estimate_sensor_pose(const CameraGrids &grids);
  void initialize(const timestamp_t &ts,
                  const CameraGrids &grids,
                  imu_data_t &imu_buf);
  void add_view(const CameraGrids &grids);
  void add_measurement(const int cam_idx, const aprilgrid_t &grid);
  void add_measurement(const timestamp_t imu_ts,
                       const vec3_t &a_m,
                       const vec3_t &w_m);

  // void solve(bool verbose = true);
  // void show_results();
  // int recover_calib_covar(matx_t &calib_covar);

  int save_results(const std::string &save_path);
  // void save_poses(const std::string &save_path);
  // void save_speed_biases(const std::string &save_path);
  // void save_cameras(const std::string &save_path);
  // void save_cam_extrinsics(const std::string &save_path);
  // // void save_imu_extrinsics(const std::string &save_path);
  // void save();
};

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
