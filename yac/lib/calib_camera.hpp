#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_params.hpp"
#include "calib_data.hpp"
#include "calib_nbv.hpp"

namespace yac {

// CAMCHAIN ///////////////////////////////////////////////////////////////////j

/**
 * Camera Chain Query Tool
 */
struct camchain_t {
  std::map<int, std::vector<int>> adj_list;  // CameraChain adjacency list
  std::map<int, std::map<int, mat4_t>> exts; // Extrinsics

  /** Constructor */
  camchain_t(const camera_data_t &cam_data,
             const std::map<int, camera_geometry_t *> &cam_geoms,
             const std::map<int, camera_params_t *> &cam_params);

  void _get_aprilgrids(const timestamp_t &ts,
                       const camera_data_t &cam_data,
                       std::vector<int> &cam_indicies,
                       std::vector<aprilgrid_t> &grids) const;

  /** Insert camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  void insert(const int cam_i, const int cam_j, const mat4_t &T_CiCj);

  /** Check if camera extrinsics `T_CiCj` between `cam_i` and `cam_j` exists */
  bool contains(const int cam_i, const int cam_j) const;

  /** Find camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  int find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const;

  /** Clear camchain */
  void clear();
};

// REPROJECTION ERROR //////////////////////////////////////////////////////////

/** Reprojection Error */
struct reproj_error_t : public ceres::CostFunction {
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

  // For debugging
  mutable matx_t J0_min;
  mutable matx_t J1_min;
  mutable matx_t J2_min;
  mutable matx_t J3_min;

  /** Constructor */
  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const camera_params_t *cam_params_,
                 const pose_t *T_BCi_,
                 const pose_t *T_C0F_,
                 const fiducial_corner_t *p_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_);

  /** Get residual */
  int get_residual(vec2_t &z_hat, vec2_t &r) const;

  /** Get residual */
  int get_residual(vec2_t &r) const;

  /** Get reprojection error */
  int get_reproj_error(real_t &error) const;

  /** Evaluate */
  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// CALIB VIEW //////////////////////////////////////////////////////////////////

struct calib_view_t {
  // Problem
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  std::deque<std::shared_ptr<reproj_error_t>> res_fns;
  std::deque<ceres::ResidualBlockId> res_ids;

  // Parameters
  fiducial_corners_t *corners = nullptr;
  camera_geometry_t *cam_geom = nullptr;
  camera_params_t *cam_params = nullptr;
  pose_t *T_BCi = nullptr;
  pose_t *T_C0F = nullptr;
  mat2_t covar;

  // Data
  const aprilgrid_t grid;

  calib_view_t(ceres::Problem *problem_,
               ceres::LossFunction *loss_,
               fiducial_corners_t *corners_,
               camera_geometry_t *cam_geom_,
               camera_params_t *cam_params_,
               pose_t *T_BCi_,
               pose_t *T_C0F_,
               const aprilgrid_t &grid_,
               const mat2_t &covar_ = I(2));
  ~calib_view_t() = default;

  std::vector<real_t> get_reproj_errors() const;
  int filter_view(const real_t outlier_threshold);
};

// Typedefs
using calib_views_t = std::map<timestamp_t, std::unique_ptr<calib_view_t>>;
using camera_calib_views_t = std::map<int, calib_views_t>;

// CALIBRATOR //////////////////////////////////////////////////////////////////

/**
 * Initialize camera intrinsics using AprilGrids `grids` observed by the camera,
 * the camera geometry `cam_geom`, and camera parameters `cam_params`.
 */
void initialize_camera(const calib_target_t &calib_target,
                       const aprilgrids_t &grids,
                       camera_geometry_t *cam_geom,
                       camera_params_t *cam_params,
                       const bool verbose = false);

/** Camera Calibrator **/
struct calib_camera_t {
  // Settings
  bool enable_nbv = true;
  bool enable_nbv_filter = false;
  bool enable_outlier_rejection = true;
  int min_nbv_views = 5;
  real_t outlier_threshold = 3.0;
  real_t info_gain_threshold = 0.5;

  // Data
  calib_target_t calib_target;
  std::map<int, camera_geometry_t *> cam_geoms;
  std::set<timestamp_t> timestamps;
  camera_data_t cam_data;
  real_t calib_info_k = 0;

  // Calib views
  camera_calib_views_t calib_views;

  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;

  // State variables
  fiducial_corners_t corners{calib_target};
  std::map<int, camera_params_t *> cam_params;
  std::map<int, extrinsics_t *> cam_exts;
  std::map<timestamp_t, pose_t *> poses;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructor / Destructor
  calib_camera_t() = delete;
  calib_camera_t(const calib_target_t &calib_target_);
  ~calib_camera_t();

  int nb_cams() const;
  aprilgrids_t get_cam_data(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  void add_camera_data(const int cam_idx, const aprilgrids_t &grids);
  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const bool fixed = false);
  void add_camera_extrinsics(const int cam_idx,
                             const mat4_t &ext = I(4),
                             const bool fixed = false);
  void add_pose(const int cam_idx,
                const aprilgrid_t &grid,
                const bool fixed = false);
  void add_view(const aprilgrid_t &grid,
                ceres::Problem *problem,
                ceres::LossFunction *loss,
                int cam_idx,
                camera_geometry_t *cam_geom,
                camera_params_t *cam_params,
                extrinsics_t *cam_exts,
                pose_t *rel_pose);
  void remove_view(const timestamp_t ts, ceres::Problem *problem);

  void _initialize_intrinsics();
  void _initialize_extrinsics();
  void _setup_problem();
  int _filter_views();
  int _filter_view(const timestamp_t ts);

  std::vector<real_t> get_all_reproj_errors();
  std::map<int, std::vector<real_t>> get_reproj_errors();

  int recover_calib_covar(matx_t &calib_covar);
  void solve();
  void show_results();
  int save_results(const std::string &save_path);
  int save_stats(const std::string &save_path);
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
