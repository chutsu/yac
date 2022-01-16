#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_params.hpp"
#include "calib_data.hpp"

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
             const std::map<int, camera_params_t> &cam_params);

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
  // -- Camera
  const camera_geometry_t *cam_geom;
  const int cam_idx;
  const int cam_res[2];
  // -- Fiducial target
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  // -- Measurement
  const vec2_t z;
  // -- Covariance, information and square-root information
  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  /** Constructor */
  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const int cam_idx_,
                 const int cam_res_[2],
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_);

  /** Evaluate */
  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// CALIB VIEW //////////////////////////////////////////////////////////////////

struct calib_view_t {
  // Problem
  ceres::Problem &problem;
  std::deque<std::shared_ptr<reproj_error_t>> cost_fns;
  std::deque<ceres::ResidualBlockId> res_ids;

  // Data
  aprilgrid_t grid;

  // Parameters
  camera_geometry_t *cam_geom;
  camera_params_t &cam_params;
  pose_t &T_C0F;
  pose_t &T_BCi;
  mat2_t covar;

  /** Constructor */
  calib_view_t(ceres::Problem &problem_,
               aprilgrid_t &grid_,
               pose_t &T_C0F_,
               pose_t &T_BCi_,
               camera_geometry_t *cam_geom_,
               camera_params_t &cam_params_,
               const mat2_t &covar_ = I(2));

  /** Destructor */
  ~calib_view_t() = default;

  /** Calibrate reprojection errors */
  std::vector<double> calculate_reproj_errors() const;
};

// CALIB VIEWS /////////////////////////////////////////////////////////////////

struct calib_views_t {
  ceres::Problem &problem;
  pose_t &extrinsics;
  camera_geometry_t *cam_geom;
  camera_params_t &cam_params;
  std::deque<calib_view_t> views;

  /** Constructor */
  calib_views_t(ceres::Problem &problem_,
                pose_t &extrinsics_,
                camera_geometry_t *cam_geom_,
                camera_params_t &cam_params_);

  /** Return number of views */
  size_t size() const;

  /** Add view */
  void add_view(aprilgrid_t &grid, pose_t &rel_pose);

  /** Calibrate reprojection errors */
  std::vector<double> calculate_reproj_errors() const;
};

// CALIBRATOR //////////////////////////////////////////////////////////////////

/**
 * Initialize camera intrinsics using AprilGrids `grids` observed by the camera,
 * the camera geometry `cam_geom`, and camera parameters `cam_params`.
 */
void initialize_camera(const aprilgrids_t &grids,
                       camera_geometry_t *cam_geom,
                       camera_params_t &cam_params,
                       const bool verbose = false);

/** Camera Calibrator **/
struct calib_camera_t {
  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;

  // State variables
  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsics_t> cam_exts;
  std::map<timestamp_t, pose_t> poses;

  // Data
  std::map<int, camera_geometry_t *> cam_geoms;
  calib_target_t target;
  std::set<timestamp_t> timestamps;
  camera_data_t cam_data;

  // Calib views
  std::map<int, std::map<timestamp_t, std::unique_ptr<calib_view_t>>>
      calib_views;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructor / Destructor
  calib_camera_t();
  ~calib_camera_t();

  int nb_cams() const;
  aprilgrids_t get_cam_data(const int cam_idx) const;
  mat4_t get_camera_extrinsics(const int cam_idx) const;
  void add_calib_target(const calib_target_t &target_);
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
  void add_view(ceres::Problem &problem,
                int cam_idx,
                aprilgrid_t &grid,
                pose_t &rel_pose,
                extrinsics_t &extrinsics,
                camera_geometry_t *cam_geom,
                camera_params_t &cam_params);
  void _initialize_intrinsics();
  void _initialize_extrinsics();
  void _setup_problem();
  void solve();
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
