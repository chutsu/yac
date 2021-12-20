#ifndef YAC_CALIB_VIEWS_HPP
#define YAC_CALIB_VIEWS_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"
#include "camchain.hpp"
#include "reproj_error.hpp"

namespace yac {

// CALIB VIEW //////////////////////////////////////////////////////////////////

struct calib_view_t {
  ceres::Problem &problem;
  aprilgrid_t grid;

  camera_geometry_t *cam_geom;
  camera_params_t &cam_params;
  pose_t &T_C0F;
  pose_t &T_BCi;

  mat2_t covar;
  std::deque<std::shared_ptr<reproj_error_t>> cost_fns;
  std::deque<ceres::ResidualBlockId> res_ids;

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
  void add_view(aprilgrid_t &grid,
                pose_t &rel_pose,
                const mat2_t &covar_ = I(2));

  /** Calibrate reprojection errors */
  std::vector<double> calculate_reproj_errors() const;
};

} //  namespace yac
#endif // YAC_CALIB_VIEWS_HPP
