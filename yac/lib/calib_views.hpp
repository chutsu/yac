#ifndef YAC_CALIB_VIEW_HPP
#define YAC_CALIB_VIEW_HPP

#include <ceres/ceres.h>
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "camchain.hpp"
#include "reproj_error.hpp"

namespace yac {

struct calib_view_t {
  ceres::Problem &problem;
  aprilgrid_t grid;

  camera_geometry_t *cam_geom;
  camera_params_t &cam_params;
  pose_t &T_C0F;
  pose_t &T_C0Ci;

  mat2_t covar;
  std::deque<std::shared_ptr<reproj_error_t>> cost_fns;
  std::deque<ceres::ResidualBlockId> res_ids;

  calib_view_t(ceres::Problem &problem_,
               aprilgrid_t &grid_,
               pose_t &T_C0F_,
               pose_t &T_C0Ci_,
               camera_geometry_t *cam_geom_,
               camera_params_t &cam_params_,
               const mat2_t &covar_ = I(2));
  ~calib_view_t() = default;
  std::vector<double> calculate_reproj_errors() const;
};

struct camera_views_t {
  ceres::Problem &problem;
  pose_t &extrinsics;
  camera_geometry_t *cam_geom;
  camera_params_t &cam_params;
  std::deque<calib_view_t> views;

  camera_views_t(ceres::Problem &problem_,
                 pose_t &extrinsics_,
                 camera_geometry_t *cam_geom_,
                 camera_params_t &cam_params_);

  size_t size() const;
  void add_view(aprilgrid_t &grid,
                pose_t &rel_pose,
                const mat2_t &covar_ = I(2));
  std::vector<double> calculate_reproj_errors() const;
};

calib_view_t::calib_view_t(ceres::Problem &problem_,
                           aprilgrid_t &grid_,
                           pose_t &T_C0F_,
                           pose_t &T_C0Ci_,
                           camera_geometry_t *cam_geom_,
                           camera_params_t &cam_params_,
                           const mat2_t &covar_)
    : problem{problem_}, grid{grid_}, T_C0F{T_C0F_}, T_C0Ci{T_C0Ci_},
      cam_geom{cam_geom_}, cam_params{cam_params_}, covar{covar_} {
  // Get AprilGrid measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t kps;
  vec3s_t pts;
  grid.get_measurements(tag_ids, corner_idxs, kps, pts);

  // Add reprojection errors
  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_idxs[i];
    const vec2_t z = kps[i];
    const vec3_t r_FFi = pts[i];
    auto cost_fn = std::make_shared<reproj_error_t>(cam_geom,
                                                    cam_params.cam_index,
                                                    cam_params.resolution,
                                                    tag_id,
                                                    corner_idx,
                                                    r_FFi,
                                                    z,
                                                    covar);
    auto res_id = problem.AddResidualBlock(cost_fn.get(),
                                           NULL,
                                           T_C0F.data(),
                                           T_C0Ci.data(),
                                           cam_params.data());
    cost_fns.push_back(cost_fn);
    res_ids.push_back(res_id);
  }
}

std::vector<double> calib_view_t::calculate_reproj_errors() const {
  // Estimate relative pose T_C0F
  const auto intrinsics = cam_params.param;
  const auto res = cam_params.resolution;
  mat4_t T_CiF;
  if (grid.estimate(cam_geom, res, intrinsics, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  // Get AprilGrid measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  vec2s_t kps;
  vec3s_t pts;
  grid.get_measurements(tag_ids, corner_idxs, kps, pts);

  // Calculate reprojection errors
  std::vector<double> reproj_errors;

  for (size_t i = 0; i < tag_ids.size(); i++) {
    // const int tag_id = tag_ids[i];
    // const int corner_idx = corner_idxs[i];
    const vec2_t z = kps[i];
    const vec3_t r_FFi = pts[i];
    const vec3_t r_CiFi = tf_point(T_CiF, r_FFi);

    vec2_t z_hat;
    cam_geom->project(res, intrinsics, r_CiFi, z_hat);
    reproj_errors.push_back((z - z_hat).norm());
  }

  return reproj_errors;
}

camera_views_t::camera_views_t(ceres::Problem &problem_,
                               pose_t &extrinsics_,
                               camera_geometry_t *cam_geom_,
                               camera_params_t &cam_params_)
    : problem{problem_}, extrinsics{extrinsics_}, cam_geom{cam_geom_},
      cam_params{cam_params_} {}

size_t camera_views_t::size() const { return views.size(); }

void camera_views_t::add_view(aprilgrid_t &grid,
                              pose_t &rel_pose,
                              const mat2_t &covar_) {
  views.emplace_back(problem, grid, rel_pose, extrinsics, cam_geom, cam_params);
}

std::vector<double> camera_views_t::calculate_reproj_errors() const {
  std::vector<double> reproj_errors;
  for (auto &view : views) {
    auto r = view.calculate_reproj_errors();
    reproj_errors.insert(reproj_errors.end(), r.begin(), r.end());
  }
  return reproj_errors;
}

} //  namespace yac
#endif // YAC_CALIB_VIEW_HPP
