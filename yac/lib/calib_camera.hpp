#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

/** Reprojection Error */
struct reproj_error_t : ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const camera_geometry_t *cam_geom;
  const int cam_index;
  const int cam_res[2];
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  const vec2_t z;

  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const int cam_index_,
                 const int cam_res_[2],
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_)
      : cam_geom{cam_geom_},
        cam_index{cam_index_},
        cam_res{cam_res_[0], cam_res_[1]},
        tag_id{tag_id_},
        corner_idx{corner_idx_},
        r_FFi{r_FFi_},
        z{z_},
        covar{covar_},
        info{covar.inverse()},
        sqrt_info{info.llt().matrixL().transpose()} {
    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7);  // camera-fiducial relative pose
    block_sizes->push_back(7);  // camera-camera extrinsics
    block_sizes->push_back(8);  // camera parameters
  }

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    const mat4_t T_C0F = tf(params[0]);
    const mat4_t T_C0Ci = tf(params[1]);
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    // -- Transform point from fiducial frame to camera-n
    const mat4_t T_CiC0 = T_C0Ci.inverse();
    const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
    // -- Project point from camera frame to image plane
    vec2_t z_hat;
    bool valid = true;
    if (cam_geom->project(cam_res, cam_params, r_CiFi, z_hat) != 0) {
      valid = false;
    }

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info * (z - z_hat);

    // Jacobians
    const matx_t Jh = cam_geom->project_jacobian(cam_params, r_CiFi);
    const matx_t J_cam_params = cam_geom->params_jacobian(cam_params, r_CiFi);
    const matx_t Jh_weighted = -1 * sqrt_info * Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0F = tf_rot(T_C0F);
          J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * r_FFi);
        }
      }

      // Jacobians w.r.t T_C0Ci
      if (jacobians[1]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0Ci = C_CiC0.transpose();
          const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
          J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
        J.setZero();

        if (valid) {
          J.block(0, 0, 2, 8) = -1 * sqrt_info * J_cam_params;
        }
      }
    }

    return true;
  }
};

/* Calibration View */
struct calib_view_t {
  aprilgrid_t grid;
  const int camera_index;
  const camera_params_t &cam;
  const pose_t &T_C0F;
  const pose_t &T_C0Ci;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_t *> cost_fns;

  calib_view_t(const aprilgrid_t &grid_,
               const int camera_index_,
               const camera_params_t &cam_,
               const pose_t &T_C0F_,
               const pose_t &T_C0Ci_)
    : grid{grid_},
      camera_index{camera_index_},
      cam{cam_},
      T_C0F{T_C0F_},
      T_C0Ci{T_C0Ci_} {}
};

struct calib_camera_t {
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;

  calib_target_t target;
  std::map<int, camera_geometry_t *> cam_geoms;
  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsics_t> cam_exts;
  std::map<timestamp_t, pose_t> poses;

  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, std::map<int, aprilgrid_t>> cam_data;

  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  calib_camera_t() {
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.enable_fast_removal = true;
    problem = new ceres::Problem(prob_options);
  }

  ~calib_camera_t() {
    delete problem;
  }

  void add_calib_target(const calib_target_t &target_) {
    target = target_;
  }

  void add_camera_data(const int cam_idx, const aprilgrids_t &grids) {
    for (const auto &grid : grids) {
      const auto ts = grid.timestamp;
      timestamps.insert(ts);
      cam_data[ts][cam_idx] = grid;
    }
  }

  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const bool fixed=false) {
    // Projection params
    vecx_t proj_params;
    if (proj_model == "pinhole") {
      const double fx = pinhole_focal(cam_res[0], 90);
      const double fy = pinhole_focal(cam_res[0], 90);
      const double cx = cam_res[0] / 2.0;
      const double cy = cam_res[1] / 2.0;
      proj_params.resize(4);
      proj_params << fx, fy, cx, cy;
    } else {
      FATAL("Unsupported [%s]!", proj_model.c_str());
    }

    // Distortion params
    vecx_t dist_params;
    if (dist_model == "radtan4" || dist_model == "equi4") {
      dist_params = zeros(4, 1);
    } else {
      FATAL("Unsupported [%s]!", dist_model.c_str());
    }

    // Camera parameters
    camera_params_t params(0, cam_idx, cam_res,
                           proj_model, dist_model,
                           proj_params, dist_params,
                           fixed);
    cam_params[cam_idx] = params;

    // Camera geometry
    if (proj_model == "pinhole" && dist_model == "radtan4") {
      cam_geoms[cam_idx] = &pinhole_radtan4;
    } else if (proj_model == "pinhole" && dist_model == "equi4") {
      cam_geoms[cam_idx] = &pinhole_equi4;
    }

    // Add parameter to problem
    int params_size = proj_params.size() + dist_params.size();
    problem->AddParameterBlock(cam_params[cam_idx].param.data(), params_size);
  }

  void add_camera_extrinsics(const int cam_idx, const mat4_t &ext=I(4)) {
    cam_exts[cam_idx] = extrinsics_t{cam_idx, ext};
  }

  pose_t &add_pose(const timestamp_t &ts, const mat4_t &T) {
    poses[ts] = pose_t{0, ts, T};
    problem->AddParameterBlock(poses[ts].param.data(), 7);
    problem->SetParameterization(poses[ts].param.data(), &pose_plus);
    return poses[ts];
  }

  void initialize_intrinsics(const int cam_idx) {
    ceres::Problem::Options prob_options;
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.enable_fast_removal = true;

    ceres::Problem problem{prob_options};
    mat2_t covar = I(2);

		// Camera
		const camera_geometry_t *cam_geom = cam_geoms[cam_idx];
		const auto cam_res = cam_params[cam_idx].resolution;
		auto &params = cam_params[cam_idx].param;
    problem.AddParameterBlock(cam_params[cam_idx].param.data(), 8);

    // Extrinsics
    extrinsics_t cam_exts{cam_idx};
    problem.AddParameterBlock(cam_exts.param.data(), 7);
    problem.SetParameterization(cam_exts.param.data(), &pose_plus);
    problem.SetParameterBlockConstant(cam_exts.param.data());

    // Poses
    std::map<timestamp_t, pose_t> poses;

    for (const auto &ts : timestamps) {
      const auto &grid = cam_data[ts][cam_idx];
      if (grid.detected == false) {
        continue;
      }

      // Estimate relative pose
      mat4_t T_CiF_k;
      if (grid.estimate(cam_geom, cam_res, params, T_CiF_k) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      // Add relative pose
      poses[ts] = pose_t{0, ts, T_CiF_k};
      problem.AddParameterBlock(poses[ts].param.data(), 7);
      problem.SetParameterization(poses[ts].param.data(), &pose_plus);

      // Add reprojection factors to problem
      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;
      vec3s_t object_points;
      grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

      for (size_t i = 0; i < tag_ids.size(); i++) {
        const int tag_id = tag_ids[i];
        const int corner_idx = corner_indicies[i];
        const vec2_t z = keypoints[i];
        const vec3_t r_FFi = object_points[i];

        auto cost_fn = new reproj_error_t{cam_geom, cam_idx, cam_res,
                                          tag_id, corner_idx,
                                          r_FFi, z, covar};
        auto res_id = problem.AddResidualBlock(cost_fn,
                                               NULL,
                                               poses[ts].param.data(),
                                               cam_exts.param.data(),
                                               cam_params[cam_idx].param.data());
      }
    }

    // Set solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 30;
    // options.check_gradients = true;

    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;

		print_vector("cam_params", cam_params[cam_idx].param);
		print_vector("cam_exts", cam_exts.param);
  }
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
