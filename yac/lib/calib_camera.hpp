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
      // valid = false;
    }

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info * (z - z_hat);

    // Jacobians
    const mat_t<2, 3> Jh = cam_geom->project_jacobian(cam_params, r_CiFi);
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
          J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0 * I(3);
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
          J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0 * I(3);
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
  calib_data_t &data;
  ceres::Problem *problem;

  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsics_t> cam_exts;
  std::map<timestamp_t, pose_t> poses;

  calib_camera_t(calib_data_t &data_)
      : data{data_}, problem{data_.problem} {
    const mat2_t covar = I(2);

    // data.cam_params[0].param << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
    // print_vector("cam_params", data.cam_params[0].param);

    std::vector<double> errors;

    for (const auto &ts : data.timestamps) {
      for (const auto &pair : data.get_grids(ts)) {
        const auto cam_idx = pair.first;
        const auto grid = pair.second;

        auto cam_geom = data.cam_geoms[cam_idx];
        auto &cam_params = data.cam_params[cam_idx];
        const auto cam_res = cam_params.resolution;
        auto &cam_exts = data.cam_exts[cam_idx];

        if (grid.detected == false) {
          continue;
        }

        // Estimate relative pose
        mat4_t T_CiF_k;
        {
          const camera_geometry_t *cam_geom = data.cam_geoms[cam_idx];
          const auto cam_res = data.cam_params[cam_idx].resolution;
          const auto cam_params = data.cam_params[cam_idx].param;
          if (grid.estimate(cam_geom, cam_res, cam_params, T_CiF_k) != 0) {
            FATAL("Failed to estimate relative pose!");
          }
        }

        // Create new pose parameter
        const auto ts = grid.timestamp;
        auto &rel_pose = data.add_pose(ts, T_CiF_k);

        if (rel_pose.tf().isApprox(T_CiF_k) == false) {
          printf("NOT THE SAME!\n");
        }

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
          auto res_id = problem->AddResidualBlock(cost_fn,
                                                  NULL,
                                                  rel_pose.param.data(),
                                                  cam_exts.param.data(),
                                                  cam_params.param.data());

          std::vector<double *> params = {
            rel_pose.param.data(),
            cam_exts.param.data(),
            cam_params.param.data(),
          };
          vec2_t r;
          cost_fn->Evaluate(params.data(), r.data(), nullptr);
          errors.push_back(r.norm());

          // print_vector("r", r);
          // print_matrix("rel_pose", rel_pose.tf());
          // print_matrix("cam_exts", cam_exts.tf());
          // print_vector("cam_params", cam_params.param);
          // return;
          // res_ids.push_back(res_id);
          // cost_fns.push_back(cost_fn);
        }
      }
    }

    printf("reproj_errors: [%f, %f, %f] (rms, mean, median)\n",
           rmse(errors), mean(errors), median(errors));
  }

  std::vector<double> reproj_errors() {
    std::vector<double> errors;

    for (const auto &ts : data.timestamps) {
      for (const auto &pair : data.get_grids(ts)) {
        const auto cam_idx = pair.first;
        const auto grid = pair.second;

        if (grid.detected == false) {
          continue;
        }

        const auto cam_geom = data.cam_geoms[cam_idx];
        const auto &cam_params = data.cam_params[cam_idx];
        const auto cam_res = cam_params.resolution;

        // Estimate relative pose
        // mat4_t T_CiF;
        // grid.estimate(cam_geom, cam_res, cam_params.param, T_CiF);

        const mat4_t T_CiF = data.poses[ts].tf();
        const mat4_t T_C0Ci = data.cam_exts[cam_idx].tf();

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
          const vec3_t r_CiFi = tf_point(T_CiF, r_FFi);

          vec2_t z_hat;
          cam_geom->project(cam_res, cam_params.param, r_CiFi, z_hat);

          vec2_t r = z - z_hat;
          errors.push_back(r.norm());;

          // print_vector("r", r);
          // print_matrix("rel_pose", data.poses[ts].tf());
          // print_matrix("cam_exts", data.cam_exts[cam_idx].tf());
          // print_vector("cam_params", cam_params.param);
          // return errors;
        }
      }
    }

    return errors;
  }

  ~calib_camera_t() {}

  // static mat4_t estimate_pose(const camera_params_t &cam,
  //                             const aprilgrid_t &grid,
  //                             const mat4_t &T_C0Ci=I(4)) {
  //   mat4_t T_CiF_k;
  //   const int *cam_res = cam.resolution;
  //   const vecx_t proj_params = cam.proj_params();
  //   const vecx_t dist_params = cam.dist_params();
  //   const C cam_geom{cam_res, proj_params, dist_params};
  //   if (grid.estimate(cam_geom, T_CiF_k) != 0) {
  //     FATAL("Failed to estimate relative pose!");
  //   }
  //
  //   const mat4_t T_C0F = T_C0Ci * T_CiF_k;
  //   return T_C0F;
  // }

  void solve() {
    // Set solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 100;
    // options.function_tolerance = 1e-12;
    // options.gradient_tolerance = 1e-12;
    // options.parameter_tolerance = 1e-20;

    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << std::endl;

    printf("cam0:\n");
    print_vector("  proj_params", data.cam_params[0].proj_params());
    print_vector("  dist_params", data.cam_params[0].dist_params());
    printf("\n");
    print_matrix("T_C0C0", data.cam_exts[0].tf());
    printf("\n");

    // std::cout << "cam1:" << std::endl;
    // std::cout << "  " << data.cam_params[1].proj_params().transpose() << std::endl;
    // std::cout << "  " << data.cam_params[1].dist_params().transpose() << std::endl;
    // std::cout << data.cam_exts[1].tf() << std::endl;

    const auto errors = reproj_errors();
    printf("reproj_errors: [%f, %f, %f] (rms, mean, median)\n",
           rmse(errors), mean(errors), median(errors));
  }
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
