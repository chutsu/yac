#ifndef YAC_CALIB_MONO_HPP
#define YAC_CALIB_MONO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "ceres_utils.hpp"
#include "calib_params.hpp"

namespace yac {

/* Monocular Camera Calibration Data */
struct calib_mono_data_t {
  mat2_t covar;
  aprilgrids_t grids;
  camera_params_t cam_params;
  mat4s_t T_CF;
};

/** Monocular Camera Residual */
template <typename CAMERA_TYPE>
struct calib_mono_residual_t : public ceres::SizedCostFunction<2, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  int tag_id_ = -1;
  int corner_id = -1;
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  mat2_t covar_;
  mat2_t info_;
  mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_mono_residual_t(const int cam_res[2],
                        const int tag_id,
                        const int corner_id,
                        const vec3_t &r_FFi,
                        const vec2_t &z,
                        const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_id{corner_id},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar_.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(2, 6)); // T_CF
    J_min.push_back(zeros(2, 8)); // cam params
  }

  ~calib_mono_residual_t() {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map camera-fiducial pose
    const mat4_t T_CF = tf(params[0]);
    const mat3_t C_CF = tf_rot(T_CF);
    const quat_t q_CF{C_CF};

    // Map camera params
    Eigen::Map<const vecx_t> cam_params(params[1], 8);

    // Transform and project point to image plane
    const vec3_t r_CFi = tf_point(T_CF, r_FFi_);
    const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
    mat_t<2, 3> Jh;
    matx_t J_params;
    vec2_t z_hat;
    bool valid = true;

    CAMERA_TYPE camera{cam_res_, cam_params};
    if (camera.project(r_CFi, z_hat, Jh) != 0) {
      valid = false;
    }
    J_params = camera.J_params(p);

    // Residuals
    const vec2_t r = sqrt_info_ * (z_ - z_hat);
    residuals[0] = r(0);
    residuals[1] = r(1);

    // Jacobians
    matx_t weighted_Jh = -1 * sqrt_info_ * Jh;

    if (jacobians) {
      // Jacobians w.r.t T_CF
      if (jacobians[0]) {
        J_min[0].block(0, 0, 2, 3) = weighted_Jh * I(3);
        J_min[0].block(0, 3, 2, 3) = weighted_Jh * -skew(C_CF * r_FFi_);
        if (valid == false) {
          J_min[0].setZero();
        }

        // // Convert from minimial jacobians to local jacobian
        // lift_pose_jacobian(J_min[0], q_CF, jacobians[0]);

        Eigen::Map<mat_t<2, 7, row_major_t>> J0(jacobians[0]);
        J0.setZero();
        J0.block(0, 0, 2, 6) = J_min[0];
      }

      // Jacobians w.r.t camera params
      if (jacobians[1]) {
        J_min[1] = -1 * sqrt_info_ * J_params;
        if (valid == false) {
          J_min[1].setZero();
        }

        Eigen::Map<mat_t<2, 8, row_major_t>> J1(jacobians[1]);
        J1 = J_min[1];
      }
    }

    return true;
  }
};

/* Monocular Camera Calibration View */
template <typename CAMERA_TYPE>
struct calib_mono_view_t {
  aprilgrid_t grid;
  camera_params_t *cam_params = nullptr;
  pose_t *T_CF = nullptr;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_mono_residual_t<CAMERA_TYPE> *> cost_fns;
};

/* Monocular Camera Calibration Views */
template <typename CAMERA_TYPE>
using calib_mono_views_t = std::deque<calib_mono_view_t<CAMERA_TYPE>>;

/* Process AprilGrid - Adding it to the calibration problem */
template <typename T>
static void process_grid(const aprilgrid_t &grid,
                         const mat2_t &covar,
                         camera_params_t &cam_params,
                         pose_t &pose,
                         ceres::Problem &problem,
                         std::vector<ceres::ResidualBlockId> &res_ids,
                         std::vector<calib_mono_residual_t<T> *> &cost_fns) {
  const int *cam_res = cam_params.resolution;

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

    auto cost_fn = new calib_mono_residual_t<T>{cam_res, tag_id, corner_idx,
                                                r_FFi, z, covar};
    auto res_id = problem.AddResidualBlock(cost_fn,
                                           NULL,
                                           pose.param.data(),
                                           cam_params.param.data());
    res_ids.push_back(res_id);
    cost_fns.push_back(cost_fn);
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrid_t &grid,
                   const camera_params_t &cam_params,
                   const mat4_t &T_CF,
                   std::vector<double> &residuals,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
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

    vec_t<8> params;
    params << cam_params.proj_params(), cam_params.dist_params();
    CAMERA_TYPE camera(cam_params.resolution, params);

    vec2_t z_hat;
    const vec3_t r_CFi = tf_point(T_CF, r_FFi);
    camera.project(r_CFi, z_hat);
    const vec2_t r = z - z_hat;

    residuals.push_back(r.norm());
    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrid_t &grid,
                   const camera_params_t &cam_params,
                   const pose_t &T_CF,
                   std::vector<double> &residuals,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  reproj_errors<CAMERA_TYPE>(grid, cam_params, T_CF.tf(), residuals, tags_meta);
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrids_t &grids,
                   const camera_params_t &cam_params,
                   const std::deque<pose_t *> &T_CF,
                   std::vector<double> &residuals) {
  for (size_t i = 0; i < grids.size(); i++) {
    reproj_errors<CAMERA_TYPE>(grids[i], cam_params, *T_CF[i], residuals);
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrids_t &grids,
                   const camera_params_t &cam_params,
                   const mat4s_t &T_CF,
                   std::vector<double> &residuals) {
  for (size_t i = 0; i < grids.size(); i++) {
    reproj_errors<CAMERA_TYPE>(grids[i], cam_params, T_CF[i], residuals);
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_mono_views_t<CAMERA_TYPE> &views,
                   const camera_params_t &cam_params,
                   // const std::deque<pose_t *> &T_CF,
                   std::vector<double> &residuals) {
  // for (size_t i = 0; i < views.size(); i++) {
  //   reproj_errors<CAMERA_TYPE>(views[i].grid, cam_params, *T_CF[i], residuals);
  // }
  for (size_t i = 0; i < views.size(); i++) {
    reproj_errors<CAMERA_TYPE>(views[i].grid, cam_params, *(views[i].T_CF), residuals);
  }
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename CAMERA_TYPE>
int calib_mono_solve(const aprilgrids_t &grids,
                     const mat2_t &covar,
                     camera_params_t &cam_params,
                     mat4s_t &T_CF) {
  // Pre-check
  if (T_CF.size() != grids.size()) {
    LOG_ERROR("T_CF.size() != grids.size()!");
    return -1;
  }

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(prob_options);
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  id_t param_id = 0;
  std::deque<pose_t *> poses;
  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_mono_residual_t<CAMERA_TYPE> *> cost_fns;

  for (size_t i = 0; i < grids.size(); i++) {
    const auto ts = grids[i].timestamp;
    pose_t *pose = new pose_t{param_id++, ts, T_CF[i]};
    poses.emplace_back(pose);

    process_grid<CAMERA_TYPE>(grids[i], covar, cam_params,
                              *pose, problem,
                              res_ids, cost_fns);
    problem.SetParameterization(pose->param.data(),
                                &pose_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Show results
  std::vector<double> errs;
  reproj_errors<CAMERA_TYPE>(grids, cam_params, poses, errs);

  printf("Optimization results:\n");
  printf("---------------------\n");
  print_vector("cam.proj_params", cam_params.proj_params());
  print_vector("cam.dist_params", cam_params.dist_params());
  printf("rms reproj error [px]: %f\n", rmse(errs));
  printf("mean reproj error [px]: %f\n", mean(errs));
  printf("\n");

  // Clean up
  T_CF.clear();
  for (auto &pose : poses) {
    T_CF.push_back(pose->tf());
    delete pose;
  }

  return 0;
}

/* Filter views */
template <typename T>
int filter_views(ceres::Problem &problem, calib_mono_views_t<T> &views) {
  int nb_outliers = 0;

  // Iterate over views
  for (auto &view : views) {
    auto &grid = view.grid;
    const auto cam = view.cam_params;
    const auto T_CF = view.T_CF;

    // Evaluate reprojection errors
    std::vector<double> errs;
    std::vector<std::pair<int, int>> tags_meta;
    reproj_errors<T>(grid, *cam, *T_CF, errs, &tags_meta);

    // Check which AprilGrid tag to remove
    std::set<int> remove_tag_ids;
    for (size_t i = 0; i < tags_meta.size(); i++) {
      auto tag_id = tags_meta[i].first;
      if (errs[i] > 1.0) {
        remove_tag_ids.insert(tag_id);
      }
    }

    // Remove residuals that are linked to the tag
    for (auto tag_id : remove_tag_ids) {
      auto res_ids_iter = view.res_ids.begin();
      auto cost_fns_iter = view.cost_fns.begin();

      while (res_ids_iter != view.res_ids.end()) {
        auto res_id = *res_ids_iter;
        auto cost_fn = *cost_fns_iter;

        if (cost_fn->tag_id_ == tag_id) {
          // Remove residual from ceres::Problem and tag id from AprilGrid
          problem.RemoveResidualBlock(res_id);
          grid.remove(tag_id);

          // Erase res_id from view
          {
            auto begin = std::begin(view.res_ids);
            auto end = std::end(view.res_ids);
            view.res_ids.erase(std::remove(begin, end, res_id), end);
          }

          // Erase cost_fn from view
          {
            auto begin = std::begin(view.cost_fns);
            auto end = std::end(view.cost_fns);
            view.cost_fns.erase(std::remove(begin, end, cost_fn), end);
          }

          nb_outliers++;
        }

        // Update iterators
        res_ids_iter++;
        cost_fns_iter++;
      }
    }
  }

  // printf("Removed %d outliers!\n", nb_outliers);
  return nb_outliers;
}

/* Estimate Monocular Camera Covariance */
int calib_mono_covar(const camera_params_t &cam_params,
                     ceres::Problem &problem,
                     matx_t &covar);

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename CAMERA_TYPE>
int calib_mono_inc_solve(calib_mono_data_t &data) {
  // Calibration data
  const mat2_t &covar = data.covar;
  aprilgrids_t &grids = data.grids;
  camera_params_t &cam_params = data.cam_params;

  // Create random index vector (same length as grids)
  // -- Create indicies vector
  std::vector<int> indicies;
  for (size_t i = 0; i < grids.size(); i++) {
    indicies.push_back(i);
  }
  // -- Randomize the indicies vector
  std::random_shuffle(std::begin(indicies), std::end(indicies));

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(prob_options);
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  id_t param_id = 0;
  std::deque<pose_t *> poses;
  calib_mono_views_t<CAMERA_TYPE> views;
  double info = -1;
  size_t counter = 0;
  size_t nb_outliers = 0;

  for (const auto &i : indicies) {
    const auto grid = grids[i];
    const auto ts = grid.timestamp;

    // Estimate T_CF at ts
    mat4_t rel_pose;
    grid.estimate(cam_params.proj_params(), cam_params.dist_params(), rel_pose);

    // Create new relative pose T_CF at ts
    auto pose = new pose_t{param_id++, ts, rel_pose};
    poses.emplace_back(pose);

    // Create view
    calib_mono_view_t<CAMERA_TYPE> view;
    view.grid = grid;
    view.cam_params = &cam_params;
    view.T_CF = pose;

    // Add AprilGrid to problem
    process_grid<CAMERA_TYPE>(grid, covar, cam_params, *pose, problem,
                              view.res_ids, view.cost_fns);
    problem.SetParameterization(pose->param.data(),
                                &pose_parameterization);
    views.push_back(view);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 3;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    // Filter views
    nb_outliers += filter_views<CAMERA_TYPE>(problem, views);

    // Evaluate current view
    matx_t covar;
    if (calib_mono_covar(cam_params, problem, covar) == 0) {
      const double info_k = covar.trace();
      const double diff = (info - info_k) / info;

      if (info < 0) {
        info = info_k;

      } else if (diff > 0.02) {
        info = info_k;
      } else {
        for (const auto &res_id : views.back().res_ids) {
          problem.RemoveResidualBlock(res_id);
        }
        poses.pop_back();
        views.pop_back();
      }
    }

    // Show progress
    std::vector<double> errs;
    reproj_errors<CAMERA_TYPE>(views, cam_params, errs);
    printf("Processed [%ld / %ld] ", counter++, indicies.size());
    printf("Reprojection Error: ");
    printf("[rmse: %.2f,", rmse(errs));
    printf(" mean: %.2f]", mean(errs));
    printf("\n");
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

  // Show results
  std::vector<double> errs;
  reproj_errors<CAMERA_TYPE>(views, cam_params, errs);

  printf("Optimization results:\n");
  printf("---------------------\n");
  printf("nb_views: %ld\n", views.size());
  printf("nb_points: %ld\n", errs.size());
  printf("nb_outliers: %ld\n", nb_outliers);
  printf("reproj_error [px]: ");
  printf("[rmse: %f", rmse(errs));
  printf(" mean: %f", mean(errs));
  printf(" median: %f]\n", median(errs));
  printf("\n");
  print_vector("cam.proj_params", cam_params.proj_params());
  print_vector("cam.dist_params", cam_params.dist_params());
  printf("\n");

  // Clean up
  data.T_CF.clear();
  for (auto &pose : poses) {
    data.T_CF.push_back(pose->tf());
    delete pose;
  }

  return 0;
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target. This function assumes that the path to `config_file`
 * is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       sigma_vision: 0.5
 *       imshow: true
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.085           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const std::string &config_file);

// /** Cost function Spec **/
// struct cost_func_spec_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//   const int frame_idx = 0;
//   const std::vector<const param_t *> params;
//   size_t nb_res = 0;
//   size_t nb_params = 0;
//
//   bool status = false;
//   vecx_t r;
//   matxs_t J;
//
//   cost_func_spec_t(const int frame_idx_,
//                    const pose_t *pose_,
//                    const camera_params_t *cam_params_,
//                    const vec2_t &z,
//                    const vec3_t &obj_pt)
//       : frame_idx{frame_idx_},
//         params{pose_, cam_params_},
//         nb_res{2},
//         nb_params{2} {
//     // Instancitate cost function
//     calib_mono_residual_t cost_fn{*cam_params_, z, obj_pt};
//
//     // Allocate memory for residual vector
//     double *r_ = (double *) malloc(sizeof(double) * nb_res);
//
//     // Allocate memory for jacobians
//     double **J_ = (double **) malloc(sizeof(double *) * nb_params);
//     for (size_t i = 0; i < nb_params; i++) {
//       J_[i] = (double *) malloc(sizeof(double) * nb_res * params[i]->global_size);
//     }
//
//     // Evaluate
//     std::vector<const double *> cost_params;
//     for (const auto &param : params) {
//       cost_params.push_back(param->param.data());
//     }
//     status = cost_fn.Evaluate(cost_params.data(), r_, J_);
//
//     // Record results
//     r.resize(nb_res);
//     for (size_t i = 0; i < nb_res; i++) {
//       r(i) = r_[i];
//     }
//     for (size_t i = 0; i < nb_params; i++) {
//       J.push_back(cost_fn.J_min[i]);
//     }
//
//     // Free residual
//     if (r_) {
//       free(r_);
//     }
//
//     // Free Jacobians
//     if (J_) {
//       for (size_t i = 0; i < nb_params; i++) {
//         free(J_[i]);
//       }
//       free(J_);
//     }
//   }
//
//   cost_func_spec_t(const int frame_idx_, camera_params_t *cam_params_)
//       : frame_idx{frame_idx_}, params{cam_params_}, nb_res{8}, nb_params{1} {
//     const matx_t covar = 0.01 * I(8);
//     camera_params_factor_t cam_factor(0, covar, cam_params_);
//     cam_factor.eval();
//     r = cam_factor.residuals;
//     J = cam_factor.jacobians;
//   }
//
//   cost_func_spec_t(const int frame_idx_, pose_t *pose_)
//       : frame_idx{frame_idx_}, params{pose_}, nb_res{6}, nb_params{1} {
//     const matx_t covar = 0.01 * I(6);
//     pose_factor_t pose_factor(0, covar, pose_);
//     pose_factor.eval();
//     r = pose_factor.residuals;
//     J = pose_factor.jacobians;
//   }
// };

// /** Monocular Camera Calibration Covariance Estimation **/
// struct calib_mono_covar_est_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//   int frame_idx = 0;
//   bool pose_prior_set = false;
//
//   std::deque<cost_func_spec_t *> cost_specs;
//   camera_params_t *cam_params = nullptr;
//   std::deque<pose_t *> poses;
//
//   calib_mono_covar_est_t(const camera_params_t &params_,
//                          const aprilgrids_t &grids) {
//     const int cam_idx = params_.cam_index;
//     const int cam_res[2] = {params_.resolution[0], params_.resolution[1]};
//     const std::string &proj_model = params_.proj_model;
//     const std::string &dist_model = params_.dist_model;
//     const vecx_t &proj_params = params_.proj_params();
//     const vecx_t &dist_params = params_.dist_params();
//     cam_params = new camera_params_t{0, cam_idx, cam_res,
//                                      proj_model, dist_model,
//                                      proj_params, dist_params};
//
//     // Add camera params prior
//     const auto cam_prior = new cost_func_spec_t(frame_idx, cam_params);
//     cost_specs.push_back(cam_prior);
//
//     // for (size_t i = 0; i < grids.size(); i++) {
//     for (size_t i = 0; i < 10; i++) {
//       add(grids[i]);
//     }
//   }
//
//   virtual ~calib_mono_covar_est_t() {
//     for (auto &spec : cost_specs) {
//       delete spec;
//     }
//
//     delete cam_params;
//     for (auto &pose : poses) {
//       delete pose;
//     }
//   }
//
//   void add(const aprilgrid_t &grid) {
//     // Add new pose
//     auto T_CF = grid.T_CF;
//     const auto pose = new pose_t{frame_idx, frame_idx, grid.T_CF};
//     poses.push_back(pose);
//
//     // Add pose prior if not already added
//     if (pose_prior_set == false) {
//       const auto pose_prior = new cost_func_spec_t(frame_idx, pose);
//       cost_specs.push_back(pose_prior);
//       pose_prior_set = true;
//     }
//
//     // Add reprojection residuals
//     for (const auto &tag_id : grid.ids) {
//       // Get keypoints
//       vec2s_t keypoints;
//       if (aprilgrid_keypoints(grid, tag_id, keypoints) != 0) {
//         FATAL("Failed to get AprilGrid keypoints!");
//       }
//
//       // Get object points
//       vec3s_t object_points;
//       if (aprilgrid_object_points(grid, tag_id, object_points) != 0) {
//         FATAL("Failed to calculate AprilGrid object points!");
//       }
//
//       // Form residual block
//       for (size_t i = 0; i < 4; i++) {
//         auto &kp = keypoints[i];
//         auto &obj_pt = object_points[i];
//         auto spec = new cost_func_spec_t{frame_idx, pose, cam_params, kp, obj_pt};
//         cost_specs.push_back(spec);
//       }
//     }
//
//     frame_idx++;
//   }
//
//   void remove_last() {
//     // Remove cost specs that belong to the last frame index
//     while (cost_specs.back()->frame_idx == (frame_idx-1)) {
//       auto last_spec = cost_specs.back();
//       cost_specs.pop_back();
//       delete last_spec;
//     }
//
//     // Remove last pose that belong to the last frame index
//     auto last_pose = poses.back();
//     poses.pop_back();
//     delete last_pose;
//
//     // Decrement frame index
//     frame_idx--;
//   }
//
//   int estimate(matx_t &covar) {
//     std::unordered_map<std::string, size_t> param_cs;
//     param_cs["pose_t"] = 0;
//     param_cs["camera_params_t"] = frame_idx * 6;
//
//     // Assign param global index
//     size_t params_size = 0;
//     std::vector<int> factor_ok;
//     std::unordered_map<const param_t *, size_t> param_index;
//
//     for (const auto &spec : cost_specs) {
//       for (size_t i = 0; i < spec->nb_params; i++) {
//         const auto param = spec->params[i];
//         const auto param_type = param->type;
//         const auto min_param_size = param->local_size;
//         if (param_index.count(param) > 0) {
//           continue; // Skip this param
//         }
//
//         param_index.insert({param, param_cs[param_type]});
//         param_cs[param_type] += min_param_size;
//         params_size += min_param_size;
//       }
//     }
//
//     // Form Hessian
//     const size_t H_size = frame_idx * 6 + 8;
//     matx_t H = zeros(H_size, H_size);
//
//     for (const auto &spec : cost_specs) {
//       // Form Hessian H
//       for (size_t i = 0; i < spec->nb_params; i++) {
//         const auto &param_i = spec->params[i];
//         const auto idx_i = param_index[param_i];
//         const auto size_i = param_i->local_size;
//         const matx_t &J_i = spec->J[i];
//
//         for (size_t j = i; j < spec->nb_params; j++) {
//           const auto &param_j = spec->params[j];
//           const auto idx_j = param_index[param_j];
//           const auto size_j = param_j->local_size;
//           const matx_t &J_j = spec->J[j];
//
//           if (i == j) {  // Diagonal
//             H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
//           } else {  // Off-diagonal
//             H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
//             H.block(idx_j, idx_i, size_j, size_i) =
//               H.block(idx_i, idx_j, size_i, size_j).transpose();
//           }
//         }
//       }
//     }
//
//     // Recover covariance matrix
//     // covar = pinv(H);
//     Eigen::FullPivLU<Eigen::MatrixXd> LU(H);
//     if (LU.rank() != H.rows()) { // Check full rank
//       return -1;
//     } else {
//       covar = H.llt().solve(I(H.rows()));
//       // covar = pinv(H);
//     }
//
//     return 0;
//   }
// };

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
