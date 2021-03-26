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
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;

  aprilgrids_t grids;
  camera_params_t &cam_params;
  std::deque<pose_t> poses;
  mat2_t covar = I(2);

  calib_mono_data_t(const aprilgrids_t &grids_,
                    camera_params_t &cam_params_,
                        const mat2_t &covar_=I(2))
      : grids{grids_}, cam_params{cam_params_}, covar{covar_} {
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.enable_fast_removal = true;
    problem = new ceres::Problem(prob_options);
  }

  ~calib_mono_data_t() {
    delete problem;
  }

  void reset() {
    delete problem;
    problem = new ceres::Problem(prob_options);
    poses.clear();
  }
};

/** Monocular Camera Residual */
template <typename CAMERA_TYPE>
struct calib_mono_residual_t : public ceres::SizedCostFunction<2, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  int tag_id_ = -1;
  int corner_idx_ = -1;
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  mat2_t covar_;
  mat2_t info_;
  mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_mono_residual_t(const int cam_res[2],
                        const int tag_id,
                        const int corner_idx,
                        const vec3_t &r_FFi,
                        const vec2_t &z,
                        const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_idx_{corner_idx},
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
    const matx_t weighted_Jh = -1 * sqrt_info_ * Jh;

    if (jacobians) {
      // Jacobians w.r.t T_CF
      if (jacobians[0]) {
        J_min[0].block(0, 0, 2, 3) = weighted_Jh * I(3);
        J_min[0].block(0, 3, 2, 3) = weighted_Jh * -skew(C_CF * r_FFi_);
        if (valid == false) {
          J_min[0].setZero();
        }
        Eigen::Map<mat_t<2, 7, row_major_t>> J0(jacobians[0]);
        J0.setZero();
        J0.block(0, 0, 2, 6) = J_min[0];

        // // Convert from minimial jacobians to local jacobian
        // lift_pose_jacobian(J_min[0], q_CF, jacobians[0]);
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

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrid_t &grid,
                   const camera_params_t &cam_params,
                   const pose_t &pose,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const auto cam_res = cam_params.resolution;
  const vecx_t proj_params = cam_params.proj_params();
  const vecx_t dist_params = cam_params.dist_params();
  CAMERA_TYPE camera(cam_res, proj_params, dist_params);

  const mat4_t T_CF = pose.tf();
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

    vec2_t z_hat;
    const vec3_t r_CFi = tf_point(T_CF, r_FFi);
    camera.project(r_CFi, z_hat);
    const vec2_t r = z - z_hat;

    errs.push_back(r.norm());
    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const aprilgrids_t &grids,
                   const camera_params_t &cam_params,
                   const std::deque<pose_t> &poses,
                   std::vector<double> &errs) {
  for (size_t i = 0; i < grids.size(); i++) {
    reproj_errors<CAMERA_TYPE>(grids[i], cam_params, poses[i], errs);
  }
}

/* Reprojection Errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_mono_data_t &data, std::vector<double> &errs) {
  const auto cam_params = data.cam_params;
  for (size_t i = 0; i < data.grids.size(); i++) {
    const auto grid = data.grids[i];
    const auto pose = data.poses[i];
    reproj_errors<CAMERA_TYPE>(grid, cam_params, pose, errs);
  }
}

/* Monocular Camera Calibration View */
template <typename CAMERA_TYPE>
struct calib_mono_view_t {
  aprilgrid_t grid;
  camera_params_t *cam_params = nullptr;
  pose_t *T_CF = nullptr;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_mono_residual_t<CAMERA_TYPE> *> cost_fns;

  void errors(std::vector<double> &errs,
              std::vector<std::pair<int, int>> *tags_meta=nullptr) const {
    reproj_errors<CAMERA_TYPE>(grid, *cam_params, *T_CF, errs, tags_meta);
  }

  void errors(std::vector<double> &errs,
              std::vector<std::pair<int, int>> *tags_meta=nullptr) {
    static_cast<const calib_mono_view_t>(*this).errors(errs, tags_meta);
  }
};

/* Monocular Camera Calibration Views */
template <typename CAMERA_TYPE>
using calib_mono_views_t = std::deque<calib_mono_view_t<CAMERA_TYPE>>;

/* Process AprilGrid - Adding it to the calibration problem */
template <typename T>
void process_grid(const aprilgrid_t &grid,
                  const mat2_t &covar,
                  camera_params_t &cam_params,
                  pose_t &pose,
                  ceres::Problem &problem,
                  std::vector<ceres::ResidualBlockId> &res_ids,
                  std::vector<calib_mono_residual_t<T> *> &cost_fns,
                  PoseLocalParameterization &pose_plus) {
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

  problem.SetParameterization(pose.param.data(), &pose_plus);
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename CAMERA_TYPE>
int calib_mono_solve(calib_mono_data_t &data) {
  // Calibration data
  ceres::Problem *problem = data.problem;
  mat2_t &covar = data.covar;
  aprilgrids_t &grids = data.grids;
  camera_params_t &cam_params = data.cam_params;
  std::deque<pose_t> &poses = data.poses;

  // Problem data
  id_t param_id = 0;
  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<calib_mono_residual_t<CAMERA_TYPE> *> cost_fns;

  // Setup camera
  const auto cam_res = cam_params.resolution;
  const vecx_t proj_params = cam_params.proj_params();
  const vecx_t dist_params = cam_params.dist_params();
  const CAMERA_TYPE cam{cam_res, proj_params, dist_params};

  // Drop AprilGrids that are not detected
  auto it = grids.begin();
  while (it != grids.end()) {
    if ((*it).detected == false) {
      it = grids.erase(it);
    } else {
      it++;
    }
  }

  // Build Problem
  for (const auto &grid : grids) {
    // Estimate relative pose
    mat4_t T_CF_k;
    grid.estimate(cam, T_CF_k);

    // Create new pose parameter
    const auto ts = grid.timestamp;
    poses.emplace_back(param_id++, ts, T_CF_k);

    // Add reprojection factors to problem
    process_grid<CAMERA_TYPE>(grid, covar, cam_params,
                              poses.back(), *problem,
                              res_ids, cost_fns,
                              data.pose_plus);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
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

  return 0;
}

/* Filter view */
template <typename CAMERA_TYPE>
static int filter_view(ceres::Problem &problem,
                       calib_mono_view_t<CAMERA_TYPE> &view,
                       double threshold=1.0) {
  auto &grid = view.grid;
  const auto cam = view.cam_params;
  int nb_outliers = 0;

  // Evaluate reprojection errors
  std::vector<double> errs;
  std::vector<std::pair<int, int>> tags_meta;
  view.errors(errs, &tags_meta);

  // Check which AprilGrid tag to remove
  std::set<std::pair<int, int>> outliers;
  for (size_t i = 0; i < tags_meta.size(); i++) {
    auto tag_id = tags_meta[i].first;
    auto corner_idx = tags_meta[i].second;
    if (errs[i] > threshold) {
      outliers.insert({tag_id, corner_idx});
    }
  }

  // Remove residuals that are linked to the tag
  for (const auto &kv : outliers) {
    auto tag_id = kv.first;
    auto corner_idx = kv.second;
    auto res_ids_iter = view.res_ids.begin();
    auto cost_fns_iter = view.cost_fns.begin();

    while (res_ids_iter != view.res_ids.end()) {
      auto res_id = *res_ids_iter;
      auto cost_fn = *cost_fns_iter;

      if (cost_fn->tag_id_ == tag_id && cost_fn->corner_idx_ == corner_idx) {
        // Remove residual from ceres::Problem, tag id and corner_idx from AprilGrid
        problem.RemoveResidualBlock(res_id);
        grid.remove(tag_id, corner_idx);

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
        break;
      }

      // Update iterators
      res_ids_iter++;
      cost_fns_iter++;
    }
  }
}

/* Filter views */
template <typename T>
static int filter_views(ceres::Problem &problem,
                        calib_mono_views_t<T> &views,
                        const double threshold=1.0) {
  int nb_outliers = 0;
  for (auto &view : views) {
    nb_outliers += filter_view(problem, view, threshold);
  }
  return nb_outliers;
}

/* Save Camera Parameters */
static int save_results(const std::string &save_path,
                        const camera_params_t &params,
                        const double rmse,
                        const double mean) {
  // Open results file
  FILE *outfile = fopen(save_path.c_str(), "w");
  if (outfile == NULL) {
    return -1;
  }

  // Save results
  const int img_w = params.resolution[0];
  const int img_h = params.resolution[1];
  const char *proj_model = params.proj_model.c_str();
  const char *dist_model = params.dist_model.c_str();
  const std::string intrinsics = arr2str(params.proj_params().data(), 4);
  const std::string distortion = arr2str(params.dist_params().data(), 4);

  fprintf(outfile, "calib_results:\n");
  fprintf(outfile, "  rms_reproj_error:  %.2f  # [px]\n", rmse);
  fprintf(outfile, "  mean_reproj_error: %.2f  # [px]\n", mean);
  fprintf(outfile, "\n");

  fprintf(outfile, "cam0:\n");
  fprintf(outfile, "  resolution: [%d, %d]\n", img_w, img_h);
  fprintf(outfile, "  proj_model: \"%s\"\n", proj_model);
  fprintf(outfile, "  dist_model: \"%s\"\n", dist_model);
  fprintf(outfile, "  proj_params: %s\n", intrinsics.c_str());
  fprintf(outfile, "  dist_params: %s\n", distortion.c_str());

  // Finsh up
  fclose(outfile);

  return 0;
}

/* Estimate Calibration Covariance */
int calib_covar(ceres::Problem &problem,
                camera_params_t &cam,
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
  std::deque<pose_t> &poses = data.poses;
  auto problem = data.problem;

  // Drop AprilGrids that are not detected
  auto it = grids.begin();
  while (it != grids.end()) {
    if ((*it).detected == false) {
      it = grids.erase(it);
    } else {
      it++;
    }
  }

  // Create random index vector (same length as grids)
  // -- Create indicies vector
  std::vector<int> indicies;
  for (size_t i = 0; i < grids.size(); i++) {
    indicies.push_back(i);
  }
  // -- Randomize the indicies vector
  std::random_shuffle(std::begin(indicies), std::end(indicies));

  // Build Problem
  id_t param_id = 0;
  calib_mono_views_t<CAMERA_TYPE> views;
  double info = -1;
  size_t processed = 0;
  size_t nb_outliers = 0;

  for (const auto &i : indicies) {
    const auto grid = grids[i];
    const auto ts = grid.timestamp;

    // Setup camera
    const auto cam_res = cam_params.resolution;
    const vecx_t proj_params = cam_params.proj_params();
    const vecx_t dist_params = cam_params.dist_params();
    const CAMERA_TYPE cam{cam_res, proj_params, dist_params};

    // Estimate T_CF at ts
    mat4_t T_CF_k;
    grid.estimate(cam, T_CF_k);

    // Create new relative pose T_CF at ts
    poses.emplace_back(param_id++, ts, T_CF_k);

    // Create view
    calib_mono_view_t<CAMERA_TYPE> view;
    view.grid = grid;
    view.cam_params = &cam_params;
    view.T_CF = &poses.back();

    // Add AprilGrid to problem
    process_grid<CAMERA_TYPE>(grid, covar, cam_params,
                                     poses.back(), *problem,
                              view.res_ids, view.cost_fns,
                                      data.pose_plus);
    views.push_back(view);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    if (views.size() > 20) {
      // Filter views
      nb_outliers += filter_views<CAMERA_TYPE>(*problem, views);

      // Evaluate current view
      matx_t covar;
      if (calib_covar(*problem, cam_params, covar) == 0) {
        const double info_k = covar.trace();
        const double diff = (info - info_k) / info;

        if (info < 0) {
          info = info_k;

        } else if (diff > 0.02) {
          info = info_k;
        } else {
          for (const auto &res_id : views.back().res_ids) {
            problem->RemoveResidualBlock(res_id);
          }
          poses.pop_back();
          views.pop_back();
        }
      }
    }

    // Show progress
    std::vector<double> errs;
    for (const auto &view : views) {
      view.errors(errs);
    }

    printf("Processed [%ld / %ld] ", processed++, indicies.size());
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
  ceres::Solve(options, problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Show results
  std::vector<double> errs;
  for (const auto &view : views) {
    view.errors(errs);
  }

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

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
