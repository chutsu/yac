#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "calib_mono.hpp"

namespace yac {

/* Stereo Camera Calibration Data */
struct calib_stereo_data_t {
  mat2_t covar;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  camera_params_t cam0;
  camera_params_t cam1;
  mat4_t T_C1C0;
  mat4s_t T_C0F;
  mat4s_t T_C1F;

  std::vector<double> cam0_errs;
  std::vector<double> cam1_errs;

  calib_stereo_data_t(const mat2_t &covar_,
                      const aprilgrids_t &cam0_grids_,
                      const aprilgrids_t &cam1_grids_,
                      const camera_params_t &cam0_,
                      const camera_params_t &cam1_)
    : covar{covar_},
      cam0_grids{cam0_grids_},
      cam1_grids{cam1_grids_},
      cam0{cam0_},
      cam1{cam1_} {}
};

/** Stereo camera calibration residual */
template <typename CAMERA_TYPE>
struct reproj_error_t : public ceres::SizedCostFunction<2, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_index_ = -1;
  int cam_res_[2] = {0, 0};
  int tag_id_ = -1;
  int corner_idx_ = -1;
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  reproj_error_t(const int cam_index,
                  const int cam_res[2],
                  const int tag_id,
                  const int corner_idx,
                  const vec3_t &r_FFi,
                  const vec2_t &z,
                  const mat2_t &covar)
      : cam_index_{cam_index},
        cam_res_{cam_res[0], cam_res[1]},
        tag_id_{tag_id}, corner_idx_{corner_idx},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    const mat4_t T_BF = tf(params[0]);
    const mat4_t T_BCi = tf(params[1]);
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    bool valid = true;
    // -- Transform point from fiducial frame to camera-n
    const mat4_t T_CiB = T_BCi.inverse();
    const vec3_t r_CiFi = tf_point(T_CiB * T_BF, r_FFi_);
    // -- Project point from camera frame to image plane
    mat_t<2, 3> cam_Jh;
    vec2_t z_hat;
    const CAMERA_TYPE cam{cam_res_, cam_params};
    if (cam.project(r_CiFi, z_hat, cam_Jh) != 0) {
      valid = false;
    }
    const vec2_t p_CiFi{r_CiFi(0) / r_CiFi(2), r_CiFi(1) / r_CiFi(2)};
    const matx_t cam_J_params = cam.J_params(p_CiFi);

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info_ * (z_ - z_hat);

    // Jacobians
    const matx_t Jh_weighted = -1 * sqrt_info_ * cam_Jh;

    if (jacobians) {
      // Jacobians w.r.t T_BF
      if (jacobians[0]) {
        const mat3_t C_CiB = tf_rot(T_CiB);
        const mat3_t C_BF = tf_rot(T_BF);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();
        J.block(0, 0, 2, 3) = Jh_weighted * C_CiB * I(3);
        J.block(0, 3, 2, 3) = Jh_weighted * C_CiB * -skew(C_BF * r_FFi_);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t T_BCi
      if (jacobians[1]) {
        const mat3_t C_CiB = tf_rot(T_CiB);
        const mat3_t C_BCi = C_CiB.transpose();
        const vec3_t r_CiFi = tf_point(T_CiB * T_BF, r_FFi_);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BCi * r_CiFi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
        J.block(0, 0, 2, 8) = -1 * sqrt_info_ * cam_J_params;
        if (valid == false) {
          J.setZero();
        }
      }
    }

    return true;
  }
};

/* Calibration View */
template <typename CAMERA_TYPE>
struct calib_view_t {
  int camera_index = -1;
  aprilgrid_t grid;
  mat2_t covar;
  pose_t &T_BF;
  pose_t &T_BCi;
  camera_params_t &cam;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_t<CAMERA_TYPE> *> cost_fns;

  calib_view_t(const int camera_index_,
               const aprilgrid_t &grid_,
               const mat2_t &covar_,
               camera_params_t &cam_,
               pose_t &T_BF_,
               pose_t &T_BCi_)
    : camera_index{camera_index_},
      grid{grid_},
      covar{covar_},
      cam{cam_},
      T_BF{T_BF_},
      T_BCi{T_BCi_} {}

  ~calib_view_t() {
    // for (const auto &cost_fn : cost_fns) {
    //   delete cost_fn;
    // }
  }

};

/* Stereo-Camera Calibration Views */
template <typename CAMERA_TYPE>
using calib_views_t = std::deque<calib_view_t<CAMERA_TYPE>>;

template <typename CAMERA_TYPE>
void save_views(const std::string &save_dir,
                const calib_views_t<CAMERA_TYPE> &views) {
  auto cam_idx = views[0].camera_index;
  auto save_path = save_dir + "/";
  save_path += "cam" + std::to_string(cam_idx) + "-views.csv";

  FILE *csv = fopen(save_path.c_str(), "w");
  fprintf(csv, "#ts,err_x,err_y,reproj_error,tag_id,corner_idx\n");
  for (const auto &view : views) {
    const auto ts = view.grid.timestamp;

    std::vector<std::pair<double, double>> residuals;
    std::vector<double> reprojection_errors;
    std::vector<std::pair<int, int>> tags_meta;
    calib_residuals(view, residuals, &tags_meta);
    reproj_errors(view, reprojection_errors);

    for (size_t i = 0; i < residuals.size(); i++) {
      const auto err_x = residuals[i].first;
      const auto err_y = residuals[i].second;
      const auto reproj_err = reprojection_errors[i];
      const auto tag_id = tags_meta[i].first;
      const auto corner_idx = tags_meta[i].second;

      fprintf(csv, "%ld,", ts);
      fprintf(csv, "%f,%f,", err_x, err_y);
      fprintf(csv, "%f,", reproj_err);
      fprintf(csv, "%d,%d\n", tag_id, corner_idx);
    }
  }
  fclose(csv);
}

/* Process AprilGrid - Adding it to the calibration problem */
template <typename CAMERA_TYPE>
static int process_grid(calib_view_t<CAMERA_TYPE> &view,
                        ceres::Problem &problem,
                        PoseLocalParameterization &pose_plus,
                        ceres::LossFunction *loss=nullptr) {
  const mat2_t covar = view.covar;
  const int cam_idx = view.camera_index;
  const int *cam_res = view.cam.resolution;
  const auto &grid = view.grid;
  auto &cam = view.cam;
  auto &T_BF = view.T_BF;
  auto &T_BCi = view.T_BCi;

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  view.grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const auto cost_fn = new reproj_error_t<CAMERA_TYPE>{
      cam_idx, cam_res, tag_id, corner_idx, r_FFi, z, covar};
    const auto res_id = problem.AddResidualBlock(cost_fn, // Cost function
                                                 loss,    // Loss function
                                                 T_BF.param.data(),
                                                 T_BCi.param.data(),
                                                 cam.param.data());

    view.res_ids.push_back(res_id);
    view.cost_fns.push_back(cost_fn);
  }
  problem.SetParameterization(T_BF.param.data(), &pose_plus);
  problem.SetParameterization(T_BCi.param.data(), &pose_plus);

  return 0;
}

template <typename CAMERA_TYPE>
static pose_t estimate_relative_pose(const aprilgrid_t &grid,
                                     const camera_params_t &cam_params,
                                     const mat4_t &T_BCi,
                                     id_t &param_counter) {
  const auto ts = grid.timestamp;
  const auto cam_res = cam_params.resolution;
  const vecx_t proj_params = cam_params.proj_params();
  const vecx_t dist_params = cam_params.dist_params();
  const CAMERA_TYPE cam{cam_res, proj_params, dist_params};

  mat4_t T_CiF;
  if (grid.estimate(cam, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  const mat4_t T_BF = T_BCi * T_CiF;
  return pose_t{param_counter++, ts, T_BF};
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void calib_residuals(const calib_view_t<CAMERA_TYPE> &view,
                     std::vector<std::pair<double, double>> &r,
                     std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const auto grid = view.grid;
  const auto cam = view.cam;
  const mat4_t T_BCi = view.T_BCi.tf();
  const mat4_t T_BF = view.T_BF.tf();
  const mat4_t T_CiB = T_BCi.inverse();

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t cam0_keypoints;
  vec2s_t cam1_keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, cam0_keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = cam0_keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const CAMERA_TYPE cam_geometry(cam.resolution, cam.param);
    const vec3_t r_CiFi = tf_point(T_CiB * T_BF, r_FFi);
    vec2_t z_hat;
    cam_geometry.project(r_CiFi, z_hat);
    r.push_back({z(0) - z_hat(0), z(1) - z_hat(1)});

    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_view_t<CAMERA_TYPE> &view,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  const auto grid = view.grid;
  const auto cam = view.cam;
  const mat4_t T_BCi = view.T_BCi.tf();
  const mat4_t T_BF = view.T_BF.tf();
  const mat4_t T_CiB = T_BCi.inverse();

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t cam0_keypoints;
  vec2s_t cam1_keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, cam0_keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    const vec2_t z = cam0_keypoints[i];
    const vec3_t r_FFi = object_points[i];

    const CAMERA_TYPE cam_geometry(cam.resolution, cam.param);
    const vec3_t r_CiFi = tf_point(T_CiB * T_BF, r_FFi);
    vec2_t z_hat;
    cam_geometry.project(r_CiFi, z_hat);

    const vec2_t r{z - z_hat};
    errs.push_back(r.norm());

    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Calculate reprojection errors */
template <typename CAMERA_TYPE>
void reproj_errors(const calib_views_t<CAMERA_TYPE> &views,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  for (const auto &view : views) {
    reproj_errors(view, errs, tags_meta);
  }
}

/* Show calibration results */
template <typename CAMERA_TYPE>
static void show_results(const calib_views_t<CAMERA_TYPE> &cam0_views,
                         const calib_views_t<CAMERA_TYPE> &cam1_views,
                         const mat4_t &T_C1C0) {
  // Show results
  std::vector<double> cam0_errs, cam1_errs;
  reproj_errors<CAMERA_TYPE>(cam0_views, cam0_errs);
  reproj_errors<CAMERA_TYPE>(cam1_views, cam1_errs);

  auto cam0 = cam0_views[0].cam;
  auto cam1 = cam1_views[0].cam;

  printf("Optimization results:\n");
  printf("---------------------\n");
  printf("cam0 reproj_error [px] [rmse: %f, ", rmse(cam0_errs));
  printf("mean: %f, ", mean(cam0_errs));
  printf("median: %f]\n", median(cam0_errs));
  printf("cam1 reproj_error [px] [rmse: %f, ", rmse(cam1_errs));
  printf("mean: %f, ", mean(cam1_errs));
  printf("median: %f]\n", median(cam1_errs));
  printf("\n");
  print_vector("cam0.proj_params", cam0.proj_params());
  print_vector("cam0.dist_params", cam0.dist_params());
  print_vector("cam1.proj_params", cam1.proj_params());
  print_vector("cam1.dist_params", cam1.dist_params());
  printf("\n");
  print_matrix("T_C1C0", T_C1C0);
  printf("\n");
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
template <typename T>
int calib_stereo_solve(const aprilgrids_t &cam0_grids,
                       const aprilgrids_t &cam1_grids,
                       const mat2_t &covar,
                       camera_params_t &cam0,
                       camera_params_t &cam1,
                       mat4_t &T_C1C0) {
  // Stereo camera calibration Setup
  id_t param_id = 0;
  mat4_t T_BC0 = I(4);
  mat4_t T_BC1 = I(4);
  pose_t cam0_exts{param_id++, 0, T_BC0};
  pose_t cam1_exts{param_id++, 1, T_BC1};
  std::map<timestamp_t, pose_t> rel_poses;

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::LossFunction *loss = nullptr;
  // ceres::LossFunction *loss = new ceres::CauchyLoss(1);
  // ceres::LossFunction *loss = new ceres::HuberLoss(1);
  PoseLocalParameterization pose_plus;

  // Process all aprilgrid data
  calib_views_t<T> views0;
  calib_views_t<T> views1;

  // -- cam0
  for (const auto &grid : cam0_grids) {
    // Estimate T_BF and add to parameters to be estimated
    const auto ts = grid.timestamp;
    const mat4_t T_BC0 = cam0_exts.tf();
    rel_poses[ts] = estimate_relative_pose<T>(grid, cam0, T_BC0, param_id);

    // Add reprojection factors to problem
    views0.emplace_back(0, grid, covar, cam0, rel_poses[ts], cam0_exts);
    if (process_grid<T>(views0.back(), problem, pose_plus, loss) != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }
  // -- cam1
  for (const auto &grid : cam1_grids) {
    // Estimate T_BF if it does not exist at ts
    const auto ts = grid.timestamp;
    const mat4_t T_BC1 = cam1_exts.tf();
    if (rel_poses.count(ts) == 0) {
      rel_poses[ts] = estimate_relative_pose<T>(grid, cam1, T_BC1, param_id);
    }

    // Add reprojection factors to problem
    views1.emplace_back(1, grid, covar, cam1, rel_poses[ts], cam1_exts);
    if (process_grid<T>(views1.back(), problem, pose_plus, loss) != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-20;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  // Finish up
  T_BC0 = cam0_exts.tf();
  T_BC1 = cam1_exts.tf();
  T_C1C0 = T_BC1.inverse() * T_BC0;
  if (loss) {
    delete loss;
  }

  // Show results
  show_results<T>(views0, views1, T_C1C0);

  return 0;
}

/* Initialize Stereo-Camera Intrinsics and Extrinsics */
template <typename CAMERA_TYPE>
static void initialize(const aprilgrids_t &grids0,
                       const aprilgrids_t &grids1,
                       const mat2_t &covar,
                       camera_params_t &cam0,
                       camera_params_t &cam1,
                       mat4_t &T_C1C0) {
  // Initialize camera intrinsics
  // -- cam0
  LOG_INFO("Initializing cam0 intrinsics ...");
  calib_mono_data_t cam0_data{grids0, cam0};
  if (calib_mono_solve<CAMERA_TYPE>(cam0_data) != 0) {
    FATAL("Failed to calibrate cam0 intrinsics!");
  }
  // {
  //   calib_mono_data_t data;
  //   data.covar = I(2);
  //   data.cam_params = cam0;
  //   data.grids = grids0;
  //
  //   if (calib_mono_inc_solve<CAMERA_TYPE>(data) != 0) {
  //     FATAL("Failed to calibrate cam0 intrinsics!");
  //   }
  //   cam0 = data.cam_params;
  // }
  // -- cam1
  LOG_INFO("Initializing cam1 intrinsics ...");
  calib_mono_data_t cam1_data{grids1, cam1};
  if (calib_mono_solve<CAMERA_TYPE>(cam1_data) != 0) {
    FATAL("Failed to calibrate cam1 intrinsics!");
  }
  // {
  //   calib_mono_data_t data;
  //   data.covar = I(2);
  //   data.cam_params = cam1;
  //   data.grids = grids1;
  //
  //   if (calib_mono_inc_solve<CAMERA_TYPE>(data) != 0) {
  //     FATAL("Failed to calibrate cam0 intrinsics!");
  //   }
  //   cam1 = data.cam_params;
  // }

  // Initialize stereo-pair extrinsics
  LOG_INFO("Initializing cam0-cam1 extrinsics ...");
	cam0 = cam0_data.cam_params;
	cam1 = cam1_data.cam_params;
  calib_stereo_solve<CAMERA_TYPE>(grids0, grids1, covar, cam0, cam1, T_C1C0);
}

/* Filter view */
template <typename CAMERA_TYPE>
int filter_view(ceres::Problem &problem,
                calib_view_t<CAMERA_TYPE> &view,
                double threshold=1.0) {
  auto &grid = view.grid;
  const auto cam = view.cam;
  int nb_outliers = 0;

  // Evaluate reprojection errors
  std::vector<double> errs;
  std::vector<std::pair<int, int>> tags_meta;
  reproj_errors<CAMERA_TYPE>(view, errs, &tags_meta);

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

  return nb_outliers;
}

/* Filter views */
template <typename CAMERA_TYPE>
int filter_views(ceres::Problem &problem,
                 calib_views_t<CAMERA_TYPE> &views,
                 double threshold=1.0) {
  int nb_outliers = 0;
  for (auto &view : views) {
    nb_outliers += filter_view<CAMERA_TYPE>(problem, view, threshold);
  }

  return nb_outliers;
}

/* Estimate Stereo Camera Covariance */
int calib_stereo_covar(camera_params_t &cam0,
                       camera_params_t &cam1,
                       pose_t &extrinsics,
                       ceres::Problem &problem,
                       matx_t &covar);

/**
 * Find random indicies.
 *
 * Aside from creating a set of random indicies to randomly sample the image
 * frames, the goal is to find a set that yeilds the lowest reprojection error
 * for the first 20 frames. The intuition is to start the incremental solver in
 * a certain state, then add / remove frames that improve the calibration.
 */
template <typename CAMERA_TYPE>
static std::vector<int> find_random_indicies(const int attempts,
                                             const int min_views,
                                             const mat2_t &covar,
                                             const aprilgrids_t &grids0,
                                             const aprilgrids_t &grids1,
                                             camera_params_t &cam0,
                                             camera_params_t &cam1,
                                             pose_t &cam0_exts,
                                             pose_t &cam1_exts,
                                             bool verbose=false) {
  id_t param_counter = 0;
  int best_idx = 0;
  double best_rmse = 0.0;
  std::vector<int> best_indicies;

  const auto cam_res = cam0.resolution;
  const vecx_t proj_params = cam0.proj_params();
  const vecx_t dist_params = cam0.dist_params();
  const CAMERA_TYPE cam0_geom{cam_res, proj_params, dist_params};

  for (int j = 0; j < attempts; j++) {
    // Create random index vector (same length as grids)
    std::vector<int> indicies;
    for (size_t i = 0; i < grids0.size(); i++) indicies.push_back(i);
    std::random_shuffle(std::begin(indicies), std::end(indicies));

    // Views
    calib_views_t<CAMERA_TYPE> views0;
    calib_views_t<CAMERA_TYPE> views1;
    std::deque<pose_t> rel_poses;

    // Evaluate indicies
    for (const auto &i : indicies) {
      const auto grid0 = grids0[i];
      const auto grid1 = grids1[i];
      const auto ts = grids0[i].timestamp;
      if (grid0.timestamp != grid1.timestamp) {
        LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
        FATAL("Bad data! Stopping stereo-calibration!");
      }

      // Estimate T_C0F
      mat4_t T_C0F_;
      grid0.estimate(cam0_geom, T_C0F_);
      rel_poses.emplace_back(param_counter++, ts, T_C0F_);

      // Create view
      views0.emplace_back(0, grid0, covar, cam0, rel_poses.back(), cam0_exts);
      views1.emplace_back(1, grid1, covar, cam1, rel_poses.back(), cam1_exts);

      if (views0.size() > min_views) {
        break;
      }
    }

    std::vector<double> cam0_errs;
    std::vector<double> cam1_errs;
    reproj_errors(views0, cam0_errs);
    reproj_errors(views1, cam1_errs);

    if (verbose) {
      printf("[%d]: ", j);
      printf("cam0: [rmse: %.2f,", rmse(cam0_errs));
      printf(" mean: %.2f]", mean(cam0_errs));
      printf("\t");
      printf("cam1: [rmse: %.2f,", rmse(cam1_errs));
      printf(" mean: %.2f]", mean(cam1_errs));
      printf("\n");
    }

    const double cand_rmse = rmse(cam0_errs) + rmse(cam1_errs);
    if (best_indicies.size() == 0 || cand_rmse < best_rmse) {
      best_idx = j;
      best_rmse = cand_rmse;
      best_indicies = indicies;
    }
  }

  if (verbose) {
    printf("best_idx: %d ", best_idx);
    printf("best_rmse: %f ", best_rmse);
    printf("\n");
  }

  return best_indicies;
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       sigma_vision: 0.5
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25[-]
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 *     cam1:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * This stereo calibrator is a more elabroate version than the standard pipeline.
 * The pipeline is as follows:
 *
 * - Initialize camera intrinsincs by solving standard BA on cam0 and cam1
 *   independently.
 * - Initialize camera extrinsics T_C1C0 using the median of T_C0F and T_C1F,
 *   using the initialized cam0, cam1 intrinsics.
 * - Find random indicies that yields the best `min_views` that yeilds lowest
 *   reprojection error, where `min_views` is a hyper-parameter.
 * - Incrementally solve the stereo-camera intrinsincs and extrinsics
 *   - if views > min_views:
 *     - filter current view whose residal block has a reproj error > threshold
 *     - remove current if uncertainty is not improved by more than 2%
 *   - Stop early if no new frames have been added for N iterations
 * - Final filter all views whose residal block has reproj error > threshold
 * - Final batch optimization
 * - Done
 */
template <typename CAMERA_TYPE>
int calib_stereo_inc_solve(calib_stereo_data_t &data) {
  // Seed random
  srand(time(NULL));

  // Options
  int random_indicies_attempts = 1000;
  size_t filter_min_views = 20;
  double filter_stddev = 3.0;

  // Map out the calibration data
  mat2_t &covar = data.covar;
  aprilgrids_t &grids0 = data.cam0_grids;
  aprilgrids_t &grids1 = data.cam1_grids;
  camera_params_t &cam0 = data.cam0;
  camera_params_t &cam1 = data.cam1;
  mat4_t &T_C1C0 = data.T_C1C0;
  mat4s_t &T_C0F = data.T_C0F;
  mat4s_t &T_C1F = data.T_C1F;

  // Pre-check
  if (grids0.size() != grids1.size()) {
    LOG_ERROR("grids0.size() != grids1.size()!");
    return -1;
  }

  // Initialize stereo intrinsics and extrinsics
  initialize<CAMERA_TYPE>(grids0, grids1, covar, cam0, cam1, T_C1C0);

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;

  // // Lock camera intrinsics
  // problem.AddParameterBlock(cam0.param.data(), 8);
  // problem.AddParameterBlock(cam1.param.data(), 8);
  // problem.SetParameterBlockConstant(cam0.param.data());
  // problem.SetParameterBlockConstant(cam1.param.data());

  // Process all aprilgrid data
  id_t param_counter = 0;
  std::deque<pose_t> rel_poses;
  mat4_t T_BC0 = I(4);
  mat4_t T_BC1 = T_BC0 * T_C1C0.inverse();
  pose_t cam0_exts{param_counter++, 0, T_BC0};
  pose_t cam1_exts{param_counter++, 1, T_BC1};
  problem.AddParameterBlock(cam0_exts.param.data(), 7);
  problem.SetParameterBlockConstant(cam0_exts.param.data());
  calib_views_t<CAMERA_TYPE> cam0_views;
  calib_views_t<CAMERA_TYPE> cam1_views;
  double info = -1;
  size_t processed = 0;
  size_t nb_outliers = 0;

  // Find best indicies that have the best first 20 views
  auto indicies = find_random_indicies<CAMERA_TYPE>(random_indicies_attempts,
                                                    filter_min_views,
                                                    covar, grids0, grids1,
                                                    cam0, cam1,
                                                    cam0_exts,
                                                    cam1_exts);

  // Incremental solve
  bool enable_early_stop = false;
  size_t stale_limit = 30;
  size_t stale_counter = 0;

  for (const auto &i : indicies) {
    size_t outliers = 0;
    const auto grid0 = grids0[i];
    const auto grid1 = grids1[i];
    const auto ts = grids0[i].timestamp;
    if (grid0.timestamp != grid1.timestamp) {
      LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
      FATAL("Bad data! Stopping stereo-calibration!");
    }

    // Estimate T_C0F
    const auto cam_res = cam0.resolution;
    const vecx_t proj_params = cam0.proj_params();
    const vecx_t dist_params = cam0.dist_params();
    const CAMERA_TYPE cam0_geom{cam_res, proj_params, dist_params};
    mat4_t T_C0F_k;
    grid0.estimate(cam0_geom, T_C0F_k);
    rel_poses.emplace_back(param_counter++, ts, T_C0F_k);

    // Create view
    cam0_views.emplace_back(0, grid0, covar, cam0, rel_poses.back(), cam0_exts);
    cam1_views.emplace_back(1, grid1, covar, cam1, rel_poses.back(), cam1_exts);

    // Add view to problem
    process_grid<CAMERA_TYPE>(cam0_views.back(), problem, pose_plus, loss);
    process_grid<CAMERA_TYPE>(cam1_views.back(), problem, pose_plus, loss);

    // Solve
    ceres::Solver::Options options;
    options.max_num_iterations = 20;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Filter last view
    if (processed > filter_min_views) {
      // Obtain cam0 and cam1 reprojection errors so far
      std::vector<double> cam0_reproj_errors;
      std::vector<double> cam1_reproj_errors;
      reproj_errors(cam0_views, cam0_reproj_errors);
      reproj_errors(cam1_views, cam1_reproj_errors);
      // Calculate filter threshold for cam0 and cam1
      const double cam0_threshold = filter_stddev * stddev(cam0_reproj_errors);
      const double cam1_threshold = filter_stddev * stddev(cam1_reproj_errors);

      // Filter last view
      outliers += filter_view(problem, cam0_views.back(), cam0_threshold);
      outliers += filter_view(problem, cam1_views.back(), cam1_threshold);
      nb_outliers += outliers;

      // Evaluate current view
      matx_t covar;
      if (calib_stereo_covar(cam0, cam1, cam1_exts, problem, covar) == 0) {
        const double info_k = covar.trace();
        const double diff = (info - info_k) / info;

        if (info < 0) {
          info = info_k;

        } else if (diff > 0.01) {
          info = info_k;
          stale_counter = 0;

        } else {
          for (const auto &res_id : cam0_views.back().res_ids) {
            problem.RemoveResidualBlock(res_id);
          }
          for (const auto &res_id : cam1_views.back().res_ids) {
            problem.RemoveResidualBlock(res_id);
          }
          cam0_views.pop_back();
          cam1_views.pop_back();
          stale_counter++;
        }
      }
    }

    // Early stop
    if (enable_early_stop && stale_counter >= stale_limit) {
      LOG_INFO("stale_counter >= stale_limit");
      LOG_INFO("stopping early!");
      break;
    }

    // Show progress
    reproj_errors(cam0_views, data.cam0_errs);
    reproj_errors(cam1_views, data.cam1_errs);

    printf("Processed [%ld / %ld]  ", processed++, indicies.size());
    printf("Outliers Removed: %ld ", outliers);
    printf("Batch size: %ld ", cam0_views.size() + cam1_views.size());
    printf("Reprojection Error: ");
    printf("\t");
    printf("cam0: [rmse: %.2f,", rmse(data.cam0_errs));
    printf(" mean: %.2f]", mean(data.cam0_errs));
    printf("\t");
    printf("cam1: [rmse: %.2f,", rmse(data.cam1_errs));
    printf(" mean: %.2f]", mean(data.cam1_errs));
    printf("\n");
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 30;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  // options.check_gradients = true;

  // Final filter before last optimization
  // -- Obtain cam0 and cam1 reprojection errors so far
  std::vector<double> cam0_reproj_errors;
  std::vector<double> cam1_reproj_errors;
  reproj_errors(cam0_views, cam0_reproj_errors);
  reproj_errors(cam1_views, cam1_reproj_errors);
  // -- Calculate filter threshold for cam0 and cam1
  const double cam0_threshold = filter_stddev * stddev(cam0_reproj_errors);
  const double cam1_threshold = filter_stddev * stddev(cam1_reproj_errors);
  // -- Filter
  nb_outliers += filter_views(problem, cam0_views, cam0_threshold);
  nb_outliers += filter_views(problem, cam1_views, cam1_threshold);

  // Solve
  ceres::Solver::Summary summary;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-20;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  // Update data
  T_BC0 = cam0_exts.tf();
  T_BC1 = cam1_exts.tf();
  T_C1C0 = T_BC1.inverse() * T_BC0;
  T_C0F.clear();
  T_C1F.clear();
  for (auto pose : rel_poses) {
    T_C0F.emplace_back(pose.tf());
    T_C1F.emplace_back(T_C1C0 * pose.tf());
  }

  // Show reuslts
  show_results(cam0_views, cam1_views, T_C1C0);
  save_views("/tmp", cam0_views);
  save_views("/tmp", cam1_views);

  // Clean up
  if (loss) {
    delete loss;
  }

  return 0;
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       sigma_vision: 0.5
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25[-]
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 *     cam1:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 */
int calib_stereo_solve(const std::string &config_file);

int save_results(const std::string &save_path,
                 const camera_params_t &cam0,
                 const camera_params_t &cam1,
                 const mat4_t &T_C1C0,
                 const std::vector<double> &cam0_errs,
                 const std::vector<double> &cam1_errs);

} //  namespace yac
#endif // YAC_CALIB_STEREO_HPP
