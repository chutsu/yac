#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "calib_mono.hpp"

namespace yac {

/** Stereo camera calibration residual */
template <typename CAMERA>
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
    const CAMERA cam{cam_res_, cam_params};
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
template <typename CAMERA>
struct calib_view_t {
  int camera_index = -1;
  aprilgrid_t grid;
  mat2_t covar;
  pose_t &T_BF;
  pose_t &T_BCi;
  camera_params_t &cam;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_t<CAMERA> *> cost_fns;

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

  ~calib_view_t() {}
};

/* Stereo-Camera Calibration Views */
template <typename CAMERA>
using calib_views_t = std::deque<calib_view_t<CAMERA>>;

template <typename CAMERA>
void save_views(const std::string &save_dir,
                const calib_views_t<CAMERA> &views) {
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
template <typename CAMERA>
static int process_grid(calib_view_t<CAMERA> &view,
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

  if (grid.detected == false) {
    return 0;
  }

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

    const auto cost_fn = new reproj_error_t<CAMERA>{
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

template <typename CAMERA>
static pose_t estimate_pose(const camera_params_t &cam,
                            const aprilgrid_t &grid,
                            const mat4_t &T_BCi,
                            id_t &param_counter) {
  mat4_t T_CiF_k;

  const auto cam_res = cam.resolution;
  const vecx_t proj_params = cam.proj_params();
  const vecx_t dist_params = cam.dist_params();
  const CAMERA cam_geom{cam_res, proj_params, dist_params};
  if (grid.estimate(cam_geom, T_CiF_k) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  const auto ts = grid.timestamp;
  const mat4_t T_BF = T_BCi * T_CiF_k;
  return pose_t{param_counter++, ts, T_BF};
}

/* Calculate residuals */
template <typename CAMERA>
void calib_residuals(const calib_view_t<CAMERA> &view,
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

    const CAMERA cam_geometry(cam.resolution, cam.param);
    const vec3_t r_CiFi = tf_point(T_CiB * T_BF, r_FFi);
    vec2_t z_hat;
    cam_geometry.project(r_CiFi, z_hat);
    r.push_back({z(0) - z_hat(0), z(1) - z_hat(1)});

    if (tags_meta) {
      tags_meta->push_back({tag_id, corner_idx});
    }
  }
}

/* Calculate residuals */
template <typename CAMERA>
void calib_residuals(const calib_views_t<CAMERA> &views,
                     std::vector<std::pair<double, double>> &r,
                     std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  for (auto view : views) {
    calib_residuals(view, r, tags_meta);
  }
}

/* Calculate reprojection errors */
template <typename CAMERA>
void reproj_errors(const calib_view_t<CAMERA> &view,
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

    const CAMERA cam_geometry(cam.resolution, cam.param);
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
template <typename CAMERA>
void reproj_errors(const calib_views_t<CAMERA> &views,
                   std::vector<double> &errs,
                   std::vector<std::pair<int, int>> *tags_meta=nullptr) {
  for (const auto &view : views) {
    reproj_errors(view, errs, tags_meta);
  }
}

/* Estimate Calibration Covariance */
int calib_covar(ceres::Problem &problem,
                camera_params_t &cam0,
                camera_params_t &cam1,
                extrinsics_t &exts,
                matx_t &covar);

/* Show calibration results */
template <typename CAMERA>
static void show_results(const calib_data_t &data) {
  // Show results
  const auto cam0_errs = data.vision_errors.at(0);
  const auto cam1_errs = data.vision_errors.at(1);
  const auto &cam0 = data.cam_params.at(0);
  const auto &cam1 = data.cam_params.at(1);
  const mat4_t T_BC0 = data.cam_exts.at(0).tf();
  const mat4_t T_BC1 = data.cam_exts.at(1).tf();
  const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;

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
int calib_stereo_solve(calib_data_t &data) {
  // Map out data
  ceres::Problem *problem = data.problem;
  ceres::LossFunction *loss = data.loss;
  PoseLocalParameterization &pose_plus = data.pose_plus;
  aprilgrids_t &cam0_grids = data.cam_grids[0];
  aprilgrids_t &cam1_grids = data.cam_grids[1];
  mat2_t &covar = data.covar;
  camera_params_t &cam0 = data.cam_params[0];
  camera_params_t &cam1 = data.cam_params[1];
  extrinsics_t &cam0_exts = data.cam_exts[0];
  extrinsics_t &cam1_exts = data.cam_exts[1];
  std::map<timestamp_t, pose_t> &poses = data.poses;

  // Fix cam0_exts
  cam0_exts.set_tf(I(3), zeros(3, 1));  // Set cam0 to be body frame
  problem->AddParameterBlock(cam0_exts.param.data(), 7);
  problem->SetParameterBlockConstant(cam0_exts.param.data());

  // Process all aprilgrid data
  id_t param_id = 0;
  calib_views_t<T> views0;
  calib_views_t<T> views1;

  // -- cam0
  for (const auto &grid : cam0_grids) {
    // check if grid is detected
    if (grid.detected == false) {
      continue;
    }

    // Estimate T_BF and add to parameters to be estimated
    const auto ts = grid.timestamp;
    const mat4_t T_BC0 = cam0_exts.tf();
    if (poses.count(ts) == 0) {
      poses[ts] = estimate_pose<T>(cam0, grid, T_BC0, param_id);
    }

    // Add reprojection factors to problem
    views0.emplace_back(0, grid, covar, cam0, poses[ts], cam0_exts);
    if (process_grid<T>(views0.back(), *problem, pose_plus, loss) != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }
  // -- cam1
  for (const auto &grid : cam1_grids) {
    // check if grid is detected
    if (grid.detected == false) {
      continue;
    }

    // Estimate T_BF if it does not exist at ts
    const auto ts = grid.timestamp;
    const mat4_t T_BC1 = cam1_exts.tf();
    if (poses.count(ts) == 0) {
      poses[ts] = estimate_pose<T>(cam1, grid, T_BC1, param_id);
    }

    // Add reprojection factors to problem
    views1.emplace_back(1, grid, covar, cam1, poses[ts], cam1_exts);
    if (process_grid<T>(views1.back(), *problem, pose_plus, loss) != 0) {
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

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  // Finish up
  reproj_errors<T>(views0, data.vision_errors.at(0));
  reproj_errors<T>(views1, data.vision_errors.at(1));

  // Show results
  show_results<T>(data);

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

/* Save calibration results */
int save_results(const std::string &save_path, const calib_data_t &data);

} //  namespace yac
#endif // YAC_CALIB_STEREO_HPP
