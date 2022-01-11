#ifndef YAC_CALIB_MOCAP_HPP
#define YAC_CALIB_MOCAP_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_mono.hpp"

namespace yac {

static void lerp_body_poses(const aprilgrids_t &grids,
                            const timestamps_t &body_timestamps,
                            const mat4s_t &body_poses,
                            aprilgrids_t &lerped_grids,
                            mat4s_t &lerped_poses,
                            timestamp_t ts_offset = 0) {
  // Make sure AprilGrids are between body poses else we can't lerp poses
  timestamps_t grid_timestamps;
  for (const auto &grid : grids) {
    if (grid.timestamp > body_timestamps.front() &&
        grid.timestamp < body_timestamps.back()) {
      lerped_grids.push_back(grid);
      grid_timestamps.push_back(grid.timestamp);
    }
  }

  // Lerp body poses using AprilGrid timestamps
  assert(body_poses.size() == body_timestamps.size());
  assert(body_timestamps.front() < grid_timestamps.front());
  timestamp_t t0 = 0;
  mat4_t pose0 = I(4);
  timestamp_t t1 = 0;
  mat4_t pose1 = I(4);

  size_t grid_idx = 0;
  for (size_t i = 0; i < body_timestamps.size(); i++) {
    // Make sure we're not going out of bounds
    if (grid_idx > (grid_timestamps.size() - 1)) {
      break;
    }

    // Get time now and desired lerp time
    const auto t_now = body_timestamps[i] + ts_offset;
    const auto t_lerp = grid_timestamps[grid_idx];

    // Update t0
    if (t_now < t_lerp) {
      t0 = t_now;
      pose0 = body_poses[i];
    }

    // Update t1
    if (t_now > t_lerp) {
      // Lerp
      t1 = t_now;
      pose1 = body_poses[i];
      const auto pose = lerp_pose(t0, pose0, t1, pose1, t_lerp);
      lerped_poses.push_back(pose);
      grid_idx++;

      // Reset
      t0 = t_now;
      pose0 = body_poses[i];
      t1 = 0;
      pose1 = I(4);
    }
  }
}

/** Mocap-Camera Calibration data */
struct calib_mocap_data_t {
  bool fix_intrinsics = false;
  bool fix_mocap_poses = false;
  bool fix_fiducial_pose = false;

  std::string data_path;
  std::string results_fpath;
  double sigma_vision = 1.0;
  bool imshow = true;

  std::string cam0_path;
  std::string grid0_path;
  std::string body0_csv_path;
  std::string target0_csv_path;

  calib_target_t calib_target;
  mat2_t covar;
  aprilgrids_t grids;
  camera_params_t cam0;
  std::vector<pose_t> T_WM;
  pose_t T_MC0;
  pose_t T_WF;

  calib_mocap_data_t(const std::string &config_file) {
    id_t param_id = 0;

    // Load calib config file
    vec2_t resolution{0.0, 0.0};
    std::string proj_model;
    std::string dist_model;

    config_t config{config_file};
    parse(config, "settings.fix_intrinsics", fix_intrinsics);
    parse(config, "settings.fix_mocap_poses", fix_mocap_poses);
    parse(config, "settings.fix_fiducial_pose", fix_fiducial_pose);
    parse(config, "settings.data_path", data_path);
    parse(config, "settings.sigma_vision", sigma_vision);
    parse(config, "settings.imshow", imshow);
    parse(config, "cam0.resolution", resolution);
    parse(config, "cam0.proj_model", proj_model);
    parse(config, "cam0.dist_model", dist_model);

    // Setup paths
    results_fpath = paths_join(data_path, "calib_results.yaml");
    cam0_path = data_path + "/cam0/data";
    grid0_path = data_path + "/grid0/cam0/data";
    body0_csv_path = data_path + "/body0/data.csv";
    target0_csv_path = data_path + "/target0/data.csv";

    // Load calibration target
    if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
      FATAL("Failed to load calib target in [%s]!", config_file.c_str());
    }

    // Set covariance matrix
    covar = I(2) * (1 / pow(sigma_vision, 2));

    // Setup AprilGrid detector
    aprilgrid_detector_t detector(calib_target.tag_rows,
                                  calib_target.tag_cols,
                                  calib_target.tag_size,
                                  calib_target.tag_spacing);
    std::vector<std::string> image_paths;
    if (list_dir(cam0_path, image_paths) != 0) {
      FATAL("Failed to traverse dir [%s]!", cam0_path.c_str());
    }
    std::sort(image_paths.begin(), image_paths.end());

    // Detect AprilGrids
    aprilgrids_t cam0_grids;
    for (const auto &image_path : image_paths) {
      const auto ts = std::stoull(parse_fname(image_path));
      const auto image = cv::imread(paths_join(cam0_path, image_path));
      const auto grid = detector.detect(ts, image);
      grid.save(grid0_path + "/" + std::to_string(ts) + ".csv");
      cam0_grids.push_back(grid);
    }

    // Load camera params
    if (yaml_has_key(config, "cam0.proj_params") == false) {
      // Perform intrinsics calibration
      LOG_INFO("Camera parameters unknown!");
      LOG_INFO("Calibrating camera intrinsics!");

      // -- Initialize camera parameters
      const int cam_idx = 0;
      const int cam_res[2] = {(int)resolution[0], (int)resolution[1]};
      this->cam0 = camera_params_t{param_id++,
                                   cam_idx,
                                   cam_res,
                                   proj_model,
                                   dist_model,
                                   4,
                                   4};
      this->cam0.initialize(cam0_grids);

      // -- Calibrate camera intrinsics
      int retval = 0;
      calib_mono_data_t data{cam0_grids, this->cam0};
      if (proj_model == "pinhole" && dist_model == "radtan4") {
        calib_mono_solve<pinhole_radtan4_t>(data);
      } else if (proj_model == "pinhole" && dist_model == "equi4") {
        calib_mono_solve<pinhole_equi4_t>(data);
      } else {
        FATAL("Unsupported [%s-%s]!", proj_model.c_str(), dist_model.c_str());
      }
      if (retval != 0) {
        FATAL("Failed to calibrate [cam0] intrinsics!");
      }

    } else {
      // Camera parameters known - proceed
      vecx_t proj_params;
      vecx_t dist_params;
      parse(config, "cam0.proj_params", proj_params);
      parse(config, "cam0.dist_params", dist_params);

      const int cam_idx = 0;
      const int cam_res[2] = {(int)resolution[0], (int)resolution[1]};
      this->cam0 = camera_params_t{param_id++,
                                   cam_idx,
                                   cam_res,
                                   proj_model,
                                   dist_model,
                                   proj_params,
                                   dist_params};
    }

    // Load dataset
    // -- April Grid
    aprilgrids_t grids_raw = load_aprilgrids(grid0_path);
    // -- Mocap marker pose
    timestamps_t body_timestamps;
    mat4s_t body_poses;
    load_poses(body0_csv_path, body_timestamps, body_poses);
    // -- Synchronize aprilgrids and body poses
    mat4s_t marker_poses;
    lerp_body_poses(grids_raw,
                    body_timestamps,
                    body_poses,
                    this->grids,
                    marker_poses);
    // -- Fiducial target pose
    const mat4_t fiducial_pose = load_pose(target0_csv_path);
    this->T_WF = pose_t(param_id++, 0, fiducial_pose);
    // -- Marker poses
    for (size_t i = 0; i < this->grids.size(); i++) {
      const auto ts = this->grids[i].timestamp;
      const mat4_t marker_pose = marker_poses[i];
      this->T_WM.emplace_back(param_id++, ts, marker_pose);
    }
    // -- Mocap marker to camera transform
    // const vec3_t euler{-90.0, 0.0, -90.0};
    const vec3_t euler{-180.0, 0.0, -90.0};
    const mat3_t C = euler321(deg2rad(euler));
    const mat4_t marker_cam_pose = tf(C, zeros(3, 1));
    this->T_MC0 = pose_t(param_id++, 0, marker_cam_pose);
  }
};

/* MOCAP marker residual */
template <typename CAMERA_TYPE>
struct calib_mocap_residual_t : public ceres::SizedCostFunction<2, 7, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  calib_mocap_residual_t(const int cam_res[2],
                         const vec3_t &r_FFi,
                         const vec2_t &z,
                         const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]}, r_FFi_{r_FFi}, z_{z}, covar_{covar},
        info_{covar.inverse()}, sqrt_info_{info_.llt().matrixL().transpose()} {
  }

  ~calib_mocap_residual_t() {
  }

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const {
    // Map optimization variables to Eigen
    const mat4_t T_WF = tf(params[0]);
    const mat4_t T_WM = tf(params[1]);
    const mat4_t T_MC0 = tf(params[2]);
    Eigen::Map<const vecx_t> cam0_params(params[3], 8);

    // Transform and project point to image plane
    const mat4_t T_MW = T_WM.inverse();
    const mat4_t T_C0M = T_MC0.inverse();
    const vec3_t r_MFi = tf_point(T_MW * T_WF, r_FFi_);
    const vec3_t r_C0Fi = tf_point(T_C0M * T_MW * T_WF, r_FFi_);
    const vec2_t p{r_C0Fi(0) / r_C0Fi(2), r_C0Fi(1) / r_C0Fi(2)};
    mat_t<2, 3> Jh;
    matx_t J_params;
    vec2_t z_hat;
    bool valid = true;

    CAMERA_TYPE camera{cam_res_, cam0_params};
    if (camera.project(r_C0Fi, z_hat, Jh) != 0) {
      valid = false;
    }
    J_params = camera.J_params(p);

    // Residual
    const vec2_t r = sqrt_info_ * (z_ - z_hat);
    residuals[0] = r(0);
    residuals[1] = r(1);

    // Jacobians
    const matx_t Jh_weighted = sqrt_info_ * Jh;
    const mat3_t C_C0W = tf_rot(T_C0M * T_MW);
    const mat3_t C_MC0 = tf_rot(T_MC0);
    const mat3_t C_C0M = C_MC0.transpose();

    if (jacobians) {
      // Jacobians w.r.t T_WF
      if (jacobians[0]) {
        const mat3_t C_WF = tf_rot(T_WF);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_C0W * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_C0W * -skew(C_WF * r_FFi_);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t T_WM
      if (jacobians[1]) {
        const mat3_t C_WM = tf_rot(T_WM);

        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0W * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0W * -skew(C_WM * r_MFi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t T_MC0
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
        J.setZero();
        J.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_C0M * I(3);
        J.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_C0M * -skew(C_MC0 * r_C0Fi);
        if (valid == false) {
          J.setZero();
        }
      }

      // Jacobians w.r.t camera params
      if (jacobians[3]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
        J = -1 * sqrt_info_ * J_params;
        if (valid == false) {
          J.setZero();
        }
      }
    }

    return true;
  }
};

template <typename T>
static void process_aprilgrid(const size_t frame_idx,
                              const mat2_t &covar,
                              calib_mocap_data_t &data,
                              ceres::Problem &problem,
                              std::vector<ceres::ResidualBlockId> &block_ids) {
  const int *cam_res = data.cam0.resolution;
  const auto &grid = data.grids[frame_idx];

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    // const int tag_id = tag_ids[i];
    // const int corner_idx = corner_indicies[i];
    const vec2_t z = keypoints[i];
    const vec3_t r_FFi = object_points[i];

    auto cost_fn = new calib_mocap_residual_t<T>{cam_res, r_FFi, z, covar};
    auto block_id = problem.AddResidualBlock(cost_fn,
                                             NULL,
                                             data.T_WF.param.data(),
                                             data.T_WM[frame_idx].param.data(),
                                             data.T_MC0.param.data(),
                                             data.cam0.param.data());
    block_ids.push_back(block_id);
  }
}

template <typename CAMERA_TYPE>
static void show_results(const calib_mocap_data_t &data) {
  // Calibration metrics
  std::deque<pose_t> poses;
  const mat4_t T_WF = data.T_WF.tf();
  const mat4_t T_C0M = data.T_MC0.tf().inverse();
  for (size_t i = 0; i < data.grids.size(); i++) {
    const mat4_t T_MW = data.T_WM[i].tf().inverse();
    const mat4_t T_C0F = T_C0M * T_MW * T_WF;
    poses.emplace_back(0, 0, T_C0F);
  }

  // Show results
  std::vector<double> errs;
  reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);

  printf("\n");
  printf("Optimization results:\n");
  printf("---------------------\n");
  printf("nb_points: %ld\n", errs.size());
  printf("reproj_error [px]: ");
  printf("[rmse: %f", rmse(errs));
  printf(" mean: %f", mean(errs));
  printf(" median: %f]\n", median(errs));
  printf("\n");
  print_vector("cam.proj_params", data.cam0.proj_params());
  print_vector("cam.dist_params", data.cam0.dist_params());
  printf("\n");
  print_matrix("T_WF", data.T_WF.tf());
  print_matrix("T_WM", data.T_WM[0].tf());
  print_matrix("T_MC0", data.T_MC0.tf());

  const auto r_MC = tf_trans(data.T_MC0.tf());
  const auto q_MC = tf_quat(data.T_MC0.tf());
  printf("r_MC: %f, %f, %f\n", r_MC(0), r_MC(1), r_MC(2));
  printf("q_MC (x, y, z, w): %f, %f, %f, %f\n",
         q_MC.x(),
         q_MC.y(),
         q_MC.z(),
         q_MC.w());
}

template <typename CAMERA_TYPE>
static void save_results(const calib_mocap_data_t &data,
                         const std::string &output_path) {
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         output_path.c_str());
  const aprilgrid_t grid = data.grids[0];
  const auto cam = data.cam0;
  const mat4_t T_WF = data.T_WF.tf();
  const mat4_t T_MC0 = data.T_MC0.tf();
  // const mat4_t T_C0M = T_MC0.inverse();

  // double err_min = *std::min_element(errs.begin(), errs.end());
  // double err_max = *std::max_element(errs.begin(), errs.end());
  // double err_median = median(errs);

  // Save calibration results to yaml file
  {
    FILE *fp = fopen(output_path.c_str(), "w");

    // // Calibration metrics
    // fprintf(fp, "calib_results:\n");
    // fprintf(fp, "  cam0.rms_reproj_error:    %f  # [px]\n", err_rmse);
    // fprintf(fp, "  cam0.mean_reproj_error:   %f  # [px]\n", err_mean);
    // fprintf(fp, "  cam0.median_reproj_error: %f  # [px]\n", err_median);
    // fprintf(fp, "  cam0.min_reproj_error:    %f  # [px]\n", err_min);
    // fprintf(fp, "  cam0.max_reproj_error:    %f  # [px]\n", err_max);
    // fprintf(fp, "\n");

    // Aprilgrid parameters
    fprintf(fp, "calib_target:\n");
    fprintf(fp, "  target_type: \"aprilgrid\"\n");
    fprintf(fp, "  tag_rows: %d\n", grid.tag_rows);
    fprintf(fp, "  tag_cols: %d\n", grid.tag_cols);
    fprintf(fp, "  tag_size: %f\n", grid.tag_size);
    fprintf(fp, "  tag_spacing: %f\n", grid.tag_spacing);
    fprintf(fp, "\n");

    // Camera parameters
    fprintf(fp, "cam0:\n");
    fprintf(fp, "  proj_model: \"%s\"\n", cam.proj_model.c_str());
    fprintf(fp, "  dist_model: \"%s\"\n", cam.dist_model.c_str());
    fprintf(fp, "  proj_params: ");
    fprintf(fp, "[");
    fprintf(fp, "%lf, ", cam.proj_params()(0));
    fprintf(fp, "%lf, ", cam.proj_params()(1));
    fprintf(fp, "%lf, ", cam.proj_params()(2));
    fprintf(fp, "%lf", cam.proj_params()(3));
    fprintf(fp, "]\n");
    fprintf(fp, "  dist_params: ");
    fprintf(fp, "[");
    fprintf(fp, "%lf, ", cam.dist_params()(0));
    fprintf(fp, "%lf, ", cam.dist_params()(1));
    fprintf(fp, "%lf, ", cam.dist_params()(2));
    fprintf(fp, "%lf", cam.dist_params()(3));
    fprintf(fp, "]\n");
    fprintf(fp, "\n");

    // T_WF
    fprintf(fp, "T_WF:\n");
    fprintf(fp, "  rows: 4\n");
    fprintf(fp, "  cols: 4\n");
    fprintf(fp, "  data: [\n");
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(0, 0));
    fprintf(fp, "%lf, ", T_WF(0, 1));
    fprintf(fp, "%lf, ", T_WF(0, 2));
    fprintf(fp, "%lf,\n", T_WF(0, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(1, 0));
    fprintf(fp, "%lf, ", T_WF(1, 1));
    fprintf(fp, "%lf, ", T_WF(1, 2));
    fprintf(fp, "%lf,\n", T_WF(1, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(2, 0));
    fprintf(fp, "%lf, ", T_WF(2, 1));
    fprintf(fp, "%lf, ", T_WF(2, 2));
    fprintf(fp, "%lf,\n", T_WF(2, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(3, 0));
    fprintf(fp, "%lf, ", T_WF(3, 1));
    fprintf(fp, "%lf, ", T_WF(3, 2));
    fprintf(fp, "%lf\n", T_WF(3, 3));
    fprintf(fp, "  ]\n");
    fprintf(fp, "\n");

    // T_MC0
    fprintf(fp, "T_MC0:\n");
    fprintf(fp, "  rows: 4\n");
    fprintf(fp, "  cols: 4\n");
    fprintf(fp, "  data: [\n");
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC0(0, 0));
    fprintf(fp, "%lf, ", T_MC0(0, 1));
    fprintf(fp, "%lf, ", T_MC0(0, 2));
    fprintf(fp, "%lf,\n", T_MC0(0, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC0(1, 0));
    fprintf(fp, "%lf, ", T_MC0(1, 1));
    fprintf(fp, "%lf, ", T_MC0(1, 2));
    fprintf(fp, "%lf,\n", T_MC0(1, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC0(2, 0));
    fprintf(fp, "%lf, ", T_MC0(2, 1));
    fprintf(fp, "%lf, ", T_MC0(2, 2));
    fprintf(fp, "%lf,\n", T_MC0(2, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC0(3, 0));
    fprintf(fp, "%lf, ", T_MC0(3, 1));
    fprintf(fp, "%lf, ", T_MC0(3, 2));
    fprintf(fp, "%lf\n", T_MC0(3, 3));
    fprintf(fp, "  ]\n");
    fprintf(fp, "\n");
    fclose(fp);
  }

  // Record poses
  {
    FILE *fp = fopen("/tmp/T_WM.csv", "w");
    for (const auto &pose : data.T_WM) {
      const quat_t q = tf_quat(pose.tf());
      const vec3_t r = tf_trans(pose.tf());
      fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
      fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_WF.csv", "w");
    const quat_t q = tf_quat(data.T_WF.tf());
    const vec3_t r = tf_trans(data.T_WF.tf());
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_MC0.csv", "w");
    const quat_t q = tf_quat(data.T_MC0.tf());
    const vec3_t r = tf_trans(data.T_MC0.tf());
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }
}

/* Calibrate mocap marker to camera extrinsics */
template <typename CAMERA_TYPE>
int calib_mocap_solve(calib_mocap_data_t &data) {
  assert(data.grids.size() > 0);
  assert(data.T_WM.size() > 0);
  assert(data.T_WM.size() == data.grids.size());

  // Setup optimization problem
  ceres::Problem::Options problem_opts;
  problem_opts.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_opts);
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  const mat2_t covar = I(2) * (1 / pow(1, 2));
  std::vector<ceres::ResidualBlockId> block_ids;
  for (size_t i = 0; i < data.grids.size(); i++) {
    process_aprilgrid<CAMERA_TYPE>(i, covar, data, problem, block_ids);

    // Set pose parameterization
    problem.SetParameterization(data.T_WM[i].param.data(),
                                &pose_parameterization);

    // Fixing the marker pose - assume mocap is calibrated and accurate
    if (data.fix_mocap_poses) {
      problem.SetParameterBlockConstant(data.T_WM[i].param.data());
    }
  }

  // Set ficuial and marker-cam pose parameterization
  problem.SetParameterization(data.T_WF.param.data(), &pose_parameterization);
  problem.SetParameterization(data.T_MC0.param.data(), &pose_parameterization);

  // Fix fiducial pose - assumes camera intrincs and PnP is good
  if (data.fix_fiducial_pose) {
    problem.SetParameterBlockConstant(data.T_WF.param.data());
  }

  // Fix camera parameters
  if (data.fix_intrinsics) {
    problem.SetParameterBlockConstant(data.cam0.param.data());
    problem.SetParameterBlockConstant(data.cam0.param.data());
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;
  options.num_threads = 1;

  // Solve
  LOG_INFO("Calibrating mocap-marker to camera extrinsics ...");
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Reject outliers
  LOG_INFO("Rejecting outliers...");
  std::deque<pose_t> poses;
  for (int i = 0; i < (int)data.grids.size(); i++) {
    const mat4_t T_WF = data.T_WF.tf();
    const mat4_t T_C0M = data.T_MC0.tf().inverse();
    const mat4_t T_MW = data.T_WM[i].tf().inverse();
    const mat4_t T_C0F = T_C0M * T_MW * T_WF;
    poses.emplace_back(0, 0, T_C0F);
  }
  std::vector<double> errs;
  reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);

  const auto nb_res_before = problem.NumResidualBlocks();
  const auto threshold = 4.0 * stddev(errs);
  for (int i = 0; i < (int)errs.size(); i++) {
    if (errs[i] > threshold) {
      problem.RemoveResidualBlock(block_ids[i]);
    }
  }
  const auto nb_res_after = problem.NumResidualBlocks();
  const auto res_diff = nb_res_before - nb_res_after;
  LOG_INFO("Removed: %d residuals out of %d", res_diff, nb_res_before);

  // Second pass
  LOG_INFO("Performing second pass ...");
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << std::endl;

  // Show results
  show_results<CAMERA_TYPE>(data);
  // save_results<CAMERA_TYPE>(data, data.results_fpath);

  return 0;
}

/* Solve the camera-mocap_marker extrinsics */
int calib_mocap_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_MOCAP_HPP
