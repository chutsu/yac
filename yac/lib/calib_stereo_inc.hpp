#ifndef YAC_CALIB_STEREO_INC_HPP
#define YAC_CALIB_STEREO_INC_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "calib_stereo.hpp"

namespace yac {

template <typename CAMERA>
class calib_stereo_inc_solver_t {
public:
  // Calib data
  const calib_target_t &target;
  const mat2_t &covar;
  aprilgrids_t &grids0;
  aprilgrids_t &grids1;

  // Optimization variables
  id_t param_counter = 0;
  std::deque<pose_t> rel_poses;
  camera_params_t &cam0;
  camera_params_t &cam1;
  extrinsics_t &cam0_exts;  // T_BC0
  extrinsics_t &cam1_exts;  // T_BC1
  calib_views_t<CAMERA> views0;
  calib_views_t<CAMERA> views1;
  std::vector<int> nbv_indicies;

  // Incremental solve settings
  int random_indicies_attempts = 1000;
  size_t min_views = 20;
  double outlier_stddev = 4.0;
  double info_gain_delta_threshold = 0.2;
  bool enable_early_stop = false;
  size_t stale_limit = 30;
  size_t stale_counter = 0;
  bool first_filter = true;

  // Incremental solve stats
  double batch_info = -1;
  size_t processed = 0;
  size_t total_outliers = 0;

  // Ceres solver settings
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;
  ceres::Solver::Summary summary;

  calib_stereo_inc_solver_t(calib_data_t &data)
      : target{data.target},
        covar{data.covar},
        grids0{data.cam_grids[0]},
        grids1{data.cam_grids[1]},
        cam0{data.cam_params[0]},
        cam1{data.cam_params[1]},
        cam0_exts{data.cam_exts[0]},
        cam1_exts{data.cam_exts[1]} {
    // Seed random
    srand(time(NULL));

    // Filter data and initialize stereo intrinsics and extrinsics
    preprocess_stereo_data();
    check_data();
    initialize();

    // Setup optimization problem
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = new ceres::Problem{prob_options};

    // Fix cam0_exts
    problem->AddParameterBlock(cam0_exts.param.data(), 7);
    problem->SetParameterBlockConstant(cam0_exts.param.data());

    // Solve
    incremental_solve();
    solve();
  }

  ~calib_stereo_inc_solver_t() {
    if (problem) {
      delete problem;
    }

    if (loss) {
      delete loss;
    }
  }

  size_t nb_grids() {
    return grids0.size();
  }

  /* Preprocess stereo data */
  void preprocess_stereo_data() {
    // Loop through both sets of calibration data
    size_t nb_grids = std::max(grids0.size(), grids1.size());
    size_t cam0_idx = 0;
    size_t cam1_idx = 0;

    aprilgrids_t cam0_grids;
    aprilgrids_t cam1_grids;

    for (size_t i = 0; i < nb_grids; i++) {
      // Get grid
      aprilgrid_t &grid0 = grids0[cam0_idx];
      aprilgrid_t &grid1 = grids1[cam1_idx];
      if (grid0.timestamp == grid1.timestamp) {
        cam0_idx++;
        cam1_idx++;
      } else if (grid0.timestamp > grid1.timestamp) {
        cam1_idx++;
        continue;
      } else if (grid0.timestamp < grid1.timestamp) {
        cam0_idx++;
        continue;
      }

      // Keep only common tags between grid0 and grid1
      // grid0.intersect(grid1);
      // assert(grid0.nb_detections == grid1.nb_detections);

      // Add to results if detected anything
      if (grid0.detected && grid1.detected) {
        cam0_grids.emplace_back(grid0);
        cam1_grids.emplace_back(grid1);
      }

      // Check if there's more data to go though
      if (cam0_idx >= grids0.size() || cam1_idx >= grids1.size()) {
        break;
      }
    }

    grids0.clear();
    grids1.clear();
    grids0 = cam0_grids;
    grids1 = cam1_grids;
  }

  /**
   * Check stereo data
   * - Timestamps are the same for cam0 and cam1
   * - AprilGrid properties are correct
   */
  void check_data() {
    bool ok = true;

    if (grids0.size() != grids1.size()) {
      LOG_ERROR("grids0.size() != grids1.size()!");
      ok = false;
    }

    std::vector<timestamp_t> timestamps;
    for (auto &grid : grids0) {
      timestamps.push_back(grid.timestamp);
    }

    for (auto &grids : {grids0, grids1}) {
      for (size_t i = 0; i < grids.size(); i++) {
        if (grids[i].timestamp != timestamps[i]) {
          LOG_ERROR("grid.timestamp != timestamps[i]!");
          ok = false;
          break;
        }

        if (grids[i].tag_rows != target.tag_rows) {
          LOG_ERROR("grid.timestamp: %ld", grids[i].timestamp);
          LOG_ERROR("grid.tag_rows != target.tag_rows!");
          ok = false;
          break;
        } else if (grids[i].tag_cols != target.tag_cols) {
          LOG_ERROR("grid.timestamp: %ld", grids[i].timestamp);
          LOG_ERROR("grid.tag_cols != target.tag_cols!");
          ok = false;
          break;
        } else if (grids[i].tag_size != target.tag_size) {
          LOG_ERROR("grid.timestamp: %ld", grids[i].timestamp);
          LOG_ERROR("grid.tag_size != target.tag_size!");
          ok = false;
          break;
        } else if (grids[i].tag_spacing != target.tag_spacing) {
          LOG_ERROR("grid.timestamp: %ld", grids[i].timestamp);
          LOG_ERROR("grid.tag_spacing != target.tag_spacing!");
          ok = false;
          break;
        }
      }
    }

    if (ok == false) {
      FATAL("Bad data!");
    }
  }

  /* Initialize Stereo-Camera Intrinsics and Extrinsics */
  void initialize() {
    // Initialize camera intrinsics
    // -- cam0
    LOG_INFO("Initializing cam0 intrinsics ...");
    calib_mono_data_t cam0_data{grids0, cam0};
    if (calib_mono_solve<CAMERA>(cam0_data) != 0) {
      FATAL("Failed to calibrate cam0 intrinsics!");
    }
    // -- cam1
    LOG_INFO("Initializing cam1 intrinsics ...");
    calib_mono_data_t cam1_data{grids1, cam1};
    if (calib_mono_solve<CAMERA>(cam1_data) != 0) {
      FATAL("Failed to calibrate cam1 intrinsics!");
    }

    // Initialize stereo-pair extrinsics
    LOG_INFO("Initializing cam0-cam1 extrinsics ...");
    auto cam0_grids = grids0;
    auto cam1_grids = grids1;
    std::map<timestamp_t, pose_t> poses;  // T_BF

    calib_data_t data;
    data.add_calib_target(target);
    data.add_camera(cam0);
    data.add_camera(cam1);
    data.add_camera_extrinsics(0);
    data.add_camera_extrinsics(1);
    data.add_grids(0, grids0);
    data.add_grids(1, grids1);
    data.preprocess_data();
    data.check_data();
    calib_stereo_solve<CAMERA>(data);

    // Show initialized parameters
    const mat4_t T_BC0 = data.cam_exts[0].tf();
    const mat4_t T_BC1 = data.cam_exts[1].tf();
    const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;
    LOG_INFO("Stereo-camera intrinsics and extrinsics initialized!");
    print_vector("cam0_params:", data.cam_params[0].param);
    print_vector("cam1_params:", data.cam_params[1].param);
    print_matrix("T_C1C0", T_C1C0);

    // Update params
    cam0.param = data.cam_params[0].param;
    cam1.param = data.cam_params[1].param;
    cam0_exts.param = data.cam_exts[0].param;
    cam1_exts.param = data.cam_exts[1].param;
  }

  int find_nbv(std::set<int> &views_seen) {
    std::map<double, int, std::greater<double>> view_entropies;

    // Incremental solve
    for (size_t i = 0; i < nb_grids(); i++) {
      if (views_seen.count(i)) {
        continue;
      }

      const auto grid0 = grids0[i];
      const auto grid1 = grids1[i];
      if (grid0.timestamp != grid1.timestamp) {
        LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
        FATAL("Bad data! Stopping stereo-calibration!");
      }

      add_view(grid0, grid1);
      solve_view();

      matx_t covar;
      if (calib_covar(*problem, cam0, cam1, cam1_exts, covar) == 0) {
        view_entropies[entropy(covar)] = i;
      }

      remove_view();
    }

    for (const auto &kv : view_entropies) {
      return kv.second;
    }
  }

  void eval_nbvs() {
    std::map<double, int, std::greater<double>> view_entropies;

    for (size_t i = 0; i < nb_grids(); i++) {
      add_view(grids0[i], grids1[i]);
      solve_view();

      matx_t covar;
      if (calib_covar(*problem, cam0, cam1, cam1_exts, covar) == 0) {
        view_entropies[entropy(covar)] = i;
      }

      remove_view();
    }

    for (const auto &kv : view_entropies) {
      const auto idx = kv.second;
      nbv_indicies.push_back(idx);
    }
  }

  /**
  * Find random indicies.
  *
  * Aside from creating a set of random indicies to randomly sample the image
  * frames, the goal is to find a set that yeilds the lowest reprojection error
  * for the first 20 frames. The intuition is to start the incremental solver in
  * a certain state, then add / remove frames that improve the calibration.
  */
  std::vector<int> find_random_indicies(bool verbose=false) {
    id_t param_counter = 0;
    int best_idx = 0;
    double best_rmse = 0.0;
    std::vector<int> best_indicies;

    const auto cam_res = cam0.resolution;
    const vecx_t proj_params = cam0.proj_params();
    const vecx_t dist_params = cam0.dist_params();
    const CAMERA cam0_geom{cam_res, proj_params, dist_params};

    for (int j = 0; j < random_indicies_attempts; j++) {
      // Create random index vector (same length as grids)
      std::vector<int> indicies;
      for (size_t i = 0; i < grids0.size(); i++) indicies.push_back(i);
      std::random_shuffle(std::begin(indicies), std::end(indicies));

      // Views
      calib_views_t<CAMERA> views0;
      calib_views_t<CAMERA> views1;
      std::deque<pose_t> rel_poses;

      // Evaluate indicies
      for (const auto &i : indicies) {
        const auto grid0 = grids0[i];
        const auto grid1 = grids1[i];
        const auto ts = grids0[i].timestamp;
        if (grid0.timestamp != grid1.timestamp) {
          LOG_ERROR("cam0_grid.timestamp != cam1_grid.timestamp");
          LOG_ERROR("cam0_grid.timestamp: %ld", grid0.timestamp);
          LOG_ERROR("cam1_grid.timestamp: %ld", grid1.timestamp);
          // FATAL("Bad data! Stopping stereo-calibration!");
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

  /* Show progress */
  void show_progress(const int nb_outliers) {
    std::vector<double>cam0_errs;
    std::vector<double>cam1_errs;
    reproj_errors(views0, cam0_errs);
    reproj_errors(views1, cam1_errs);

    printf("Processed [%ld / %ld] views, ", processed++, grids0.size());
    printf("nb_kept_views: %ld, ", views0.size());
    printf("nb_outliers: %d, ", nb_outliers);
    printf("cam0_rmse: %.2f, ", rmse(cam0_errs));
    printf("cam1_rmse: %.2f", rmse(cam1_errs));
    printf("\n");
  }

  void show_results() {
    // Show results
    const mat4_t T_BC0 = cam0_exts.tf();
    const mat4_t T_BC1 = cam1_exts.tf();
    const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;

    std::vector<double> cam0_errs, cam1_errs;
    reproj_errors<CAMERA>(views0, cam0_errs);
    reproj_errors<CAMERA>(views1, cam1_errs);

    printf("Optimization results:\n");
    printf("---------------------\n");
    printf("nb_total_outliers: %ld\n", total_outliers);
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

  /* Estimate pose T_C0F */
  pose_t estimate_pose(const aprilgrid_t &grid0, const aprilgrid_t &grid1) {
    mat4_t T_C0F_k;

    if (grid0.nb_detections > grid1.nb_detections) {
      const auto cam_res = cam0.resolution;
      const vecx_t proj_params = cam0.proj_params();
      const vecx_t dist_params = cam0.dist_params();
      const CAMERA cam0_geom{cam_res, proj_params, dist_params};
      if (grid0.estimate(cam0_geom, T_C0F_k) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

    } else {
      const auto cam_res = cam1.resolution;
      const vecx_t proj_params = cam1.proj_params();
      const vecx_t dist_params = cam1.dist_params();
      const CAMERA cam1_geom{cam_res, proj_params, dist_params};
      mat4_t T_C1F_k;
      if (grid1.estimate(cam1_geom, T_C1F_k) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      const mat4_t T_C0C1 = cam1_exts.tf();
      T_C0F_k = T_C0C1 * T_C1F_k;
    }

    const auto ts = grid0.timestamp;
    return pose_t{param_counter++, ts, T_C0F_k};
  }

  void add_view(const aprilgrid_t &grid0, const aprilgrid_t &grid1) {
    // Estimate T_C0F
    auto rel_pose = estimate_pose(grid0, grid1);
    rel_poses.push_back(rel_pose);

    // Create and add view to problem
    views0.emplace_back(0, grid0, covar, cam0, rel_poses.back(), cam0_exts);
    views1.emplace_back(1, grid1, covar, cam1, rel_poses.back(), cam1_exts);
    process_grid<CAMERA>(views0.back(), *problem, pose_plus, loss);
    process_grid<CAMERA>(views1.back(), *problem, pose_plus, loss);
  }

  void remove_view() {
    problem->RemoveParameterBlock(rel_poses.back().param.data());
    rel_poses.pop_back();
    // for (const auto &res_id : views0.back().res_ids) {
    //   problem->RemoveResidualBlock(res_id);
    // }
    // for (const auto &res_id : views1.back().res_ids) {
    //   problem->RemoveResidualBlock(res_id);
    // }
    views0.pop_back();
    views1.pop_back();
  }

  int remove_outliers(const bool last_view_only) {
    int nb_outliers = 0;
    std::vector<std::pair<double, double>> errs0;
    std::vector<std::pair<double, double>> errs1;
    calib_residuals(views0, errs0);
    calib_residuals(views1, errs1);

    std::vector<double> errs0_x;
    std::vector<double> errs0_y;
    std::vector<double> errs1_x;
    std::vector<double> errs1_y;
    for (size_t i = 0; i < errs0.size(); i++) {
      errs0_x.push_back(errs0[i].first);
      errs0_y.push_back(errs0[i].second);
    }
    for (size_t i = 0; i < errs1.size(); i++) {
      errs1_x.push_back(errs1[i].first);
      errs1_y.push_back(errs1[i].second);
    }

    const auto cam0_stddev_x = stddev(errs0_x);
    const auto cam0_stddev_y = stddev(errs0_y);
    const auto cam1_stddev_x = stddev(errs1_x);
    const auto cam1_stddev_y = stddev(errs1_y);

    const auto th0_x = outlier_stddev * cam0_stddev_x;
    const auto th0_y = outlier_stddev * cam0_stddev_y;
    const auto th1_x = outlier_stddev * cam1_stddev_x;
    const auto th1_y = outlier_stddev * cam1_stddev_y;

    if (last_view_only) {
      // Filter last view
      nb_outliers += filter_view(*problem, views0.back(), th0_x, th0_y);
      nb_outliers += filter_view(*problem, views1.back(), th1_x, th1_y);
    } else {
      // Filter all views
      nb_outliers += filter_views(*problem, views0, th0_x, th0_y);
      nb_outliers += filter_views(*problem, views1, th1_x, th1_y);
    }

    return nb_outliers;
  }

  /* Filter view */
  int filter_view(ceres::Problem &problem,
                  calib_view_t<CAMERA> &view,
                  double threshold_x=1.0,
                  double threshold_y=1.0) {
    auto &grid = view.grid;
    const auto cam = view.cam;
    int nb_outliers = 0;

    // Evaluate reprojection errors
    std::vector<std::pair<double, double>> errs;
    std::vector<std::pair<int, int>> tags_meta;
    calib_residuals(view, errs, &tags_meta);

    // Check which AprilGrid tag to remove
    std::set<std::pair<int, int>> outliers;
    for (size_t i = 0; i < tags_meta.size(); i++) {
      auto tag_id = tags_meta[i].first;
      auto corner_idx = tags_meta[i].second;

      const auto err_x = std::fabs(errs[i].first);
      const auto err_y = std::fabs(errs[i].second);
      if (err_x > threshold_x || err_y > threshold_y) {
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
          // Remove residual from problem
          problem.RemoveResidualBlock(res_id);

          // Remove tag id and corner idx from grid
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
  int filter_views(ceres::Problem &problem,
                   calib_views_t<CAMERA> &views,
                   double threshold_x=1.0,
                   double threshold_y=1.0) {
    int nb_outliers = 0;
    for (auto &view : views) {
      nb_outliers += filter_view(problem,
                                 view,
                                 threshold_x,
                                 threshold_y);
    }

    return nb_outliers;
  }

  void solve_view() {
    ceres::Solver::Options options;
    options.max_num_iterations = 20;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    ceres::Solve(options, problem, &summary);
  }

  double entropy(matx_t &covar) {
    // Calculate entropy in bits
    const Eigen::SelfAdjointEigenSolver<matx_t> eig(covar);
    const auto eigvals = eig.eigenvalues().array();
    const double entropy = -1.0 * eigvals.log().sum() / std::log(2);
    return entropy;
  }

  bool keep_view(double &view_info) {
    matx_t covar;
    if (calib_covar(*problem, cam0, cam1, cam1_exts, covar) == 0) {
      view_info = entropy(covar);
      const double info_gain = 0.5 * (view_info - batch_info);
      // printf("entropy: %f\t", info_k);
      // printf("info_gain: %f\n", info_gain);

      if (info_gain > info_gain_delta_threshold) {
        stale_counter = 0;

      } else {
        stale_counter++;
        return false;
      }
    }

    return true;
  }

  void evaluate_view(int &nb_outliers) {
    // Evaluate current view
    double view_info = 0.0;
    bool keep = keep_view(view_info);
    if (keep == false) {
      remove_view();
      return;
    }

      // Check if we have min-views before removing outliers
    if (processed < min_views) {
      batch_info = view_info;
      return;
    }

    // Remove outliers
    if (first_filter) {
      nb_outliers = remove_outliers(false);
      first_filter = false;
    } else {
      nb_outliers = remove_outliers(true);
    }

    // If there are outliers in the current view, lets check the entropy
    // again to see if we should remove it or keep it
    if (nb_outliers && keep_view(view_info) == false) {
      remove_view();
    } else {
      batch_info = view_info;
      total_outliers += nb_outliers;
    }
  }

  void incremental_solve() {
    // eval_nbvs();

    // Incremental solve
    for (const auto &i : find_random_indicies()) {
      int nb_outliers = 0;
      add_view(grids0[i], grids1[i]);
      solve_view();
      evaluate_view(nb_outliers);

      if (enable_early_stop && stale_counter >= stale_limit) {
        LOG_INFO("stale_counter >= stale_limit");
        LOG_INFO("stopping early!");
        break;
      }

      show_progress(nb_outliers);
    }

    // std::set<int> views_seen;
    // for (size_t i = 0; i < nb_grids(); i++) {
    //   auto view_idx = find_nbv(views_seen);
    //   views_seen.insert(view_idx);
    //
    //   int nb_outliers = 0;
    //   const auto grid0 = grids0[view_idx];
    //   const auto grid1 = grids1[view_idx];
    //
    //   add_view(grid0, grid1);
    //   solve_view();
    //   evaluate_view(nb_outliers);
    //
    //   if (enable_early_stop && stale_counter >= stale_limit) {
    //     LOG_INFO("stale_counter >= stale_limit");
    //     LOG_INFO("stopping early!");
    //     break;
    //   }
    //
    //   show_progress(nb_outliers);
    // }
  }

  void solve() {
    // Set solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 30;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;

    // Final filter before last optimization
    total_outliers += remove_outliers(false);

    // Solve
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-20;
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << std::endl;

    // Show reuslts
    show_results();
    save_views("/tmp", views0);
    save_views("/tmp", views1);
  }
};

} //  namespace yac
#endif // YAC_CALIB_STEREO_INC_HPP
