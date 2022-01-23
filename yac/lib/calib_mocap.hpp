#ifndef YAC_CALIB_MOCAP_HPP
#define YAC_CALIB_MOCAP_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

// MOCAP MARKER RESIDUAL ///////////////////////////////////////////////////////

struct mocap_residual_t : public ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const camera_geometry_t *cam_geom_;
  const int cam_idx_;
  const int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;

  mocap_residual_t(const camera_geometry_t *cam_geom,
                   const int cam_idx,
                   const int cam_res[2],
                   const vec3_t &r_FFi,
                   const vec2_t &z,
                   const mat2_t &covar);
  ~mocap_residual_t() = default;

  bool Evaluate(double const *const *params,
                double *residuals,
                double **jacobians) const;
};

// MOCAP-CAMERA CALIBRATION DATA ///////////////////////////////////////////////

struct calib_mocap_data_t {
  // Flags
  bool fix_intrinsics = false;
  bool fix_mocap_poses = false;
  bool fix_fiducial_pose = false;

  // Settings
  double sigma_vision = 1.0;
  bool imshow = true;

  // Paths
  std::string data_path;
  std::string results_fpath;
  std::string cam0_path;
  std::string grid0_path;
  std::string body0_csv_path;
  std::string target0_csv_path;

  // Data
  calib_target_t calib_target;
  aprilgrids_t grids;
  camera_params_t cam0;
  std::vector<pose_t> T_WM;
  pose_t T_MC0;
  pose_t T_WF;
  mat2_t covar;

  // Constructor
  calib_mocap_data_t(const std::string &config_file);
};

static void process_aprilgrid(const camera_geometry_t *cam_geom,
                              const size_t frame_idx,
                              const mat2_t &covar,
                              calib_mocap_data_t &data,
                              ceres::Problem &problem,
                              std::vector<ceres::ResidualBlockId> &block_ids) {
  const int *cam_res = data.cam0.resolution;
  const int cam_idx = data.cam0.cam_index;
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

    auto cost_fn =
        new mocap_residual_t{cam_geom, cam_idx, cam_res, r_FFi, z, covar};
    auto block_id = problem.AddResidualBlock(cost_fn,
                                             NULL,
                                             data.T_WF.param.data(),
                                             data.T_WM[frame_idx].param.data(),
                                             data.T_MC0.param.data(),
                                             data.cam0.param.data());
    block_ids.push_back(block_id);
  }
}

// template <typename CAMERA_TYPE>
// static void show_results(const calib_mocap_data_t &data) {
//   // Calibration metrics
//   std::deque<pose_t> poses;
//   const mat4_t T_WF = data.T_WF.tf();
//   const mat4_t T_C0M = data.T_MC0.tf().inverse();
//   for (size_t i = 0; i < data.grids.size(); i++) {
//     const mat4_t T_MW = data.T_WM[i].tf().inverse();
//     const mat4_t T_C0F = T_C0M * T_MW * T_WF;
//     poses.emplace_back(0, 0, T_C0F);
//   }
//
//   // Show results
//   std::vector<double> errs;
//   reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);
//
//   printf("\n");
//   printf("Optimization results:\n");
//   printf("---------------------\n");
//   printf("nb_points: %ld\n", errs.size());
//   printf("reproj_error [px]: ");
//   printf("[rmse: %f", rmse(errs));
//   printf(" mean: %f", mean(errs));
//   printf(" median: %f]\n", median(errs));
//   printf("\n");
//   print_vector("cam.proj_params", data.cam0.proj_params());
//   print_vector("cam.dist_params", data.cam0.dist_params());
//   printf("\n");
//   print_matrix("T_WF", data.T_WF.tf());
//   print_matrix("T_WM", data.T_WM[0].tf());
//   print_matrix("T_MC0", data.T_MC0.tf());
//
//   const auto r_MC = tf_trans(data.T_MC0.tf());
//   const auto q_MC = tf_quat(data.T_MC0.tf());
//   printf("r_MC: %f, %f, %f\n", r_MC(0), r_MC(1), r_MC(2));
//   printf("q_MC (x, y, z, w): %f, %f, %f, %f\n",
//          q_MC.x(),
//          q_MC.y(),
//          q_MC.z(),
//          q_MC.w());
// }
//
// template <typename CAMERA_TYPE>
// static void save_results(const calib_mocap_data_t &data,
//                          const std::string &output_path) {
//   printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
//          output_path.c_str());
//   const aprilgrid_t grid = data.grids[0];
//   const auto cam = data.cam0;
//   const mat4_t T_WF = data.T_WF.tf();
//   const mat4_t T_MC0 = data.T_MC0.tf();
//   // const mat4_t T_C0M = T_MC0.inverse();
//
//   // double err_min = *std::min_element(errs.begin(), errs.end());
//   // double err_max = *std::max_element(errs.begin(), errs.end());
//   // double err_median = median(errs);
//
//   // Save calibration results to yaml file
//   {
//     FILE *fp = fopen(output_path.c_str(), "w");
//
//     // // Calibration metrics
//     // fprintf(fp, "calib_results:\n");
//     // fprintf(fp, "  cam0.rms_reproj_error:    %f  # [px]\n", err_rmse);
//     // fprintf(fp, "  cam0.mean_reproj_error:   %f  # [px]\n", err_mean);
//     // fprintf(fp, "  cam0.median_reproj_error: %f  # [px]\n", err_median);
//     // fprintf(fp, "  cam0.min_reproj_error:    %f  # [px]\n", err_min);
//     // fprintf(fp, "  cam0.max_reproj_error:    %f  # [px]\n", err_max);
//     // fprintf(fp, "\n");
//
//     // Aprilgrid parameters
//     fprintf(fp, "calib_target:\n");
//     fprintf(fp, "  target_type: \"aprilgrid\"\n");
//     fprintf(fp, "  tag_rows: %d\n", grid.tag_rows);
//     fprintf(fp, "  tag_cols: %d\n", grid.tag_cols);
//     fprintf(fp, "  tag_size: %f\n", grid.tag_size);
//     fprintf(fp, "  tag_spacing: %f\n", grid.tag_spacing);
//     fprintf(fp, "\n");
//
//     // Camera parameters
//     fprintf(fp, "cam0:\n");
//     fprintf(fp, "  proj_model: \"%s\"\n", cam.proj_model.c_str());
//     fprintf(fp, "  dist_model: \"%s\"\n", cam.dist_model.c_str());
//     fprintf(fp, "  proj_params: ");
//     fprintf(fp, "[");
//     fprintf(fp, "%lf, ", cam.proj_params()(0));
//     fprintf(fp, "%lf, ", cam.proj_params()(1));
//     fprintf(fp, "%lf, ", cam.proj_params()(2));
//     fprintf(fp, "%lf", cam.proj_params()(3));
//     fprintf(fp, "]\n");
//     fprintf(fp, "  dist_params: ");
//     fprintf(fp, "[");
//     fprintf(fp, "%lf, ", cam.dist_params()(0));
//     fprintf(fp, "%lf, ", cam.dist_params()(1));
//     fprintf(fp, "%lf, ", cam.dist_params()(2));
//     fprintf(fp, "%lf", cam.dist_params()(3));
//     fprintf(fp, "]\n");
//     fprintf(fp, "\n");
//
//     // T_WF
//     fprintf(fp, "T_WF:\n");
//     fprintf(fp, "  rows: 4\n");
//     fprintf(fp, "  cols: 4\n");
//     fprintf(fp, "  data: [\n");
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_WF(0, 0));
//     fprintf(fp, "%lf, ", T_WF(0, 1));
//     fprintf(fp, "%lf, ", T_WF(0, 2));
//     fprintf(fp, "%lf,\n", T_WF(0, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_WF(1, 0));
//     fprintf(fp, "%lf, ", T_WF(1, 1));
//     fprintf(fp, "%lf, ", T_WF(1, 2));
//     fprintf(fp, "%lf,\n", T_WF(1, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_WF(2, 0));
//     fprintf(fp, "%lf, ", T_WF(2, 1));
//     fprintf(fp, "%lf, ", T_WF(2, 2));
//     fprintf(fp, "%lf,\n", T_WF(2, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_WF(3, 0));
//     fprintf(fp, "%lf, ", T_WF(3, 1));
//     fprintf(fp, "%lf, ", T_WF(3, 2));
//     fprintf(fp, "%lf\n", T_WF(3, 3));
//     fprintf(fp, "  ]\n");
//     fprintf(fp, "\n");
//
//     // T_MC0
//     fprintf(fp, "T_MC0:\n");
//     fprintf(fp, "  rows: 4\n");
//     fprintf(fp, "  cols: 4\n");
//     fprintf(fp, "  data: [\n");
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_MC0(0, 0));
//     fprintf(fp, "%lf, ", T_MC0(0, 1));
//     fprintf(fp, "%lf, ", T_MC0(0, 2));
//     fprintf(fp, "%lf,\n", T_MC0(0, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_MC0(1, 0));
//     fprintf(fp, "%lf, ", T_MC0(1, 1));
//     fprintf(fp, "%lf, ", T_MC0(1, 2));
//     fprintf(fp, "%lf,\n", T_MC0(1, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_MC0(2, 0));
//     fprintf(fp, "%lf, ", T_MC0(2, 1));
//     fprintf(fp, "%lf, ", T_MC0(2, 2));
//     fprintf(fp, "%lf,\n", T_MC0(2, 3));
//     fprintf(fp, "    ");
//     fprintf(fp, "%lf, ", T_MC0(3, 0));
//     fprintf(fp, "%lf, ", T_MC0(3, 1));
//     fprintf(fp, "%lf, ", T_MC0(3, 2));
//     fprintf(fp, "%lf\n", T_MC0(3, 3));
//     fprintf(fp, "  ]\n");
//     fprintf(fp, "\n");
//     fclose(fp);
//   }
//
//   // Record poses
//   {
//     FILE *fp = fopen("/tmp/T_WM.csv", "w");
//     for (const auto &pose : data.T_WM) {
//       const quat_t q = tf_quat(pose.tf());
//       const vec3_t r = tf_trans(pose.tf());
//       fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
//       fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
//       fprintf(fp, "\n");
//     }
//     fclose(fp);
//   }
//
//   {
//     FILE *fp = fopen("/tmp/T_WF.csv", "w");
//     const quat_t q = tf_quat(data.T_WF.tf());
//     const vec3_t r = tf_trans(data.T_WF.tf());
//     fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
//     fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
//     fprintf(fp, "\n");
//     fclose(fp);
//   }
//
//   {
//     FILE *fp = fopen("/tmp/T_MC0.csv", "w");
//     const quat_t q = tf_quat(data.T_MC0.tf());
//     const vec3_t r = tf_trans(data.T_MC0.tf());
//     fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
//     fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
//     fprintf(fp, "\n");
//     fclose(fp);
//   }
// }

// /* Calibrate mocap marker to camera extrinsics */
// int calib_mocap_solve(calib_mocap_data_t &data) {
//   assert(data.grids.size() > 0);
//   assert(data.T_WM.size() > 0);
//   assert(data.T_WM.size() == data.grids.size());
//
//   // Setup optimization problem
//   ceres::Problem::Options problem_opts;
//   problem_opts.local_parameterization_ownership =
//   ceres::DO_NOT_TAKE_OWNERSHIP; ceres::Problem problem(problem_opts);
//   PoseLocalParameterization pose_parameterization;
//
//   // Process all aprilgrid data
//   const mat2_t covar = I(2) * (1 / pow(1, 2));
//   std::vector<ceres::ResidualBlockId> block_ids;
//   for (size_t i = 0; i < data.grids.size(); i++) {
//     // process_aprilgrid<CAMERA_TYPE>(i, covar, data, problem, block_ids);
//
//     // Set pose parameterization
//     problem.SetParameterization(data.T_WM[i].param.data(),
//                                 &pose_parameterization);
//
//     // Fixing the marker pose - assume mocap is calibrated and accurate
//     if (data.fix_mocap_poses) {
//       problem.SetParameterBlockConstant(data.T_WM[i].param.data());
//     }
//   }
//
//   // Set ficuial and marker-cam pose parameterization
//   problem.SetParameterization(data.T_WF.param.data(),
//   &pose_parameterization);
//   problem.SetParameterization(data.T_MC0.param.data(),
//   &pose_parameterization);
//
//   // Fix fiducial pose - assumes camera intrincs and PnP is good
//   if (data.fix_fiducial_pose) {
//     problem.SetParameterBlockConstant(data.T_WF.param.data());
//   }
//
//   // Fix camera parameters
//   if (data.fix_intrinsics) {
//     problem.SetParameterBlockConstant(data.cam0.param.data());
//     problem.SetParameterBlockConstant(data.cam0.param.data());
//   }
//
//   // Set solver options
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = true;
//   options.max_num_iterations = 100;
//   // options.check_gradients = true;
//   options.num_threads = 1;
//
//   // Solve
//   LOG_INFO("Calibrating mocap-marker to camera extrinsics ...");
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   // std::cout << summary.FullReport() << std::endl;
//   std::cout << summary.BriefReport() << std::endl;
//   std::cout << std::endl;
//
//   // Reject outliers
//   LOG_INFO("Rejecting outliers...");
//   std::deque<pose_t> poses;
//   for (int i = 0; i < (int)data.grids.size(); i++) {
//     const mat4_t T_WF = data.T_WF.tf();
//     const mat4_t T_C0M = data.T_MC0.tf().inverse();
//     const mat4_t T_MW = data.T_WM[i].tf().inverse();
//     const mat4_t T_C0F = T_C0M * T_MW * T_WF;
//     poses.emplace_back(0, 0, T_C0F);
//   }
//   std::vector<double> errs;
//   reproj_errors<CAMERA_TYPE>(data.grids, data.cam0, poses, errs);
//
//   const auto nb_res_before = problem.NumResidualBlocks();
//   const auto threshold = 4.0 * stddev(errs);
//   for (int i = 0; i < (int)errs.size(); i++) {
//     if (errs[i] > threshold) {
//       problem.RemoveResidualBlock(block_ids[i]);
//     }
//   }
//   const auto nb_res_after = problem.NumResidualBlocks();
//   const auto res_diff = nb_res_before - nb_res_after;
//   LOG_INFO("Removed: %d residuals out of %d", res_diff, nb_res_before);
//
//   // Second pass
//   LOG_INFO("Performing second pass ...");
//   ceres::Solve(options, &problem, &summary);
//   // std::cout << summary.FullReport() << std::endl;
//   std::cout << summary.BriefReport() << std::endl;
//   std::cout << std::endl;
//
//   // Show results
//   show_results<CAMERA_TYPE>(data);
//   // save_results<CAMERA_TYPE>(data, data.results_fpath);
//
//   return 0;
// }

/* Solve the camera-mocap_marker extrinsics */
int calib_mocap_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_MOCAP_HPP
