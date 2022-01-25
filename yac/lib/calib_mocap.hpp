#ifndef YAC_CALIB_MOCAP_HPP
#define YAC_CALIB_MOCAP_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_camera.hpp"

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

  std::vector<int> resolution;
  std::string proj_model;
  std::string dist_model;
  vecx_t proj_params;
  vecx_t dist_params;

  std::map<timestamp_t, mat4_t> T_WM;
  mat4_t T_WF;
  mat4_t T_MC0;
  mat2_t covar = I(2);

  calib_mocap_data_t(const std::string &config_file);

  camera_params_t get_camera_params() const;
  extrinsics_t get_extrinsics() const;
  pose_t get_fiducial_pose() const;
  std::map<timestamp_t, pose_t> get_marker_poses() const;
};

// static void process_aprilgrid(const camera_geometry_t *cam_geom,
//                               const size_t frame_idx,
//                               const mat2_t &covar,
//                               calib_mocap_data_t &data,
//                               ceres::Problem &problem,
//                               std::vector<ceres::ResidualBlockId> &block_ids)
//                               {
//   const int *cam_res = data.cam0.resolution;
//   const int cam_idx = data.cam0.cam_index;
//   const auto &grid = data.grids[frame_idx];
//
//   std::vector<int> tag_ids;
//   std::vector<int> corner_indicies;
//   vec2s_t keypoints;
//   vec3s_t object_points;
//   grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);
//
//   for (size_t i = 0; i < tag_ids.size(); i++) {
//     // const int tag_id = tag_ids[i];
//     // const int corner_idx = corner_indicies[i];
//     const vec2_t z = keypoints[i];
//     const vec3_t r_FFi = object_points[i];
//
//     auto cost_fn =
//         new mocap_residual_t{cam_geom, cam_idx, cam_res, r_FFi, z, covar};
//     auto block_id = problem.AddResidualBlock(cost_fn,
//                                              NULL,
//                                              data.T_WF.param.data(),
//                                              data.T_WM[frame_idx].param.data(),
//                                              data.T_MC0.param.data(),
//                                              data.cam0.param.data());
//     block_ids.push_back(block_id);
//   }
// }

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
//   // double err_max = *std::max_element(errs.begin(), errs.end());
//   // double err_median = median(errs);
//
//   // Save calibration results to yaml file
//   {
//     FILE *fp = fopen(output_path.c_str(), "w");
//
//     // Calibration metrics
//     fprintf(fp, "calib_results:\n");
//     fprintf(fp, "  rmse:   %f # [px]\n", rmse(reproj_errors));
//     fprintf(fp, "  mean:   %f # [px]\n", mean(reproj_errors));
//     fprintf(fp, "  median: %f # [px]\n", median(reproj_errors));
//     fprintf(fp, "  stddev: %f # [px]\n", stddev(reproj_errors));
//     fprintf(fp, "\n");
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
//     fprintf(fp, "cam%d:\n", cam_idx);
//     fprintf(fp, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
//     fprintf(fp, "  proj_model: \"%s\"\n", proj_model);
//     fprintf(fp, "  dist_model: \"%s\"\n", dist_model);
//     fprintf(fp, "  proj_params: %s\n", proj_params.c_str());
//     fprintf(fp, "  dist_params: %s\n", dist_params.c_str());
//     fprintf(fp, "\n");
//
//     // T_WF
//     fprintf(fp, "T_WF:\n");
//     fprintf(fp, "  rows: 4\n");
//     fprintf(fp, "  cols: 4\n");
//     fprintf(fp, "  data: [\n");
//     fprintf(fp, "%s\n", mat2str(T_WF, "    ").c_str());
//     fprintf(fp, "  ]\n");
//     fprintf(fp, "\n");
//
//     // T_MC0
//     fprintf(fp, "T_MC0:\n");
//     fprintf(fp, "  rows: 4\n");
//     fprintf(fp, "  cols: 4\n");
//     fprintf(fp, "  data: [\n");
//     fprintf(fp, "%s\n", mat2str(T_MC0, "    ").c_str());
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
//       fprintf(fp, "%lf,%lf,%lf,", r.x(), r.y(), r.z());
//       fprintf(fp, "%lf,%lf,%lf,%lf", q.x(), q.y(), q.z(), q.w());
//       fprintf(fp, "\n");
//     }
//     fclose(fp);
//   }
//
//   {
//     FILE *fp = fopen("/tmp/T_WF.csv", "w");
//     const quat_t q = tf_quat(data.T_WF.tf());
//     const vec3_t r = tf_trans(data.T_WF.tf());
//     fprintf(fp, "%lf,%lf,%lf,", r.x(), r.y(), r.z());
//     fprintf(fp, "%lf,%lf,%lf,%lf", q.x(), q.y(), q.z(), q.w());
//     fprintf(fp, "\n");
//     fclose(fp);
//   }
//
//   {
//     FILE *fp = fopen("/tmp/T_MC0.csv", "w");
//     const quat_t q = tf_quat(data.T_MC0.tf());
//     const vec3_t r = tf_trans(data.T_MC0.tf());
//     fprintf(fp, "%lf,%lf,%lf,", r.x(), r.y(), r.z());
//     fprintf(fp, "%lf,%lf,%lf,%lf", q.x(), q.y(), q.z(), q.w());
//     fprintf(fp, "\n");
//     fclose(fp);
//   }
// }

/* Solve the camera-mocap_marker extrinsics */
int calib_mocap_solve(const calib_mocap_data_t &data);

/* Solve the camera-mocap_marker extrinsics */
int calib_mocap_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_MOCAP_HPP
