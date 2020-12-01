#ifndef YAC_NBV_HPP
#define YAC_NBV_HPP

#include "core.hpp"
#include "calib_data.hpp"

namespace yac {

mat4_t calib_target_origin(const calib_target_t &target,
                           const vec2_t &cam_res,
                           const double hfov);

mat4s_t calib_generate_poses(const calib_target_t &target);

mat4s_t generate_initial_poses(const calib_target_t &target);

mat4s_t generate_nbv_poses(const calib_target_t &target);

// template <typename CAMERA>
// struct test_grid_t {
//   const int grid_rows = 5;
//   const int grid_cols = 5;
//   const double grid_depth = 5.0;
//   vec2s_t keypoints;
//   vec3s_t object_points;
//
//   test_grid_t(const camera_params_t &cam_params) {
//     const double dx = cam_params.resolution[0] / (grid_cols + 1);
//     const double dy = cam_params.resolution[1] / (grid_rows + 1);
//
//     auto cam_res = cam_params.resolution;
//     auto proj_params = cam_params.proj_params();
//     auto dist_params = cam_params.dist_params();
//     CAMERA camera{cam_res, proj_params, dist_params};
//
//     double kp_x = 0;
//     double kp_y = 0;
//     for (int i = 1; i < (grid_cols + 1); i++) {
//       kp_y += dy;
//       for (int j = 1; j < (grid_rows + 1); j++) {
//         kp_x += dx;
//
//         // Keypoint
//         const vec2_t kp{kp_x, kp_y};
//         keypoints.push_back(kp);
//
//         // Object point
//         Eigen::Vector3d ray;
//         camera.back_project(kp, ray);
//         object_points.push_back(ray * grid_depth);
//       }
//       kp_x = 0;
//     }
//   }
// };

// /** NBV for Monocular Camera Calibration **/
// struct nbv_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   calib_target_t calib_target;
//   calib_mono_covar_est_t covar_est;
//
//   nbv_t(const calib_target_t &target_,
//         const camera_params_t &cam_params_,
//         const aprilgrids_t &aprilgrids_)
//     : calib_target{target_}, covar_est{cam_params_, aprilgrids_} {}
//
//   virtual ~nbv_t() {}
//
//   int find_nbv(const calib_target_t &target) {
//     mat4s_t nbv_poses = generate_nbv_poses(target);
//     auto cam_params = covar_est.cam_params;
//
//     int cam_res[2] = {cam_params->resolution[0], cam_params->resolution[1]};
//     auto proj_params = cam_params->proj_params();
//     auto dist_params = cam_params->dist_params();
//     pinhole_radtan4_t camera{cam_res, proj_params, dist_params};
//
//     matx_t covar;
//     int retval = covar_est.estimate(covar);
//     if (retval == -1) {
//       LOG_WARN("Failed to recover covariance!");
//       return -1;
//     }
//     auto base_score = covar.trace();
//     printf("base score: %f\n", base_score);
//
//     real_t best_score = 0.0;
//     mat4_t best_pose = I(4);
//     for (const auto &pose : nbv_poses) {
//       aprilgrid_t grid;
//       aprilgrid_set_properties(grid,
//                                target.tag_rows,
//                                target.tag_cols,
//                                target.tag_size,
//                                target.tag_spacing);
//       grid.T_CF = pose;
//
//       for (int tag_id = 0; tag_id < 36; tag_id++) {
//         vec3s_t object_points;
//         vec3s_t test_points;
//         aprilgrid_object_points(grid, tag_id, object_points);
//         for (int i = 0; i < 4; i++) {
//           // auto test_point = object_points[i];
//           // test_point(2) = 5.0;
//           auto test_point = tf_point(pose, object_points[i]);
//           test_points.push_back(test_point);
//         }
//
//         vec2s_t keypoints;
//         for (int i = 0; i < 4; i++) {
//           vec2_t kp;
//           camera.project(test_points[i], kp);
//           keypoints.push_back(kp);
//         }
//
//         aprilgrid_add(grid, tag_id, keypoints);
//       }
//       covar_est.add(grid);
//
//       matx_t covar_nbv;
//       int retval = covar_est.estimate(covar_nbv);
//       if (retval == -1) {
//         LOG_WARN("Failed to recover covariance! Skipping this pose!");
//         continue;
//       }
//
//       auto score = covar_nbv.trace();
//       // auto score =  shannon_entropy(covar);
//
//       printf("score: %f\n", score);
//       printf("diff score: %f\n", base_score - score);
//       if (score > best_score) {
//         best_score = score;
//         best_pose = pose;
//       }
//       covar_est.remove_last();
//     }
//
//     printf("best score: %f\n", best_score);
//     printf("diff: %f\n", base_score - best_score);
//     print_matrix("best pose", best_pose);
//
//     return 0;
//   }
// };

} //  namespace yac
#endif // YAC_NBV_HPP
