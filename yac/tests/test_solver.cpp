#include "munit.hpp"
#include "solver.hpp"

namespace yac {

#define TEST_CAM_CALIB_PATH TEST_PATH "/test_data/calib/cam_april"

int test_yac_solver() {
  // Load test data
  std::vector<std::string> cam0_files;
  list_files(TEST_CAM_CALIB_PATH "/cam0", cam0_files);

  aprilgrids_t grids;
  for (auto grid_path : cam0_files) {
    grid_path = std::string(TEST_CAM_CALIB_PATH "/cam0/") + grid_path;
    grids.emplace_back(grid_path);
  }

  // Setup camera
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-5};
  pinhole_radtan4_t cam_geom;
  camera_params_t cam_params(cam_idx,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params);
  mat4_t T_C0C0 = I(4);
  extrinsics_t cam_exts{T_C0C0, true};

  // Setup fiducial target
  calib_target_t calib_target;
  fiducial_corners_t corners{calib_target};

  // Setup solver
  ceres_solver_t solver;
  std::vector<pose_t *> cam_poses;
  std::vector<calib_residual_t *> res_fns;

  solver.add_param(&cam_params);
  solver.add_param(&cam_exts);
  for (const auto grid : grids) {
    // Pre-check
    if (grid.detected == false) {
      continue;
    }

    // Timestamp
    const timestamp_t ts = grid.timestamp;

    // Estimate camera-fiducial pose
    mat4_t T_C0F;
    if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_C0F) != 0) {
      FATAL("Failed to estimate relative pose!");
    }
    auto cam_pose = new pose_t{ts, T_C0F};
    cam_poses.push_back(cam_pose);
    solver.add_param(cam_pose);

    // Get AprilGrid measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t pts;
    grid.get_measurements(tag_ids, corner_idxs, kps, pts);

    // Add reprojection errors
    mat2_t covar = I(2);
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_idxs[i];
      const vec2_t z = kps[i];
      auto p_FFi = corners.get_corner(tag_id, corner_idx);
      auto res_fn = new reproj_residual_t(&cam_geom,
                                          &cam_params,
                                          &cam_exts,
                                          cam_pose,
                                          p_FFi,
                                          z,
                                          covar);
      solver.add_residual(res_fn);
      res_fns.push_back(res_fn);
    }
  }

  // // Old eval
  // {
  //   profiler_t prof;
  //   prof.start("eval_residuals");
  //   ParameterOrder param_order;
  //   std::vector<calib_residual_t *> res_evaled;
  //   ResidualJacobians res_jacs;
  //   ResidualJacobians res_min_jacs;
  //   ResidualValues res_vals;
  //   size_t residuals_length;
  //   size_t params_length;
  //   solver._eval_residuals(param_order,
  //                          res_evaled,
  //                          res_jacs,
  //                          res_min_jacs,
  //                          res_vals,
  //                          residuals_length,
  //                          params_length);
  //   prof.stop("eval_residuals");
  //   prof.print("eval_residuals");
  //
  //   printf("nb residuals evaluated: %ld\n", res_evaled.size());
  //   printf("residuals length: %ld\n", residuals_length);
  //   printf("params length: %ld\n", params_length);
  //
  //   print_matrix("J0", res_min_jacs[res_evaled.back()][0]);
  //   print_matrix("J1", res_min_jacs[res_evaled.back()][1]);
  //   print_matrix("J2", res_min_jacs[res_evaled.back()][2]);
  // }

  // // New eval
  // {
  //   profiler_t prof;
  //   prof.start("eval_residuals");
  //   ParameterOrder param_order;
  //   std::vector<calib_residual_t *> res_evaled;
  //   size_t residuals_length;
  //   size_t params_length;
  //   solver._eval_residuals(param_order,
  //                          res_evaled,
  //                          residuals_length,
  //                          params_length);
  //   prof.stop("eval_residuals");
  //   prof.print("eval_residuals");
  //
  //   printf("nb residuals evaluated: %ld\n", res_evaled.size());
  //   printf("residuals length: %ld\n", residuals_length);
  //   printf("params length: %ld\n", params_length);
  //
  //   print_matrix("J0", res_evaled.back()->min_jacobian_blocks[0]);
  //   print_matrix("J1", res_evaled.back()->min_jacobian_blocks[1]);
  //   print_matrix("J2", res_evaled.back()->min_jacobian_blocks[2]);
  // }

  {
    profiler_t prof;

    std::vector<param_t *> params;
    params.push_back(&cam_params);

    prof.start("solver.estimate_log_covariance_determinant()");
    real_t covar_det = 0.0;
    solver.estimate_log_covariance_determinant(params, covar_det);
    prof.stop("solver.estimate_log_covariance_determinant()");

    printf("log(det(covar)): %.10f\n", covar_det);
    prof.print("solver.estimate_log_covariance_determinant()");
  }

  // // Solve
  // printf("num_residuals: %ld\n", solver.num_residuals());
  // printf("num_parameters: %ld\n", solver.num_params());
  // solver.solve(30, true, 1);

  // Clean up
  for (auto res_fn : res_fns) {
    delete res_fn;
  }
  for (auto pose : cam_poses) {
    delete pose;
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_yac_solver); }

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
