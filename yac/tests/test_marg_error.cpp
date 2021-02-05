#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_stereo.hpp"
#include "marg_error.hpp"

namespace yac {

int test_residual_info() {
  // Test data
  test_data_t test_data = setup_test_data();

  // Filter out grids not detected
  aprilgrid_t grid;
  for (size_t i = 0; i < test_data.grids0.size(); i++) {
    if (test_data.grids0[i].detected) {
      grid = test_data.grids0[i];
      break;
    }
  }

  // Setup camera
  id_t param_id = 0;
  const int cam0_idx = 0;
  const int cam0_res[2] = {752, 480};
  const std::string cam0_proj_model = "pinhole";
  const std::string cam0_dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{param_id++, cam0_idx, cam0_res,
                             cam0_proj_model, cam0_dist_model,
                             proj_params, dist_params};
  const pinhole_radtan4_t cam{cam0_res, proj_params, dist_params};

  // Setup cam extrinsics and relative poses
  mat4_t T_CiF;
  if (grid.estimate(cam, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }
  mat4_t T_BCi = I(4);
  mat4_t T_BF = T_BCi * T_CiF;
  pose_t cam_extrinsics{param_id++, grid.timestamp, T_BCi};
  pose_t rel_pose{param_id++, grid.timestamp, T_BF};

  // Form reprojection residual block
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);
  reproj_error_t<pinhole_radtan4_t> err(cam0_idx, cam0_res,
                                        tag_ids[1],
                                        corner_indicies[1],
                                        object_points[1],
                                        keypoints[1],
                                        I(2));

  // Marginalization block
  std::vector<param_t *> param_blocks;
  param_blocks.push_back((param_t *) &rel_pose);
  param_blocks.push_back((param_t *) &cam_extrinsics);
  param_blocks.push_back((param_t *) &cam_params);
  residual_info_t residual_info{&err, param_blocks};

  // Assert
  MU_CHECK(residual_info.cost_fn == &err);
  MU_CHECK(residual_info.loss_fn == nullptr);
  MU_CHECK(residual_info.param_blocks.size() == 3);

  return 0;
}

struct view_t {
  camera_params_t *cam_params;
  extrinsics_t *extrinsics;
  pose_t * pose;
  std::vector<reproj_error_t<pinhole_radtan4_t> *> errors;
};

int test_marg_error() {
  // Test data
  test_data_t test_data = setup_test_data();

  // Filter out grids not detected
  aprilgrids_t grids;
  for (size_t i = 0; i < test_data.grids0.size(); i++) {
    if (test_data.grids0[i].detected) {
      grids.push_back(test_data.grids0[i]);
    }

    if (grids.size() == 5) {
      break;
    }
  }

  // Setup camera
  id_t param_id = 0;
  const int cam0_idx = 0;
  const int cam0_res[2] = {752, 480};
  const std::string cam0_proj_model = "pinhole";
  const std::string cam0_dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{param_id++, cam0_idx, cam0_res,
                             cam0_proj_model, cam0_dist_model,
                             proj_params, dist_params};
  const pinhole_radtan4_t cam{cam0_res, proj_params, dist_params};

  // Setup problem
  ceres::Problem problem;
  marg_error_t *marg_error = new marg_error_t();

  mat4_t T_BCi = I(4);
  extrinsics_t cam_extrinsics{param_id++, T_BCi};
  std::vector<pose_t *> rel_poses;
  std::vector<reproj_error_t<pinhole_radtan4_t> *> errors;

  // std::deque<

  for (const auto &grid : grids) {
    // Form relative pose
    mat4_t T_CiF;
    if (grid.estimate(cam, T_CiF) != 0) {
      FATAL("Failed to estimate relative pose!");
    }
    mat4_t T_BF = T_BCi * T_CiF;
    auto rel_pose = new pose_t{param_id++, grid.timestamp, T_BF};
    rel_poses.push_back(rel_pose);

    // Form reprojection residual block
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    for (size_t i = 0; i < tag_ids.size(); i++) {
      auto err = new reproj_error_t<pinhole_radtan4_t>(cam0_idx,
                                                       cam0_res,
                                                       tag_ids[i],
                                                       corner_indicies[i],
                                                       object_points[i],
                                                       keypoints[i],
                                                       I(2));
      errors.push_back(err);

      // std::vector<param_t *> param_blocks;
      // param_blocks.push_back((param_t *) rel_pose);
      // param_blocks.push_back((param_t *) &cam_extrinsics);
      // param_blocks.push_back((param_t *) &cam_params);
      // marg_error->add(err, param_blocks);

      problem.AddResidualBlock(err,
                               nullptr,
                               rel_pose->param.data(),
                               cam_extrinsics.param.data(),
                               cam_params.param.data());
    }
  }

  // matx_t H;
  // vecx_t b;
  // marg_error.form_hessian(H, b, true);

  // rel_poses[0]->marginalize = true;
  // rel_poses[1]->marginalize = true;
  // marg_error->marginalize(problem);
  // problem.AddResidualBlock(marg_error, nullptr, marg_error->get_param_ptrs());

  printf("nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  printf("nb_residual_blocks: %d\n", problem.NumResidualBlocks());

  print_vector("cam_params", cam_params.param);
  print_vector("extrinsics", cam_extrinsics.param);

  // vecx_t residuals{marg_error->get_residual_size()};
  // std::vector<double *> params = marg_error->get_param_ptrs();
  // marg_error->Evaluate(params.data(), residuals.data());

  // Solve
  problem.SetParameterBlockConstant(cam_extrinsics.param.data());
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  print_vector("cam_params", cam_params.param);
  print_vector("extrinsics", cam_extrinsics.param);

  // mat2csv("/tmp/H.csv", H);
  // mat2csv("/tmp/b.csv", b);
  // print_matrix("H", H);
  // print_vector("b", b);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_residual_info);
  MU_ADD_TEST(test_marg_error);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
