#include "munit.hpp"
#include "euroc_test_data.hpp"
#include "calib_stereo.hpp"
#include "calib_nbv.hpp"
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
  std::vector<camera_params_t *> cam_params;
  std::vector<extrinsics_t *> cam_exts;
  pose_t * pose;
  std::map<int, std::vector<reproj_error_t<pinhole_radtan4_t> *>> reproj_errors;
};

int test_marg_error() {
  // Test data
  test_data_t test_data = setup_test_data();

  // Filter out grids not detected
  const size_t max_grids = 10;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;

  const size_t grids0_end = test_data.grids0.size();
  const size_t grids1_end = test_data.grids1.size();
  size_t grid0_idx = 0;
  size_t grid1_idx = 0;
  while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
    auto grid0 = test_data.grids0[grid0_idx];
    auto grid1 = test_data.grids1[grid1_idx];

    if (grid0.timestamp > grid1.timestamp) {
      grid1_idx++;
    } else if (grid0.timestamp < grid1.timestamp) {
      grid0_idx++;
    } else if (grid0.timestamp == grid1.timestamp) {
      if (grid0.detected && grid1.detected) {
        cam0_grids.push_back(grid0);
        cam1_grids.push_back(grid1);
      }
      if (cam0_grids.size() == max_grids) {
        break;
      }
      grid0_idx++;
      grid1_idx++;
    }
  }

  // Setup camera
  const int cam_res[2] = {752, 480};
  id_t param_id = 0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam0_params{param_id++, 0, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};
  camera_params_t cam1_params{param_id++, 1, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};

  // Setup problem
  ceres::Problem problem;
  marg_error_t *marg_error = new marg_error_t();

  mat4_t T_BC0 = I(4);
  mat4_t T_BC1;
  T_BC1 << 0.99998, -0.00254, -0.00498,  0.10995,
           0.00247,  0.99989, -0.01440, -0.00025,
           0.00502,  0.01439,  0.99988,  0.00062,
           0.00000,  0.00000,  0.00000,  1.00000;
  extrinsics_t cam0_exts{param_id++, T_BC0};
  extrinsics_t cam1_exts{param_id++, T_BC1};

  std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
  std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
  std::map<timestamp_t, pose_t *> rel_poses;
  std::vector<view_t> views;

  for (size_t k = 0; k < max_grids; k++) {
    std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
    view_t view;
    view.cam_params = cam_params;
    view.cam_exts = cam_exts;

    for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
      const auto grid = cam_grids[cam_idx];
      const vec4_t proj_params = cam0_params.proj_params();
      const vec4_t dist_params = cam0_params.dist_params();
      const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};

      // Estimate relative pose (T_C0F)
      mat4_t T_CiF;
      if (grid->estimate(cam, T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }
      mat4_t T_BCi = cam_exts[cam_idx]->tf();
      mat4_t T_BF = T_BCi * T_CiF;

      // Form relative pose
      const auto ts = grid->timestamp;
      pose_t *rel_pose;
      if (rel_poses.count(ts) == 0) {
        rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
        rel_poses[ts] = rel_pose;
      } else {
        rel_pose = rel_poses[ts];
      }
      view.pose = rel_pose;

      // Form reprojection residual block
      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;
      vec3s_t object_points;
      grid->get_measurements(tag_ids, corner_indicies, keypoints, object_points);

      for (size_t i = 0; i < tag_ids.size(); i++) {
        auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
                                                         cam_res,
                                                         tag_ids[i],
                                                         corner_indicies[i],
                                                         object_points[i],
                                                         keypoints[i],
                                                         I(2));

        view.reproj_errors[cam_idx].push_back(err);
        problem.AddResidualBlock(err,
                                 nullptr,
                                 rel_pose->param.data(),
                                 cam_exts[cam_idx]->param.data(),
                                 cam_params[cam_idx]->param.data());
      }
    }

    views.push_back(view);
  }

  printf("[before marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  printf("[before marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());

  // Marginalize
  const auto view = views[0];
  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
      view.pose->marginalize = true;
      std::vector<param_t *> param_blocks;
      param_blocks.push_back((param_t *) view.pose);
      param_blocks.push_back((param_t *) view.cam_exts[cam_idx]);
      param_blocks.push_back((param_t *) view.cam_params[cam_idx]);
      marg_error->add(reproj_error, param_blocks);
    }
  }
  marg_error->marginalize(problem);
  problem.AddResidualBlock(marg_error, nullptr, marg_error->get_param_ptrs());

  printf("[after marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  printf("[after marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());

  print_vector("cam0_params", cam_params[0]->param);
  print_vector("cam0_exts", cam_exts[0]->param);
  print_vector("cam1_params", cam_params[1]->param);
  print_vector("cam1_exts", cam_exts[1]->param);

  // Solve
  problem.SetParameterBlockConstant(cam_exts[0]->param.data());
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << std::endl;

  print_vector("cam0_params", cam_params[0]->param);
  print_vector("cam0_exts", cam_exts[0]->param);
  print_vector("cam1_params", cam_params[1]->param);
  print_vector("cam1_exts", cam_exts[1]->param);

  return 0;
}

int test_marg_error_check_gradients() {
  // Test data
  test_data_t test_data = setup_test_data();

  // Filter out grids not detected
  const size_t max_grids = 10;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;

  const size_t grids0_end = test_data.grids0.size();
  const size_t grids1_end = test_data.grids1.size();
  size_t grid0_idx = 0;
  size_t grid1_idx = 0;
  while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
    auto grid0 = test_data.grids0[grid0_idx];
    auto grid1 = test_data.grids1[grid1_idx];

    if (grid0.timestamp > grid1.timestamp) {
      grid1_idx++;
    } else if (grid0.timestamp < grid1.timestamp) {
      grid0_idx++;
    } else if (grid0.timestamp == grid1.timestamp) {
      if (grid0.detected && grid1.detected) {
        cam0_grids.push_back(grid0);
        cam1_grids.push_back(grid1);
      }
      if (cam0_grids.size() == max_grids) {
        break;
      }
      grid0_idx++;
      grid1_idx++;
    }
  }

  // Setup camera
  const int cam_res[2] = {752, 480};
  id_t param_id = 0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam0_params{param_id++, 0, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};
  camera_params_t cam1_params{param_id++, 1, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};

  // Setup problem
  ceres::Problem problem;
  marg_error_t *marg_error = new marg_error_t();

  mat4_t T_BC0 = I(4);
  mat4_t T_BC1;
  T_BC1 << 0.99998, -0.00254, -0.00498,  0.10995,
           0.00247,  0.99989, -0.01440, -0.00025,
           0.00502,  0.01439,  0.99988,  0.00062,
           0.00000,  0.00000,  0.00000,  1.00000;
  extrinsics_t cam0_exts{param_id++, T_BC0};
  extrinsics_t cam1_exts{param_id++, T_BC1};

  std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
  std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
  std::map<timestamp_t, pose_t *> rel_poses;
  std::vector<view_t> views;

  for (size_t k = 0; k < max_grids; k++) {
    std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
    view_t view;
    view.cam_params = cam_params;
    view.cam_exts = cam_exts;

    for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
      const auto grid = cam_grids[cam_idx];
      const vec4_t proj_params = cam0_params.proj_params();
      const vec4_t dist_params = cam0_params.dist_params();
      const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};

      // Estimate relative pose (T_C0F)
      mat4_t T_CiF;
      if (grid->estimate(cam, T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }
      mat4_t T_BCi = cam_exts[cam_idx]->tf();
      mat4_t T_BF = T_BCi * T_CiF;

      // Form relative pose
      const auto ts = grid->timestamp;
      pose_t *rel_pose;
      if (rel_poses.count(ts) == 0) {
        rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
        rel_poses[ts] = rel_pose;
      } else {
        rel_pose = rel_poses[ts];
      }
      view.pose = rel_pose;

      // Form reprojection residual block
      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;
      vec3s_t object_points;
      grid->get_measurements(tag_ids, corner_indicies, keypoints, object_points);

      for (size_t i = 0; i < tag_ids.size(); i++) {
        auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
                                                         cam_res,
                                                         tag_ids[i],
                                                         corner_indicies[i],
                                                         object_points[i],
                                                         keypoints[i],
                                                         I(2));

        view.reproj_errors[cam_idx].push_back(err);
        problem.AddResidualBlock(err,
                                 nullptr,
                                 rel_pose->param.data(),
                                 cam_exts[cam_idx]->param.data(),
                                 cam_params[cam_idx]->param.data());
      }
    }

    views.push_back(view);
  }

  printf("[before marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  printf("[before marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());

  // Marginalize
  const auto view = views[0];
  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
      view.pose->marginalize = true;
      std::vector<param_t *> param_blocks;
      param_blocks.push_back((param_t *) view.pose);
      param_blocks.push_back((param_t *) view.cam_exts[cam_idx]);
      param_blocks.push_back((param_t *) view.cam_params[cam_idx]);
      marg_error->add(reproj_error, param_blocks);
    }
  }
  marg_error->marginalize(problem, true);
  problem.AddResidualBlock(marg_error, nullptr, marg_error->get_param_ptrs());

  printf("[after marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  printf("[after marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());

  std::vector<double *> param_ptrs = marg_error->get_param_ptrs();

  const auto r_size = marg_error->e0_.size();
  vecx_t r = zeros(r_size, 1);

  std::vector<param_t *> params = marg_error->get_params();
  double **jacobians = new double *[params.size()];
  for (size_t i = 0; i < params.size(); i++) {
    auto param = params[i];
    jacobians[i] = (double *) malloc(sizeof(double) * r.size() * param->global_size);
  }
  marg_error->Evaluate(param_ptrs.data(), r.data(), jacobians);
  for (const auto param : params) {
    printf("param[%s]\n", param->type.c_str());
  }

  // Check jacobians
  double step = 1e-8;
  double threshold = 1e-4;

  printf("marg_error->x0.size(): %ld\n", marg_error->x0_.size());
  mat2csv("/tmp/e0.csv", marg_error->e0_);
  mat2csv("/tmp/J0.csv", marg_error->J0_);

  for (size_t param_idx = 0; param_idx < params.size(); param_idx++) {
    auto &param = params[param_idx];

    matx_t fdiff = zeros(r_size, param->local_size);
    for (int i = 0; i < param->local_size; i++) {
      vecx_t r_fd = zeros(r_size, 1);
      param->perturb(i, step);
      marg_error->Evaluate(param_ptrs.data(), r_fd.data(), nullptr);
      fdiff.col(i) = (r_fd - r) / step;
      param->perturb(i, -step);
    }

    auto J_name = "J" + std::to_string(param_idx);
    auto J_rows = r_size;
    auto J_cols = param->global_size;
    auto J_min_cols = param->local_size;

    Eigen::Map<matx_row_major_t> J(jacobians[param_idx], J_rows, J_cols);
    const matx_t J_min = J.block(0, 0, J_rows, J_min_cols);

    MU_CHECK(check_jacobian(J_name, fdiff, J_min, threshold, true) == 0);
  }

  return 0;
}

int test_marg_error_swe() {
  // Test data
  test_data_t test_data = setup_test_data();

  // Filter out grids not detected
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;

  const size_t grids0_end = test_data.grids0.size();
  const size_t grids1_end = test_data.grids1.size();
  size_t grid0_idx = 0;
  size_t grid1_idx = 0;
  while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
    auto grid0 = test_data.grids0[grid0_idx];
    auto grid1 = test_data.grids1[grid1_idx];

    if (grid0.timestamp > grid1.timestamp) {
      grid1_idx++;
    } else if (grid0.timestamp < grid1.timestamp) {
      grid0_idx++;
    } else if (grid0.timestamp == grid1.timestamp) {
      if (grid0.detected && grid1.detected) {
        cam0_grids.push_back(grid0);
        cam1_grids.push_back(grid1);
      }
      grid0_idx++;
      grid1_idx++;
    }
  }

  // Setup camera
  const int cam_res[2] = {752, 480};
  id_t param_id = 0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam0_params{param_id++, 0, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};
  camera_params_t cam1_params{param_id++, 1, cam_res,
                              proj_model, dist_model,
                              proj_params, dist_params};

  // Setup camera extrinsics
  mat4_t T_BC0 = I(4);
  mat4_t T_BC1 = I(4);
  // T_BC1 << 0.99998, -0.00254, -0.00498,  0.10995,
  //          0.00247,  0.99989, -0.01440, -0.00025,
  //          0.00502,  0.01439,  0.99988,  0.00062,
  //          0.00000,  0.00000,  0.00000,  1.00000;
  extrinsics_t cam0_exts{param_id++, T_BC0};
  extrinsics_t cam1_exts{param_id++, T_BC1};

  // Sliding window estimation
  std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
  std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
  std::map<timestamp_t, pose_t *> rel_poses;
  std::deque<view_t> sliding_window;

  ceres::Problem problem;
  PoseLocalParameterization pose_plus;

  problem.AddParameterBlock(cam_exts[0]->param.data(), 7);
  problem.AddParameterBlock(cam_exts[1]->param.data(), 7);
  problem.SetParameterBlockConstant(cam_exts[0]->param.data());
  problem.SetParameterization(cam_exts[0]->param.data(), &pose_plus);
  problem.SetParameterization(cam_exts[1]->param.data(), &pose_plus);

  marg_error_t *marg_error = new marg_error_t();
  ceres::ResidualBlockId marg_error_id = nullptr;

  size_t max_window_size = 10;
  auto nb_cams = 2;
  auto nb_grids = cam0_grids.size();

  for (size_t k = 0; k < nb_grids; k++) {
    std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
    view_t view;
    view.cam_params = cam_params;
    view.cam_exts = cam_exts;

    // Keep sliding window bounded
    if (sliding_window.size() >= max_window_size) {
      // Remove previous residual block if it exists
      if (marg_error_id != nullptr) {
        problem.RemoveResidualBlock(marg_error_id);

        // Replace old marginalization error, and keep track of previous info
        auto marg_error_new = new marg_error_t();
        marg_error_new->add(marg_error, marg_error->get_params());
        delete marg_error;
        marg_error = marg_error_new;
      }

      // Pop the oldest view from the sliding window
      const auto view = sliding_window.front();
      sliding_window.pop_front();
      // problem.RemoveParameterBlock(view.pose->param.data());

      // Marginalize the oldest pose and add marginalization prior to problem
      for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
        for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
          view.pose->marginalize = true;
          std::vector<param_t *> param_blocks;
          param_blocks.push_back((param_t *) view.pose);
          param_blocks.push_back((param_t *) view.cam_exts[cam_idx]);
          param_blocks.push_back((param_t *) view.cam_params[cam_idx]);
          marg_error->add(reproj_error, param_blocks);
        }
      }
      marg_error->marginalize(problem);
      marg_error_id = problem.AddResidualBlock(marg_error,
                                               nullptr,
                                               marg_error->get_param_ptrs());
    }

    // Add view to sliding window
    for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
      // Estimate relative pose (T_C0F)
      const auto grid = cam_grids[cam_idx];
      const vec4_t proj_params = cam0_params.proj_params();
      const vec4_t dist_params = cam0_params.dist_params();
      const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
      mat4_t T_CiF;
      if (grid->estimate(cam, T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }
      mat4_t T_BCi = cam_exts[cam_idx]->tf();
      mat4_t T_BF = T_BCi * T_CiF;

      // Form relative pose
      const auto ts = grid->timestamp;
      pose_t *rel_pose;
      if (rel_poses.count(ts) == 0) {
        rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
        rel_poses[ts] = rel_pose;
      } else {
        rel_pose = rel_poses[ts];
      }
      view.pose = rel_pose;

      // Form reprojection residual block for each observation from each camera
      std::vector<int> tag_ids;
      std::vector<int> corner_indicies;
      vec2s_t keypoints;
      vec3s_t object_points;
      grid->get_measurements(tag_ids, corner_indicies, keypoints, object_points);

      for (size_t i = 0; i < tag_ids.size(); i++) {
        auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
                                                         cam_res,
                                                         tag_ids[i],
                                                         corner_indicies[i],
                                                         object_points[i],
                                                         keypoints[i],
                                                         I(2));

        view.reproj_errors[cam_idx].push_back(err);
        problem.AddResidualBlock(err,
                                 nullptr,
                                 rel_pose->param.data(),
                                 cam_exts[cam_idx]->param.data(),
                                 cam_params[cam_idx]->param.data());
      }
      problem.SetParameterization(rel_pose->param.data(), &pose_plus);
    }
    sliding_window.push_back(view);

    // Solve window
    if (sliding_window.size() >= max_window_size) {
      ceres::Solver::Options options;
      options.max_num_iterations = 10;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      // Obtain the calibration covariance matrix and calculate entropy. This
      // is one way to make sure the marginalization error is working. If the
      // marg_error_t() is working the entropy should be increasing over time.
      matx_t covar;
      if (calib_covar(problem, cam0_params, cam1_params, cam1_exts, covar) == 0) {
        printf("entropy: %f\t", entropy(covar));
      }
      std::cout << summary.BriefReport() << std::endl;
    }

  }

  print_vector("cam0_params", cam_params[0]->param);
  print_vector("cam0_exts", cam_exts[0]->param);
  print_vector("cam1_params", cam_params[1]->param);
  print_vector("cam1_exts", cam_exts[1]->param);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_residual_info);
  MU_ADD_TEST(test_marg_error);
  MU_ADD_TEST(test_marg_error_check_gradients);
  MU_ADD_TEST(test_marg_error_swe);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
