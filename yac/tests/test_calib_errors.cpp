#include "munit.hpp"
#include "calib_errors.hpp"

namespace yac {

std::map<int, aprilgrids_t> setup_test_data() {
  // Calibration data
  const calib_target_t calib_target;
  const std::string data_path = "/data/euroc/cam_april";
  const std::map<int, std::string> cam_paths = {
      {0, data_path + "/mav0/cam0/data"},
      {1, data_path + "/mav0/cam1/data"},
  };
  const std::string grids_path = "/data/euroc/cam_april/mav0/grid0";
  return calib_data_preprocess(calib_target, cam_paths, grids_path);
}

timeline_t setup_camimu_test_data() {
  // Load calibration data
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> grid_paths = {
      {0, data_path + "/mav0/grid0/cam0"},
      {1, data_path + "/mav0/grid0/cam1"},
  };
  LOG_INFO("Loading EuRoC data [%s]", data_path.c_str());

  // Preprocess aprilgrids
  LOG_INFO("Preprocessing AprilGrid data ...");
  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    if (system(("mkdir -p " + grid_paths[cam_idx]).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", grid_paths[cam_idx].c_str());
    }
  }

  // Loop through timeline
  euroc_calib_t calib_data{data_path};
  auto timeline = calib_data.timeline();
  size_t i = 0;

  for (const auto &ts : timeline.timestamps) {
    if (i++ % 100 == 0) {
      printf(".");
      fflush(stdout);
    }

    for (const auto &event : timeline.get_events(ts)) {
      if (auto cam_event = dynamic_cast<camera_event_t *>(event)) {
        // Setup
        const timestamp_t ts = cam_event->ts;
        const int cam_idx = cam_event->cam_idx;
        const auto flag = cv::IMREAD_GRAYSCALE;
        const cv::Mat image = cv::imread(cam_event->img_path, flag);
        const std::string grid_fname = std::to_string(ts) + ".csv";
        const std::string grid_file = grid_paths[cam_idx] + "/" + grid_fname;

        // Detect or load
        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
          grid = detector.detect(ts, image);
          grid.save(grid_file);
        } else {
          grid.load(grid_file);
          if (grid.detected == false) {
            grid.timestamp = ts;
            grid.tag_rows = detector.tag_rows;
            grid.tag_cols = detector.tag_cols;
            grid.tag_size = detector.tag_size;
            grid.tag_spacing = detector.tag_spacing;
          }
        }

        // Add aprilgrid to timeline
        timeline.add(ts, cam_idx, grid);
      }
    }
  }
  printf("\n");

  return timeline;
}

int test_pose_error() {
  const vec3_t r{0.1, 0.2, 0.3};
  const vec3_t rpy{0.1, 0.2, 0.3};
  const quat_t q = euler2quat(rpy);
  pose_t pose{0, tf(q, r)};
  matx_t covar = I(6);
  pose_error_t err(&pose, covar);
  MU_CHECK(err.check_jacs(0, "J_pose"));

  return 0;
}

int test_reproj_error() {
  // Load data
  auto cam_grids = setup_test_data();

  // Get 1 detected aprilgrid
  aprilgrid_t grid;
  for (size_t k = 0; k < cam_grids[0].size(); k++) {
    if (cam_grids[0][k].detected) {
      grid = cam_grids[0][k];
      break;
    }
  }

  // Form reprojection error
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{cam_idx,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};
  pinhole_radtan4_t cam_geom;

  mat4_t T_CiF;
  if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  const calib_target_t calib_target;
  mat4_t T_C0Ci = I(4);
  mat4_t T_C0F = T_C0Ci * T_CiF;
  pose_t cam_exts{grid.timestamp, T_C0Ci};
  pose_t rel_pose{grid.timestamp, T_C0F}; // Fiducial corners
  fiducial_corners_t corners{calib_target};
  auto corner = corners.get_corner(tag_ids[0], corner_indicies[0]);

  reproj_error_t err(&cam_geom,
                     &cam_params,
                     &cam_exts,
                     &rel_pose,
                     corner,
                     keypoints[0],
                     I(2));

  MU_CHECK(err.check_jacs(0, "J_cam_exts"));
  MU_CHECK(err.check_jacs(1, "J_rel_pose"));
  MU_CHECK(err.check_jacs(2, "J_fiducial"));
  MU_CHECK(err.check_jacs(3, "J_cam"));

  return 0;
}

int test_fiducial_error() {
  // Fiducial pose in world frame
  // clang-format off
  mat4_t T_WF;
  T_WF << 0.0, 0.0, 1.0, -1.0,
          1.0, 0.0, 0.0, -0.4,
          0.0, 1.0, 0.0, -0.5,
          0.0, 0.0, 0.0, 1.0;
  T_WF = tf_perturb_rot(T_WF, 0.01, 0);
  fiducial_t fiducial{T_WF};
  // clang-format on

  // Imu pose in world frame
  // clang-format off
  mat4_t T_WS;
  T_WS << -1.0, 0.0, 0.0, 0.1,
          0.0, -1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  T_WS = tf_perturb_rot(T_WS, 0.01, 1);
  pose_t pose{0, T_WS};
  // ^ Note: Due to numerical stability issues the translation component
  // cannot be 0 for checking jacobians
  // clang-format on

  // Camera extrinsics
  // clang-format off
  mat4_t T_BC0;
  T_BC0 << 0.0, -1.0, 0.0, 0.001,
           1.0, 0.0, 0.0, 0.001,
           0.0, 0.0, 1.0, 0.001,
           0.0, 0.0, 0.0, 1.0;
  T_BC0 = tf_perturb_rot(T_BC0, 0.01, 2);
  extrinsics_t cam_exts{T_BC0};
  // clang-format on

  // Imu extrinsics
  // clang-format off
  mat4_t T_BS;
  T_BS << 0.0, 0.0, 1.0, 0.001,
          0.0, -1.0, 0.0, 0.001,
          1.0, 0.0, 0.0, 0.001,
          0.0, 0.0, 0.0, 1.0;
  T_BS = tf_perturb_rot(T_BS, -0.01, 2);
  extrinsics_t imu_exts{T_BS};
  // clang-format on

  // Get AprilGrid data
  aprilgrid_t grid{0, 6, 6, 0.088, 0.3};
  const int tag_id = 0;
  const int corner_idx = 2;
  const timestamp_t ts = 0;
  const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);

  // Camera parameters
  pinhole_radtan4_t cam_geom;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam_params{cam_idx,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};

  // Project image point
  const mat2_t covar = I(2);
  const mat4_t T_C0B = T_BC0.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_C0Fi = tf_point(T_C0B * T_BS * T_SW * T_WF, r_FFi);
  vec2_t z = zeros(2, 1);
  cam_geom.project(cam_res, cam_params.param, r_C0Fi, z);

  // Form fiducial error
  fiducial_error_t err(ts,
                       &cam_geom,
                       &cam_params,
                       &cam_exts,
                       &imu_exts,
                       &fiducial,
                       &pose,
                       tag_id,
                       corner_idx,
                       r_FFi,
                       z,
                       covar);

  MU_CHECK(err.check_jacs(0, "J_fiducial"));
  MU_CHECK(err.check_jacs(1, "J_pose"));
  MU_CHECK(err.check_jacs(2, "J_imu_exts"));
  MU_CHECK(err.check_jacs(2, "J_cam_exts"));
  MU_CHECK(err.check_jacs(3, "J_cam"));

  return 0;
}

struct view_t {
  std::map<int, camera_params_t *> cam_params;
  std::map<int, extrinsics_t *> cam_exts;
  std::map<int, std::vector<reproj_error_t *>> reproj_errors;
  pose_t *pose;
};

int test_marg_error() {
  // Test data
  auto cam_grids = setup_test_data();

  // Filter out grids not detected
  const size_t max_nb_grids = 10;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;

  const size_t grids0_end = cam_grids[0].size();
  const size_t grids1_end = cam_grids[1].size();
  size_t grid0_idx = 0;
  size_t grid1_idx = 0;
  while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
    auto grid0 = cam_grids[0][grid0_idx];
    auto grid1 = cam_grids[1][grid1_idx];

    if (grid0.timestamp > grid1.timestamp) {
      grid1_idx++;
    } else if (grid0.timestamp < grid1.timestamp) {
      grid0_idx++;
    } else if (grid0.timestamp == grid1.timestamp) {
      if (grid0.detected && grid1.detected) {
        cam0_grids.push_back(grid0);
        cam1_grids.push_back(grid1);
      }
      if (cam0_grids.size() == max_nb_grids) {
        break;
      }
      grid0_idx++;
      grid1_idx++;
    }
  }

  // Setup camera
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
  const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  camera_params_t cam0_params{0,
                              cam_res,
                              proj_model,
                              dist_model,
                              proj_params,
                              dist_params};
  camera_params_t cam1_params{1,
                              cam_res,
                              proj_model,
                              dist_model,
                              proj_params,
                              dist_params};

  // Setup problem
  ceres::Problem problem;
  // std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
  // std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
  // std::map<timestamp_t, pose_t *> rel_poses;
  // std::vector<view_t> views;
  marg_error_t *marg_error = new marg_error_t();

  // -- Camera extrinsics
  mat4_t T_BC0 = I(4);
  mat4_t T_BC1;
  // clang-format off
  T_BC1 << 0.99998, -0.00254, -0.00498, 0.10995,
           0.00247, 0.99989, -0.01440, -0.00025,
           0.00502, 0.01439, 0.99988, 0.00062,
           0.00000, 0.00000, 0.00000, 1.00000;
  // clang-format on
  extrinsics_t cam0_exts{T_BC0};
  extrinsics_t cam1_exts{T_BC1};

  for (size_t k = 0; k < max_nb_grids; k++) {
    // std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
    // view_t view;
    // view.cam_params = cam_params;
    // view.cam_exts = cam_exts;

    // for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
    //   const auto grid = cam_grids[cam_idx];
    //   const vec4_t proj_params = cam0_params.proj_params();
    //   const vec4_t dist_params = cam0_params.dist_params();
    //   const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
    //
    //   // Estimate relative pose (T_C0F)
    //   mat4_t T_CiF;
    //   if (grid->estimate(cam, T_CiF) != 0) {
    //     FATAL("Failed to estimate relative pose!");
    //   }
    //   mat4_t T_BCi = cam_exts[cam_idx]->tf();
    //   mat4_t T_BF = T_BCi * T_CiF;
    //
    //   // Form relative pose
    //   const auto ts = grid->timestamp;
    //   pose_t *rel_pose;
    //   if (rel_poses.count(ts) == 0) {
    //     rel_pose = new pose_t{grid->timestamp, T_BF};
    //     rel_poses[ts] = rel_pose;
    //   } else {
    //     rel_pose = rel_poses[ts];
    //   }
    //   view.pose = rel_pose;
    //
    //   // Form reprojection residual block
    //   std::vector<int> tag_ids;
    //   std::vector<int> corner_indicies;
    //   vec2s_t keypoints;
    //   vec3s_t object_points;
    //   grid->get_measurements(tag_ids,
    //                          corner_indicies,
    //                          keypoints,
    //                          object_points);
    //
    //   for (size_t i = 0; i < tag_ids.size(); i++) {
    //     auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
    //                                                      cam_res,
    //                                                      tag_ids[i],
    //                                                      corner_indicies[i],
    //                                                      object_points[i],
    //                                                      keypoints[i],
    //                                                      I(2));
    //
    //     view.reproj_errors[cam_idx].push_back(err);
    //     problem.AddResidualBlock(err,
    //                              nullptr,
    //                              rel_pose->param.data(),
    //                              cam_exts[cam_idx]->param.data(),
    //                              cam_params[cam_idx]->param.data());
    //   }
    // }

    // views.push_back(view);
  }

  // clang-format off
  // printf("[before marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  // printf("[before marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());
  // clang-format on

  // Marginalize
  // const auto view = views[0];
  // for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
  //   for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
  //     view.pose->marginalize = true;
  //     std::vector<param_t *> param_blocks;
  //     param_blocks.push_back((param_t *)view.pose);
  //     param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
  //     param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
  //     marg_error->add(reproj_error, param_blocks);
  //   }
  // }
  // marg_error->marginalize(problem);
  // problem.AddResidualBlock(marg_error, nullptr,
  // marg_error->get_param_ptrs());

  // clang-format off
  // printf("[after marg] nb_parameter_blocks: %d\n", problem.NumParameterBlocks());
  // printf("[after marg] nb_residual_blocks: %d\n", problem.NumResidualBlocks());
  // print_vector("cam0_params", cam_params[0]->param);
  // print_vector("cam0_exts", cam_exts[0]->param);
  // print_vector("cam1_params", cam_params[1]->param);
  // print_vector("cam1_exts", cam_exts[1]->param);
  // clang-format on

  // Solve
  // problem.SetParameterBlockConstant(cam_exts[0]->param.data());
  // ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  // std::cout << std::endl;

  // print_vector("cam0_params", cam_params[0]->param);
  // print_vector("cam0_exts", cam_exts[0]->param);
  // print_vector("cam1_params", cam_params[1]->param);
  // print_vector("cam1_exts", cam_exts[1]->param);

  return 0;
}

// int test_marg_error_check_gradients() {
//   // Test data
//   test_data_t test_data = setup_test_data();
//
//   // Filter out grids not detected
//   const size_t max_grids = 10;
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//
//   const size_t grids0_end = test_data.grids0.size();
//   const size_t grids1_end = test_data.grids1.size();
//   size_t grid0_idx = 0;
//   size_t grid1_idx = 0;
//   while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
//     auto grid0 = test_data.grids0[grid0_idx];
//     auto grid1 = test_data.grids1[grid1_idx];
//
//     if (grid0.timestamp > grid1.timestamp) {
//       grid1_idx++;
//     } else if (grid0.timestamp < grid1.timestamp) {
//       grid0_idx++;
//     } else if (grid0.timestamp == grid1.timestamp) {
//       if (grid0.detected && grid1.detected) {
//         cam0_grids.push_back(grid0);
//         cam1_grids.push_back(grid1);
//       }
//       if (cam0_grids.size() == max_grids) {
//         break;
//       }
//       grid0_idx++;
//       grid1_idx++;
//     }
//   }
//
//   // Setup camera
//   const int cam_res[2] = {752, 480};
//   id_t param_id = 0;
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam0_params{
//                               0,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//   camera_params_t cam1_params{
//                               1,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//
//   // Setup problem
//   ceres::Problem problem;
//   marg_error_t *marg_error = new marg_error_t();
//
//   mat4_t T_BC0 = I(4);
//   mat4_t T_BC1;
//   T_BC1 << 0.99998, -0.00254, -0.00498, 0.10995, 0.00247, 0.99989, -0.01440,
//       -0.00025, 0.00502, 0.01439, 0.99988, 0.00062, 0.00000, 0.00000,
//       0.00000, 1.00000;
//   extrinsics_t cam0_exts{param_id++, T_BC0};
//   extrinsics_t cam1_exts{param_id++, T_BC1};
//
//   std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
//   std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
//   std::map<timestamp_t, pose_t *> rel_poses;
//   std::vector<view_t> views;
//
//   for (size_t k = 0; k < max_grids; k++) {
//     std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
//     view_t view;
//     view.cam_params = cam_params;
//     view.cam_exts = cam_exts;
//
//     for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
//       const auto grid = cam_grids[cam_idx];
//       const vec4_t proj_params = cam0_params.proj_params();
//       const vec4_t dist_params = cam0_params.dist_params();
//       const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
//
//       // Estimate relative pose (T_C0F)
//       mat4_t T_CiF;
//       if (grid->estimate(cam, T_CiF) != 0) {
//         FATAL("Failed to estimate relative pose!");
//       }
//       mat4_t T_BCi = cam_exts[cam_idx]->tf();
//       mat4_t T_BF = T_BCi * T_CiF;
//
//       // Form relative pose
//       const auto ts = grid->timestamp;
//       pose_t *rel_pose;
//       if (rel_poses.count(ts) == 0) {
//         rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
//         rel_poses[ts] = rel_pose;
//       } else {
//         rel_pose = rel_poses[ts];
//       }
//       view.pose = rel_pose;
//
//       // Form reprojection residual block
//       std::vector<int> tag_ids;
//       std::vector<int> corner_indicies;
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       grid->get_measurements(tag_ids,
//                              corner_indicies,
//                              keypoints,
//                              object_points);
//
//       for (size_t i = 0; i < tag_ids.size(); i++) {
//         auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
//                                                          cam_res,
//                                                          tag_ids[i],
//                                                          corner_indicies[i],
//                                                          object_points[i],
//                                                          keypoints[i],
//                                                          I(2));
//
//         view.reproj_errors[cam_idx].push_back(err);
//         problem.AddResidualBlock(err,
//                                  nullptr,
//                                  rel_pose->param.data(),
//                                  cam_exts[cam_idx]->param.data(),
//                                  cam_params[cam_idx]->param.data());
//       }
//     }
//
//     views.push_back(view);
//   }
//
//   printf("[before marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[before marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   // Marginalize
//   const auto view = views[0];
//   for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
//     for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
//       view.pose->marginalize = true;
//       std::vector<param_t *> param_blocks;
//       param_blocks.push_back((param_t *)view.pose);
//       param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
//       param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
//       marg_error->add(reproj_error, param_blocks);
//     }
//   }
//   marg_error->marginalize(problem, true);
//   problem.AddResidualBlock(marg_error, nullptr,
//   marg_error->get_param_ptrs());
//
//   printf("[after marg] nb_parameter_blocks: %d\n",
//          problem.NumParameterBlocks());
//   printf("[after marg] nb_residual_blocks: %d\n",
//   problem.NumResidualBlocks());
//
//   std::vector<double *> param_ptrs = marg_error->get_param_ptrs();
//
//   const auto r_size = marg_error->e0_.size();
//   vecx_t r = zeros(r_size, 1);
//
//   std::vector<param_t *> params = marg_error->get_params();
//   double **jacobians = new double *[params.size()];
//   for (size_t i = 0; i < params.size(); i++) {
//     auto param = params[i];
//     jacobians[i] =
//         (double *)malloc(sizeof(double) * r.size() * param->global_size);
//   }
//   marg_error->Evaluate(param_ptrs.data(), r.data(), jacobians);
//   for (const auto param : params) {
//     printf("param[%s]\n", param->type.c_str());
//   }
//
//   // Check jacobians
//   double step = 1e-8;
//   double threshold = 1e-4;
//
//   printf("marg_error->x0.size(): %ld\n", marg_error->x0_.size());
//   mat2csv("/tmp/e0.csv", marg_error->e0_);
//   mat2csv("/tmp/J0.csv", marg_error->J0_);
//
//   for (size_t param_idx = 0; param_idx < params.size(); param_idx++) {
//     auto &param = params[param_idx];
//
//     matx_t fdiff = zeros(r_size, param->local_size);
//     for (int i = 0; i < param->local_size; i++) {
//       vecx_t r_fd = zeros(r_size, 1);
//       param->perturb(i, step);
//       marg_error->Evaluate(param_ptrs.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       param->perturb(i, -step);
//     }
//
//     auto J_name = "J" + std::to_string(param_idx);
//     auto J_rows = r_size;
//     auto J_cols = param->global_size;
//     auto J_min_cols = param->local_size;
//
//     Eigen::Map<matx_row_major_t> J(jacobians[param_idx], J_rows, J_cols);
//     const matx_t J_min = J.block(0, 0, J_rows, J_min_cols);
//
//     MU_CHECK(check_jacobian(J_name, fdiff, J_min, threshold, true) == 0);
//   }
//
//   return 0;
// }
//
// int test_marg_error_swe() {
//   // Test data
//   test_data_t test_data = setup_test_data();
//
//   // Filter out grids not detected
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//
//   const size_t grids0_end = test_data.grids0.size();
//   const size_t grids1_end = test_data.grids1.size();
//   size_t grid0_idx = 0;
//   size_t grid1_idx = 0;
//   while (grid0_idx < grids0_end && grid1_idx < grids1_end) {
//     auto grid0 = test_data.grids0[grid0_idx];
//     auto grid1 = test_data.grids1[grid1_idx];
//
//     if (grid0.timestamp > grid1.timestamp) {
//       grid1_idx++;
//     } else if (grid0.timestamp < grid1.timestamp) {
//       grid0_idx++;
//     } else if (grid0.timestamp == grid1.timestamp) {
//       if (grid0.detected && grid1.detected) {
//         cam0_grids.push_back(grid0);
//         cam1_grids.push_back(grid1);
//       }
//       grid0_idx++;
//       grid1_idx++;
//     }
//   }
//
//   // Setup camera
//   const int cam_res[2] = {752, 480};
//   id_t param_id = 0;
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam0_params{param_id++,
//                               0,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//   camera_params_t cam1_params{param_id++,
//                               1,
//                               cam_res,
//                               proj_model,
//                               dist_model,
//                               proj_params,
//                               dist_params};
//
//   // Setup camera extrinsics
//   mat4_t T_BC0 = I(4);
//   mat4_t T_BC1 = I(4);
//   // T_BC1 << 0.99998, -0.00254, -0.00498,  0.10995,
//   //          0.00247,  0.99989, -0.01440, -0.00025,
//   //          0.00502,  0.01439,  0.99988,  0.00062,
//   //          0.00000,  0.00000,  0.00000,  1.00000;
//   extrinsics_t cam0_exts{param_id++, T_BC0};
//   extrinsics_t cam1_exts{param_id++, T_BC1};
//
//   // Sliding window estimation
//   std::vector<camera_params_t *> cam_params = {&cam0_params, &cam1_params};
//   std::vector<extrinsics_t *> cam_exts = {&cam0_exts, &cam1_exts};
//   std::map<timestamp_t, pose_t *> rel_poses;
//   std::deque<view_t> sliding_window;
//
//   ceres::Problem problem;
//   PoseLocalParameterization pose_plus;
//
//   problem.AddParameterBlock(cam_exts[0]->param.data(), 7);
//   problem.AddParameterBlock(cam_exts[1]->param.data(), 7);
//   problem.SetParameterBlockConstant(cam_exts[0]->param.data());
//   problem.SetParameterization(cam_exts[0]->param.data(), &pose_plus);
//   problem.SetParameterization(cam_exts[1]->param.data(), &pose_plus);
//
//   marg_error_t *marg_error = new marg_error_t();
//   ceres::ResidualBlockId marg_error_id = nullptr;
//
//   size_t max_window_size = 10;
//   auto nb_cams = 2;
//   auto nb_grids = cam0_grids.size();
//
//   for (size_t k = 0; k < nb_grids; k++) {
//     std::vector<aprilgrid_t *> cam_grids = {&cam0_grids[k], &cam1_grids[k]};
//     view_t view;
//     view.cam_params = cam_params;
//     view.cam_exts = cam_exts;
//
//     // Keep sliding window bounded
//     if (sliding_window.size() >= max_window_size) {
//       // Remove previous residual block if it exists
//       if (marg_error_id != nullptr) {
//         problem.RemoveResidualBlock(marg_error_id);
//
//         // Replace old marginalization error, and keep track of previous info
//         auto marg_error_new = new marg_error_t();
//         marg_error_new->add(marg_error, marg_error->get_params());
//         delete marg_error;
//         marg_error = marg_error_new;
//       }
//
//       // Pop the oldest view from the sliding window
//       const auto view = sliding_window.front();
//       sliding_window.pop_front();
//       // problem.RemoveParameterBlock(view.pose->param.data());
//
//       // Marginalize the oldest pose and add marginalization prior to problem
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         for (auto &reproj_error : view.reproj_errors.at(cam_idx)) {
//           view.pose->marginalize = true;
//           std::vector<param_t *> param_blocks;
//           param_blocks.push_back((param_t *)view.pose);
//           param_blocks.push_back((param_t *)view.cam_exts[cam_idx]);
//           param_blocks.push_back((param_t *)view.cam_params[cam_idx]);
//           marg_error->add(reproj_error, param_blocks);
//         }
//       }
//       marg_error->marginalize(problem);
//       marg_error_id = problem.AddResidualBlock(marg_error,
//                                                nullptr,
//                                                marg_error->get_param_ptrs());
//     }
//
//     // Add view to sliding window
//     for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
//       // Estimate relative pose (T_C0F)
//       const auto grid = cam_grids[cam_idx];
//       const vec4_t proj_params = cam0_params.proj_params();
//       const vec4_t dist_params = cam0_params.dist_params();
//       const pinhole_radtan4_t cam{cam_res, proj_params, dist_params};
//       mat4_t T_CiF;
//       if (grid->estimate(cam, T_CiF) != 0) {
//         FATAL("Failed to estimate relative pose!");
//       }
//       mat4_t T_BCi = cam_exts[cam_idx]->tf();
//       mat4_t T_BF = T_BCi * T_CiF;
//
//       // Form relative pose
//       const auto ts = grid->timestamp;
//       pose_t *rel_pose;
//       if (rel_poses.count(ts) == 0) {
//         rel_pose = new pose_t{param_id++, grid->timestamp, T_BF};
//         rel_poses[ts] = rel_pose;
//       } else {
//         rel_pose = rel_poses[ts];
//       }
//       view.pose = rel_pose;
//
//       // Form reprojection residual block for each observation from each
//       camera std::vector<int> tag_ids; std::vector<int> corner_indicies;
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       grid->get_measurements(tag_ids,
//                              corner_indicies,
//                              keypoints,
//                              object_points);
//
//       for (size_t i = 0; i < tag_ids.size(); i++) {
//         auto err = new reproj_error_t<pinhole_radtan4_t>(cam_idx,
//                                                          cam_res,
//                                                          tag_ids[i],
//                                                          corner_indicies[i],
//                                                          object_points[i],
//                                                          keypoints[i],
//                                                          I(2));
//
//         view.reproj_errors[cam_idx].push_back(err);
//         problem.AddResidualBlock(err,
//                                  nullptr,
//                                  rel_pose->param.data(),
//                                  cam_exts[cam_idx]->param.data(),
//                                  cam_params[cam_idx]->param.data());
//       }
//       problem.SetParameterization(rel_pose->param.data(), &pose_plus);
//     }
//     sliding_window.push_back(view);
//
//     // Solve window
//     if (sliding_window.size() >= max_window_size) {
//       ceres::Solver::Options options;
//       options.max_num_iterations = 10;
//       ceres::Solver::Summary summary;
//       ceres::Solve(options, &problem, &summary);
//
//       // Obtain the calibration covariance matrix and calculate entropy. This
//       // is one way to make sure the marginalization error is working. If the
//       // marg_error_t() is working the entropy should be increasing over
//       time. matx_t covar; if (calib_covar(problem, cam0_params, cam1_params,
//       cam1_exts, covar) ==
//           0) {
//         printf("entropy: %f\t", entropy(covar));
//       }
//       std::cout << summary.BriefReport() << std::endl;
//     }
//   }
//
//   print_vector("cam0_params", cam_params[0]->param);
//   print_vector("cam0_exts", cam_exts[0]->param);
//   print_vector("cam1_params", cam_params[1]->param);
//   print_vector("cam1_exts", cam_exts[1]->param);
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_pose_error);
  MU_ADD_TEST(test_reproj_error);
  MU_ADD_TEST(test_fiducial_error);
  MU_ADD_TEST(test_marg_error);
  // MU_ADD_TEST(test_marg_error_check_gradients);
  // MU_ADD_TEST(test_marg_error_swe);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
