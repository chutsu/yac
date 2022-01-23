#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

// Global variables
std::map<int, aprilgrids_t> cam_grids;

void test_setup() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/cam_april";
  std::map<int, std::string> grids_path = {
      {0, data_path + "/grid0/cam0"},
      {1, data_path + "/grid0/cam1"},
  };

  for (int cam_idx = 0; cam_idx < 2; cam_idx++) {
    // Create grids path
    if (system(("mkdir -p " + grids_path[cam_idx]).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", grids_path[cam_idx].c_str());
    }

    // Get image files
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    const std::string img_dir = data_path + "/mav0/" + cam_str + "/data";
    std::vector<std::string> img_paths;
    list_files(img_dir, img_paths);

    // Loop through image files
    for (const auto img_fname : img_paths) {
      printf(".");
      fflush(stdout);

      const std::string img_path = img_dir + "/" + img_fname;
      const std::string ts_str = img_fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      const std::string grid_fname = ts_str + ".csv";
      const std::string grid_path = grids_path[cam_idx] + "/" + grid_fname;
      const auto image = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

      // Load or detect apriltag
      aprilgrid_t grid;
      if (file_exists(grid_path)) {
        grid.load(grid_path);
        if (grid.detected == false) {
          grid.timestamp = ts;
          grid.tag_rows = detector.tag_rows;
          grid.tag_cols = detector.tag_cols;
          grid.tag_size = detector.tag_size;
          grid.tag_spacing = detector.tag_spacing;
        }
      } else {
        grid = detector.detect(ts, image);
        grid.save(grid_path);
      }
      // grid.imshow("viz", image);
      // cv::waitKey(1);

      cam_grids[cam_idx].push_back(grid);
    }
  }
  printf("\n");
}

struct blake_zisserman_loss_t {
  const double epsilon = 0.0;

  blake_zisserman_loss_t(const size_t df,
                         const double p_cut = 0.999,
                         const double w_cut = 0.1) {
    // // Calculate epsilon - parameter for Blake-Zisserman Loss
    // const std::chi_squared_distribution<double> chi_dist(df);
    // const double cdf = boost::math::quantile(chi_dist, p_cut);
    // epsilon = (1 - w_cut) / w_cut * exp(-cdf);
  }
  ~blake_zisserman_loss_t() = default;

  virtual void Evaluate(double s, double out[3]) const {
    // out[0] = exp(-s) / (exp(-s) + epsilon);
    // out[1] = -(exp(s) * epsilon) / pow((exp(s) * epsilon + 1.0), 2);
    // out[2] = exp(-s) * (2.0 * exp(-2.0 * s) / pow(exp(-s) + epsilon, 3));
  }
};

int test_blake_zisserman_loss() {
  blake_zisserman_loss_t loss(2);
  UNUSED(loss);
  return 0;
}

// int test_reproj_error() {
//   aprilgrid_t grid;
//   for (size_t k = 0; k < cam_grids[0].size(); k++) {
//     if (cam_grids[0][k].detected) {
//       grid = cam_grids[0][k];
//       break;
//     }
//   }
//
//   std::vector<int> tag_ids;
//   std::vector<int> corner_indicies;
//   vec2s_t keypoints;
//   vec3s_t object_points;
//   grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);
//
//   const int cam_idx = 0;
//   const int cam_res[2] = {752, 480};
//   const std::string proj_model = "pinhole";
//   const std::string dist_model = "radtan4";
//   const vec4_t proj_params{458.654, 457.296, 367.215, 248.375};
//   const vec4_t dist_params{-0.28340811, 0.07395907,
//   0.00019359, 1.76187114e-05}; camera_params_t cam_params{cam_idx,
//                              cam_res,
//                              proj_model,
//                              dist_model,
//                              proj_params,
//                              dist_params};
//   const pinhole_radtan4_t cam_geom;
//
//   mat4_t T_CiF;
//   if (grid.estimate(&cam_geom, cam_res, cam_params.param, T_CiF) != 0) {
//     FATAL("Failed to estimate relative pose!");
//   }
//
//   mat4_t T_C0Ci = I(4);
//   mat4_t T_C0F = T_C0Ci * T_CiF;
//   pose_t cam_exts{grid.timestamp, T_C0Ci};
//   pose_t rel_pose{grid.timestamp, T_C0F};
//
//   reproj_error_t err(&cam_geom,
//                      &cam_params,
//                      &cam_exts,
//                      &rel_pose,
//                      tag_ids[0],
//                      corner_indicies[0],
//                      object_points[0],
//                      keypoints[0],
//                      I(2));
//
//   std::vector<double *> params = {
//       rel_pose.param.data(),
//       cam_exts.param.data(),
//       cam_params.param.data(),
//   };
//   vec2_t r;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J0;
//   Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J1;
//   Eigen::Matrix<double, 2, 8, Eigen::RowMajor> J2;
//   std::vector<double *> jacobians = {J0.data(), J1.data(), J2.data()};
//
//   // Baseline
//   err.Evaluate(params.data(), r.data(), jacobians.data());
//
//   // Check Jacobians
//   double step = 1e-8;
//   double threshold = 1e-4;
//
//   // -- Test relative pose T_C0F jacobian
//   {
//     mat_t<2, 6> fdiff;
//
//     for (int i = 0; i < 6; i++) {
//       vec2_t r_fd;
//       rel_pose.perturb(i, step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       rel_pose.perturb(i, -step);
//     }
//
//     const mat_t<2, 6> J0_min = J0.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J0", fdiff, J0_min, threshold, true) == 0);
//   }
//
//   // -- Test cam exts T_C0Ci jacobian
//   {
//     mat_t<2, 6> fdiff;
//
//     for (int i = 0; i < 6; i++) {
//       vec2_t r_fd;
//       cam_exts.perturb(i, step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam_exts.perturb(i, -step);
//     }
//
//     const mat_t<2, 6> J1_min = J1.block(0, 0, 2, 6);
//     MU_CHECK(check_jacobian("J1", fdiff, J1_min, threshold, true) == 0);
//   }
//
//   // -- Test camera-parameters jacobian
//   {
//     mat_t<2, 8> fdiff;
//
//     for (int i = 0; i < 8; i++) {
//       vec2_t r_fd;
//       cam_params.perturb(i, step);
//       err.Evaluate(params.data(), r_fd.data(), nullptr);
//       fdiff.col(i) = (r_fd - r) / step;
//       cam_params.perturb(i, -step);
//     }
//
//     MU_CHECK(check_jacobian("J2", fdiff, J2, threshold, true) == 0);
//   }
//
//   return 0;
// }

int test_calib_view() {
  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);

  ceres::Problem problem;
  ceres::CauchyLoss loss(1.0);
  auto grid = cam_grids[0][0];
  const mat4_t T_C0F = I(4);
  const mat4_t T_C0Ci = I(4);
  pose_t extrinsics{0, T_C0Ci};
  pose_t rel_pose{0, T_C0F};
  pinhole_radtan4_t cam_geom;

  calib_view_t(&problem, &loss, &cam_geom, &cam0, &extrinsics, &rel_pose, grid);

  return 0;
}

int test_initialize_camera() {
  const int img_w = 752;
  const int img_h = 480;
  const int res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  camera_params_t cam0 = camera_params_t::init(0, res, proj_model, dist_model);
  pinhole_radtan4_t cam_geom;

  initialize_camera(cam_grids[0], &cam_geom, &cam0, true);
  return 0;
}

int test_calib_camera() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  calib_camera_t calib;
  calib.add_calib_target(target);
  calib.add_camera_data(0, cam_grids[0]);
  calib.add_camera_data(1, cam_grids[1]);
  calib.add_camera(0, cam_res, proj_model, dist_model);
  calib.add_camera(1, cam_res, proj_model, dist_model);
  calib.solve();
  calib.save_results("/tmp/calib-cameras.yaml");

  return 0;
}

void test_suite() {
  test_setup();
  MU_ADD_TEST(test_blake_zisserman_loss);
  // MU_ADD_TEST(test_reproj_error);
  MU_ADD_TEST(test_calib_view);
  MU_ADD_TEST(test_initialize_camera);
  MU_ADD_TEST(test_calib_camera);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
