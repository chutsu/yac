#include "munit.hpp"
#include "calib_mono.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define APRILGRID_DATA "/tmp/aprilgrid_test/mono/cam0"
#define APRILGRID_IMAGE TEST_PATH "/test_data/calib/aprilgrid/aprilgrid.png"
#define CAM0_APRILGRID_DATA "/tmp/aprilgrid_test/stereo/cam0"
#define CAM1_APRILGRID_DATA "/tmp/aprilgrid_test/stereo/cam1"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
  }

  // Test preprocess data
  const std::string image_dir = IMAGE_DIR;
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  int retval = preprocess_camera_data(target,
                                      image_dir,
                                      image_size,
                                      lens_hfov,
                                      lens_vfov,
                                      APRILGRID_DATA);
  if (retval == -1) {
    FATAL("Failed to preprocess camera data!");
  }
}

// int test_calib_mono_residual() {
//   // Test load
//   aprilgrids_t aprilgrids;
//   timestamps_t timestamps;
//   int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
//   MU_CHECK(retval == 0);
//   MU_CHECK(aprilgrids.size() > 0);
//   MU_CHECK(aprilgrids[0].ids.size() > 0);
//
//   // Setup intrinsic and distortion initialization
//   calib_params_t calib_params("pinhole", "radtan4", 752, 480, 98.0, 73.0);
//
//
//   for (size_t i = 0; i < aprilgrids.size(); i++) {
//     const auto grid = aprilgrids[i];
//     const auto tag_id = grid.ids[0];
//     const int corner_id = 0;
//     const auto kp = grid.keypoints[corner_id];
//     const auto T_CF = grid.T_CF;
//     const auto T_FC = T_CF.inverse();
//
//     // Get object point
//     vec3_t object_point;
//     if (aprilgrid_object_point(grid, tag_id, corner_id, object_point) != 0) {
//       LOG_ERROR("Failed to calculate AprilGrid object point!");
//     }
//
//     // Form residual and call the functor
//     // const quat_t q_CF = tf_quat(T_CF);
//     // const vec3_t r_CF = tf_trans(T_CF);
//     const quat_t q_FC = tf_quat(T_FC);
//     const vec3_t r_FC = tf_trans(T_FC);
//     double residuals[2] = {0.0, 0.0};
// 		const double *parameters[4] = {q_FC.coeffs().data(),
// 																	 r_FC.data(),
// 																	 calib_params.proj_params.data(),
// 																	 calib_params.dist_params.data()};
//     calib_mono_residual_t residual{calib_params.resolution().data(),
// 																 	 calib_params.proj_model,
//                                    calib_params.dist_model,
//                                    kp,
//                                    object_point};
//     residual.Evaluate(parameters, residuals, nullptr);
//
//     // Just some arbitrary test to make sure reprojection error is not larger
//     // than 100pixels in x or y direction. But often this can be the case ...
// 		printf("residuals: (%.2f, %.2f)\n", residuals[0], residuals[1]);
//     MU_CHECK(residuals[0] < 200.0);
//     MU_CHECK(residuals[1] < 200.0);
//   }
//
//   return 0;
// }

int test_calib_mono_solve() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  std::vector<timestamp_t> timestamps;
  MU_CHECK(load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps) == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{id, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  // Test
  mat4s_t T_CF;
  MU_CHECK(calib_mono_solve(aprilgrids, cam_params, T_CF) == 0);
  MU_CHECK(aprilgrids.size() == T_CF.size());
  calib_mono_stats(aprilgrids, cam_params, T_CF);

  // // Show results
  // std::cout << "Optimized results:" << std::endl;
  // print_vector("cam0.proj_params", calib_params.proj_params);
  // print_vector("cam0.dist_params", calib_params.dist_params);

  return 0;
}

int test_calib_mono_estimate_covariance() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  std::vector<timestamp_t> timestamps;
  MU_CHECK(load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps) == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{0, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  // // Test
  // mat4s_t T_CF;
  // MU_CHECK(calib_mono_solve(aprilgrids, cam_params, T_CF) == 0);
  // MU_CHECK(aprilgrids.size() == T_CF.size());
  // calib_mono_stats(aprilgrids, cam_params, T_CF);

  // Estimate covariance
  calib_mono_covar_est_t covar_est(cam_params, aprilgrids);

  matx_t covar;
  MU_CHECK(covar_est.estimate(covar) == 0);
  mat2csv("/tmp/covar.csv", covar);

  return 0;
}

int test_calib_mono_find_nbv() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  std::vector<timestamp_t> timestamps;
  MU_CHECK(load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps) == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_params{0, cam_idx, cam_res,
                             proj_model, dist_model,
                             proj_params, dist_params};

  // // Test
  // mat4s_t T_CF;
  // MU_CHECK(calib_mono_solve(aprilgrids, cam_params, T_CF) == 0);
  // MU_CHECK(aprilgrids.size() == T_CF.size());
  // calib_mono_stats(aprilgrids, cam_params, T_CF);

  // Find next best view
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
  }
  nbv_t nbv(target, cam_params, aprilgrids);
  nbv.find_nbv(target);

  return 0;
}

void test_suite() {
  test_setup();

  // MU_ADD_TEST(test_calib_mono_residual);
  // MU_ADD_TEST(test_calib_mono_solve);
  // MU_ADD_TEST(test_calib_mono_estimate_covariance);
  MU_ADD_TEST(test_calib_mono_find_nbv);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
