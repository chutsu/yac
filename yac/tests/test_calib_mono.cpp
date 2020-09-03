#include "munit.hpp"
#include "calib_mono.hpp"
// #include "factor.hpp"

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

int test_calib_mono_residual() {
  // Test load
  aprilgrids_t aprilgrids;
  timestamps_t timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup intrinsic and distortion initialization
  calib_params_t calib_params("pinhole", "radtan4", 752, 480, 98.0, 73.0);


  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto grid = aprilgrids[i];
    const auto tag_id = grid.ids[0];
    const int corner_id = 0;
    const auto kp = grid.keypoints[corner_id];
    const auto T_CF = grid.T_CF;
    const auto T_FC = T_CF.inverse();

    // Get object point
    vec3_t object_point;
    if (aprilgrid_object_point(grid, tag_id, corner_id, object_point) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object point!");
    }

    // Form residual and call the functor
    // const quat_t q_CF = tf_quat(T_CF);
    // const vec3_t r_CF = tf_trans(T_CF);
    const quat_t q_FC = tf_quat(T_FC);
    const vec3_t r_FC = tf_trans(T_FC);
    double residuals[2] = {0.0, 0.0};
		const double *parameters[4] = {q_FC.coeffs().data(),
																	 r_FC.data(),
																	 calib_params.proj_params.data(),
																	 calib_params.dist_params.data()};
    calib_mono_residual_t residual{calib_params.resolution().data(),
																 	 calib_params.proj_model,
                                   calib_params.dist_model,
                                   kp,
                                   object_point};
    residual.Evaluate(parameters, residuals, nullptr);

    // Just some arbitrary test to make sure reprojection error is not larger
    // than 100pixels in x or y direction. But often this can be the case ...
		printf("residuals: (%.2f, %.2f)\n", residuals[0], residuals[1]);
    MU_CHECK(residuals[0] < 200.0);
    MU_CHECK(residuals[1] < 200.0);
  }

  return 0;
}

int test_calib_mono_solve() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  std::vector<timestamp_t> timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  calib_params_t calib_params("pinhole", "radtan4",
                              752, 480, 98.0, 73.0);

  // Test
  mat4s_t T_CF;
  MU_CHECK(calib_mono_solve(aprilgrids, calib_params, T_CF) == 0);
  MU_CHECK(aprilgrids.size() == T_CF.size());
  calib_mono_stats(aprilgrids, calib_params, T_CF);

  // Show results
  // std::cout << "Optimized intrinsics and distortions:" << std::endl;
  // std::cout << cam << std::endl;
  // std::cout << radtan << std::endl;

  return 0;
}

// int test_calib_nbv() {
//   // Setup calibration target
//   calib_target_t target;
//   if (calib_target_load(target, APRILGRID_CONF) != 0) {
//     LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
//     return -1;
//   }
//
//   // // Setup camera
//   // cv::VideoCapture camera(0);
//   // if (camera.isOpened() == false) {
//   //   return -1;
//   // }
//   // sleep(2);
//
//   // // Guess the camera intrinsics and distortion
//   // cv::Mat frame;
//   // camera.read(frame);
//   // const auto detector = aprilgrid_detector_t();
//   // const double fx = pinhole_focal(frame.cols, 120.0);
//   // const double fy = pinhole_focal(frame.rows, 120.0);
//   // const double cx = frame.cols / 2.0;
//   // const double cy = frame.rows / 2.0;
//   // const mat3_t cam_K = pinhole_K(fx, fy, cx, cy);
//   // const vec4_t cam_D = zeros(4, 1);
//
//   graph_t graph;
//
//   graph_add_landmark(
//
//   // Generate nbv poses
//   // mat4s_t nbv_poses = calib_generate_poses(target);
//   // mat4s_t nbv_poses = generate_nbv_poses(target);
//   // mat4s_t nbv_poses = generate_initial_poses(target);
//
//   // FILE *poses_csv = fopen("/tmp/poses.csv", "w");
//   // for (const auto &pose : nbv_poses) {
//   //   const auto &q = tf_quat(pose);
//   //   const auto &r = tf_trans(pose);
//   //
//   //   fprintf(poses_csv, "%f,", q.w());
//   //   fprintf(poses_csv, "%f,", q.x());
//   //   fprintf(poses_csv, "%f,", q.y());
//   //   fprintf(poses_csv, "%f,", q.z());
//   //
//   //   fprintf(poses_csv, "%f,", r(0));
//   //   fprintf(poses_csv, "%f,", r(1));
//   //   fprintf(poses_csv, "%f\n", r(2));
//   // }
//   // fclose(poses_csv);
//
//
//   // // Loop camera feed
//   // // int pose_idx = randi(0, nbv_poses.size());
//   // while (true) {
//   //   // Get image
//   //   cv::Mat frame;
//   //   camera.read(frame);
//   //
//   //   // Detect AprilGrid
//   //   aprilgrid_t grid;
//   //   aprilgrid_set_properties(grid,
//   //                            target.tag_rows,
//   //                            target.tag_cols,
//   //                            target.tag_size,
//   //                            target.tag_spacing);
//   //   aprilgrid_detect(grid, detector, frame, cam_K, cam_D);
//   //
//   // //   // Calculate calibration target from camera view
//   // //   vec3s_t object_points;
//   // //   aprilgrid_object_points(grid, object_points);
//   // //   const size_t nb_pts = object_points.size();
//   // //   const matx_t hp_T = vecs2mat(object_points);
//   // //   const mat4_t T_TC = nbv_poses[pose_idx];
//   // //   const mat4_t T_CT = T_TC.inverse();
//   // //   const matx_t hp_C = T_CT * hp_T;
//   // //   const matx_t p_C = hp_C.block(0, 0, 3, nb_pts);
//   // //
//   // //   // Project target corners to camera frame
//   // //   for (size_t i = 0; i < nb_pts; i++) {
//   // //     const vec3_t p = p_C.block(0, i, 3, 1);
//   // //     const vec3_t pt{p(0) / p(2), p(1) / p(2), 1.0};
//   // //     const vec2_t pixel = (K * pt).head(2);
//   // //     cv::Point2f cv_pixel(pixel(0), pixel(1));
//   // //     if (i < 4) {
//   // //       cv::circle(frame, cv_pixel, 3, cv::Scalar(0, 255, 0), -1);
//   // //     } else {
//   // //       cv::circle(frame, cv_pixel, 3, cv::Scalar(0, 0, 255), -1);
//   // //     }
//   // //   }
//   // //
//   // //   const vec3_t pos_desired = tf_trans(T_CT);
//   // //   const vec3_t pos_actual = tf_trans(grid.T_CF);
//   // //   const double pos_diff = (pos_desired - pos_actual).norm();
//   // //   bool pos_ok = (pos_diff < 0.25) ? true : false;
//   // //
//   // //   const vec3_t rpy_desired = quat2euler(tf_quat(T_CT));
//   // //   const vec3_t rpy_actual = quat2euler(tf_quat(grid.T_CF));
//   // //   const double rpy_diff = (rpy_desired - rpy_actual).norm();
//   // //   bool rpy_ok = (rad2deg(rpy_diff) < 10.0) ? true : false;
//   // //
//   // //   printf("pos diff [%.2f]\t rpy_diff [%.2f]\n", pos_diff, rpy_diff);
//   // //   if (pos_ok && rpy_ok) {
//   // //     pose_idx = randf(0, nbv_poses.size());
//   // //   }
//   // //
//   // //   // const std::string title = "AprilGrid";
//   // //   // aprilgrid_imshow(grid, title, frame);
//   // //
//   //   // Show image and get user input
//   //   cv::Mat frame_flip;
//   //   cv::flip(frame, frame_flip, 1);
//   //   cv::imshow("Image", frame_flip);
//   //   char key = (char) cv::waitKey(1);
//   //   if (key == 'q') {
//   //     break;
//   //   }
//   // }
//
//   return 0;
// }

void test_suite() {
  test_setup();

  // MU_ADD_TEST(test_calib_mono_residual);
  MU_ADD_TEST(test_calib_mono_solve);
  // MU_ADD_TEST(test_calib_nbv);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
