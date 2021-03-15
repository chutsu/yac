#include <gtest/gtest.h>
#include <glog/logging.h>

#define EST_TIMEDELAY 0
#include "autocal/common/Calibrator.hpp"
#include "autocal/Estimator.hpp"

#include "EuRoCTestSetup.hpp"
#define TEST_DATA "/data/euroc/calib/imu_april"

namespace autocal {
namespace ceres {


TEST(Estimator, optimize) {
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  // Setup Estimator
  Estimator est;
  // -- Add calib target
  est.mode_ = "batch";
  est.addCalibTarget(test_data.target_params);
  // -- Add IMU
  est.addImu(test_data.imu0_params);
  est.addImuTimeDelay(0.0);
  // -- Add camera
  est.addCamera(0, test_data.cam0_geometry, true);
  est.addCamera(1, test_data.cam1_geometry, true);
  // -- Add extrinsics
  mat4_t T_SC0;
  mat4_t T_SC1;
  T_SC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
           0.0, 0.0, 0.0, 1.0;
  T_SC1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
           0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
           -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
           0.0, 0.0, 0.0, 1.0;
  mat4_t T_BC0 = I(4);
  mat4_t T_BC1 = T_SC0.inverse() * T_SC1;
  mat4_t T_BS = T_BC0 * T_SC0.inverse();
  est.addImuExtrinsics(T_BS);
  est.addCameraExtrinsics(0, T_BC0, true);
  est.addCameraExtrinsics(1, T_BC1, true);

  std::map<int, std::string> cam_image_paths;
  for (const auto &ts : test_data.timeline.timestamps) {
    printf(".");
    const auto result = test_data.timeline.data.equal_range(ts);
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;

      // AprilGrid event
      if (event.type == APRILGRID_EVENT) {
        const int cam_idx = event.camera_index;
        est.addMeasurement(cam_idx, event.grid);
      }

      // // Camera event
      // if (event.type == CAMERA_EVENT) {
      //   const auto ts = event.ts;
      //   const int cam_idx = event.camera_index;
      //   cam_image_paths[cam_idx] = event.image_path;
      //
      //   if (cam_image_paths.size() == est.nb_cams()) {
      //     std::vector<cv::Mat> cam_images;
      //     for (size_t i = 0; i < est.nb_cams(); i++) {
      //       cam_images.push_back(cv::imread(cam_image_paths[i]));
      //     }
      //     est.addMeasurement(ts, cam_images);
      //     cam_image_paths.clear();
      //   }
      // }

      // Imu event
      if (event.type == IMU_EVENT) {
        auto ts = Time(ns2sec(event.ts));
        const vec3_t w_m = event.w_m;
        const vec3_t a_m = event.a_m;
        est.addMeasurement(ts, w_m, a_m);
      }
    }
  }

  printf("\n");

  est.optimizeBatch(30, 4, true);
  // est.saveResults("/tmp/calib-stereo_imu.yaml");

	// Compare estimation to ground truth
  // -- cam0
  {
    vec4_t gnd_proj_params{458.654, 457.296, 367.215, 248.375};
    vec4_t gnd_dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    vec4_t est_proj_params = est.getCameraParameterEstimate(0).head(4);
    vec4_t est_dist_params = est.getCameraParameterEstimate(0).tail(4);

    print_vector("cam0 proj params [gnd]", gnd_proj_params);
    print_vector("cam0 proj params [est]", est_proj_params);
    print_vector("cam0 dist params [gnd]", gnd_dist_params);
    print_vector("cam0 dist params [est]", est_dist_params);
    printf("\n");

    EXPECT_TRUE(fabs(gnd_proj_params[0] - est_proj_params[0]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[1] - est_proj_params[1]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[2] - est_proj_params[2]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[3] - est_proj_params[3]) < 10.0);

    EXPECT_TRUE(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);
  }
  // -- cam1
  {
    vec4_t gnd_proj_params{457.587, 456.134, 379.999, 255.238};
    vec4_t gnd_dist_params{-0.28368365,  0.07451284, -0.00010473, -3.55590700e-05};
    vec4_t est_proj_params = est.getCameraParameterEstimate(1).head(4);
    vec4_t est_dist_params = est.getCameraParameterEstimate(1).tail(4);

    print_vector("cam1 proj params [gnd]", gnd_proj_params);
    print_vector("cam1 proj params [est]", est_proj_params);
    print_vector("cam1 dist params [gnd]", gnd_dist_params);
    print_vector("cam1 dist params [est]", est_dist_params);
    printf("\n");

    EXPECT_TRUE(fabs(gnd_proj_params[0] - est_proj_params[0]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[1] - est_proj_params[1]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[2] - est_proj_params[2]) < 10.0);
    EXPECT_TRUE(fabs(gnd_proj_params[3] - est_proj_params[3]) < 10.0);

    EXPECT_TRUE(fabs(gnd_dist_params[0] - est_dist_params[0]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[1] - est_dist_params[1]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[2] - est_dist_params[2]) < 0.1);
    EXPECT_TRUE(fabs(gnd_dist_params[3] - est_dist_params[3]) < 0.1);
  }
  // -- Extrinsics
  {
    // clang-format off
    mat4_t T_SC0_gnd;
    T_SC0_gnd << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            	 	 0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            	 	 -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            	 	 0.0, 0.0, 0.0, 1.0;
    mat4_t T_SC1_gnd;
    T_SC1_gnd << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            	 	 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            	 	 -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            	 	 0.0, 0.0, 0.0, 1.0;

		const mat4_t T_BS_est = est.getImuExtrinsicsEstimate();
		const mat4_t T_BC0_est = est.getCameraExtrinsicsEstimate(0);
		const mat4_t T_BC1_est = est.getCameraExtrinsicsEstimate(1);
		const mat4_t T_SC0_est = T_BS_est.inverse() * T_BC0_est;
		const mat4_t T_SC1_est = T_BS_est.inverse() * T_BC1_est;

		const vec3_t r_SC0_gnd = tf_trans(T_SC0_gnd);
		const vec3_t r_SC0_est = tf_trans(T_SC0_est);
		const vec3_t rpy_SC0_gnd = rad2deg(quat2euler(tf_quat(T_SC0_gnd)));
		const vec3_t rpy_SC0_est = rad2deg(quat2euler(tf_quat(T_SC0_est)));

		const vec3_t r_SC1_gnd = tf_trans(T_SC1_gnd);
		const vec3_t r_SC1_est = tf_trans(T_SC1_est);
		const vec3_t rpy_SC1_gnd = rad2deg(quat2euler(tf_quat(T_SC1_gnd)));
		const vec3_t rpy_SC1_est = rad2deg(quat2euler(tf_quat(T_SC1_est)));

		print_matrix("T_SC0 [gnd]", T_SC0_gnd);
    print_matrix("T_SC0 [est]", T_SC0_est);
    print_vector("trans (imu0-cam0) [gnd] [m]", r_SC0_gnd);
    print_vector("trans (imu0-cam0) [est] [m]", r_SC0_est);
    print_vector("rot (imu0-cam0) [gnd] [deg]", rpy_SC0_gnd);
    print_vector("rot (imu0-cam0) [est] [deg]", rpy_SC0_est);
		printf("\n");

		print_matrix("T_SC1 [gnd]", T_SC1_gnd);
    print_matrix("T_SC1 [est]", T_SC1_est);
    print_vector("trans (imu0-cam1) [gnd] [m]", r_SC1_gnd);
    print_vector("trans (imu0-cam1) [est] [m]", r_SC1_est);
    print_vector("rot (imu0-cam1) [gnd] [deg]", rpy_SC1_gnd);
    print_vector("rot (imu0-cam1) [est] [deg]", rpy_SC1_est);

    printf("time_delay: %f\n", est.getTimeDelayEstimate());

    // clang-format on
	}
}

}  // namespace ceres
}  // namespace autocal
