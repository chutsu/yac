#include <memory>
#include <sys/time.h>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "autocal/core/core.hpp"
#include "autocal/calib/calib_data.hpp"
#include "autocal/calib/euroc.hpp"
#include "autocal/time/Time.hpp"
#include "autocal/util/assert_macros.hpp"
#include "autocal/kinematics/Transformation.hpp"
#include "autocal/common/FrameTypedefs.hpp"
#include "autocal/common/Measurements.hpp"
#include "autocal/common/timeline.hpp"

#include "autocal/cv/CameraGeometry.hpp"
#include "autocal/ceres/Calibrator.hpp"

#include "EuRoCTestSetup.hpp"

#define CALIB_FILE "./test_data/calib/calibrator/visensor.yaml"

#define OCTAVE_SCRIPT(A)                                                       \
  if (system("octave " A) != 0) {                                              \
    FATAL("Octave script [%s] failed !", A);                                   \
  }

TEST(Calibrator, config_nb_cameras) {
  EXPECT_EQ(autocal::ceres::config_nb_cameras(CALIB_FILE), 2);
}

TEST(Calibrator, config_nb_imus) {
  EXPECT_EQ(autocal::ceres::config_nb_imus(CALIB_FILE), 1);
}

TEST(Calibrator, load_calib_target_params) {
  autocal::config_t config{CALIB_FILE};
  autocal::calib_target_t target;

  autocal::ceres::load_calib_target_params(config, target);
  std::cout << "target_type: " << target.target_type << std::endl;
  std::cout << "tag_rows: " << target.tag_rows << std::endl;
  std::cout << "tag_cols: " << target.tag_cols << std::endl;
  std::cout << "tag_size: " << target.tag_size << std::endl;
  std::cout << "tag_spacing: " << target.tag_spacing << std::endl;

  EXPECT_EQ(target.target_type, "aprilgrid");
  EXPECT_EQ(target.tag_rows, 6);
  EXPECT_EQ(target.tag_cols, 6);
  EXPECT_EQ(target.tag_size, 0.088);
  EXPECT_EQ(target.tag_spacing, 0.3);
}

TEST(Calibrator, load_optimization_settings) {
  autocal::ceres::OptSettings opt;
  autocal::config_t config{CALIB_FILE};

  autocal::ceres::load_optimization_settings(config, opt);
}

TEST(Calibrator, load_camera_params) {
  autocal::config_t config{CALIB_FILE};

  autocal::ceres::CameraParameters params;
  autocal::ceres::load_camera_params(config, 0, params);

  EXPECT_EQ(params.camera_model, "pinhole");
  EXPECT_EQ(params.distortion_model, "radtan");
  EXPECT_EQ(params.resolution(0), 752);
  EXPECT_EQ(params.resolution(1), 480);
  EXPECT_FALSE((params.intrinsics - autocal::zeros(4, 1)).norm() < 1e-5);
  EXPECT_TRUE((params.distortion - autocal::zeros(4, 1)).norm() < 1e-5);
}

TEST(Calibrator, load_imu_params) {
  ImuParameters imu_params;
  autocal::config_t config{CALIB_FILE};

  autocal::ceres::load_imu_params(config, 0, imu_params);
  EXPECT_EQ(imu_params.a_max, 176.0);
  EXPECT_EQ(imu_params.g_max, 7.8);
  EXPECT_EQ(imu_params.sigma_g_c, 12.0e-4);
  EXPECT_EQ(imu_params.sigma_a_c, 8.0e-3);
  EXPECT_EQ(imu_params.sigma_bg, 0.03);
  EXPECT_EQ(imu_params.sigma_ba, 0.1);
  EXPECT_EQ(imu_params.sigma_gw_c, 4.0e-6);
  EXPECT_EQ(imu_params.sigma_aw_c, 4.0e-5);
  EXPECT_EQ(imu_params.tau, 3600.0);
  EXPECT_EQ(imu_params.g, 9.81007);
  EXPECT_TRUE((imu_params.a0 - autocal::zeros(4, 1)).norm() < 1e-5);
  EXPECT_EQ(imu_params.rate, 200);
}

TEST(Calibrator, load_extrinsic_params) {
  autocal::config_t config{CALIB_FILE};

  {
    mat4_t T_SC0;
    autocal::ceres::load_extrinsic_params(config, 0, 0, T_SC0);
    // std::cout << "T_SC0:\n" << T_SC0.T() << std::endl;

    // clang-format off
    autocal::mat4_t expected;
    expected << 0.0,-1.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE((T_SC0 - expected).norm() < 1e-5);
    // clang-format on
  }

  {
    mat4_t T_SC1;
    autocal::ceres::load_extrinsic_params(config, 0, 1, T_SC1);
    // std::cout << "T_SC1:\n" << T_SC1.T() << std::endl;

    // clang-format off
    autocal::mat4_t expected;
    expected << 0.0,-1.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE((T_SC1 - expected).norm() < 1e-5);
    // clang-format on
  }
}

TEST(Calibrator, trim_imu_data) {
  // Setup
  ImuMeasurementDeque imu_data;
  for (int i = 0; i < 10; i++) {
    vec3_t gyro{0.1 * i, 0.1 * i, 0.1 * i};
    vec3_t accel{0.1 * i, 0.1 * i, 0.1 * i};
    ImuSensorReadings meas{gyro, accel};
    Time ts{0.1 * i};
    imu_data.emplace_back(ts, meas);
  }

  // Before trim
  printf("imu_data [before trim]:\n");
  for (auto &meas : imu_data) {
    auto gyro = meas.measurement.gyroscopes;
    auto accel = meas.measurement.accelerometers;
    printf("gyro: %.4f %.4f %.4f", gyro(0), gyro(1), gyro(2));
    printf("\t");
    printf("accel: %.4f %.4f %.4f", accel(0), accel(1), accel(2));
    printf("\n");
  }

  // Trim data
  const timestamp_t t1 = 0.5 * 1e9;
  autocal::ceres::trim_imu_data(imu_data, t1);

  // After trim
  printf("imu_data [after trim]:\n");
  for (auto &meas : imu_data) {
    auto gyro = meas.measurement.gyroscopes;
    auto accel = meas.measurement.accelerometers;
    printf("gyro: %.4f %.4f %.4f", gyro(0), gyro(1), gyro(2));
    printf("\t");
    printf("accel: %.4f %.4f %.4f", accel(0), accel(1), accel(2));
    printf("\n");
  }
}
