#include <gtest/gtest.h>
#include <glog/logging.h>

#include "autocal/common/Calibrator.hpp"
#include "autocal/Initializer.hpp"

#include "EuRoCTestSetup.hpp"

#define TEST_DATA "/data/euroc/calib/imu_april"
#define CALIB_FILE "./test_data/calib/calibrator/visensor.yaml"

namespace autocal {

TEST(VisualInertialInitializer, configure) {
  VisualInertialInitializer<PinholeRadtan> est;
  est.configure(CALIB_FILE);
}

TEST(VisualInertialInitializer, optimize) {
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  // Setup VisualInertialInitializer
  VisualInertialInitializer<PinholeRadtan> est;
  est.addCalibTarget(test_data.target_params);
  est.addImu(test_data.imu0_params);
  est.addImuTimeDelay(0.0);
  est.addCamera(0, test_data.cam0_geometry);
  est.addCamera(1, test_data.cam1_geometry);
  // est.addCameraExtrinsics(0, test_data.T_SC0);
  // est.addCameraExtrinsics(1, test_data.T_SC1);

  // Optimize
  printf("Optimizing ...\n");
  // est.setupProblem(test_data.timeline);
  est.optimize(30, 4, true);
  printf("nb_residuals: %d\n", est.problem_->problem_->NumResidualBlocks());
  printf("nb_params: %d\n", est.problem_->problem_->NumParameterBlocks());
}

}  // namespace autocal
