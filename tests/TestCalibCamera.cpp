#include <gtest/gtest.h>

#include "CalibCamera.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(CalibCamera, initializeIntrinsics) {
  CalibCamera calib{TEST_CONFIG};
  calib.initializeIntrinsics();
}

TEST(CalibCamera, initializeExtrinsic) {
  CalibCamera calib{TEST_CONFIG};
  calib.initializeIntrinsics();
  calib.initializeExtrinsics();
}

TEST(CalibCamera, solve) {
  CalibCamera calib{TEST_CONFIG};
  calib.solve();
  calib.saveResults("/tmp/calib_camera-results.yaml");
}

} // namespace yac
