#include <gtest/gtest.h>

#include "CalibCamera.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(CalibCamera, initializeCamera) {
  CalibCamera calib{TEST_CONFIG};
  calib.initializeCamera(0);
}

TEST(CalibCamera, solve) {
  CalibCamera calib{TEST_CONFIG};
  calib.initializeCamera(0);
  calib.solve();
}

} // namespace yac
