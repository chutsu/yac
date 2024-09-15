#include <gtest/gtest.h>

#include "CalibCamera.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(CalibCamera, testCalibMono) {
  CalibCamera calib{TEST_CONFIG};
  calib.initializeCamera(0);
}

} // namespace yac
