#include <gtest/gtest.h>

#include "CameraChain.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(CalibData, load) {
  CalibData calib_data{TEST_CONFIG};
  ASSERT_EQ(calib_data.getNumCameras(), 2);
}

}
