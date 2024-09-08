#include <gtest/gtest.h>

#include "CalibData.hpp"

#define TEST_CONFIG TEST_DATA "/calib_mono.yaml"

namespace yac {

TEST(CalibData, cameraData) {
  CalibData calib_data{TEST_CONFIG};
}

}
