#include "munit.hpp"
#include "calib_data.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define APRILGRID_CONF TEST_PATH "/test_data/calib/aprilgrid/target.yaml"
#define CAM0_IMAGE_DIR "/data/euroc/calib/cam_april/mav0/cam0/data"
#define CAM1_IMAGE_DIR "/data/euroc/calib/cam_april/mav0/cam1/data"
#define MONO_OUTPUT_DIR "/tmp/aprilgrid_test/mono"

int test_calib_target() {
  calib_target_t calib_target;
  if (calib_target.load(APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  MU_CHECK(calib_target.tag_rows == 6);
  MU_CHECK(calib_target.tag_cols == 6);
  MU_CHECK(fltcmp(calib_target.tag_size, 0.088) == 0);
  MU_CHECK(fltcmp(calib_target.tag_spacing, 0.3) == 0);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_calib_target); }

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
