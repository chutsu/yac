#include "munit.hpp"
#include "euroc_test_data.hpp"
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

int test_calib_data() {
  test_data_t test_data = setup_test_data();
  const auto image_dir = CAM0_IMAGE_DIR;
  const auto output_dir = MONO_OUTPUT_DIR "/cam0";

  calib_data_t calib_data;
  calib_data.add_calib_target(test_data.target);
  calib_data.add_camera(test_data.cam0);
  calib_data.add_camera(test_data.cam1);
  calib_data.add_camera_extrinsics(0);
  calib_data.add_camera_extrinsics(1);
  // calib_data.add_grids(0, test_data.grids0);
  // calib_data.add_grids(1, test_data.grids1);
  calib_data.preprocess_camera_data(image_dir, output_dir);
  // calib_data.check_data();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_target);
  MU_ADD_TEST(test_calib_data);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
