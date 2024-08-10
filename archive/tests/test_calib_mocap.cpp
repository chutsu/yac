#include "munit.hpp"
#include "calib_mocap.hpp"

namespace yac {

#ifndef TEST_PATH
#define TEST_PATH "."
#endif

#define TEST_CONF TEST_PATH "/test_data/mocap_data/calib_mocap-intel_d435i.yaml"
#define TEST_DATA TEST_PATH "/test_data/mocap_data"

int test_calib_mocap() {
  calib_mocap_t calib(TEST_CONF, TEST_DATA);
  calib.solve();
  calib.show_results();
  return 0;
}

void test_suite() { MU_ADD_TEST(test_calib_mocap); }

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
