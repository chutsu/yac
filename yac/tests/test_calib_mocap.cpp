#include "munit.hpp"
#include "calib_mocap.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TEST_CALIB TEST_PATH "/test_data/calib_mocap-intel_d435i.yaml"

int test_calib_mocap_data() {
  calib_mocap_data_t data(TEST_CALIB);

  return 0;
}

int test_calib_mocap_solve() {
  calib_mocap_solve(TEST_CALIB);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_mocap_data);
  MU_ADD_TEST(test_calib_mocap_solve);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
