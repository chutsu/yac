#include "munit.hpp"
#include "calib_camera.hpp"

namespace yac {

int test_calib_view() {

  return 0;
}


void test_suite() {
  MU_ADD_TEST(test_calib_view);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
