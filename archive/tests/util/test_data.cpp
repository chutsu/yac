#include <unistd.h>

#include "../munit.hpp"
#include "util/data.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TEST_DATA TEST_PATH "/test_data/core/data/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"

int test_csv_rows() {
  int rows;
  rows = csv_rows(TEST_DATA);
  MU_CHECK(rows == 281);
  return 0;
}

int test_csv_cols() {
  int cols;
  cols = csv_cols(TEST_DATA);
  MU_CHECK(cols == 2);
  return 0;
}

int test_csv2mat() {
  matx_t data;

  csv2mat(TEST_DATA, true, data);
  MU_CHECK(data.rows() == 280);
  MU_CHECK(data.cols() == 2);
  MU_CHECK_NEAR(-2.22482078596, data(0, 0), 1e-4);
  MU_CHECK_NEAR(9.9625789766, data(0, 1), 1e-4);
  MU_CHECK_NEAR(47.0485650525, data(279, 0), 1e-4);
  MU_CHECK_NEAR(613.503760567, data(279, 1), 1e-4);

  return 0;
}

int test_mat2csv() {
  matx_t x;
  matx_t y;

  csv2mat(TEST_DATA, true, x);
  mat2csv(TEST_OUTPUT, x);
  csv2mat(TEST_OUTPUT, false, y);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      MU_CHECK_NEAR(x(i, j), y(i, j), 0.1);
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_csv_rows);
  MU_ADD_TEST(test_csv_cols);
  MU_ADD_TEST(test_csv2mat);
  MU_ADD_TEST(test_mat2csv);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
