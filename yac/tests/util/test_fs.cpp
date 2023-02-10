#include "../munit.hpp"
#include "util/fs.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TEST_CONFIG TEST_PATH "/test_data/core/config/config.yaml"
#define TEST_BAD_CONFIG TEST_PATH "/test_data/core/config/bogus.yaml"

int test_file_exists() {
  bool retval;

  retval = file_exists(TEST_CONFIG);
  MU_CHECK(retval == true);

  retval = file_exists(TEST_BAD_CONFIG);
  MU_CHECK(retval == false);

  return 0;
}

int test_path_split() {
  std::vector<std::string> splits;

  splits = path_split("/a/b/c.yaml");
  MU_CHECK(3 == (int) splits.size());
  MU_CHECK("a" == splits[0]);
  MU_CHECK("b" == splits[1]);
  MU_CHECK("c.yaml" == splits[2]);

  return 0;
}

int test_paths_join() {
  std::string out;

  out = paths_join("/a/b/c", "../");
  std::cout << out << std::endl;
  MU_CHECK("/a/b" == out);

  out = paths_join("/a/b/c", "../..");
  std::cout << out << std::endl;
  MU_CHECK("/a" == out);

  out = paths_join("/a/b/c", "d/e");
  std::cout << out << std::endl;
  MU_CHECK("/a/b/c/d/e" == out);

  out = paths_join("./a/b/c", "../d/e");
  std::cout << out << std::endl;
  MU_CHECK("./a/b/d/e" == out);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_file_exists);
  MU_ADD_TEST(test_path_split);
  MU_ADD_TEST(test_paths_join);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
