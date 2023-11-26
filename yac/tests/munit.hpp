#ifndef MUNIT_HPP
#define MUNIT_HPP

#include <stdio.h>
#include <string.h>
#include <string>
#include <glog/logging.h>

/* GLOBAL VARIABLES */
static int tests = 0;
static int passed = 0;
static int failed = 0;
static std::string test_target = "";

/* MACROS */
#define KNRM "\x1B[1;0m"
#define KRED "\x1B[1;31m"
#define KGRN "\x1B[1;32m"
#define KYEL "\x1B[1;33m"
#define KBLU "\x1B[1;34m"
#define KMAG "\x1B[1;35m"
#define KCYN "\x1B[1;36m"
#define KWHT "\x1B[1;37m"

/* MUNIT */
#define MU_CHECK(TEST)                                                         \
  do {                                                                         \
    if ((TEST) == false) {                                                     \
      printf("%sERROR!%s [%s:%d] %s %sFAILED!%s\n",                            \
             KRED,                                                             \
             KNRM,                                                             \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST,                                                            \
             KRED,                                                             \
             KNRM);                                                            \
      return -1;                                                               \
    }                                                                          \
  } while (0)

#define MU_CHECK_NEAR(expected, actual, tolerance)                             \
  do {                                                                         \
    if (!(fabs(expected - actual) < tolerance)) {                              \
      printf("%sERROR!%s [%s:%d] %sFAILED!%s\n",                               \
             KRED,                                                             \
             KNRM,                                                             \
             __func__,                                                         \
             __LINE__,                                                         \
             KRED,                                                             \
             KNRM);                                                            \
      return -1;                                                               \
    }                                                                          \
  } while (0)

#define MU_CHECK_FLOAT(expected, actual)                                       \
  do {                                                                         \
    if (!(fabs(expected - actual) < 1e-6)) {                                   \
      printf("%sERROR!%s [%s:%d] %sFAILED!%s\n",                               \
             KRED,                                                             \
             KNRM,                                                             \
             __func__,                                                         \
             __LINE__,                                                         \
             KRED,                                                             \
             KNRM);                                                            \
      printf("%sReason: %f != %f%s\n",                                         \
             KRED,                                                             \
             (float)expected,                                                  \
             (float)actual,                                                    \
             KNRM);                                                            \
      return -1;                                                               \
    }                                                                          \
  } while (0)

#define MU_ADD_TEST(TEST)                                                      \
  do {                                                                         \
    if (test_target != "" && test_target != #TEST) {                           \
      continue;                                                                \
    }                                                                          \
                                                                               \
    printf("%s-> %s %s\n", KBLU, #TEST, KNRM);                                 \
    fflush(stdout);                                                            \
    if (TEST() == -1) {                                                        \
      printf("%sTEST FAILED!%s\n\n", KRED, KNRM);                              \
      failed++;                                                                \
    } else {                                                                   \
      printf("%sTEST PASSED!%s\n\n", KGRN, KNRM);                              \
      passed++;                                                                \
    }                                                                          \
    tests++;                                                                   \
  } while (0)

#define MU_REPORT()                                                            \
  do {                                                                         \
    printf("\n");                                                              \
    printf(KBLU);                                                              \
    printf("%d tests, ", tests);                                               \
    printf("%d passed, ", passed);                                             \
    printf("%d failed\n", tests - passed);                                     \
    printf("\n");                                                              \
    printf(KNRM);                                                              \
    if (failed != 0) {                                                         \
      return -1;                                                               \
    } else {                                                                   \
      return 0;                                                                \
    }                                                                          \
  } while (0)

#define MU_RUN_TESTS(TEST_SUITE)                                               \
  int main(int argc, char *argv[]) {                                           \
    google::InitGoogleLogging(argv[0]);                                        \
                                                                               \
    if (argc == 3 && strcmp(argv[1], "--target") == 0) {                       \
      test_target = std::string{argv[2]};                                      \
      printf("%sTEST TARGET [%s]%s\n", KYEL, test_target.c_str(), KNRM);       \
    }                                                                          \
                                                                               \
    TEST_SUITE();                                                              \
    MU_REPORT();                                                               \
    return 0;                                                                  \
  }

#define PYTHON_SCRIPT(A)                                                       \
  if (system("python3 " A) != 0) {                                             \
    printf("Python script [%s] failed !", A);                                  \
    return -1;                                                                 \
  }

#define OCTAVE_SCRIPT(A)                                                       \
  if (system("octave " A) != 0) {                                              \
    printf("Octave script [%s] failed !", A);                                  \
    exit(-1);                                                                  \
  }

#endif // MUNIT_HPP
