#pragma once

/*****************************************************************************
 * Contents:
 * - Filesystem
 * - Data Type
 * - Macros
 * - Algebra
 * - Linear Algebra
 * - Geometry
 * - Statistics
 * - Transform
 * - Time
 * - Interpolation
 * - Measurements
 * - Parameters
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <dirent.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <random>
#include <set>
#include <list>
#include <deque>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace yac {

/******************************************************************************
 *                                MACROS
 *****************************************************************************/

#define KNRM "\x1B[1;0m"
#define KRED "\x1B[1;31m"
#define KGRN "\x1B[1;32m"
#define KYEL "\x1B[1;33m"
#define KBLU "\x1B[1;34m"
#define KMAG "\x1B[1;35m"
#define KCYN "\x1B[1;36m"
#define KWHT "\x1B[1;37m"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          "\033[31m[ERROR] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)
#define LOG_WARN(M, ...)                                                       \
  fprintf(stdout, "\033[33m[WARN] " M "\033[0m\n", ##__VA_ARGS__)

#define FATAL(M, ...)                                                          \
  {                                                                            \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                          \
            __FILENAME__,                                                      \
            __LINE__,                                                          \
            ##__VA_ARGS__);                                                    \
    throw std::runtime_error(msg);                                             \
  }

#define FATAL_ASSERT(X, M, ...)                                                \
  if (!(X)) {                                                                  \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                          \
            __FILENAME__,                                                      \
            __LINE__,                                                          \
            ##__VA_ARGS__);                                                    \
    throw std::runtime_error(msg);                                             \
  }

// #ifdef NDEBUG
// #define DEBUG(M, ...)
// #else
// #define DEBUG(M, ...) fprintf(stdout, "[DEBUG] " M "\n", ##__VA_ARGS__)
// #endif

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void)(expr);                                                              \
  } while (0)

#ifndef CHECK
#define CHECK(A)                                                               \
  if (!(A)) {                                                                  \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[CHECK FAILED] [%s:%d] " TOSTRING(A) "\033[0m\n",         \
            __FILENAME__,                                                      \
            __LINE__);                                                         \
    throw std::runtime_error(msg);                                             \
  }
#endif

/******************************************************************************
 *                                PROGRESS
 *****************************************************************************/

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void print_progress(double percentage, const std::string &prefix = "");

/******************************************************************************
 *                                DATA TYPE
 *****************************************************************************/

// -- TIMESTAMP ----------------------------------------------------------------

typedef int64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

// -- VECTOR ------------------------------------------------------------------

#define dynamic_t Eigen::Dynamic
#define col_major_t Eigen::ColMajor
#define row_major_t Eigen::RowMajor

using vec2i_t = Eigen::Matrix<int, 2, 1>;
using vec3i_t = Eigen::Matrix<int, 3, 1>;
using vec4i_t = Eigen::Matrix<int, 4, 1>;
using vec5i_t = Eigen::Matrix<int, 5, 1>;
using vec6i_t = Eigen::Matrix<int, 6, 1>;
using vec7i_t = Eigen::Matrix<int, 7, 1>;
using vecxi_t = Eigen::Matrix<int, dynamic_t, 1>;

using vec2_t = Eigen::Matrix<double, 2, 1>;
using vec3_t = Eigen::Matrix<double, 3, 1>;
using vec4_t = Eigen::Matrix<double, 4, 1>;
using vec5_t = Eigen::Matrix<double, 5, 1>;
using vec6_t = Eigen::Matrix<double, 6, 1>;
using vec7_t = Eigen::Matrix<double, 7, 1>;
using vecx_t = Eigen::Matrix<double, dynamic_t, 1>;

using vec2s_t = std::vector<vec2_t, Eigen::aligned_allocator<vec2_t>>;
using vec3s_t = std::vector<vec3_t, Eigen::aligned_allocator<vec3_t>>;
using vec4s_t = std::vector<vec4_t, Eigen::aligned_allocator<vec4_t>>;
using vec5s_t = std::vector<vec5_t, Eigen::aligned_allocator<vec5_t>>;
using vec6s_t = std::vector<vec6_t, Eigen::aligned_allocator<vec6_t>>;
using vec7s_t = std::vector<vec7_t, Eigen::aligned_allocator<vec7_t>>;
using vecxs_t = std::vector<vecx_t, Eigen::aligned_allocator<vecx_t>>;

using row_vector_t = Eigen::Matrix<double, 1, dynamic_t>;
using col_vector_t = Eigen::Matrix<double, dynamic_t, 1>;

template <int LENGTH, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using vec_t = Eigen::Matrix<double, LENGTH, 1, STRIDE_TYPE>;

// -- MATRIX -------------------------------------------------------------------

// clang-format off
using mat2_t = Eigen::Matrix<double, 2, 2>;
using mat3_t = Eigen::Matrix<double, 3, 3>;
using mat4_t = Eigen::Matrix<double, 4, 4>;
using matx_t = Eigen::Matrix<double, dynamic_t, dynamic_t>;
using matx_row_major_t = Eigen::Matrix<double, dynamic_t, dynamic_t, row_major_t>;
using mat34_t = Eigen::Matrix<double, 3, 4>;

using mat2s_t = std::vector<mat2_t, Eigen::aligned_allocator<mat2_t>>;
using mat3s_t = std::vector<mat3_t, Eigen::aligned_allocator<mat3_t>>;
using mat4s_t = std::vector<mat4_t, Eigen::aligned_allocator<mat4_t>>;
using matxs_t = std::vector<matx_t, Eigen::aligned_allocator<matx_t>>;
using matxs_row_major_t = std::vector<matx_row_major_t, Eigen::aligned_allocator<matx_row_major_t>>;

using mat_hash_t = std::unordered_map<long, std::unordered_map<long, double>>;
using mat_indicies_t = std::vector<std::pair<long int, long int>>;

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using mat_t = Eigen::Matrix<double, ROWS, COLS, STRIDE_TYPE>;

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using map_mat_t = Eigen::Map<Eigen::Matrix<double, ROWS, COLS, STRIDE_TYPE>>;

template <int ROWS>
using map_vec_t = Eigen::Map<Eigen::Matrix<double, ROWS, 1>>;
// clang-format on

// -- GEOMETRY -----------------------------------------------------------------

using quat_t = Eigen::Quaternion<double>;
using quats_t = std::vector<quat_t, Eigen::aligned_allocator<quat_t>>;
using angle_axis_t = Eigen::AngleAxis<double>;
using arrayx_t = Eigen::Array<double, dynamic_t, 1>;

/******************************************************************************
 *                                FILESYSTEM
 *****************************************************************************/

/**
 * Open file in `path` with `mode` and set `nb_rows`.
 * @returns File pointer on success, nullptr on failure.
 */
FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows = nullptr);

/**
 * Skip file line in file `fp`.
 */
void skip_line(FILE *fp);

/**
 * Get number of rows in file.
 * @returns Number of rows in file else -1 for failure.
 */
int file_rows(const std::string &file_path);

/**
 * Copy file from path `src` to path `dest.
 *
 * @returns 0 for success else -1 if `src` file could not be opend, or -2 if
 * `dest` file could not be opened.
 */
int file_copy(const std::string &src, const std::string &dest);

/**
 * Return file extension in `path`.
 */
std::string parse_fext(const std::string &path);

/**
 * Return basename
 */
std::string parse_fname(const std::string &path);

/**
 * Check if file exists
 *
 * @param path Path to file
 * @returns true or false
 */
bool file_exists(const std::string &path);

/**
 * Check if path exists
 *
 * @param path Path
 * @returns true or false
 */
bool dir_exists(const std::string &path);

/**
 * Create directory
 *
 * @param path Path
 * @returns 0 for success, -1 for failure
 */
int dir_create(const std::string &path);

/**
 * Return directory name
 *
 * @param path Path
 * @returns directory name
 */
std::string dir_name(const std::string &path);

/**
 * Strips a target character from the start and end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip(const std::string &s, const std::string &target = " ");

/**
 * Strips a target character from the end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip_end(const std::string &s, const std::string &target = " ");

/**
 * Create directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int create_dir(const std::string &path);

/**
 * Remove directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int remove_dir(const std::string &path);

/**
 * Remove file extension
 *
 * @param path Path to directory
 * @returns File path without extension
 */
std::string remove_ext(const std::string &path);

/**
 * List directory
 *
 * @param path Path to directory
 * @param results List of files and directories
 * @returns 0 for success, -1 for failure
 */
int list_dir(const std::string &path, std::vector<std::string> &results);

/**
 * List files in directory
 *
 * @param path Path to directory
 * @param files List of files only
 * @returns 0 for success, -1 for failure
 */
int list_files(const std::string &path, std::vector<std::string> &files);

/**
 * Split path into a number of elements
 *
 * @param path Path
 * @returns List of path elements
 */
std::vector<std::string> path_split(const std::string path);

/**
 * Combine `path1` and `path2`
 *
 * @param path1 Path 1
 * @param path2 Path 22
 * @returns Combined path
 */
std::string paths_join(const std::string path1, const std::string path2);

/*****************************************************************************
 *                               CONFIG
 *****************************************************************************/

/** Configuration **/
struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t() = default;
  config_t(const std::string &file_path_);
  ~config_t() = default;
};

/**
 * Load YAML file.
 * @returns 0 for success or -1 for failure.
 */
int yaml_load_file(const std::string file_path, YAML::Node &root);

/**
 * Get YAML node containing the parameter value.
 * @returns 0 for success or -1 for failure.
 */
int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const config_t &config, const std::string &key);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const std::string &file_path, const std::string &key);

/**
 * Check size of vector in config file and returns the size.
 */
template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional);

/**
 * Check matrix fields.
 */
void yaml_check_matrix_fields(const YAML::Node &node,
                              const std::string &key,
                              size_t &rows,
                              size_t &cols);

/**
 * Check matrix to make sure that the parameter has the data field "rows",
 * "cols" and "data". It also checks to make sure the number of values is the
 * same size as the matrix.
 */
template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols);

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional = false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional);

int parse(const config_t &config,
          const std::string &key,
          vec2_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec3_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec4_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec5_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec6_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec7_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vecx_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec2i_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec3i_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec4i_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vecxi_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat2_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat3_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat4_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          matx_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          cv::Mat &mat,
          const bool optional = false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          const int rows,
          const int cols,
          T &mat,
          const bool optional = false);

////////// CONFIG IMPLEMENTATION

template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional) {
  UNUSED(optional);
  assert(node);

  // Get expected vector size
  size_t vector_size = 0;
  if (std::is_same<T, vec2_t>::value) {
    vector_size = 2;
  } else if (std::is_same<T, vec3_t>::value) {
    vector_size = 3;
  } else if (std::is_same<T, vec4_t>::value) {
    vector_size = 4;
  } else if (std::is_same<T, vec5_t>::value) {
    vector_size = 5;
  } else if (std::is_same<T, vecx_t>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
  } else if (std::is_same<T, vec2i_t>::value) {
    vector_size = 2;
  } else if (std::is_same<T, vec3i_t>::value) {
    vector_size = 3;
  } else if (std::is_same<T, vec4i_t>::value) {
    vector_size = 4;
  } else if (std::is_same<T, vec5i_t>::value) {
    vector_size = 5;
  } else if (std::is_same<T, vecxi_t>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
  } else {
    LOG_ERROR("Failed to check vector [%s]!", key.c_str());
    FATAL("Unsupported vector type!");
  }

  // Check number of values in the param
  if (node.size() == 0 && node.size() != vector_size) {
    FATAL("Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          static_cast<int>(vector_size),
          static_cast<int>(node.size()));
  }

  return vector_size;
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols) {
  UNUSED(optional);
  assert(node);
  yaml_check_matrix_fields(node, key, rows, cols);

  // Check number of elements
  size_t nb_elements = 0;
  if (std::is_same<T, mat2_t>::value) {
    nb_elements = 4;
  } else if (std::is_same<T, mat3_t>::value) {
    nb_elements = 9;
  } else if (std::is_same<T, mat4_t>::value) {
    nb_elements = 16;
  } else if (std::is_same<T, matx_t>::value) {
    nb_elements = node["data"].size();

  } else if (std::is_same<T, cv::Mat>::value) {
    nb_elements = node["data"].size();
  } else {
    FATAL("Unsportted matrix type!");
  }
  if (node["data"].size() != nb_elements) {
    FATAL("Matrix [%s] rows and cols do not match number of values!",
          key.c_str());
  }
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional) {
  size_t rows;
  size_t cols;
  yaml_check_matrix<T>(node, key, optional, rows, cols);
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional) {
  try {
    // Get node
    YAML::Node node;
    if (yaml_get_node(config, key, optional, node) != 0) {
      return -1;
    }

    // Parse
    out = node.as<T>();
  } catch (const std::exception &e) {
    FATAL("Failed to parse [%s]: %s", key.c_str(), e.what());
  }

  return 0;
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional) {
  // Get node
  try {
    YAML::Node node;
    if (yaml_get_node(config, key, optional, node) != 0) {
      return -1;
    }

    // Parse
    std::vector<T> array;
    for (auto n : node) {
      out.push_back(n.as<T>());
    }
  } catch (const std::exception &e) {
    FATAL("Failed to parse [%s]: %s", key.c_str(), e.what());
  }

  return 0;
}

/******************************************************************************
 *                                  ALGEBRA
 *****************************************************************************/

/**
 * Sign of number
 *
 * @param[in] x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const double x);

/**
 * Floating point comparator
 *
 * @param[in] f1 First value
 * @param[in] f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(const double f1, const double f2);

/**
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
double binomial(const double n, const double k);

/**
 * Return evenly spaced numbers over a specified interval.
 */
template <typename T>
std::vector<T> linspace(const T start, const T end, const int num) {
  std::vector<T> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  const double diff = static_cast<double>(end - start);
  const double delta = diff / static_cast<double>(num - 1);
  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 *****************************************************************************/

/**
 * Print shape of a matrix
 *
 * @param[in] name Name of matrix
 * @param[in] A Matrix
 */
void print_shape(const std::string &name, const matx_t &A);

/**
 * Print shape of a vector
 *
 * @param[in] name Name of vector
 * @param[in] v Vector
 */
void print_shape(const std::string &name, const vecx_t &v);

/**
 * Print array
 *
 * @param[in] name Name of array
 * @param[in] array Target array
 * @param[in] size Size of target array
 */
void print_array(const std::string &name,
                 const double *array,
                 const size_t size);

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const vecx_t &v);

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const double *v, const int N);

/**
 * Print matrix `m` with a `name`.
 */
void print_matrix(const std::string &name,
                  const matx_t &m,
                  const std::string &indent = "");

/**
 * Print quaternion `q` with a `name`.
 */
void print_quaternion(const std::string &name, const quat_t &q);

/**
 * Array to string
 *
 * @param[in] array Target array
 * @param[in] size Size of target array
 * @returns String of array
 */
std::string array2str(const double *array, const size_t size);

/**
 * Convert double array to Eigen::Vector
 *
 * @param[in] x Input array
 * @param[in] size Size of input array
 * @param[out] y Output vector
 */
void array2vec(const double *x, const size_t size, vecx_t &y);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @returns Array
 */
double *vec2array(const vecx_t &v);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @returns Array
 */
double *mat2array(const matx_t &m);

/**
 * Quaternion to array
 *
 * *VERY IMPORTANT*: The returned array is (x, y, z, w).
 *
 * @param[in] q Quaternion
 * @returns Array
 */
double *quat2array(const quat_t &q);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @param[out] out Output array
 */
void vec2array(const vecx_t &v, double *out);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @param[in] out Output array
 */
void mat2array(const matx_t &m, double *out);

/**
 * Matrix to list of vectors
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
std::vector<vecx_t> mat2vec(const matx_t &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
vec3s_t mat2vec3(const matx_t &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
vec2s_t mat2vec2(const matx_t &m, const bool row_wise = true);

/**
 * Vectors to matrix
 */
matx_t vecs2mat(const vec3s_t &vs);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @param[in] max_digits Max digits
 * @returns Vector as a string
 */
std::string vec2str(const vecx_t &v,
                    const bool brackets = true,
                    const bool max_digits = false);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @param[in] max_digits Max digits
 * @returns Vector as a string
 */
template <typename T>
std::string vec2str(const std::vector<T> &v,
                    const bool brackets = true,
                    const bool max_digits = false) {
  std::ostringstream ss;

  if (max_digits) {
    typedef std::numeric_limits<double> numeric_limits;
    ss << std::setprecision(numeric_limits::max_digits10);
  }

  if (brackets) {
    ss << "[";
  }

  for (int i = 0; i < v.size(); i++) {
    ss << v.at(i);
    if ((i + 1) != v.size()) {
      ss << ", ";
    }
  }

  if (brackets) {
    ss << "]";
  }

  return ss.str();
}

/**
 * Array to string
 *
 * @param[in] arr Array
 * @param[in] len Length of array
 * @param[in] brackets Brakcets around vector string
 * @returns Array as a string
 */
std::string arr2str(const double *arr, const size_t len, bool brackets = true);

/**
 * Matrix to string
 *
 * @param[in] m Matrix
 * @param[in] indent Indent string
 * @returns Array as a string
 */
std::string mat2str(const matx_t &m,
                    const std::string &indent = "  ",
                    const bool max_digits = false);

/**
 * Normalize vector x
 */
vec2_t normalize(const vec2_t &x);

/**
 * Normalize vector `v`.
 */
vec3_t normalize(const vec3_t &v);

/**
 * Condition number of `A`.
 */
double cond(const matx_t &A);

/**
 * Zeros-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Zeros matrix
 */
matx_t zeros(const int rows, const int cols);

/**
 * Zeros square matrix
 *
 * @param size Square size of matrix
 * @returns Zeros matrix
 */
matx_t zeros(const int size);

/**
 * Identity-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Identity matrix
 */
matx_t I(const int rows, const int cols);

/**
 * Identity square matrix
 *
 * @param size Square size of matrix
 * @returns Identity square matrix
 */
matx_t I(const int size);

/**
 * Ones-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Ones square matrix
 */
matx_t ones(const int rows, const int cols);

/**
 * Ones square matrix
 *
 * @param size Square size of matrix
 * @returns Ones square matrix
 */
matx_t ones(const int size);

/**
 * Horizontally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t hstack(const matx_t &A, const matx_t &B);

/**
 * Vertically stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t vstack(const matx_t &A, const matx_t &B);

/**
 * Diagonally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t dstack(const matx_t &A, const matx_t &B);

/**
 * Skew symmetric-matrix
 *
 * @param w Input vector
 * @returns Skew symmetric matrix
 */
mat3_t skew(const vec3_t &w);

/**
 * Skew symmetric-matrix squared
 *
 * @param w Input vector
 * @returns Skew symmetric matrix squared
 */
mat3_t skewsq(const vec3_t &w);

/**
 * Enforce Positive Semi-Definite
 *
 * @param A Input matrix
 * @returns Positive semi-definite matrix
 */
matx_t enforce_psd(const matx_t &A);

/**
 * Null-space of A
 *
 * @param A Input matrix
 * @returns Null space of A
 */
matx_t nullspace(const matx_t &A);

/**
 * Check if two matrices `A` and `B` are equal.
 */
bool equals(const matx_t &A, const matx_t &B);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 *
 * @param[in] x Matrix values
 * @param[in] rows Number of matrix rows
 * @param[in] cols Number of matrix colums
 * @param[out] y Output matrix
 */
void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 matx_t &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 *
 * @param[in] A Matrix
 * @param[out] x Output vector of matrix values
 */
void load_matrix(const matx_t A, std::vector<double> &x);

/** Pseudo Inverse via SVD **/
matx_t pinv(const matx_t &A, const double tol = 1e-4);

/** Rank of matrix A **/
long int rank(const matx_t &A);

/** Check if matrix A is full rank */
bool full_rank(const matx_t &A);

/**
 * Perform Schur's Complement
 */
int schurs_complement(matx_t &H,
                      vecx_t &b,
                      const size_t m,
                      const size_t r,
                      const bool precond = false);

/******************************************************************************
 *                                 Geometry
 *****************************************************************************/

/**
 * Sinc function.
 */
double sinc(const double x);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(const double d);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
vec3_t deg2rad(const vec3_t d);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
vec3_t rad2deg(const vec3_t &r);

/**
 * Wrap angle in degrees to 180
 *
 * @param[in] d Degrees
 * @return Angle wraped to 180
 */
double wrap180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
double wrap360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
double wrapPi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
double wrap2Pi(const double r);

/**
 * Create a circle point of radius `r` at angle `theta` radians.
 */
vec2_t circle(const double r, const double theta);

/**
 * Create the sphere point with sphere radius `rho` at longitude `theta`
 * [radians] and latitude `phi` [radians].
 */
vec3_t sphere(const double rho, const double theta, const double phi);

/**
 * Create look at matrix.
 */
mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis = vec3_t{0.0, -1.0, 0.0});

/**
 * Cross-Track error based on waypoint line between p1, p2, and robot position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @return Cross track error
 */
double cross_track_error(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

/**
 * Check if point `pos` is left or right of line formed by `p1` and `p2`
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @returns
 *    - 1: Point is left of waypoint line formed by `p1` and `p2`
 *    - 2: Point is right of waypoint line formed by `p1` and `p2`
 *    - 0: Point is colinear with waypoint line formed by `p1` and `p2`
 */
int point_left_right(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

/**
 * Calculate closest point given waypoint line between `p1`, `p2` and robot
 * position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] p3 Robot position
 * @param[out] closest Closest point
 * @returns
 *    Unit number denoting where the closest point is on waypoint line. For
 *    example, a return value of 0.5 denotes the closest point is half-way
 *    (50%) of the waypoint line, alternatively a negative number denotes the
 *    closest point is behind the first waypoint.
 */
double closest_point(const vec2_t &p1,
                     const vec2_t &p2,
                     const vec2_t &p3,
                     vec2_t &closest);

/**
 * Fit a circle from a list of points.
 */
void fit_circle(const vec2s_t &points, double &cx, double &cy, double &radius);

/**
 * Find the intersect of two circles.
 */
vec2s_t intersect_circles(const double cx0,
                          const double cy0,
                          const double r0,
                          const double cx1,
                          const double cy1,
                          const double r1);

#define EARTH_RADIUS_M 6378137.0

/**
 * Calculate new latitude and logitude coordinates with an offset in North and
 * East direction.
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude offsets.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param offset_N Offset in North direction (meters)
 * @param offset_E Offset in East direction (meters)
 * @param lat_new New latitude (decimal format)
 * @param lon_new New longitude (decimal format)
 */
void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new);

/**
 * Calculate difference in distance in North and East from two GPS coordinates
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude diffs.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 * @param dist_N Distance of point of interest in North axis [m]
 * @param dist_E Distance of point of interest in East axis [m]
 */
void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E);

/**
 * Calculate Euclidean distance between two GPS coordintes
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude distance.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 *
 * @returns Euclidean distance between two GPS coordinates [m]
 */
double latlon_dist(double lat_ref, double lon_ref, double lat, double lon);

/******************************************************************************
 *                                STATISTICS
 *****************************************************************************/

/**
 * Create random integer
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random integer
 */
int randi(const int ub, const int lb);

/**
 * Create random double
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random floating point
 */
double randf(const double ub, const double lb);

/**
 * Sum values in vector.
 *
 * @param[in] x Array of numbers
 * @return Sum of vector
 */
double sum(const std::vector<double> &x);

/** Calculate median given an array of numbers */
double median(const std::vector<double> &v);

/** Max */
double max(const std::vector<double> &x);

/** Mean */
double mean(const std::vector<double> &x);

/** Mean */
vec2_t mean(const vec2s_t &x);

/** Mean */
vec3_t mean(const vec3s_t &x);

/** Variance */
double var(const std::vector<double> &x);

/** Variance */
vec2_t var(const vec2s_t &vecs);

/** Variance */
vec3_t var(const vec3s_t &vecs);

/** Standard Deviation */
double stddev(const std::vector<double> &x);

/** Standard Deviation */
vec2_t stddev(const vec2s_t &x);

/** Standard Deviation */
vec3_t stddev(const vec3s_t &x);

/** Root Mean Squared Error.  */
double rmse(const std::vector<double> &x);

/** Root Mean Squared Error.  */
vec2_t rmse(const vec2s_t &vecs);

/** Root Mean Squared Error.  */
vec3_t rmse(const vec3s_t &vecs);

/**
 * Multivariate normal.
 */
vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu = vec3_t{0.0, 0.0, 0.0},
           const vec3_t &stdev = vec3_t{1.0, 1.0, 1.0});

/**
 * Gassian normal.
 * http://c-faq.com/lib/gaussian.html
 */
double gauss_normal();

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> tf(const Eigen::Matrix<T, 3, 3> &C,
                          const Eigen::Matrix<T, 3, 1> &r) {
  Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
  transform.block(0, 0, 3, 3) = C;
  transform.block(0, 3, 3, 1) = r;
  return transform;
}

/**
 * Form a 4x4 homogeneous transformation matrix from a pointer to double array
 * containing (quaternion + translation) 7 elements: (qw, qx, qy, qz, x, y, z)
 */
mat4_t tf(const double *params);

/**
 * Form a 4x4 homogeneous transformation matrix from a pointer to double array
 * containing (quaternion + translation) 7 elements: (qw, qx, qy, qz, x, y, z)
 */
mat4_t tf(const vecx_t &params);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
mat4_t tf(const mat3_t &C, const vec3_t &r);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * Hamiltonian quaternion `q` and translation vector `r`.
 */
mat4_t tf(const quat_t &q, const vec3_t &r);

/**
 * Invert a 4x4 homogeneous transformation matrix
 */
mat4_t tf_inv(const mat4_t &T);

/**
 * Form a 7x1 pose vector
 */
vecx_t tf_vec(const mat4_t &T);

/**
 * Extract rotation from transform
 */
inline mat3_t tf_rot(const mat4_t &tf) { return tf.block<3, 3>(0, 0); }

/**
 * Extract rotation and convert to quaternion from transform
 */
inline quat_t tf_quat(const mat4_t &tf) { return quat_t{tf.block<3, 3>(0, 0)}; }

/**
 * Extract translation from transform
 */
inline vec3_t tf_trans(const mat4_t &tf) { return tf.block<3, 1>(0, 3); }

/**
 * Perturb the rotation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_rot(const mat4_t &T, double step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_trans(const mat4_t &T, double step_size, const int i);

/**
 * Perturb the translation and rotation component.
 */
mat4_t tf_perturb(const mat4_t &T, const double dr, const double drot);

/**
 * Transform point `p` with transform `T`.
 */
vec3_t tf_point(const mat4_t &T, const vec3_t &p);

/**
 * Pose difference between pose0 and pose1.
 */
void pose_diff(const double pose0[7],
               const double pose1[7],
               double *dr,
               double *drot);

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotx(const double theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t roty(const double theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotz(const double theta);

/**
 * Convert euler sequence 123 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
mat3_t euler123(const vec3_t &euler);

/**
 * Convert euler sequence 321 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
mat3_t euler321(const vec3_t &euler);

/**
 * Convert roll, pitch and yaw to quaternion.
 */
quat_t euler2quat(const vec3_t &euler);

/**
 * Convert rotation vectors to rotation matrix using measured acceleration
 * `a_m` from an IMU and gravity vector `g`.
 */
mat3_t vecs2rot(const vec3_t &a_m, const vec3_t &g);

/**
 * Convert rotation vector `rvec` to rotation matrix.
 */
mat3_t rvec2rot(const vec3_t &rvec, const double eps = 1e-5);

/**
 * Convert quaternion to euler angles.
 */
vec3_t quat2euler(const quat_t &q);

/**
 * Convert quaternion to rotation matrix.
 */
mat3_t quat2rot(const quat_t &q);

/**
 * Convert small angle euler angle to quaternion.
 */
quat_t quat_delta(const vec3_t &dalpha);

/**
 * Return left quaternion product matrix.
 */
mat4_t quat_lmul(const quat_t &q);

/**
 * Return left quaternion product matrix (but only for x, y, z components).
 */
mat3_t quat_lmul_xyz(const quat_t &q);

/**
 * Return right quaternion product matrix.
 */
mat4_t quat_rmul(const quat_t &q);

/**
 * Return right quaternion product matrix (but only for x, y, z components).
 */
mat3_t quat_rmul_xyz(const quat_t &q);

/**
 * Return only the x, y, z, components of a quaternion matrix.
 */
mat3_t quat_mat_xyz(const mat4_t &Q);

/**
 * Add noise to rotation matrix `rot`, where noise `n` is in degrees.
 */
mat3_t add_noise(const mat3_t &rot, const double n);

/**
 * Add noise to position vector `pos`, where noise `n` is in meters.
 */
vec3_t add_noise(const vec3_t &pos, const double n);

/**
 * Add noise to transform `pose`, where `pos_n` is in meters and `rot_n` is in
 * degrees.
 */
mat4_t add_noise(const mat4_t &pose, const double pos_n, const double rot_n);

/**
 * Initialize attitude using IMU gyroscope `w_m` and accelerometer `a_m`
 * measurements. The calculated attitude outputted into to `C_WS`. Note: this
 * function does not calculate initial yaw angle in the world frame. Only the
 * roll, and pitch are inferred from IMU measurements.
 */
void imu_init_attitude(const vec3s_t w_m,
                       const vec3s_t a_m,
                       mat3_t &C_WS,
                       const size_t buffer_size = 50);

/*****************************************************************************
 *                                    TIME
 *****************************************************************************/

/**
 * Print timestamp.
 */
void timestamp_print(const timestamp_t &ts, const std::string &prefix = "");

/**
 * Convert seconds to timestamp.
 */
timestamp_t sec2ts(const double sec);

/**
 * Convert ts to second.
 */
double ts2sec(const timestamp_t &ts);

/**
 * Decompose timestamp to seconds and nano-seconds.
 */
void tsdecomp(const timestamp_t &ts, long int &sec, long int &nsec);

/**
 * Form timestamp from seconds and nano-seconds.
 */
timestamp_t tsform(const long int sec, const long int &nsec);

/**
 * Convert nano-second to second.
 */
double ns2sec(const int64_t ns);

/**
 * Start timer.
 */
struct timespec tic();

/**
 * Stop timer and return number of seconds.
 */
float toc(struct timespec *tic);

/**
 * Stop timer and return miliseconds elasped.
 */
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

/**
 * Profiler
 */
struct profiler_t {
  std::map<std::string, timespec> timers;
  std::map<std::string, std::vector<double>> record;

  profiler_t() {}

  void start(const std::string &key) { timers[key] = tic(); }

  float stop(const std::string &key) {
    if (timers.count(key) == 0) {
      FATAL("Key [%s] does not exist in profiler!", key.c_str());
    }

    record[key].push_back(toc(&timers[key]));
    return record[key].back();
  }

  void print(const std::string &key, const bool show_last = true) {
    if (record.count(key) == 0) {
      FATAL("Key [%s] does not exist in profiler!", key.c_str());
    }

    if (show_last) {
      printf("[%s]: %.4fs\n", key.c_str(), record[key].back());
    } else {
      printf("Total [%s]: %.4fs\n", key.c_str(), sum(record[key]));
    }
  }
};

/******************************************************************************
 *                              INTERPOLATION
 *****************************************************************************/

/**
 * Linear interpolation between two points.
 *
 * @param[in] a First point
 * @param[in] b Second point
 * @param[in] t Unit number
 * @returns Linear interpolation
 */
template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Slerp
 */
quat_t slerp(const quat_t &q_start, const quat_t &q_end, const double alpha);

/**
 * Interpolate between two poses `p0` and `p1` with parameter `alpha`.
 */
mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const double alpha);

/**
 * Interpolate `poses` where each pose has a timestamp in `timestamps` and the
 * interpolation points in time are in `interp_ts`. The results are recorded
 * in `interp_poses`.
 * @returns 0 for success, -1 for failure
 */
void interp_poses(const timestamps_t &timestamps,
                  const mat4s_t &poses,
                  const timestamps_t &interp_ts,
                  mat4s_t &interped_poses,
                  const double threshold = 0.001);

/**
 * Get the closest pose in `poses` where each pose has a timestamp in
 * `timestamps` and the target points in time are in `target_ts`. The results
 * are recorded in `result`.
 * @returns 0 for success, -1 for failure
 */
void closest_poses(const timestamps_t &timestamps,
                   const mat4s_t &poses,
                   const timestamps_t &interp_ts,
                   mat4s_t &result);

/**
 * Let `t0` and `t1` be timestamps from two signals. If one of them is measured
 * at a higher rate, the goal is to interpolate the lower rate signal so that
 * it aligns with the higher rate one.
 *
 * This function will determine which timestamp deque will become the reference
 * signal and the other will become the target signal. Based on this the
 * interpolation points will be based on the reference signal.
 *
 * Additionally, this function ensures the interpolation timestamps are
 * achievable by:
 *
 * - interp start > target start
 * - interp end < target end
 *
 * **Note**: This function will not include timestamps from the target
 * (lower-rate) signal. The returned interpolation timestamps only returns
 * **interpolation points** to match the reference signal (higher-rate).
 *
 * @returns Interpolation timestamps from two timestamp deques `t0` and `t1`.
 */
std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1);

/**
 * Given the interpolation timestamps `lerp_ts`, target timestamps
 * `target_ts` and target data `target_data`. This function will:
 *
 * 1: Interpolate the `target_data` at the interpolation points defined by
 *    `target_ts`.
 * 2: Disgard data that are not in the target timestamp
 */
void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old = false);

/** Lerp pose */
mat4_t lerp_pose(const timestamp_t &t0,
                 const mat4_t &pose0,
                 const timestamp_t &t1,
                 const mat4_t &pose1,
                 const timestamp_t &t_lerp);

/**
 * Given two data signals with timestamps `ts0`, `vs0`, `ts1`, and `vs1`, this
 * function determines which data signal is at a lower rate and performs linear
 * interpolation inorder to synchronize against the higher rate data signal.
 *
 * The outcome of this function is that both data signals will have:
 *
 * - Same number of timestamps.
 * - Lower-rate data will be interpolated against the higher rate data.
 *
 * **Note**: This function will drop values from the start and end of both
 * signals inorder to synchronize them.
 */
void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1);

/**
 * SIM IMU
 */
struct sim_imu_t {
  // IMU parameters
  double rate = 0.0;       // IMU rate [Hz]
  double tau_a = 3600.0;   // Reversion time constant for accel [s]
  double tau_g = 3600.0;   // Reversion time constant for gyro [s]
  double sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
  double sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
  double sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  double sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
  double g = 9.81;         // Gravity vector [ms-2]

  // IMU flags and biases
  bool started = false;
  vec3_t b_g = zeros(3, 1);
  vec3_t b_a = zeros(3, 1);
  timestamp_t ts_prev = 0;
};

/**
 * Reset IMU
 */
void sim_imu_reset(sim_imu_t &imu);

/**
 * Simulate IMU measurement
 */
void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const mat4_t &T_WS_W,
                         const vec3_t &w_WS_W,
                         const vec3_t &a_WS_W,
                         vec3_t &a_WS_S,
                         vec3_t &w_WS_S);

/******************************************************************************
 *                               MEASUREMENTS
 *****************************************************************************/

struct imu_meas_t {
  timestamp_t ts = 0;
  vec3_t accel{0.0, 0.0, 0.0};
  vec3_t gyro{0.0, 0.0, 0.0};

  imu_meas_t() = default;
  ~imu_meas_t() = default;
  imu_meas_t(const timestamp_t &ts_, const vec3_t &accel_, const vec3_t &gyro_)
      : ts{ts_}, accel{accel_}, gyro{gyro_} {}
};

struct imu_data_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  std::deque<timestamp_t> timestamps;
  std::deque<vec3_t, Eigen::aligned_allocator<vec3_t>> accel;
  std::deque<vec3_t, Eigen::aligned_allocator<vec3_t>> gyro;

  imu_data_t() = default;
  ~imu_data_t() = default;

  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr) {
    timestamps.push_back(ts);
    accel.push_back(acc);
    gyro.push_back(gyr);
  }

  double time_span() const {
    const auto ts_first = timestamps.front();
    const auto ts_last = timestamps.back();
    return ts2sec(ts_last - ts_first);
  }

  size_t size() const { return timestamps.size(); }

  size_t size() { return static_cast<const imu_data_t &>(*this).size(); }

  timestamp_t last_ts() const { return timestamps.back(); }

  timestamp_t last_ts() {
    return static_cast<const imu_data_t &>(*this).last_ts();
  }

  void clear() {
    timestamps.clear();
    accel.clear();
    gyro.clear();
  }

  void trim(const timestamp_t ts_end) {
    // Pre-check
    if (timestamps.size() == 0) {
      return;
    }

    // Make sure the trim timestamp is after the first imu measurement
    if (ts_end < timestamps.front()) {
      return;
    }

    // Trim IMU measurements
    // while (ts_end > timestamps.front()) {
    //   timestamps.pop_front();
    //   accel.pop_front();
    //   gyro.pop_front();
    // }

    std::deque<timestamp_t> timestamps_new;
    std::deque<vec3_t, Eigen::aligned_allocator<vec3_t>> accel_new;
    std::deque<vec3_t, Eigen::aligned_allocator<vec3_t>> gyro_new;
    for (size_t k = 0; k < timestamps.size(); k++) {
      if (timestamps[k] >= ts_end) {
        if (k > 0 && timestamps_new.size() == 0) {
          timestamps_new.push_back(timestamps[k - 1]);
          accel_new.push_back(accel[k - 1]);
          gyro_new.push_back(gyro[k - 1]);
        }

        timestamps_new.push_back(timestamps[k]);
        accel_new.push_back(accel[k]);
        gyro_new.push_back(gyro[k]);
      }
    }
    timestamps = timestamps_new;
    accel = accel_new;
    gyro = gyro_new;
  }

  imu_data_t extract(const timestamp_t ts_start, const timestamp_t ts_end) {
    assert(ts_start >= timestamps.front());
    assert(ts_end <= timestamps.back());

    // Extract data between ts_start and ts_end
    imu_data_t buf;
    size_t remove_idx = 0;
    timestamp_t ts_km1;
    vec3_t acc_km1;
    vec3_t gyr_km1;

    for (size_t k = 0; k < timestamps.size(); k++) {
      // Check if within the extraction zone
      auto ts_k = timestamps[k];
      if (ts_k < ts_start) {
        continue;
      }

      // Setup
      const vec3_t acc_k = accel[k];
      const vec3_t gyr_k = gyro[k];

      // Interpolate start or end?
      if (buf.timestamps.size() == 0 and ts_k > ts_start) {
        // Interpolate start
        ts_km1 = timestamps[k - 1];
        acc_km1 = accel[k - 1];
        gyr_km1 = gyro[k - 1];

        const double alpha = (ts_start - ts_km1) / (ts_k - ts_km1);
        const vec3_t acc_km1 = (1.0 - alpha) * acc_km1 + alpha * acc_k;
        const vec3_t gyr_km1 = (1.0 - alpha) * gyr_km1 + alpha * gyr_k;
        ts_km1 = ts_start;

        buf.add(ts_km1, acc_km1, gyr_km1);

      } else if (ts_k > ts_end) {
        // Interpolate end
        ts_km1 = timestamps[k - 1];
        acc_km1 = accel[k - 1];
        gyr_km1 = gyro[k - 1];

        const double alpha = (ts_end - ts_km1) / (ts_k - ts_km1);
        const vec3_t acc_k = (1.0 - alpha) * acc_km1 + alpha * acc_k;
        const vec3_t gyr_k = (1.0 - alpha) * gyr_km1 + alpha * gyr_k;
        ts_k = ts_end;
      }

      // Add to subset
      buf.add(ts_k, acc_k, gyr_k);

      // End?
      if (ts_k == ts_end) {
        break;
      }

      // Update
      remove_idx = k;
    }

    // Remove data before ts_end
    for (size_t k = 0; k < remove_idx; k++) {
      timestamps.pop_front();
      accel.pop_front();
      gyro.pop_front();
    }

    return buf;
  }

  mat3_t initial_attitude(ssize_t limit = -1) {
    // Sample IMU measurements
    vec3_t sum_angular_vel = vec3_t::Zero();
    vec3_t sum_linear_acc = vec3_t::Zero();
    auto buf_size = (limit == -1) ? timestamps.size() : limit;
    for (size_t k = 0; k < buf_size; k++) {
      sum_angular_vel += gyro[k];
      sum_linear_acc += accel[k];
    }

    // Initialize the initial orientation, so that the estimation
    // is consistent with the inertial frame.
    const vec3_t mean_accel = sum_linear_acc / buf_size;
    const vec3_t gravity{0.0, 0.0, -9.81};
    mat3_t C_WS = vecs2rot(mean_accel, -gravity);

    // Extract roll, pitch and set yaw to 0
    // const quat_t q_WS = quat_t(C_WS);
    // const vec3_t rpy = quat2euler(q_WS);
    // const double roll = rpy(0);
    // const double pitch = rpy(1);
    // const double yaw = 0.0;
    // C_WS = euler321(vec3_t{roll, pitch, yaw});

    return C_WS;
  }

  friend std::ostream &operator<<(std::ostream &os, const imu_data_t &data) {
    os << "nb_measurements: " << data.size() << std::endl;
    for (size_t k = 0; k < data.timestamps.size(); k++) {
      os << "ts: " << std::to_string(data.timestamps[k]) << " ";
      os << "acc: " << vec2str(data.accel[k]) << " ";
      os << "gyr: " << vec2str(data.gyro[k]) << std::endl;
    }
    return os;
  }
};

/******************************************************************************
 *                               PARAMETERS
 *****************************************************************************/

struct imu_params_t {
  double rate = 0.0;       // IMU rate [Hz]
  double a_max = 160.0;    // Max accelerometer measurement [m/s^2]
  double g_max = 10.0;     // Max gyroscope measurement [rad/s]
  double sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
  double sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
  double sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  double sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
  double sigma_bg = 0.0;   // Gyro bias prior [rad/s]
  double sigma_ba = 0.0;   // Accel bias prior [m/s^2]
  double g = 9.81;         // Gravity vector [ms^-2]
};

/******************************************************************************
 *                                  OPENCV
 *****************************************************************************/

/**
 * Convert gray-scale image to rgb image
 *
 * @param image
 *
 * @returns RGB image
 */
cv::Mat gray2rgb(const cv::Mat &image);

/**
 * Convert rgb image to gray-scale image
 *
 * @param image
 *
 * @returns Gray-scale image
 */
cv::Mat rgb2gray(const cv::Mat &image);

} //  namespace yac
