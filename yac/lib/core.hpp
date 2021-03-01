#ifndef YAC_CORE_HPP
#define YAC_CORE_HPP

/*****************************************************************************
 * This file is huge, it contains everything from parsing yaml files, linear
 * algebra functions to networking code used for robotics.
 *
 * Contents:
 * - Data Type
 * - Macros
 * - Data
 * - Filesystem
 * - Configuration
 * - Algebra
 * - Linear Algebra
 * - Geometry
 * - Differential Geometry
 * - Statistics
 * - Transform
 * - Time
 * - Networking
 * - Interpolation
 * - Control
 * - Measurements
 * - Models
 * - Vision
 * - Parameters
 * - Simulation
 * - Factor Graph
 *
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
#include <errno.h>
#include <pthread.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

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

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#define ENABLE_MACROS 1

namespace yac {

/******************************************************************************
 *                                DATA TYPE
 *****************************************************************************/

/* PRECISION TYPE */
// #define PRECISION 1 // Single Precision
#define PRECISION 2 // Double Precision

#if PRECISION == 1
  #define real_t float
#elif PRECISION == 2
  #define real_t double
#else
  #define real_t double
#endif

#define dynamic_t Eigen::Dynamic
#define col_major_t Eigen::ColMajor
#define row_major_t Eigen::RowMajor

typedef Eigen::Matrix<real_t, 2, 1> vec2_t;
typedef Eigen::Matrix<real_t, 3, 1> vec3_t;
typedef Eigen::Matrix<real_t, 4, 1> vec4_t;
typedef Eigen::Matrix<real_t, 5, 1> vec5_t;
typedef Eigen::Matrix<real_t, 6, 1> vec6_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> vecx_t;
typedef Eigen::Matrix<real_t, 2, 2> mat2_t;
typedef Eigen::Matrix<real_t, 3, 3> mat3_t;
typedef Eigen::Matrix<real_t, 4, 4> mat4_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> matx_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matx_row_major_t;
typedef Eigen::Matrix<real_t, 3, 4> mat34_t;
typedef Eigen::Quaternion<real_t> quat_t;
typedef Eigen::AngleAxis<real_t> angle_axis_t;
typedef Eigen::Matrix<real_t, 1, Eigen::Dynamic> row_vector_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> col_vector_t;
typedef Eigen::Array<real_t, Eigen::Dynamic, 1> arrayx_t;

typedef Eigen::SparseMatrix<real_t> sp_mat_t;
typedef Eigen::SparseVector<real_t> sp_vec_t;

typedef std::vector<vec2_t, Eigen::aligned_allocator<vec2_t>> vec2s_t;
typedef std::vector<vec3_t, Eigen::aligned_allocator<vec3_t>> vec3s_t;
typedef std::vector<vec4_t, Eigen::aligned_allocator<vec4_t>> vec4s_t;
typedef std::vector<vec5_t, Eigen::aligned_allocator<vec5_t>> vec5s_t;
typedef std::vector<vec6_t, Eigen::aligned_allocator<vec6_t>> vec6s_t;
typedef std::vector<vecx_t> vecxs_t;
typedef std::vector<mat2_t, Eigen::aligned_allocator<mat2_t>> mat2s_t;
typedef std::vector<mat3_t, Eigen::aligned_allocator<mat3_t>> mat3s_t;
typedef std::vector<mat4_t, Eigen::aligned_allocator<mat4_t>> mat4s_t;
typedef std::vector<matx_t, Eigen::aligned_allocator<matx_t>> matxs_t;
typedef std::vector<quat_t, Eigen::aligned_allocator<quat_t>> quats_t;

template <int LENGTH, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using vec_t = Eigen::Matrix<real_t, LENGTH, 1, STRIDE_TYPE>;

template <int ROWS,
          int COLS,
          Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using mat_t = Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>;

template <int ROWS,
          int COLS,
          Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using map_mat_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>>;

template <int ROWS>
using map_vec_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, 1>>;

typedef std::unordered_map<long, std::unordered_map<long, real_t>> mat_hash_t;
typedef std::vector<std::pair<long int, long int>> mat_indicies_t;

typedef int64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#ifdef ENABLE_MACROS

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
  fprintf(stdout,                                                              \
          "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

#ifdef NDEBUG
#define DEBUG(M, ...)
#else
#define DEBUG(M, ...) fprintf(stdout, "[DEBUG] " M "\n", ##__VA_ARGS__)
#endif

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

#endif // ENABLE_MACROS ------------------------------------------------------

/******************************************************************************
 *                                  DATA
 *****************************************************************************/

/**
 * Convert bytes to signed 8bit number
 */
int8_t int8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 8bit number
 */
uint8_t uint8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 16bit number
 */
int16_t int16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 16bit number
 */
uint16_t uint16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 32bit number
 */
int32_t int32(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 32bit number
 */
uint32_t uint32(const uint8_t *data, const size_t offset);

/**
 * Allocate memory for a C-style string
 */
char *malloc_string(const char *s);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const char *fp);

/**
 * Get number of cols in CSV file.
 * @returns Number of cols in CSV file else -1 for failure.
 */
int csv_cols(const char *fp);

/**
 * Return csv fields as strings and number of fields in csv file.
 */
char **csv_fields(const char *fp, int *nb_fields);

/**
 * Load data in csv file `fp`. Assumming the data are real_ts. Also returns
 * number of rows and cols in `nb_rows` and `nb_cols` respectively.
 */
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);

/**
 * Load integer arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
int **load_iarrays(const char *csv_path, int *nb_arrays);

/**
 * Load real_t arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
real_t **load_darrays(const char *csv_path, int *nb_arrays);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const std::string &file_path);

/**
 * Get number of columns in CSV file.
 * @returns Number of columns in CSV file else -1 for failure.
 */
int csv_cols(const std::string &file_path);

/**
 * Convert CSV file to matrix.
 * @returns 0 for success, -1 for failure
 */
int csv2mat(const std::string &file_path, const bool header, matx_t &data);

/**
 * Convert matrix to csv file.
 * @returns 0 for success, -1 for failure
 */
int mat2csv(const std::string &file_path, const matx_t &data);

/**
 * Convert vector to csv file.
 * @returns 0 for success, -1 for failure
 */
int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data);

/**
 * Convert timestamps to csv file.
 * @returns 0 for success, -1 for failure
 */
int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data);

/**
 * Print progress to screen
 */
void print_progress(const real_t percentage);

/**
 * Check if vector `x` is all true.
 */
bool all_true(const std::vector<bool> x);

/**
 * Pop front of an `std::vector`.
 */
template <typename T>
void pop_front(std::vector<T> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Pop front of an `std::vector`.
 */
template <typename T1, typename T2>
void pop_front(std::vector<T1, T2> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Extend `std::vector`.
 */
template <typename T>
void extend(std::vector<T> &x, std::vector<T> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Extend `std::vector`.
 */
template <typename T1, typename T2>
void extend(std::vector<T1, T2> &x, std::vector<T1, T2> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Slice `std::vector`.
 */
template<typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T> vec(first, last);
  return vec;
}

/**
 * Slice `std::vector`.
 */
template<typename T1, typename T2>
std::vector<T1, T2> slice(std::vector<T1, T2> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T1, T2> vec(first, last);
  return vec;
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
const V *lookup(const std::map<K, V> &map, K key) {
  typename std::map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
V *lookup(std::map<K, V> &map, K key) {
  return const_cast<V *>(lookup(const_cast<const std::map<K, V> &>(map), key));
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
const V *lookup(const std::unordered_map<K, V> &map, K key) {
  typename std::unordered_map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
V *lookup(std::unordered_map<K, V> &map, K key) {
  return const_cast<V *>(
      lookup(const_cast<const std::unordered_map<K, V> &>(map), key));
}

/**
 * Union between set `a` and set `b`.
 */
template <typename T>
T set_union(const T &s1, const T &s2) {
  T result = s1;
  result.insert(s2.begin(), s2.end());
  return result;
}

/**
 * Difference between `a` and set `b`.
 */
template <typename T>
T set_diff(const T &a, const T &b) {
  T results;
  std::set_difference(a.begin(),
                      a.end(),
                      b.begin(),
                      b.end(),
                      std::inserter(results, results.end()));
  return results;
}

/**
 * Symmetric difference between `a` and `b`.
 */
template <typename T>
T set_symmetric_diff(const T &a, const T &b) {
  T results;
  std::set_symmetric_difference(a.begin(),
                                a.end(),
                                b.begin(),
                                b.end(),
                                std::back_inserter(results));
  return results;
}

/**
 * Intersection between std::vectors `vecs`.
 * @returns Number of common elements
 */
template <typename T>
std::set<T> intersection(const std::list<std::vector<T>> &vecs) {
  // Obtain element count across all vectors
  std::unordered_map<T, size_t> counter;
  for (const auto &vec : vecs) { // Loop over all vectors
    for (const auto &p : vec) {  // Loop over elements in vector
      counter[p] += 1;
    }
  }

  // Build intersection result
  std::set<T> retval;
  for (const auto &el : counter) {
    if (el.second == vecs.size()) {
      retval.insert(el.first);
    }
  }

  return retval;
}

/**
 * Ordered Set
 */
template <class T>
class ordered_set_t {
public:
  using iterator                     = typename std::vector<T>::iterator;
  using const_iterator               = typename std::vector<T>::const_iterator;

  iterator begin()                   { return vector.begin(); }
  iterator end()                     { return vector.end(); }
  const_iterator begin() const       { return vector.begin(); }
  const_iterator end() const         { return vector.end(); }
  const T& at(const size_t i) const  { return vector.at(i); }
  const T& front() const             { return vector.front(); }
  const T& back() const              { return vector.back(); }
  void insert(const T& item)         { if (set.insert(item).second) vector.push_back(item); }
  size_t count(const T& item) const  { return set.count(item); }
  bool empty() const                 { return set.empty(); }
  size_t size() const                { return set.size(); }
  void clear()                       { vector.clear(); set.clear(); }

private:
  std::vector<T> vector;
  std::set<T>    set;
};

/* Save vector of size 3 `y` with timestamps `ts` to `save_path`. */
void save_data(const std::string &save_path,
							 const timestamps_t &ts,
							 const vec3s_t &y);

/**
 * Save 3D features to csv file defined in `path`.
 */
void save_features(const std::string &path, const vec3s_t &features);

/**
 * Save pose to `csv_file` incrementally.
 */
void save_pose(FILE *csv_file,
               const timestamp_t &ts,
               const quat_t &rot,
               const vec3_t &pos);

/**
 * Save pose to `csv_file` incrementally.
 */
void save_pose(FILE *csv_file, const timestamp_t &ts, const vecx_t &pose);

/**
 * Save poses to csv file in `path`.
 */
void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const quats_t &orientations,
                const vec3s_t &positions);

/**
 * Save poses to csv file in `path`.
 */
void save_poses(const std::string &path, const mat4s_t &poses);

/**
 * Save poses to csv file in `path`.
 */
void save_poses(const std::string &path,
							  const timestamps_t &timestamps,
								const mat4s_t &poses);

/**
 * Save pose to csv file in `path`.
 */
void save_pose(const std::string &path, const mat4_t &pose);

/**
 * Save pose to csv file in `path`.
 */
void save_pose(const std::string &path,
               const timestamp_t &ts,
               const mat4_t &pose);

/**
 * Save IMU measurements
 */
void save_imu_data(const std::string &path,
							     const timestamps_t &imu_ts,
								   const vec3s_t &imu_acc,
								   const vec3s_t &imu_gyr);

/** Load pose */
mat4_t load_pose(const std::string &fpath);

/** Load poses */
void load_poses(const std::string &fpath,
                timestamps_t &timestamps,
                mat4s_t &poses);

/**
 * Check jacobian
 */
int check_jacobian(const std::string &jac_name,
                   const matx_t &fdiff,
                   const matx_t &jac,
                   const real_t threshold,
                   const bool print=false);

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


/******************************************************************************
 *                              CONFIGURATION
 *****************************************************************************/

struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t();
  config_t(const std::string &file_path_);
  ~config_t();
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
          vecx_t &vec,
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
  } else {
    FATAL("Unsportted vector type!");
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
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  out = node.as<T>();
  return 0;
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  std::vector<T> array;
  for (auto n : node) {
    out.push_back(n.as<T>());
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
int sign(const real_t x);

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
int fltcmp(const real_t f1, const real_t f2);

/**
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
real_t binomial(const real_t n, const real_t k);

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

  const real_t diff = static_cast<real_t>(end - start);
  const real_t delta = diff / static_cast<real_t>(num - 1);
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
                 const real_t *array,
                 const size_t size);

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const vecx_t &v);

/**
 * Print matrix `m` with a `name`.
 */
void print_matrix(const std::string &name, const matx_t &m);

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
std::string array2str(const real_t *array, const size_t size);

/**
 * Convert real_t array to Eigen::Vector
 *
 * @param[in] x Input array
 * @param[in] size Size of input array
 * @param[out] y Output vector
 */
void array2vec(const real_t *x, const size_t size, vecx_t &y);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @returns Array
 */
real_t *vec2array(const vecx_t &v);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @returns Array
 */
real_t *mat2array(const matx_t &m);

/**
 * Quaternion to array
 *
 * *VERY IMPORTANT*: The returned array is (x, y, z, w).
 *
 * @param[in] q Quaternion
 * @returns Array
 */
real_t *quat2array(const quat_t &q);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @param[out] out Output array
 */
void vec2array(const vecx_t &v, real_t *out);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @param[in] out Output array
 */
void mat2array(const matx_t &m, real_t *out);

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
 * @returns Vector as a string
 */
std::string vec2str(const vecx_t &v, const bool brackets = true);

/**
 * Array to string
 *
 * @param[in] arr Array
 * @param[in] len Length of array
 * @param[in] brackets Brakcets around vector string
 * @returns Array as a string
 */
std::string arr2str(const real_t *arr, const size_t len, bool brackets = true);

/**
 * Matrix to string
 *
 * @param[in] m Matrix
 * @param[in] indent Indent string
 * @returns Array as a string
 */
std::string mat2str(const matx_t &m, const std::string &indent = "  ");

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
real_t cond(const matx_t &A);

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
 * Load std::vector of real_ts to an Eigen::Matrix
 *
 * @param[in] x Matrix values
 * @param[in] rows Number of matrix rows
 * @param[in] cols Number of matrix colums
 * @param[out] y Output matrix
 */
void load_matrix(const std::vector<real_t> &x,
                 const int rows,
                 const int cols,
                 matx_t &y);

/**
 * Load an Eigen::Matrix into a std::vector of real_ts
 *
 * @param[in] A Matrix
 * @param[out] x Output vector of matrix values
 */
void load_matrix(const matx_t A, std::vector<real_t> &x);

/** Pseudo Inverse via SVD **/
matx_t pinv(const matx_t &A, const real_t tol=1e-4);

/** Rank of matrix A **/
long int rank(const matx_t &A);

/** Check if matrix A is full rank */
bool full_rank(const matx_t &A);

/**
 * Perform Schur's Complement
 */
int schurs_complement(matx_t &H, vecx_t &b,
                      const size_t m, const size_t r,
                      const bool precond=false, const bool debug=false);


// /**
//  * Recover covariance(i, l) (a specific value in the covariance matrix) from
//  * the upper triangular matrix `U` with precomputed diagonal vector containing
//  * `diag(U)^{-1}`. Computed covariances will be stored in the `hash` to avoid
//  * recomputing the value again.
//  */
// real_t covar_recover(const long i, const long l,
//                      const matx_t &U, const vecx_t &diag,
//                      mat_hash_t &hash);
//
// /**
//  * From the Hessian matrix `H`, recover the covariance values defined in
//  * `indicies`. Returns a matrix hashmap of covariance values.
//  */
// mat_hash_t covar_recover(const matx_t &H, const mat_indicies_t &indicies);

/******************************************************************************
 *                                 Geometry
 *****************************************************************************/

/**
 * Sinc function.
 */
real_t sinc(const real_t x);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
real_t deg2rad(const real_t d);

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
real_t rad2deg(const real_t r);

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
real_t wrap180(const real_t d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
real_t wrap360(const real_t d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
real_t wrapPi(const real_t r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
real_t wrap2Pi(const real_t r);

/**
 * Create a circle point of radius `r` at angle `theta` radians.
 */
vec2_t circle(const real_t r, const real_t theta);

/**
 * Create the sphere point with sphere radius `rho` at longitude `theta`
 * [radians] and latitude `phi` [radians].
 */
vec3_t sphere(const real_t rho, const real_t theta, const real_t phi);

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
real_t cross_track_error(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

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
real_t closest_point(const vec2_t &p1,
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
vec2s_t intersect_circles(const double cx0, const double cy0, const double r0,
                          const double cx1, const double cy1, const double r1);

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
void latlon_offset(real_t lat_ref,
                   real_t lon_ref,
                   real_t offset_N,
                   real_t offset_E,
                   real_t *lat_new,
                   real_t *lon_new);

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
void latlon_diff(real_t lat_ref,
                 real_t lon_ref,
                 real_t lat,
                 real_t lon,
                 real_t *dist_N,
                 real_t *dist_E);

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
real_t latlon_dist(real_t lat_ref, real_t lon_ref, real_t lat, real_t lon);

/*****************************************************************************
 *                         DIFFERENTIAL GEOMETRY
 *****************************************************************************/

namespace lie {

mat3_t Exp(const vec3_t &phi);
vec3_t Log(const mat3_t &C);
mat3_t Jr(const vec3_t &psi);

} // namespace lie

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
 * Create random real_t
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random floating point
 */
real_t randf(const real_t ub, const real_t lb);

/**
 * Sum values in vector.
 *
 * @param[in] x Array of numbers
 * @return Sum of vector
 */
real_t sum(const std::vector<real_t> &x);

/** Calculate median given an array of numbers */
real_t median(const std::vector<real_t> &v);

/** Mean */
real_t mean(const std::vector<real_t> &x);

/** Mean */
vec3_t mean(const vec3s_t &x);

/** Variance */
real_t var(const std::vector<real_t> &x);

/** Variance */
vec3_t var(const vec3s_t &vecs);

/** Standard Deviation */
real_t stddev(const std::vector<real_t> &x);

/** Root Mean Squared Error.  */
real_t rmse(const std::vector<real_t> &x);

/**
 * Shannon Entropy of a given covariance matrix `covar`.
 */
real_t shannon_entropy(const matx_t &covar);

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
real_t gauss_normal();

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
 * Form a 4x4 homogeneous transformation matrix from a pointer to real_t array
 * containing (quaternion + translation) 7 elements: (qw, qx, qy, qz, x, y, z)
 */
mat4_t tf(const double *params);

/**
 * Form a 4x4 homogeneous transformation matrix from a pointer to real_t array
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
mat4_t tf_perturb_rot(const mat4_t &T, real_t step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_trans(const mat4_t &T, real_t step_size, const int i);

/**
 * Transform point `p` with transform `T`.
 */
vec3_t tf_point(const mat4_t &T, const vec3_t &p);

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotx(const real_t theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t roty(const real_t theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotz(const real_t theta);

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
mat3_t rvec2rot(const vec3_t &rvec, const real_t eps = 1e-5);

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
mat3_t add_noise(const mat3_t &rot, const real_t n);

/**
 * Add noise to position vector `pos`, where noise `n` is in meters.
 */
vec3_t add_noise(const vec3_t &pos, const real_t n);

/**
 * Add noise to transform `pose`, where `pos_n` is in meters and `rot_n` is in
 * degrees.
 */
mat4_t add_noise(const mat4_t &pose, const real_t pos_n, const real_t rot_n);

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
timestamp_t sec2ts(const real_t sec);

/**
 * Convert ts to second.
 */
real_t ts2sec(const timestamp_t &ts);

/**
 * Convert nano-second to second.
 */
real_t ns2sec(const int64_t ns);

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
real_t time_now();

/**
 * Profiler
 */
struct profiler_t {
  std::map<std::string, timespec> timers;
  std::map<std::string, float> record;

  profiler_t() {}

  void start(const std::string &key) {
    timers[key] = tic();
  }

  float stop(const std::string &key) {
    record[key] = toc(&timers[key]);
    return record[key];
  }

  void print(const std::string &key) {
    printf("[%s]: %.4fs\n", key.c_str(), stop(key));
  }
};

/*****************************************************************************
 *                               NETWORKING
 ****************************************************************************/

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 */
int ip_port_info(const int sockfd, char *ip, int *port);

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 */
int ip_port_info(const int sockfd, std::string &ip, int &port);

/**
 * TCP server
 */
struct tcp_server_t {
  int port = 8080;
  int sockfd = -1;
  std::vector<int> conns;
  void *(*conn_thread)(void *) = nullptr;

  tcp_server_t(int port_ = 8080);
};

/**
 * TCP client
 */
struct tcp_client_t {
  std::string server_ip;
  int server_port = 8080;
  int sockfd = -1;
  int (*loop_cb)(tcp_client_t &) = nullptr;

  tcp_client_t(const std::string &server_ip_ = "127.0.0.1",
               int server_port_ = 8080);
};

/**
 * Configure TCP server
 */
int tcp_server_config(tcp_server_t &server);

/**
 * Loop TCP server
 */
int tcp_server_loop(tcp_server_t &server);

/**
 * Configure TCP client
 */
int tcp_client_config(tcp_client_t &client);

/**
 * Loop TCP client
 */
int tcp_client_loop(tcp_client_t &client);

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
T lerp(const T &a, const T &b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Slerp
 */
quat_t slerp(const quat_t &q_start, const quat_t &q_end, const real_t alpha);

/**
 * Interpolate between two poses `p0` and `p1` with parameter `alpha`.
 */
mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const real_t alpha);

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
                  const real_t threshold = 0.001);

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

typedef Eigen::Spline<real_t, 1> Spline1D;
typedef Eigen::Spline<real_t, 2> Spline2D;
typedef Eigen::Spline<real_t, 3> Spline3D;

#define SPLINE1D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline1D>::Interpolate(X, DEG, Y)

#define SPLINE2D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline2D>::Interpolate(X, DEG, Y)

#define SPLINE3D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline3D>::Interpolate(X, DEG, Y)

/**
 * Continuous trajectory generator
 */
struct ctraj_t {
  const timestamps_t timestamps;
  const vec3s_t positions;
  const quats_t orientations;

  const real_t ts_s_start;
  const real_t ts_s_end;
  const real_t ts_s_gap;

  Spline3D pos_spline;
  Spline3D rvec_spline;

  ctraj_t(const timestamps_t &timestamps,
          const vec3s_t &positions,
          const quats_t &orientations);
};

/**
 * Container for multiple continuous trajectories
 */
typedef std::vector<ctraj_t> ctrajs_t;

/**
 * Initialize continuous trajectory.
 */
void ctraj_init(ctraj_t &ctraj);

/**
 * Calculate pose `T_WB` at timestamp `ts`.
 */
mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate velocity `v_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate acceleration `a_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate angular velocity `w_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Save trajectory to file
 */
int ctraj_save(const ctraj_t &ctraj, const std::string &save_path);

/**
 * SIM IMU
 */
struct sim_imu_t {
  // IMU parameters
  real_t rate = 0.0;        // IMU rate [Hz]
  real_t tau_a = 3600.0;    // Reversion time constant for accel [s]
  real_t tau_g = 3600.0;    // Reversion time constant for gyro [s]
  real_t sigma_g_c = 0.0;   // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;   // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0;  // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0;  // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t g = 9.81;          // Gravity vector [ms-2]

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

/*****************************************************************************
 *                                CONTROL
 *****************************************************************************/

/**
 * PID Controller
 */
struct pid_t {
  real_t error_prev = 0.0;
  real_t error_sum = 0.0;

  real_t error_p = 0.0;
  real_t error_i = 0.0;
  real_t error_d = 0.0;

  real_t k_p = 0.0;
  real_t k_i = 0.0;
  real_t k_d = 0.0;

  pid_t();
  pid_t(const real_t k_p, const real_t k_i, const real_t k_d);
  ~pid_t();
};

/**
 * `pid_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const pid_t &pid);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p,
                  const real_t setpoint,
                  const real_t actual,
                  const real_t dt);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p, const real_t error, const real_t dt);

/**
 * Reset controller
 */
void pid_reset(pid_t &p);

/**
 * Carrot control
 */
struct carrot_ctrl_t {
  vec3s_t waypoints;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
  size_t wp_index = 0;
  real_t look_ahead_dist = 0.0;

  carrot_ctrl_t();
  ~carrot_ctrl_t();
};

/**
 * Configure carrot control using a list of position `waypoints` (x, y, z), and
 * a `look_ahead` distance in [m].
 *
 * @returns 0 for success, -1 for failure
 */
int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const real_t look_ahead_dist);

/**
 * Calculate closest point along current trajectory using current position
 * `pos`, and outputs the closest point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result);

/**
 * Calculate carrot point using current position `pos`, and outputs the carrot
 * point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result);

/**
 * Update carrot controller using current position `pos` and outputs the carrot
 * point in `result`.
 *
 * @returns 0 for success, 1 for all waypoints reached and -1 for failure
 */
int carrot_ctrl_update(carrot_ctrl_t &cc, const vec3_t &pos, vec3_t &carrot_pt);

/******************************************************************************
 *                               MEASUREMENTS
 *****************************************************************************/

struct meas_t {
  timestamp_t ts = 0;

  meas_t() {}
  meas_t(const timestamp_t &ts_) : ts{ts_} {}
  virtual ~meas_t() {}
};

struct imu_meas_t {
  timestamp_t ts = 0;
  vec3_t accel{0.0, 0.0, 0.0};
  vec3_t gyro{0.0, 0.0, 0.0};

  imu_meas_t() {}

  imu_meas_t(const timestamp_t &ts_, const vec3_t &accel_, const vec3_t &gyro_)
    : ts{ts_}, accel{accel_}, gyro{gyro_} {}

  ~imu_meas_t() {}
};

struct imu_data_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  timestamps_t timestamps;
  vec3s_t accel;
  vec3s_t gyro;

  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr) {
    timestamps.push_back(ts);
    accel.push_back(acc);
    gyro.push_back(gyr);
  }

  size_t size() const { return timestamps.size(); }
  size_t size() { return static_cast<const imu_data_t &>(*this).size(); }

  timestamp_t last_ts() const { return timestamps.back(); }
  timestamp_t last_ts() { return static_cast<const imu_data_t &>(*this).last_ts(); }

  void clear() {
    timestamps.clear();
    accel.clear();
    gyro.clear();
  }

  friend std::ostream& operator<<(std::ostream &os, const imu_data_t &data) {
    os << "nb_measurements: " << data.size() << std::endl;
    for (size_t k = 0; k < data.timestamps.size(); k++) {
      os << "ts: " << std::to_string(data.timestamps[k]) << " ";
      os << "acc: " << vec2str(data.accel[k]) << " ";
      os << "gyr: " << vec2str(data.gyro[k]) << std::endl;
    }
    return os;
  }
};

// struct image_t : meas_t {
//   int width = 0;
//   int height = 0;
//   float *data = nullptr;
//
//   image_t() {}
//
//   image_t(const timestamp_t ts_, const int width_, const int height_)
//       : meas_t{ts_}, width{width_}, height{height_} {
//     data = new float[width * height];
//   }
//
//   image_t(const timestamp_t ts_,
//           const int width_,
//           const int height_,
//           float *data_)
//       : meas_t{ts_}, width{width_}, height{height_}, data{data_} {}
//
//   virtual ~image_t() {
//     if (data) {
//       free(data);
//     }
//   }
// };

struct cam_frame_t {
  timestamp_t ts = 0;
  vec2s_t keypoints;
  std::vector<size_t> feature_ids;

  cam_frame_t() {}

  cam_frame_t(const timestamp_t &ts_,
              const vec2s_t &keypoints_,
              const std::vector<size_t> feature_ids_)
    : ts{ts_}, keypoints{keypoints_}, feature_ids{feature_ids_} {}

  ~cam_frame_t() {}
};

/******************************************************************************
 *                                 MODELS
 *****************************************************************************/

/**
 * Create DH transform from link n to link n-1 (end to front)
 *
 * @param[in] theta
 * @param[in] d
 * @param[in] a
 * @param[in] alpha
 *
 * @returns DH transform
 */
mat4_t dh_transform(const real_t theta,
                    const real_t d,
                    const real_t a,
                    const real_t alpha);

/**
 * 2-DOF Gimbal Model
 */
struct gimbal_model_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Parameter vector of transform from
  // static camera to base-mechanism
  vecx_t tau_s = zeros(6, 1);

  // Parameter vector of transform from
  // end-effector to dynamic camera
  vecx_t tau_d = zeros(6, 1);

  // First gibmal-joint
  real_t Lambda1 = 0.0;
  vec3_t w1 = zeros(3, 1);

  // Second gibmal-joint
  real_t Lambda2 = 0.0;
  vec3_t w2 = zeros(3, 1);

  real_t theta1_offset = 0.0;
  real_t theta2_offset = 0.0;

  gimbal_model_t();
  gimbal_model_t(const vec6_t &tau_s,
                 const vec6_t &tau_d,
                 const real_t Lambda1,
                 const vec3_t w1,
                 const real_t Lambda2,
                 const vec3_t w2,
                 const real_t theta1_offset = 0.0,
                 const real_t theta2_offset = 0.0);
  virtual ~gimbal_model_t();
};

/**
 * Set gimbal attitude
 *
 * @param[in,out] model Model
 * @param[in] roll Roll [rads]
 * @param[in] pitch Pitch [rads]
 */
void gimbal_model_set_attitude(gimbal_model_t &model,
                               const real_t roll,
                               const real_t pitch);

/**
 * Get gimbal joint angle
 *
 * @param[in] model Model
 * @returns Gimbal joint angles
 */
vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model);

/**
 * Returns transform from static camera to base mechanism
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_BS(const gimbal_model_t &model);

/**
 * Returns transform from base mechanism to end-effector
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_EB(const gimbal_model_t &model);

/**
 * Returns transform from end-effector to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DE(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DS(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in,out] model Model
 * @param[in] theta Gimbal roll and pitch [radians]
 * @returns Transform from static to dynamic camera
 */
mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta);

/**
 * gimbal_model_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_model_t &gimbal);

/**
 * Calculate target angular velocity and time taken to traverse a desired
 * circle * trajectory of radius r and velocity v
 *
 * @param[in] r Desired circle radius
 * @param[in] v Desired trajectory velocity
 * @param[in] w Target angular velocity
 * @param[in] time Target time taken to complete circle trajectory
 **/
void circle_trajectory(const real_t r, const real_t v, real_t *w, real_t *time);

/**
 * Two wheel robot
 */
struct two_wheel_t {
  vec3_t r_G = vec3_t::Zero();
  vec3_t v_G = vec3_t::Zero();
  vec3_t a_G = vec3_t::Zero();
  vec3_t rpy_G = vec3_t::Zero();
  vec3_t w_G = vec3_t::Zero();

  real_t vx_desired = 0.0;
  real_t yaw_desired = 0.0;

  pid_t vx_controller{0.1, 0.0, 0.1};
  pid_t yaw_controller{0.1, 0.0, 0.1};

  vec3_t a_B = vec3_t::Zero();
  vec3_t v_B = vec3_t::Zero();
  vec3_t w_B = vec3_t::Zero();

  two_wheel_t() {}

  two_wheel_t(const vec3_t &r_G_, const vec3_t &v_G_, const vec3_t &rpy_G_)
      : r_G{r_G_}, v_G{v_G_}, rpy_G{rpy_G_} {}

  ~two_wheel_t() {}

  void update(const real_t dt) {
    const vec3_t r_G_prev = r_G;
    const vec3_t v_G_prev = v_G;
    const vec3_t rpy_G_prev = rpy_G;

    r_G += euler321(rpy_G) * v_B * dt;
    v_G = (r_G - r_G_prev) / dt;
    a_G = (v_G - v_G_prev) / dt;

    rpy_G += euler321(rpy_G) * w_B * dt;
    w_G = rpy_G - rpy_G_prev;
    a_B = euler123(rpy_G) * a_G;

    // Wrap angles to +/- pi
    for (int i = 0; i < 3; i++) {
      rpy_G(i) = (rpy_G(i) > M_PI) ? rpy_G(i) - 2 * M_PI : rpy_G(i);
      rpy_G(i) = (rpy_G(i) < -M_PI) ? rpy_G(i) + 2 * M_PI : rpy_G(i);
    }
  }
};

/**
 * MAV model
 */
struct mav_model_t {
  vec3_t attitude{0.0, 0.0, 0.0};         ///< Attitude in global frame
  vec3_t angular_velocity{0.0, 0.0, 0.0}; ///< Angular velocity in global frame
  vec3_t position{0.0, 0.0, 0.0};         ///< Position in global frame
  vec3_t linear_velocity{0.0, 0.0, 0.0};  ///< Linear velocity in global frame

  real_t Ix = 0.0963; ///< Moment of inertia in x-axis
  real_t Iy = 0.0963; ///< Moment of inertia in y-axis
  real_t Iz = 0.1927; ///< Moment of inertia in z-axis

  real_t kr = 0.1; ///< Rotation drag constant
  real_t kt = 0.2; ///< Translation drag constant

  real_t l = 0.9; ///< MAV arm length
  real_t d = 1.0; ///< drag constant

  real_t m = 1.0;  ///< Mass
  real_t g = 9.81; ///< Gravity
};

/**
 * Update
 *
 * @param[in,out] qm Model
 * @param[in] motor_inputs Motor inputs (m1, m2, m3, m4)
 * @param[in] dt Time difference (s)
 * @returns 0 for success, -1 for failure
 */
int mav_model_update(mav_model_t &qm,
                     const vec4_t &motor_inputs,
                     const real_t dt);

/*****************************************************************************
 *                                  CV
 ****************************************************************************/

/**
 * Compare `cv::Mat` whether they are equal
 *
 * @param m1 First matrix
 * @param m2 Second matrix
 * @returns true or false
 */
bool is_equal(const cv::Mat &m1, const cv::Mat &m2);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const cv::Mat &x, matx_t &y);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const matx_t &x, cv::Mat &y);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @returns Matrix as Eigen::Matrix
 */
matx_t convert(const cv::Mat &x);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @returns Matrix as cv::Mat
 */
cv::Mat convert(const matx_t &x);

/**
 * Sort Keypoints
 *
 * @param keypoints
 * @param limit
 * @returns Sorted keypoints by response
 */
std::vector<cv::KeyPoint> sort_keypoints(
    const std::vector<cv::KeyPoint> keypoints, const size_t limit = 0);

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

/**
 * Create ROI from an image
 *
 * @param[in] image Input image
 * @param[in] width ROI width
 * @param[in] height ROI height
 * @param[in] cx ROI center x-axis
 * @param[in] cy ROI center y-axis
 *
 * @returns ROI
 */
cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const real_t cx,
            const real_t cy);

/**
 * Compare two keypoints based on the response.
 *
 * @param[in] kp1 First keypoint
 * @param[in] kp2 Second keypoint
 * @returns Boolean to denote if first keypoint repose is larger than second
 */
bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 *
 * @returns Undistorted image using radial-tangential distortion
 */
cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 * @param[in] balance Balance
 * @param[in,out] Knew New camera matrix K
 *
 * @returns Undistorted image using equidistant distortion
 */
cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew);
/**
 * Illumination invariant transform.
 *
 * @param[in] image Image
 * @param[in] lambda_1 Lambad 1
 * @param[in] lambda_2 Lambad 2
 * @param[in] lambda_3 Lambad 3
 */
void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3);

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Image frame 0
 * @param[in] img1 Image frame 1
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] status Inlier vector
 *
 * @returns Image with feature matches between frame 0 and 1
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Previous image frame
 * @param[in] img1 Current image frame
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] matches Feature matches
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features);

/**
 * Grid fast
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] threshold Fast threshold
 * @param[in] nonmax_suppression Nonmax Suppression
 *
 * @returns List of keypoints
 */
std::vector<cv::Point2f> grid_fast(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const real_t threshold = 10.0,
                                   const bool nonmax_suppression = true);

/**
 * Grid good
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] quality_level Quality level
 * @param[in] min_distance Min distance
 * @param[in] mask Mask
 * @param[in] block_size Block size
 * @param[in] use_harris_detector Use Harris detector
 * @param[in] k Free parameter for Harris detector
 *
 * @returns List of points
 */
std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const real_t quality_level = 0.01,
                                   const real_t min_distance = 10,
                                   const cv::Mat mask = cv::Mat(),
                                   const int block_size = 3,
                                   const bool use_harris_detector = false,
                                   const real_t k = 0.04);

/**
 * Distortion model
 */
struct distortion_t {
  vecx_t params;

  distortion_t() {}

  distortion_t(const vecx_t &params_)
    : params{params_} {}

  distortion_t(const real_t *params_, const size_t params_size_) {
    params.resize(params_size_);
    for (size_t i = 0; i < params_size_; i++) {
      params(i) = params_[i];
    }
  }

  virtual ~distortion_t() {}

  virtual vec2_t distort(const vec2_t &p) = 0;
  virtual vec2_t distort(const vec2_t &p) const = 0;

  virtual vec2_t undistort(const vec2_t &p) = 0;
  virtual vec2_t undistort(const vec2_t &p) const = 0;

  virtual mat2_t J_point(const vec2_t &p) = 0;
  virtual mat2_t J_point(const vec2_t &p) const = 0;

  virtual matx_t J_dist(const vec2_t &p) = 0;
  virtual matx_t J_dist(const vec2_t &p) const = 0;

  // virtual void operator=(const distortion_t &src) throw() = 0;
};

/**
 * No distortion
 */
struct nodist_t : distortion_t {
  static const size_t params_size = 0;

  nodist_t() {}
  nodist_t(const vecx_t &) {}
  nodist_t(const real_t *) {}
  ~nodist_t() {}

  vec2_t distort(const vec2_t &p) {
    return static_cast<const nodist_t &>(*this).distort(p);
  }

  vec2_t distort(const vec2_t &p) const {
    return p;
  }

  vec2_t undistort(const vec2_t &p) {
    return static_cast<const nodist_t &>(*this).undistort(p);
  }

  vec2_t undistort(const vec2_t &p) const {
    return p;
  }

  mat2_t J_point(const vec2_t &p) {
    return static_cast<const nodist_t &>(*this).J_point(p);
  }

  mat2_t J_point(const vec2_t &p) const {
    UNUSED(p);
    return I(2);
  }

  matx_t J_dist(const vec2_t &p) {
    return static_cast<const nodist_t &>(*this).J_dist(p);
  }

  matx_t J_dist(const vec2_t &p) const {
    UNUSED(p);
    matx_t J;
    J.resize(2, 0);
    return J;
  }
};

/**
 * Radial-tangential distortion
 */
struct radtan4_t : distortion_t {
  static const size_t params_size = 4;

  radtan4_t() {}

  radtan4_t(const vecx_t &params_)
    : distortion_t{params_} {}

  radtan4_t(const real_t *dist_params)
    : distortion_t{dist_params, params_size} {}

  radtan4_t(const real_t k1,
            const real_t k2,
            const real_t p1,
            const real_t p2)
    : distortion_t{vec4_t{k1, k2, p1, p2}} {}

  virtual ~radtan4_t() {}

  real_t k1() { return static_cast<const radtan4_t &>(*this).k1(); }
  real_t k2() { return static_cast<const radtan4_t &>(*this).k2(); }
  real_t p1() { return static_cast<const radtan4_t &>(*this).p1(); }
  real_t p2() { return static_cast<const radtan4_t &>(*this).p2(); }
  real_t k1() const { return params(0); }
  real_t k2() const { return params(1); }
  real_t p1() const { return params(2); }
  real_t p2() const { return params(3); }

  vec2_t distort(const vec2_t &p) {
    return static_cast<const radtan4_t &>(*this).distort(p);
  }

  vec2_t distort(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    // Apply radial distortion
    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;
    const real_t radial_factor = 1 + (k1() * r2) + (k2() * r4);
    const real_t x_dash = x * radial_factor;
    const real_t y_dash = y * radial_factor;

    // Apply tangential distortion
    const real_t xy = x * y;
    const real_t x_ddash = x_dash + (2 * p1() * xy + p2() * (r2 + 2 * x2));
    const real_t y_ddash = y_dash + (p1() * (r2 + 2 * y2) + 2 * p2() * xy);

    return vec2_t{x_ddash, y_ddash};
  }

  vec2_t undistort(const vec2_t &p0) {
    return static_cast<const radtan4_t &>(*this).undistort(p0);
  }

  vec2_t undistort(const vec2_t &p0) const {
    vec2_t p = p0;
    int max_iter = 5;

    for (int i = 0; i < max_iter; i++) {
      // Error
      const vec2_t p_distorted = distort(p);
      const vec2_t err = (p0 - p_distorted);

      // Jacobian
      mat2_t J = J_point(p);
      const mat2_t pinv = (J.transpose() * J).inverse() * J.transpose();
      const vec2_t dp = pinv * err;
      p = p + dp;

      if ((err.transpose() * err) < 1.0e-15) {
        break;
      }
    }

    return p;
  }

  mat2_t J_point(const vec2_t &p) {
    return static_cast<const radtan4_t &>(*this).J_point(p);
  }

  mat2_t J_point(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;

    // Let p = [x; y] normalized point
    // Let p' be the distorted p
    // The jacobian of p' w.r.t. p (or dp'/dp) is:
    mat2_t J_point;
    J_point(0, 0) = 1 + k1() * r2 + k2() * r4;
    J_point(0, 0) += 2 * p1() * y + 6 * p2() * x;
    J_point(0, 0) += x * (2 * k1() * x + 4 * k2() * x * r2);
    J_point(1, 0) = 2 * p1() * x + 2 * p2() * y;
    J_point(1, 0) += y * (2 * k1() * x + 4 * k2() * x * r2);
    J_point(0, 1) = J_point(1, 0);
    J_point(1, 1) = 1 + k1() * r2 + k2() * r4;
    J_point(1, 1) += 6 * p1() * y + 2 * p2() * x;
    J_point(1, 1) += y * (2 * k1() * y + 4 * k2() * y * r2);
    // Above is generated using sympy

    return J_point;
  }

  matx_t J_dist(const vec2_t &p) {
    return static_cast<const radtan4_t &>(*this).J_dist(p);
  }

  matx_t J_dist(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    const real_t xy = x * y;
    const real_t x2 = x * x;
    const real_t y2 = y * y;
    const real_t r2 = x2 + y2;
    const real_t r4 = r2 * r2;

    mat_t<2, 4> J_dist = zeros(2, 4);
    J_dist(0, 0) = x * r2;
    J_dist(0, 1) = x * r4;
    J_dist(0, 2) = 2 * xy;
    J_dist(0, 3) = 3 * x2 + y2;

    J_dist(1, 0) = y * r2;
    J_dist(1, 1) = y * r4;
    J_dist(1, 2) = x2 + 3 * y2;
    J_dist(1, 3) = 2 * xy;

    return J_dist;
  }
};

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4);

/**
 * Equi-distant distortion
 */
struct equi4_t : distortion_t {
  static const size_t params_size = 4;

  equi4_t() {}

  equi4_t(const vecx_t &dist_params)
    : distortion_t{dist_params} {}

  equi4_t(const real_t *dist_params)
    : distortion_t{dist_params, params_size} {}

  equi4_t(const real_t k1,
          const real_t k2,
          const real_t k3,
          const real_t k4) {
    params.resize(4);
    params << k1, k2, k3, k4;
  }

  ~equi4_t() {}

  real_t k1() { return static_cast<const equi4_t &>(*this).k1(); }
  real_t k2() { return static_cast<const equi4_t &>(*this).k2(); }
  real_t k3() { return static_cast<const equi4_t &>(*this).k3(); }
  real_t k4() { return static_cast<const equi4_t &>(*this).k4(); }

  real_t k1() const { return this->params(0); }
  real_t k2() const { return this->params(1); }
  real_t k3() const { return this->params(2); }
  real_t k4() const { return this->params(3); }

  vec2_t distort(const vec2_t &p) {
    return static_cast<const equi4_t &>(*this).distort(p);
  }

  vec2_t distort(const vec2_t &p) const {
    const real_t r = p.norm();
    if (r < 1e-8) {
      return p;
    }

    // Apply equi distortion
    const real_t th = atan(r);
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    const real_t thd = th * (1 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    const real_t x_dash = (thd / r) * p(0);
    const real_t y_dash = (thd / r) * p(1);

    return vec2_t{x_dash, y_dash};
  }

  vec2_t undistort(const vec2_t &p) {
    return static_cast<const equi4_t &>(*this).undistort(p);
  }

  vec2_t undistort(const vec2_t &p) const {
    const real_t thd = sqrt(p(0) * p(0) + p(1) * p(1));

    real_t th = thd; // Initial guess
    for (int i = 20; i > 0; i--) {
      const real_t th2 = th * th;
      const real_t th4 = th2 * th2;
      const real_t th6 = th4 * th2;
      const real_t th8 = th4 * th4;
      th = thd / (1 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    }

    const real_t scaling = tan(th) / thd;
    return vec2_t{p(0) * scaling, p(1) * scaling};
  }

  mat2_t J_point(const vec2_t &p) {
    return static_cast<const equi4_t &>(*this).J_point(p);
  }

  mat2_t J_point(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);
    const real_t r = p.norm();
    const real_t th = atan(r);
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    const real_t thd = th * (1.0 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8);
    const real_t s = thd / r;

    // Form jacobian
    const real_t th_r = 1.0 / (r * r + 1.0);
    real_t thd_th = 1.0 + 3.0 * k1() * th2;
    thd_th += 5.0 * k2() * th4;
    thd_th += 7.0 * k3() * th6;
    thd_th += 9.0 * k4() * th8;
    const real_t s_r = thd_th * th_r / r - thd / (r * r);
    const real_t r_x = 1.0 / r * x;
    const real_t r_y = 1.0 / r * y;

    mat2_t J_point = I(2);
    J_point(0, 0) = s + x * s_r * r_x;
    J_point(0, 1) = x * s_r * r_y;
    J_point(1, 0) = y * s_r * r_x;
    J_point(1, 1) = s + y * s_r * r_y;

    return J_point;
  }

  matx_t J_dist(const vec2_t &p) {
    return static_cast<const equi4_t &>(*this).J_dist(p);
  }

  matx_t J_dist(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);
    const real_t r = p.norm();
    const real_t th = atan(r);

    const real_t th3 = th * th * th;
    const real_t th5 = th3 * th * th;
    const real_t th7 = th5 * th * th;
    const real_t th9 = th7 * th * th;

    matx_t J_dist = zeros(2, 4);
    J_dist(0, 0) = x * th3 / r;
    J_dist(0, 1) = x * th5 / r;
    J_dist(0, 2) = x * th7 / r;
    J_dist(0, 3) = x * th9 / r;

    J_dist(1, 0) = y * th3 / r;
    J_dist(1, 1) = y * th5 / r;
    J_dist(1, 2) = y * th7 / r;
    J_dist(1, 3) = y * th9 / r;

    return J_dist;
  }
};

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4);

/* Projection model */
template <typename DM = nodist_t>
struct projection_t {
  int resolution[2] = {0, 0};
  vecx_t params;
  DM distortion;

  projection_t() {}

  projection_t(const int resolution_[2],
               const vecx_t &proj_params_,
               const vecx_t &dist_params_)
    : resolution{resolution_[0], resolution_[1]},
      params{proj_params_},
      distortion{dist_params_} {}

  projection_t(const int resolution_[2],
               const vecx_t &params_,
               const size_t proj_params_size_,
               const size_t dist_params_size_)
    : projection_t{resolution_,
                   params_.head(proj_params_size_),
                   params_.tail(dist_params_size_)} {}

  ~projection_t() {}

  virtual mat2_t J_point() = 0;
  virtual mat2_t J_point() const = 0;

  virtual matx_t J_proj(const vec2_t &p) = 0;
  virtual matx_t J_proj(const vec2_t &p) const = 0;

  virtual matx_t J_dist(const vec2_t &p) = 0;
  virtual matx_t J_dist(const vec2_t &p) const = 0;
};

/**
 * Pinhole projection model
 */
template <typename DM = nodist_t>
struct pinhole_t : projection_t<DM> {
  static const size_t proj_params_size = 4;
  static const size_t dist_params_size = DM::params_size;
  static const size_t params_size = proj_params_size + dist_params_size;

  pinhole_t() {}

  pinhole_t(const int resolution[2],
            const vecx_t &proj_params,
            const vecx_t &dist_params)
    : projection_t<DM>{resolution, proj_params, dist_params} {}

  pinhole_t(const int resolution[2],
            const vecx_t &params)
    : projection_t<DM>{resolution, params, proj_params_size, DM::params_size} {}

  pinhole_t(const int resolution[2],
            const real_t fx,
            const real_t fy,
            const real_t cx,
            const real_t cy)
      : projection_t<DM>{resolution, vec4_t{fx, fy, cx, cy}, zeros(0)} {}

  ~pinhole_t() {}

  real_t fx() { return static_cast<const pinhole_t &>(*this).fx(); }
  real_t fy() { return static_cast<const pinhole_t &>(*this).fy(); }
  real_t cx() { return static_cast<const pinhole_t &>(*this).cx(); }
  real_t cy() { return static_cast<const pinhole_t &>(*this).cy(); }

  real_t fx() const { return this->params(0); }
  real_t fy() const { return this->params(1); }
  real_t cx() const { return this->params(2); }
  real_t cy() const { return this->params(3); }

  vecx_t proj_params() {
    return static_cast<const pinhole_t &>(*this).proj_params();
  }

  vecx_t proj_params() const {
    return this->params;
  }

  vecx_t dist_params() {
    return static_cast<const pinhole_t &>(*this).dist_params();
  }

  vecx_t dist_params() const {
    return this->distortion.params;
  }

  mat3_t K() {
    return static_cast<const pinhole_t &>(*this).K();
  }

  mat3_t K() const {
    mat3_t K = zeros(3, 3);
    K(0, 0) = fx();
    K(1, 1) = fy();
    K(0, 2) = cx();
    K(1, 2) = cy();
    K(2, 2) = 1.0;
    return K;
  }

  int project(const vec3_t &p_C, vec2_t &z_hat) {
    return static_cast<const pinhole_t &>(*this).project(p_C, z_hat);
  }

  int project(const vec3_t &p_C, vec2_t &z_hat) const {
    // Check validity of the point, simple depth test.
    const real_t x = p_C(0);
    const real_t y = p_C(1);
    const real_t z = p_C(2);
    if (z < 0.0) {
      return -1;
    }

    // Project, distort and then scale and center
    const vec2_t p{x / z, y / z};
    const vec2_t p_dist = this->distortion.distort(p);
    z_hat(0) = fx() * p_dist(0) + cx();
    z_hat(1) = fy() * p_dist(1) + cy();

    // Check projection
    const bool x_ok = (z_hat(0) >= 0 && z_hat(0) <= this->resolution[0]);
    const bool y_ok = (z_hat(1) >= 0 && z_hat(1) <= this->resolution[1]);
    if (x_ok == false || y_ok == false) {
      return -2;
    }

    return 0;
  }

  int project(const vec3_t &p_C, vec2_t &z_hat, mat_t<2, 3> &J_h) {
    return static_cast<const pinhole_t &>(*this).project(p_C, z_hat, J_h);
  }

  int project(const vec3_t &p_C, vec2_t &z_hat, mat_t<2, 3> &J_h) const {
    int retval = project(p_C, z_hat);

    // Projection Jacobian
    const real_t x = p_C(0);
    const real_t y = p_C(1);
    const real_t z = p_C(2);
    mat_t<2, 3> J_proj;
    J_proj.setZero();
    J_proj(0, 0) = 1.0 / z;
    J_proj(1, 1) = 1.0 / z;
    J_proj(0, 2) = -x / (z * z);
    J_proj(1, 2) = -y / (z * z);

    // Measurement Jacobian
    const vec2_t p{x / z, y / z};
    J_h = J_point() * this->distortion.J_point(p) * J_proj;

    return retval;
  }

  int back_project(const vec2_t &kp, vec3_t &ray) {
    return static_cast<const pinhole_t &>(*this).back_project(kp, ray);
	}

  int back_project(const vec2_t &kp, vec3_t &ray) const {
    const real_t px = (kp(0) - cx()) / fx();
    const real_t py = (kp(1) - cy()) / fy();
    const vec2_t p{px, py};

    const vec2_t p_undist = this->distortion.undistort(p);
    ray(0) = p_undist(0);
    ray(1) = p_undist(1);
    ray(2) = 1.0;

    return 0;
  }

  vec2_t undistort_keypoint(const vec2_t &z) {
    return static_cast<const pinhole_t &>(*this).undistort(z);
  }

  vec2_t undistort_keypoint(const vec2_t &z) const {
    // Back-project and undistort
    const real_t px = (z(0) - cx()) / fx();
    const real_t py = (z(1) - cy()) / fy();
    const vec2_t p{px, py};
    const vec2_t p_undist = this->distortion.undistort(p);

    // Project undistorted point to image plane
    const real_t x = p_undist(0) * fx() + cx();
    const real_t y = p_undist(1) * fy() + cy();
    const vec2_t z_undist = {x, y};

    return z_undist;
  }

  mat2_t J_point() {
    return static_cast<const pinhole_t &>(*this).J_point();
  }

  mat2_t J_point() const {
    mat2_t J_K = zeros(2, 2);
    J_K(0, 0) = fx();
    J_K(1, 1) = fy();
    return J_K;
  }

  matx_t J_proj(const vec2_t &p) {
    return static_cast<const pinhole_t &>(*this).J_proj(p);
  }

  matx_t J_proj(const vec2_t &p) const {
    const real_t x = p(0);
    const real_t y = p(1);

    mat_t<2, 4> J_proj = zeros(2, 4);
    J_proj(0, 0) = x;
    J_proj(1, 1) = y;
    J_proj(0, 2) = 1;
    J_proj(1, 3) = 1;

    return J_proj;
  }

  matx_t J_dist(const vec2_t &p) {
    return static_cast<const pinhole_t &>(*this).J_dist(p);
  }

  matx_t J_dist(const vec2_t &p) const {
    return J_point() * this->distortion.J_dist(p);
  }

  matx_t J_params(const vec2_t &p) {
    return static_cast<const pinhole_t &>(*this).J_params(p);
  }

  matx_t J_params(const vec2_t &p) const {
    const vec2_t p_dist = this->distortion.distort(p);

    matx_t J = zeros(2, params_size);
    J.block(0, 0, 2, proj_params_size) = J_proj(p_dist);
    J.block(0, dist_params_size, 2, proj_params_size) = J_dist(p);
    return J;
  }
};

typedef pinhole_t<radtan4_t> pinhole_radtan4_t;
typedef pinhole_t<equi4_t> pinhole_equi4_t;
typedef pinhole_t<nodist_t> pinhole_ideal_t;

template <typename DM>
std::ostream &operator<<(std::ostream &os, const pinhole_t<DM> &pinhole) {
  os << "fx: " << pinhole.fx() << std::endl;
  os << "fy: " << pinhole.fy() << std::endl;
  os << "cx: " << pinhole.cx() << std::endl;
  os << "cy: " << pinhole.cy() << std::endl;

  os << std::endl;
  os << pinhole.distortion << std::endl;
  return os;
}

real_t pinhole_focal(const int image_size, const real_t fov);

mat3_t pinhole_K(const real_t fx,
                 const real_t fy,
                 const real_t cx,
                 const real_t cy);

mat3_t pinhole_K(const vec4_t &params);

mat3_t pinhole_K(const int img_w,
                 const int img_h,
                 const real_t lens_hfov,
                 const real_t lens_vfov);

template <typename CAMERA_TYPE>
int solvepnp(const CAMERA_TYPE &cam,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_CF) {
  assert(keypoints.size() == object_points.size());

  // Create object points (counter-clockwise, from bottom left)
  size_t nb_points = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  for (size_t i = 0; i < nb_points; i++) {
    const vec2_t kp = cam.undistort_keypoint(keypoints[i]);
    const vec3_t pt = object_points[i];
    img_pts.emplace_back(kp(0), kp(1));
    obj_pts.emplace_back(pt(0), pt(1), pt(2));
  }

  // Extract out camera intrinsics
  const vecx_t proj_params = cam.proj_params();
  const double fx = proj_params(0);
  const double fy = proj_params(1);
  const double cx = proj_params(2);
  const double cy = proj_params(3);

  // Solve pnp
  cv::Mat camera_matrix(3, 3, CV_32FC1, 0.0f);
  camera_matrix.at<float>(0, 0) = fx;
  camera_matrix.at<float>(1, 1) = fy;
  camera_matrix.at<float>(0, 2) = cx;
  camera_matrix.at<float>(1, 2) = cy;
  camera_matrix.at<float>(2, 2) = 1.0;
  cv::Vec4f distortion_params(0, 0, 0, 0); // SolvPnP assumes radtan

  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(obj_pts,
              img_pts,
              camera_matrix,
              distortion_params,
              rvec,
              tvec,
              false,
              CV_ITERATIVE);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF = tf(convert(R), convert(tvec));

  return 0;
}

/******************************************************************************
 *                               PARAMETERS
 *****************************************************************************/

struct imu_params_t {
  real_t rate = 0.0;        // IMU rate [Hz]
  real_t a_max = 160.0;     // Max accelerometer measurement [m/s^2]
  real_t g_max = 10.0;      // Max gyroscope measurement [rad/s]
  real_t sigma_g_c = 0.0;   // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;   // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0;  // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0;  // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t sigma_bg = 0.03;   // Gyro bias prior [rad/s]
  real_t sigma_ba = 0.1;    // Accel bias prior [m/s^2]
  real_t g = 9.81;          // Gravity vector [ms^-2]
};

/*****************************************************************************
 *                               SIMULATION
 *****************************************************************************/

enum sim_event_type_t {
  NOT_SET,
  CAMERA,
  IMU,
};

struct sim_event_t {
  sim_event_type_t type = NOT_SET;
  int sensor_id = 0;
  timestamp_t ts = 0;
  imu_meas_t imu;
  cam_frame_t frame;

  // Camera event
  sim_event_t(const int sensor_id_,
              const timestamp_t &ts_,
              const vec2s_t &keypoints_,
              const std::vector<size_t> &feature_idxs_)
    : type{CAMERA},
      sensor_id{sensor_id_},
      ts{ts_},
      frame{ts_, keypoints_, feature_idxs_} {}

  // IMU event
  sim_event_t(const int sensor_id_, const timestamp_t &ts_,
              const vec3_t &accel_, const vec3_t &gyro_)
    : type{IMU}, sensor_id{sensor_id_}, ts{ts_}, imu{ts_, accel_, gyro_} {}
};

struct vio_sim_data_t {
  // Settings
  real_t sensor_velocity = 0.3;
  real_t cam_rate = 30;
  real_t imu_rate = 400;

  // Scene data
  vec3s_t features;

  // Camera data
  timestamps_t cam_ts;
  vec3s_t cam_pos_gnd;
  quats_t cam_rot_gnd;
  mat4s_t cam_poses_gnd;
  vec3s_t cam_pos;
  quats_t cam_rot;
  mat4s_t cam_poses;
  std::vector<std::vector<size_t>> observations;
  std::vector<vec2s_t> keypoints;

  // IMU data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  vec3s_t imu_pos;
  quats_t imu_rot;
  mat4s_t imu_poses;
  vec3s_t imu_vel;

  // Simulation timeline
  std::multimap<timestamp_t, sim_event_t> timeline;

  // Add IMU measurement to timeline
  void add(const int sensor_id,
           const timestamp_t &ts,
           const vec3_t &accel,
           const vec3_t &gyro);

  // Add camera frame to timeline
  void add(const int sensor_id,
           const timestamp_t &ts,
           const vec2s_t &keypoints,
           const std::vector<size_t> &feature_idxs);

  void save(const std::string &dir);
};

void sim_circle_trajectory(const real_t circle_r, vio_sim_data_t &sim_data);

} //  namespace yac
#endif // YAC_CORE_HPP
