#ifndef YAC_CORE_HPP
#define YAC_CORE_HPP

/*****************************************************************************
 * This file is huge, it contains everything from parsing yaml files, linear
 * algebra functions to networking code used for robotics.
 *
 * Contents:
 * - Data Type
 * - Macros
 * - Filesystem
 * - Algebra
 * - Linear Algebra
 * - Geometry
 * - Differential Geometry
 * - Statistics
 * - Transform
 * - Time
 * - Interpolation
 * - Control
 * - Measurements
 * - Parameters
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

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/StdDeque>
#include <unsupported/Eigen/Splines>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "progressbar.hpp"

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

// -- TIMESTAMP ----------------------------------------------------------------

typedef int64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

// -- VECTOR ------------------------------------------------------------------

#define dynamic_t Eigen::Dynamic
#define col_major_t Eigen::ColMajor
#define row_major_t Eigen::RowMajor

using veci2_t = Eigen::Matrix<int, 2, 1>;
using veci3_t = Eigen::Matrix<int, 3, 1>;
using veci4_t = Eigen::Matrix<int, 4, 1>;
using veci5_t = Eigen::Matrix<int, 5, 1>;
using veci6_t = Eigen::Matrix<int, 6, 1>;
using vecix_t = Eigen::Matrix<int, dynamic_t, 1>;

using vec2_t = Eigen::Matrix<real_t, 2, 1>;
using vec3_t = Eigen::Matrix<real_t, 3, 1>;
using vec4_t = Eigen::Matrix<real_t, 4, 1>;
using vec5_t = Eigen::Matrix<real_t, 5, 1>;
using vec6_t = Eigen::Matrix<real_t, 6, 1>;
using vecx_t = Eigen::Matrix<real_t, dynamic_t, 1>;

using vec2s_t = std::vector<vec2_t, Eigen::aligned_allocator<vec2_t>>;
using vec3s_t = std::vector<vec3_t, Eigen::aligned_allocator<vec3_t>>;
using vec4s_t = std::vector<vec4_t, Eigen::aligned_allocator<vec4_t>>;
using vec5s_t = std::vector<vec5_t, Eigen::aligned_allocator<vec5_t>>;
using vec6s_t = std::vector<vec6_t, Eigen::aligned_allocator<vec6_t>>;
using vecxs_t = std::vector<vecx_t, Eigen::aligned_allocator<vec6_t>>;

using row_vector_t = Eigen::Matrix<real_t, 1, dynamic_t>;
using col_vector_t = Eigen::Matrix<real_t, dynamic_t, 1>;

template <int LENGTH, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using vec_t = Eigen::Matrix<real_t, LENGTH, 1, STRIDE_TYPE>;

// -- MATRIX -------------------------------------------------------------------

// clang-format off
using mat2_t = Eigen::Matrix<real_t, 2, 2>;
using mat3_t = Eigen::Matrix<real_t, 3, 3>;
using mat4_t = Eigen::Matrix<real_t, 4, 4>;
using matx_t = Eigen::Matrix<real_t, dynamic_t, dynamic_t>;
using matx_row_major_t = Eigen::Matrix<real_t, dynamic_t, dynamic_t, row_major_t>;
using mat34_t = Eigen::Matrix<real_t, 3, 4>;

using mat2s_t = std::vector<mat2_t, Eigen::aligned_allocator<mat2_t>>;
using mat3s_t = std::vector<mat3_t, Eigen::aligned_allocator<mat3_t>>;
using mat4s_t = std::vector<mat4_t, Eigen::aligned_allocator<mat4_t>>;
using matxs_t = std::vector<matx_t, Eigen::aligned_allocator<matx_t>>;
using matxs_row_major_t = std::vector<matx_row_major_t, Eigen::aligned_allocator<matx_row_major_t>>;

using mat_hash_t = std::unordered_map<long, std::unordered_map<long, real_t>>;
using mat_indicies_t = std::vector<std::pair<long int, long int>>;

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using mat_t = Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>;

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using map_mat_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>>;

template <int ROWS>
using map_vec_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, 1>>;
// clang-format on

// -- GEOMETRY -----------------------------------------------------------------

using quat_t = Eigen::Quaternion<real_t>;
using quats_t = std::vector<quat_t, Eigen::aligned_allocator<quat_t>>;
using angle_axis_t = Eigen::AngleAxis<real_t>;
using arrayx_t = Eigen::Array<real_t, dynamic_t, 1>;

using sp_mat_t = Eigen::SparseMatrix<real_t>;
using sp_vec_t = Eigen::SparseVector<real_t>;

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
    typedef std::numeric_limits<real_t> numeric_limits;
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
std::string arr2str(const real_t *arr, const size_t len, bool brackets = true);

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
matx_t pinv(const matx_t &A, const real_t tol = 1e-4);

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

// /**
//  * Recover covariance(i, l) (a specific value in the covariance matrix) from
//  * the upper triangular matrix `U` with precomputed diagonal vector
//  containing
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

/** Max */
real_t max(const std::vector<real_t> &x);

/** Mean */
real_t mean(const std::vector<real_t> &x);

/** Mean */
vec2_t mean(const vec2s_t &x);

/** Mean */
vec3_t mean(const vec3s_t &x);

/** Variance */
real_t var(const std::vector<real_t> &x);

/** Variance */
vec2_t var(const vec2s_t &vecs);

/** Variance */
vec3_t var(const vec3s_t &vecs);

/** Standard Deviation */
real_t stddev(const std::vector<real_t> &x);

/** Standard Deviation */
vec2_t stddev(const vec2s_t &x);

/** Standard Deviation */
vec3_t stddev(const vec3s_t &x);

/** Root Mean Squared Error.  */
real_t rmse(const std::vector<real_t> &x);

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
mat4_t tf_perturb_rot(const mat4_t &T, real_t step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_trans(const mat4_t &T, real_t step_size, const int i);

/**
 * Perturb the translation and rotation component.
 */
mat4_t tf_perturb(const mat4_t &T, const real_t dr, const real_t drot);

/**
 * Transform point `p` with transform `T`.
 */
vec3_t tf_point(const mat4_t &T, const vec3_t &p);

/**
 * Pose difference between pose0 and pose1.
 */
void pose_diff(const real_t pose0[7],
               const real_t pose1[7],
               real_t *dr,
               real_t *drot);

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
  std::map<std::string, std::vector<real_t>> record;

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
  const timestamp_t ts_start;
  const timestamp_t ts_end;
  const vec3s_t positions;
  const quats_t orientations;

  const real_t ts_s_start;
  const real_t ts_s_end;
  const real_t ts_s_gap;

  Spline3D pos_spline;
  Spline3D rvec_spline;

  ctraj_t(const timestamp_t &ts_start_,
          const timestamp_t &ts_end_,
          const vec3s_t &positions_,
          const quats_t &orientations_);
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
  real_t rate = 0.0;       // IMU rate [Hz]
  real_t tau_a = 3600.0;   // Reversion time constant for accel [s]
  real_t tau_g = 3600.0;   // Reversion time constant for gyro [s]
  real_t sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t g = 9.81;         // Gravity vector [ms-2]

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

// /*****************************************************************************
//  *                                CONTROL
//  *****************************************************************************/
//
// /**
//  * PID Controller
//  */
// struct pid_t {
//   real_t error_prev = 0.0;
//   real_t error_sum = 0.0;
//
//   real_t error_p = 0.0;
//   real_t error_i = 0.0;
//   real_t error_d = 0.0;
//
//   real_t k_p = 0.0;
//   real_t k_i = 0.0;
//   real_t k_d = 0.0;
//
//   pid_t();
//   pid_t(const real_t k_p, const real_t k_i, const real_t k_d);
//   ~pid_t();
// };
//
// /**
//  * `pid_t` to output stream
//  */
// std::ostream &operator<<(std::ostream &os, const pid_t &pid);
//
// /**
//  * Update controller
//  *
//  * @returns Controller command
//  */
// real_t pid_update(pid_t &p,
//                   const real_t setpoint,
//                   const real_t actual,
//                   const real_t dt);
//
// /**
//  * Update controller
//  *
//  * @returns Controller command
//  */
// real_t pid_update(pid_t &p, const real_t error, const real_t dt);
//
// /**
//  * Reset controller
//  */
// void pid_reset(pid_t &p);
//
// /**
//  * Carrot control
//  */
// struct carrot_ctrl_t {
//   vec3s_t waypoints;
//   vec3_t wp_start = vec3_t::Zero();
//   vec3_t wp_end = vec3_t::Zero();
//   size_t wp_index = 0;
//   real_t look_ahead_dist = 0.0;
//
//   carrot_ctrl_t();
//   ~carrot_ctrl_t();
// };
//
// /**
//  * Configure carrot control using a list of position `waypoints` (x, y, z),
//  and
//  * a `look_ahead` distance in [m].
//  *
//  * @returns 0 for success, -1 for failure
//  */
// int carrot_ctrl_configure(carrot_ctrl_t &cc,
//                           const vec3s_t &waypoints,
//                           const real_t look_ahead_dist);
//
// /**
//  * Calculate closest point along current trajectory using current position
//  * `pos`, and outputs the closest point in `result`.
//  *
//  * @returns A number to denote progress along the waypoint, if -1 then the
//  * position is before `wp_start`, 0 if the position is between `wp_start` and
//  * `wp_end`, and finally 1 if the position is after `wp_end`.
//  */
// int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
//                               const vec3_t &pos,
//                               vec3_t &result);
//
// /**
//  * Calculate carrot point using current position `pos`, and outputs the
//  carrot
//  * point in `result`.
//  *
//  * @returns A number to denote progress along the waypoint, if -1 then the
//  * position is before `wp_start`, 0 if the position is between `wp_start` and
//  * `wp_end`, and finally 1 if the position is after `wp_end`.
//  */
// int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
//                              const vec3_t &pos,
//                              vec3_t &result);
//
// /**
//  * Update carrot controller using current position `pos` and outputs the
//  carrot
//  * point in `result`.
//  *
//  * @returns 0 for success, 1 for all waypoints reached and -1 for failure
//  */
// int carrot_ctrl_update(carrot_ctrl_t &cc, const vec3_t &pos, vec3_t
// &carrot_pt);

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

  real_t time_span() const {
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

        const real_t alpha = (ts_start - ts_km1) / (ts_k - ts_km1);
        const vec3_t acc_km1 = (1.0 - alpha) * acc_km1 + alpha * acc_k;
        const vec3_t gyr_km1 = (1.0 - alpha) * gyr_km1 + alpha * gyr_k;
        ts_km1 = ts_start;

        buf.add(ts_km1, acc_km1, gyr_km1);

      } else if (ts_k > ts_end) {
        // Interpolate end
        ts_km1 = timestamps[k - 1];
        acc_km1 = accel[k - 1];
        gyr_km1 = gyro[k - 1];

        const real_t alpha = (ts_end - ts_km1) / (ts_k - ts_km1);
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
    // const real_t roll = rpy(0);
    // const real_t pitch = rpy(1);
    // const real_t yaw = 0.0;
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
}; // namespace yac

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

// /******************************************************************************
//  *                                 MODELS
//  *****************************************************************************/
//
// /**
//  * Create DH transform from link n to link n-1 (end to front)
//  *
//  * @param[in] theta
//  * @param[in] d
//  * @param[in] a
//  * @param[in] alpha
//  *
//  * @returns DH transform
//  */
// mat4_t dh_transform(const real_t theta,
//                     const real_t d,
//                     const real_t a,
//                     const real_t alpha);
//
// /**
//  * 2-DOF Gimbal Model
//  */
// struct gimbal_model_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//   // Parameter vector of transform from
//   // static camera to base-mechanism
//   vecx_t tau_s = zeros(6, 1);
//
//   // Parameter vector of transform from
//   // end-effector to dynamic camera
//   vecx_t tau_d = zeros(6, 1);
//
//   // First gibmal-joint
//   real_t Lambda1 = 0.0;
//   vec3_t w1 = zeros(3, 1);
//
//   // Second gibmal-joint
//   real_t Lambda2 = 0.0;
//   vec3_t w2 = zeros(3, 1);
//
//   real_t theta1_offset = 0.0;
//   real_t theta2_offset = 0.0;
//
//   gimbal_model_t();
//   gimbal_model_t(const vec6_t &tau_s,
//                  const vec6_t &tau_d,
//                  const real_t Lambda1,
//                  const vec3_t w1,
//                  const real_t Lambda2,
//                  const vec3_t w2,
//                  const real_t theta1_offset = 0.0,
//                  const real_t theta2_offset = 0.0);
//   virtual ~gimbal_model_t();
// };
//
// /**
//  * Set gimbal attitude
//  *
//  * @param[in,out] model Model
//  * @param[in] roll Roll [rads]
//  * @param[in] pitch Pitch [rads]
//  */
// void gimbal_model_set_attitude(gimbal_model_t &model,
//                                const real_t roll,
//                                const real_t pitch);
//
// /**
//  * Get gimbal joint angle
//  *
//  * @param[in] model Model
//  * @returns Gimbal joint angles
//  */
// vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model);
//
// /**
//  * Returns transform from static camera to base mechanism
//  *
//  * @param[in] model Model
//  * @returns Transform
//  */
// mat4_t gimbal_model_T_BS(const gimbal_model_t &model);
//
// /**
//  * Returns transform from base mechanism to end-effector
//  *
//  * @param[in] model Model
//  * @returns Transform
//  */
// mat4_t gimbal_model_T_EB(const gimbal_model_t &model);
//
// /**
//  * Returns transform from end-effector to dynamic camera
//  *
//  * @param[in] model Model
//  * @returns Transform
//  */
// mat4_t gimbal_model_T_DE(const gimbal_model_t &model);
//
// /**
//  * Returns transform from static to dynamic camera
//  *
//  * @param[in] model Model
//  * @returns Transform
//  */
// mat4_t gimbal_model_T_DS(const gimbal_model_t &model);
//
// /**
//  * Returns transform from static to dynamic camera
//  *
//  * @param[in,out] model Model
//  * @param[in] theta Gimbal roll and pitch [radians]
//  * @returns Transform from static to dynamic camera
//  */
// mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta);
//
// /**
//  * gimbal_model_t to output stream
//  */
// std::ostream &operator<<(std::ostream &os, const gimbal_model_t &gimbal);
//
// /**
//  * Calculate target angular velocity and time taken to traverse a desired
//  * circle * trajectory of radius r and velocity v
//  *
//  * @param[in] r Desired circle radius
//  * @param[in] v Desired trajectory velocity
//  * @param[in] w Target angular velocity
//  * @param[in] time Target time taken to complete circle trajectory
//  **/
// void circle_trajectory(const real_t r, const real_t v, real_t *w, real_t
// *time);
//
// /**
//  * Two wheel robot
//  */
// struct two_wheel_t {
//   vec3_t r_G = vec3_t::Zero();
//   vec3_t v_G = vec3_t::Zero();
//   vec3_t a_G = vec3_t::Zero();
//   vec3_t rpy_G = vec3_t::Zero();
//   vec3_t w_G = vec3_t::Zero();
//
//   real_t vx_desired = 0.0;
//   real_t yaw_desired = 0.0;
//
//   pid_t vx_controller{0.1, 0.0, 0.1};
//   pid_t yaw_controller{0.1, 0.0, 0.1};
//
//   vec3_t a_B = vec3_t::Zero();
//   vec3_t v_B = vec3_t::Zero();
//   vec3_t w_B = vec3_t::Zero();
//
//   two_wheel_t() {}
//
//   two_wheel_t(const vec3_t &r_G_, const vec3_t &v_G_, const vec3_t &rpy_G_)
//       : r_G{r_G_}, v_G{v_G_}, rpy_G{rpy_G_} {}
//
//   ~two_wheel_t() {}
//
//   void update(const real_t dt) {
//     const vec3_t r_G_prev = r_G;
//     const vec3_t v_G_prev = v_G;
//     const vec3_t rpy_G_prev = rpy_G;
//
//     r_G += euler321(rpy_G) * v_B * dt;
//     v_G = (r_G - r_G_prev) / dt;
//     a_G = (v_G - v_G_prev) / dt;
//
//     rpy_G += euler321(rpy_G) * w_B * dt;
//     w_G = rpy_G - rpy_G_prev;
//     a_B = euler123(rpy_G) * a_G;
//
//     // Wrap angles to +/- pi
//     for (int i = 0; i < 3; i++) {
//       rpy_G(i) = (rpy_G(i) > M_PI) ? rpy_G(i) - 2 * M_PI : rpy_G(i);
//       rpy_G(i) = (rpy_G(i) < -M_PI) ? rpy_G(i) + 2 * M_PI : rpy_G(i);
//     }
//   }
// };
//
// /**
//  * MAV model
//  */
// struct mav_model_t {
//   vec3_t attitude{0.0, 0.0, 0.0};         ///< Attitude in global frame
//   vec3_t angular_velocity{0.0, 0.0, 0.0}; ///< Angular velocity in global
//   frame vec3_t position{0.0, 0.0, 0.0};         ///< Position in global
//   frame vec3_t linear_velocity{0.0, 0.0, 0.0};  ///< Linear velocity in
//   global frame
//
//   real_t Ix = 0.0963; ///< Moment of inertia in x-axis
//   real_t Iy = 0.0963; ///< Moment of inertia in y-axis
//   real_t Iz = 0.1927; ///< Moment of inertia in z-axis
//
//   real_t kr = 0.1; ///< Rotation drag constant
//   real_t kt = 0.2; ///< Translation drag constant
//
//   real_t l = 0.9; ///< MAV arm length
//   real_t d = 1.0; ///< drag constant
//
//   real_t m = 1.0;  ///< Mass
//   real_t g = 9.81; ///< Gravity
// };
//
// /**
//  * Update
//  *
//  * @param[in,out] qm Model
//  * @param[in] motor_inputs Motor inputs (m1, m2, m3, m4)
//  * @param[in] dt Time difference (s)
//  * @returns 0 for success, -1 for failure
//  */
// int mav_model_update(mav_model_t &qm,
//                      const vec4_t &motor_inputs,
//                      const real_t dt);

/******************************************************************************
 *                               PARAMETERS
 *****************************************************************************/

struct imu_params_t {
  real_t rate = 0.0;       // IMU rate [Hz]
  real_t a_max = 160.0;    // Max accelerometer measurement [m/s^2]
  real_t g_max = 10.0;     // Max gyroscope measurement [rad/s]
  real_t sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t sigma_bg = 0.0;   // Gyro bias prior [rad/s]
  real_t sigma_ba = 0.0;   // Accel bias prior [m/s^2]
  real_t g = 9.81;         // Gravity vector [ms^-2]
};

} //  namespace yac
#endif // YAC_CORE_HPP
