#ifndef YAC_DATA_HPP
#define YAC_DATA_HPP

#include "core.hpp"
#include "fs.hpp"

namespace yac {

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
void extend(std::vector<T> &x, const std::vector<T> &add) {
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
template <typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T> vec(first, last);
  return vec;
}

/**
 * Slice `std::vector`.
 */
template <typename T1, typename T2>
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
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  iterator begin() { return vector.begin(); }
  iterator end() { return vector.end(); }
  const_iterator begin() const { return vector.begin(); }
  const_iterator end() const { return vector.end(); }
  const T &at(const size_t i) const { return vector.at(i); }
  const T &front() const { return vector.front(); }
  const T &back() const { return vector.back(); }
  void insert(const T &item) {
    if (set.insert(item).second)
      vector.push_back(item);
  }
  size_t count(const T &item) const { return set.count(item); }
  bool empty() const { return set.empty(); }
  size_t size() const { return set.size(); }
  void clear() {
    vector.clear();
    set.clear();
  }

private:
  std::vector<T> vector;
  std::set<T> set;
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
 * Save poses to csv file in `path`.
 */
void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const mat4s_t &poses);

/**
 * Save extrinsics to csv file in `path`.
 */
void save_extrinsics(const std::string &path, const mat4_t &ext);

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
                   const bool print = false);

} // namespace yac
#endif // YAC_DATA_HPP
