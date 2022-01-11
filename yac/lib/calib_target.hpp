#ifndef YAC_CALIB_TARGET_HPP
#define YAC_CALIB_TARGET_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

/** Calibration target */
struct calib_target_t {
  std::string target_type = "aprilgrid";
  int tag_rows = 6;
  int tag_cols = 6;
  real_t tag_size = 0.088;
  real_t tag_spacing = 0.3;

  calib_target_t() = default;
  calib_target_t(const std::string target_type_,
                 const int tag_rows_,
                 const int tag_cols_,
                 const real_t tag_size_,
                 const real_t tag_spacing_);
  ~calib_target_t() = default;

  /**
   * Load calibration target.
   * @returns 0 or -1 for success or failure
   */
  int load(const std::string &target_file, const std::string &prefix = "");

  /** Print calibration target */
  void print();
};

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
