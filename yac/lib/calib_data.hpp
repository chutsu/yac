#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include "util/util.hpp"

namespace yac {

// CALIB UTILS /////////////////////////////////////////////////////////////////

/** Camera Data **/
using camera_data_t = std::map<timestamp_t, std::map<int, aprilgrid_t>>;

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses,
                     timestamp_t ts_offset = 0);

void load_imu_data(const std::string &csv_path,
                   timestamps_t &timestamps,
                   vec3s_t &a_B,
                   vec3s_t &w_B);

// CALIB TARGET ////////////////////////////////////////////////////////////////

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
  void print() const;
};

// CALIBRATION DETECTION //////////////////////////////////////////////////////

/***
 * Preprocess / Load AprilGrid data
 *
 * @param[in] calib_target Calibration Target Configuration
 * @param[in] cam_paths Paths to camera images
 * @param[in] grids_path Path to save detected AprilGrids
 * @param[in] imshow Show detections
 *
 * @returns AprilGrid data observed by each camera
 */
std::map<int, aprilgrids_t>
calib_data_preprocess(const calib_target_t &calib_target,
                      const std::map<int, std::string> cam_paths,
                      const std::string &grids_path,
                      const bool imshow = false,
                      const bool format_v2 = false);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
