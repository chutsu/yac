#include "calib_data.hpp"

namespace yac {

// CALIB UTILS /////////////////////////////////////////////////////////////////

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses,
                     timestamp_t ts_offset) {
  // Make sure AprilGrids are between body poses else we can't lerp poses
  timestamps_t grid_timestamps;
  for (const auto &grid : grids) {
    if (grid.timestamp > body_timestamps.front() &&
        grid.timestamp < body_timestamps.back()) {
      lerped_grids.push_back(grid);
      grid_timestamps.push_back(grid.timestamp);
    }
  }

  // Lerp body poses using AprilGrid timestamps
  assert(body_poses.size() == body_timestamps.size());
  assert(body_timestamps.front() < grid_timestamps.front());
  timestamp_t t0 = 0;
  mat4_t pose0 = I(4);
  timestamp_t t1 = 0;
  mat4_t pose1 = I(4);

  size_t grid_idx = 0;
  for (size_t i = 0; i < body_timestamps.size(); i++) {
    // Make sure we're not going out of bounds
    if (grid_idx > (grid_timestamps.size() - 1)) {
      break;
    }

    // Get time now and desired lerp time
    const auto t_now = body_timestamps[i] + ts_offset;
    const auto t_lerp = grid_timestamps[grid_idx];

    // Update t0
    if (t_now < t_lerp) {
      t0 = t_now;
      pose0 = body_poses[i];
    }

    // Update t1
    if (t_now > t_lerp) {
      // Lerp
      t1 = t_now;
      pose1 = body_poses[i];
      const auto pose = lerp_pose(t0, pose0, t1, pose1, t_lerp);
      lerped_poses.push_back(pose);
      grid_idx++;

      // Reset
      t0 = t_now;
      pose0 = body_poses[i];
      t1 = 0;
      pose1 = I(4);
    }
  }
}

// CALIB TARGET ////////////////////////////////////////////////////////////////

calib_target_t::calib_target_t(const std::string target_type_,
                               const int tag_rows_,
                               const int tag_cols_,
                               const real_t tag_size_,
                               const real_t tag_spacing_)
    : target_type{target_type_}, tag_rows{tag_rows_}, tag_cols{tag_cols_},
      tag_size{tag_size_}, tag_spacing{tag_spacing_} {}

int calib_target_t::load(const std::string &target_file,
                         const std::string &prefix) {
  config_t config{target_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  const auto parent = (prefix == "") ? "" : prefix + ".";
  parse(config, parent + "target_type", target_type);
  parse(config, parent + "tag_rows", tag_rows);
  parse(config, parent + "tag_cols", tag_cols);
  parse(config, parent + "tag_size", tag_size);
  parse(config, parent + "tag_spacing", tag_spacing);

  return 0;
}

void calib_target_t::print() const {
  printf("target_type: %s\n", target_type.c_str());
  printf("tag_rows: %d\n", tag_rows);
  printf("tag_cols: %d\n", tag_cols);
  printf("tag_size: %f\n", tag_size);
  printf("tag_spacing: %f\n", tag_spacing);
}

} // namespace yac
