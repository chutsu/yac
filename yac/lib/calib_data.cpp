#include "calib_data.hpp"

namespace yac {

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

} // namespace yac
