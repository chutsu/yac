#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

/** Camera Data **/
using camera_data_t = std::map<timestamp_t, std::map<int, aprilgrid_t>>;

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses,
                     timestamp_t ts_offset = 0);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
