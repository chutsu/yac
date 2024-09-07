#include "CalibTarget.hpp"

namespace yac {

CalibTarget::CalibTarget(const timestamp_t ts,
                         const int num_rows,
                         const int num_cols)
    : ts_{ts}, num_rows_{num_rows}, num_cols_{num_cols} {}

timestamp_t CalibTarget::getTimestamp() const { return ts_; }

int CalibTarget::getNumRows() const { return num_rows_; }

int CalibTarget::getNumCols() const { return num_cols_; }

// int CalibTarget::estimate(const CameraModel *cam,
//                         const int cam_res[2],
//                         const vecx_t &cam_params,
//                         mat4_t &T_CF) const {
//
//
//   // Check if we actually have data to work with
//   if (nb_detections == 0) {
//     return -1;
//   }
//
//   // Create object points (counter-clockwise, from bottom left)
//   vec2s_t img_pts;
//   vec3s_t obj_pts;
//   for (int i = 0; i < (tag_rows * tag_cols * 4); i++) {
//     if (data(i, 0) > 0) {
//       img_pts.emplace_back(data(i, 1), data(i, 2));
//       obj_pts.emplace_back(data(i, 3), data(i, 4), data(i, 5));
//     }
//   }
//
//   return solvepnp(cam, cam_res, cam_params, img_pts, obj_pts, T_CF);
// }

} // namespace yac
