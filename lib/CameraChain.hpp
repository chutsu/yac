#pragma once
#include <deque>
#include <vector>
#include <unordered_map>

#include "Core.hpp"

namespace yac {

/** Camera Chain Query Tool */
class CameraChain {
private:
  std::unordered_map<int, std::vector<int>> adj_list;  // CameraChain adjacency list
  std::unordered_map<int, std::unordered_map<int, mat4_t>> exts; // Extrinsics

  void getCalibTargets(const timestamp_t &ts,
                       const camera_data_t &cam_data,
                       std::vector<int> &cam_indicies,
                       std::vector<CalibTarget> &calib_targets) const;

public:
  /** Constructor */
  CameraChain(const CameraData &cam_data,
              const std::unordered_map<int, CameraGeometry> &cam_geoms);

  // /** Insert camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  // void insert(const int cam_i, const int cam_j, const mat4_t &T_CiCj);
  //
  // /** Check if camera extrinsics `T_CiCj` between `cam_i` and `cam_j` exists */
  // bool contains(const int cam_i, const int cam_j) const;
  //
  // /** Find camera extrinsics `T_CiCj` between `cam_i` and `cam_j` */
  // int find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const;
  //
  // /** Clear camchain */
  // void clear();
};

} // namespace yac
