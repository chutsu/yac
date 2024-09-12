#pragma once
#include <deque>
#include <vector>
#include <unordered_map>

#include "Core.hpp"
#include "CalibData.hpp"

namespace yac {

/** Camera Chain Query Tool */
class CameraChain {
private:
  std::map<int, std::vector<int>> adjlist_;             // Adjacency list
  std::map<int, std::unordered_map<int, mat4_t>> exts_; // Extrinsics

  // void getCalibTargets(const timestamp_t &ts,
  //                      const camera_data_t &cam_data,
  //                      std::vector<int> &cam_indicies,
  //                      std::vector<CalibTarget> &calib_targets) const;

public:
  /** Constructor */
  CameraChain(const CameraData &cam_data,
              const std::unordered_map<int, CameraGeometry> &cam_geoms);

  /** Insert camera extrinsics `T_ij` between `i` and `j` */
  void insert(const int i, const int j, const mat4_t &T_ij);

  /** Check if camera extrinsics `T_ij` between `i` and `j` exists */
  bool contains(const int i, const int j) const;

  /** Find camera extrinsics `T_ij` between `i` and `j` */
  int find(const int i, const int j, mat4_t &T_ij) const;

  /** Clear camchain */
  void clear();
};

} // namespace yac
