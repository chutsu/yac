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
  std::map<int, std::map<int, std::vector<mat4_t>>> adjlist_;

public:
  CameraChain() = default;
  CameraChain(const std::map<int, CameraGeometryPtr> &camera_geometries,
              const std::map<int, CameraData> &camera_data);
  virtual ~CameraChain() = default;

  /** Insert link beteen camera i and j */
  void insert(const int i, const int j, const mat4_t &T_ij);

  /** Find camera extrinsics `T_ij` between `i` and `j` */
  int find(const int i, const int j, mat4_t &T_ij) const;
};

} // namespace yac
