#include "CameraChain.hpp"

namespace yac {

CameraChain::CameraChain(
    const CameraData &cam_data,
    const std::unordered_map<int, CameraGeometry> &cam_geoms) {
  // for (const auto &kv : cam_data) {
  //   const auto &ts = kv.first;
  //
  //   // Get all detected AprilGrid at ts
  //   std::vector<int> cam_indicies;
  //   std::vector<aprilgrid_t> grids;
  //   _get_aprilgrids(ts, cam_data, cam_indicies, grids);
  //
  //   // Check we have more than 1
  //   if (cam_indicies.size() < 2) {
  //     continue;
  //   }
  //
  //   // Add to camchain if we haven't already
  //   const int cam_i = cam_indicies[0];
  //   const auto params_i = cam_params.at(cam_i);
  //   const auto res_i = params_i->resolution;
  //   const auto geom_i = cam_geoms.at(cam_i).get();
  //   const auto &grid_i = grids[0];
  //
  //   for (size_t i = 1; i < cam_indicies.size(); i++) {
  //     const auto cam_j = cam_indicies[i];
  //     const auto params_j = cam_params.at(cam_j);
  //     const auto res_j = params_j->resolution;
  //     const auto geom_j = cam_geoms.at(cam_j).get();
  //     const auto &grid_j = grids[i];
  //
  //     if (contains(cam_i, cam_j) == false) {
  //       mat4_t T_CiF;
  //       if (grid_i.estimate(geom_i, res_i, params_i->param, T_CiF) != 0) {
  //         FATAL("Failed to estimate relative pose!");
  //       }
  //
  //       mat4_t T_CjF;
  //       if (grid_j.estimate(geom_j, res_j, params_j->param, T_CjF) != 0) {
  //         FATAL("Failed to estimate relative pose!");
  //       }
  //
  //       const mat4_t T_CiCj = T_CiF * tf_inv(T_CjF);
  //       insert(cam_i, cam_j, T_CiCj);
  //     }
  //   }
  // }
}

// void CameraChain::getCalibTargets(const timestamp_t &ts,
//                                   const camera_data_t &cam_data,
//                                   std::vector<int> &cam_indicies,
//                                   std::vector<aprilgrid_t> &grids) const {
//   // Get all detected AprilGrid at ts
//   for (const auto &[cam_idx, grid] : cam_data.at(ts)) {
//     // Check aprilgrid is detected
//     if (grid.detected == false && grid.nb_detections < 12) {
//       continue;
//     }
//
//     // Update
//     cam_indicies.push_back(cam_idx);
//     grids.push_back(grid);
//   }
// }

void CameraChain::insert(const int cam_i,
                         const int cam_j,
                         const mat4_t &T_CiCj) {
  if (contains(cam_i, cam_j)) {
    return;
  }

  adjlist_[cam_i].push_back(cam_j);
  adjlist_[cam_j].push_back(cam_i);
  exts_[cam_i][cam_j] = T_CiCj;
  exts_[cam_j][cam_i] = tf_inv(T_CiCj);
}

bool CameraChain::contains(const int cam_i, const int cam_j) const {
  return (exts_.count(cam_i) && exts_.at(cam_i).count(cam_j));
}

int CameraChain::find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const {
  // Straight-forward case
  if (cam_i == cam_j) {
    T_CiCj = I(4);
    return 0;
  }

  // Check if we have even inserted the cameras before
  if (exts_.count(cam_i) == 0 || exts_.count(cam_j) == 0) {
    return -1;
  }

  // Check if we already have the extrinsics pair
  if (contains(cam_i, cam_j)) {
    T_CiCj = exts_.at(cam_i).at(cam_j);
    return 0;
  }

  // Iterative BFS - To get path from cam_i to cam_j
  bool found_target = false;
  std::deque<int> queue;
  std::map<int, bool> visited;
  std::map<int, int> path_map;

  queue.push_back(cam_i);
  while (!queue.empty()) {
    const auto parent = queue.front();
    queue.pop_front();
    visited.at(parent) = true;

    for (const int &child : adjlist_.at(parent)) {
      if (visited.at(child)) {
        continue;
      }

      queue.push_back(child);
      path_map[child] = parent;

      if (child == cam_j) {
        found_target = true;
        break;
      }
    }
  }

  // Check if we've found the target
  if (found_target == false) {
    return -2;
  }

  // Traverse the path backwards and chain the transforms
  mat4_t T_CjCi = I(4);
  int child = cam_j;
  while (path_map.count(child)) {
    const int parent = path_map[child];
    T_CjCi = T_CjCi * exts_.at(child).at(parent);
    child = parent;
  }
  T_CiCj = tf_inv(T_CjCi);

  return 0;
}

void CameraChain::clear() {
  adjlist_.clear();
  exts_.clear();
}

} // namespace yac
