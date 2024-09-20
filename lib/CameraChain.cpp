#include "CameraChain.hpp"
#include "SolvePnp.hpp"

namespace yac {

CameraChain::CameraChain(
    const std::map<int, CameraGeometryPtr> &camera_geometries,
    const std::map<int, CameraData> &camera_data) {
  // Get camera timestamps
  std::map<timestamp_t, std::vector<int>> camera_timestamps;
  for (const auto &[camera_index, camera_measurements] : camera_data) {
    for (const auto &[ts, calib_target] : camera_measurements) {
      camera_timestamps[ts].push_back(camera_index);
    }
  }

  // Loop through timestamps and get co-observations
  for (auto &[ts, camera_indicies] : camera_timestamps) {
    // Pre-check
    if (camera_indicies.size() < 2) {
      continue;
    }

    // Obtain co-observations
    std::map<int, CalibTargetPtr> observations;
    for (auto camera_index : camera_indicies) {
      observations[camera_index] = camera_data.at(camera_index).at(ts);
    }

    // Estimate relative pose T_CiF
    const int index_i = camera_indicies[0];
    const auto &target_i = observations[index_i];
    const auto camera_i = camera_geometries.at(index_i);
    mat4_t T_CiF;
    if (SolvePnp::estimate(camera_i, target_i, T_CiF) != 0) {
      continue;
    }

    // Estimate relative pose T_CjF
    for (size_t j = 1; j < camera_indicies.size(); j++) {
      const int index_j = camera_indicies[j];
      const auto &target_j = observations[index_j];
      const auto camera_j = camera_geometries.at(index_j);

      // Solvepnp T_CjF
      mat4_t T_CjF;
      if (SolvePnp::estimate(camera_j, target_j, T_CjF) != 0) {
        continue;
      }

      // Insert into adjacency list
      insert(index_i, index_j, T_CiF * T_CjF.inverse());
    }
  }
}

void CameraChain::insert(const int i, const int j, const mat4_t &T_ij) {
  adjlist_[i][j].push_back(T_ij);
  adjlist_[j][i].push_back(T_ij.inverse());
}

int CameraChain::find(const int i, const int j, mat4_t &T_CiCj) const {
  // Average extrinsic
  auto average_extrinsic = [&](const int i, const int j) {
    vec3s_t positions;
    std::vector<quat_t> rotations;
    for (const auto &extrinsic : adjlist_.at(i).at(j)) {
      positions.push_back(tf_trans(extrinsic));
      rotations.push_back(tf_quat(extrinsic));
    }

    vec3_t pos = mean(positions);
    quat_t rot = quat_average(rotations);
    return tf(rot, pos);
  };

  // Straight-forward case
  if (i == j) {
    T_CiCj = I(4);
    return 0;
  }

  // Check if we have even inserted the cameras before
  if (adjlist_.count(i) == 0 || adjlist_.count(j) == 0) {
    return -1;
  }

  // Check if we already have the extrinsics pair
  if (adjlist_.count(i) && adjlist_.at(i).count(j)) {
    T_CiCj = average_extrinsic(i, j);
    return 0;
  }

  // Iterative BFS - To get path from camera i to j
  bool found_target = false;
  std::deque<int> queue;
  std::map<int, bool> visited;
  std::map<int, int> path_map;

  queue.push_back(i);
  while (!queue.empty()) {
    const auto parent = queue.front();
    queue.pop_front();
    visited[parent] = true;

    for (const auto &[child, _] : adjlist_.at(parent)) {
      if (visited[child]) {
        continue;
      }

      queue.push_back(child);
      path_map[child] = parent;

      if (child == j) {
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
  int child = j;
  while (path_map.count(child)) {
    const int parent = path_map[child];
    T_CiCj = T_CjCi * average_extrinsic(child, parent);
    child = parent;
  }
  T_CiCj = T_CjCi.inverse();

  return 0;
}

} // namespace yac
