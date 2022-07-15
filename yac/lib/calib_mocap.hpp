#ifndef YAC_CALIB_MOCAP_HPP
#define YAC_CALIB_MOCAP_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_camera.hpp"

namespace yac {

struct calib_mocap_t {
  // Settings
  bool fix_intrinsics = false;
  bool fix_mocap_poses = false;
  bool fix_fiducial_pose = false;
  bool imshow = true;
  bool show_progress = true;
  int max_iter = 50;

  // Paths
  std::string config_file;
  std::string data_path;

  // Calibration data
  calib_target_t calib_target;
  aprilgrids_t grids;

  // Calibration parameters
  std::shared_ptr<camera_geometry_t> camera_geometry;
  camera_params_t camera;
  pose_t fiducial_pose;
  std::map<timestamp_t, pose_t> mocap_poses;
  extrinsics_t mocap_camera_extrinsics;

  // Calibration problem
  std::unique_ptr<ceres::Problem> problem;
  PoseLocalParameterization pose_plus;
  std::vector<std::shared_ptr<mocap_residual_t>> residuals;

  calib_mocap_t(const std::string &config_file, const std::string &data_path);

  int get_num_views() const;
  std::vector<real_t> get_reproj_errors() const;
  mat4_t get_fiducial_pose() const;
  mat4_t get_mocap_camera_extrinsics() const;
  vecx_t get_camera_params() const;

  aprilgrids_t _preprocess(const calib_target_t &calib_target,
                           const std::string &cam_path,
                           const std::string &grid_path);
  void _add_view(const aprilgrid_t &grid);

  int solve();
  void print_settings(FILE *out) const;
  void print_calib_target(FILE *out) const;
  void print_metrics(FILE *out) const;
  void print_fiducial_pose(FILE *out) const;
  void print_mocap_camera_extrinsics(FILE *out) const;
  void print_camera_params(FILE *out) const;
  void print_mocap_poses(FILE *out) const;
  void show_results() const;
  int save_results(const std::string &save_path) const;
};

} //  namespace yac
#endif // YAC_CALIB_MOCAP_HPP
