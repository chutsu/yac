#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include <string>
#include <thread>

#include <ceres/ceres.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

/* Calibration target. */
struct calib_target_t {
  std::string target_type;
  int tag_rows = 0;
  int tag_cols = 0;
  real_t tag_size = 0.0;
  real_t tag_spacing = 0.0;

  calib_target_t() {}
  calib_target_t(const std::string target_type_,
                 const int tag_rows_,
                 const int tag_cols_,
                 const real_t tag_size_,
                 const real_t tag_spacing_)
    : target_type{target_type_},
      tag_rows{tag_rows_},
      tag_cols{tag_cols_},
      tag_size{tag_size_},
      tag_spacing{tag_spacing_} {}
  ~calib_target_t() {}
};

/**
 * Load calibration target.
 * @returns 0 or -1 for success or failure
 */
int calib_target_load(calib_target_t &ct,
                      const std::string &target_file,
                      const std::string &prefix = "");

/* Calibration config */
struct calib_data_t {
  bool ok = false;
  std::string node_name;
  config_t config;

  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;
  std::map<int, std::vector<double>> vision_errors;

  // Params
  calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, std::pair<int, aprilgrid_t>> cam_grids;

  int nb_cams = 0;
  int nb_imus = 0;
  imu_params_t imu_params;
  std::map<int, camera_geometry_t *> cam_geoms;
  std::map<int, camera_params_t> cam_params;
  mat2_t covar = I(2);

  id_t param_counter = 0;
  std::map<timestamp_t, pose_t> poses;
  std::map<int, extrinsics_t> cam_exts;
  std::map<int, extrinsics_t> imu_exts;

  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  calib_data_t();
  calib_data_t(const std::string &config_file);
  ~calib_data_t();

  void add_calib_target(const calib_target_t &target_);
  void add_imu(const imu_params_t &params);
  void add_camera(const camera_params_t &params);
  void add_camera_extrinsics(const int cam_idx, const mat4_t &ext=I(4));
  void add_imu_extrinsics(const int imu_idx, const mat4_t &ext=I(4));
  void add_grids(const int cam_idx, const aprilgrids_t &grids);
  pose_t &add_pose(const timestamp_t &ts, const mat4_t &T);
  // void check_data();
  static int config_nb_cameras(const std::string &config_file);
  static int config_nb_imus(const std::string &config_file);
  static void load_calib_target_params(const config_t &config,
                                       calib_target_t &params,
                                       bool verbose=false);
  camera_params_t load_cam_params(const config_t &config,
                                  const int cam_idx,
                                  bool verbose=false);
  static imu_params_t load_imu_params(const config_t &config,
                                      const int index,
                                      bool verbose=false);
  extrinsics_t load_cam_exts(const config_t &config,
                             const int cam_idx,
                             bool verbose=false);

  int nb_grids(const timestamp_t &ts);
  std::vector<std::pair<int, aprilgrid_t>> get_grids(const timestamp_t &ts);

  void show_results();
};

/**
 * Preprocess camera image data and output AprilGrid detection data as
 * csv. Once the data is preprocessed the data is saved to `output_dir`.
 *
 * @returns 0 for Success, -1 for failure, and 1 where the output directory
 * contains data.
 */
int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const std::string &output_dir,
                           const bool show_progress = true);

/**
 * Load preprocess-ed camera calibration data located in `data_dir` where the
 * data will be loaded in `aprilgrids`. By default, this function will only
 * return aprilgrids that are detected. To return all calibration data
 * including camera frames where aprilgrids were not detected, change
 * `detected_only` to false.
 *
 * @returns 0 or -1 for success or failure
 */
int load_camera_calib_data(const std::string &data_dir,
                           aprilgrids_t &aprilgrids,
                           bool detected_only = true);

/**
 * Extract and only keep common aprilgrid corners between `grids0` and `grids1`.
 */
void extract_common_calib_data(aprilgrids_t &grids0, aprilgrids_t &grids1);

/**
 * Load preprocessed stereo calibration data, where `cam0_data_dir` and
 * `cam1_data_dir` are preprocessed calibration data observed from cam0 and
 * cam1. The preprocessed calibration data will be loaded into
 * `cam0_aprilgrids` and `cam1_aprilgrids` respectively, where the data
 * contains AprilGrids observed by both cameras at the same timestamp.**
 *
 * This function assumes:
 *
 * - Stereo camera images are synchronized
 * - Images that are synchronized are expected to have the **same exact
 *   timestamp**
 *
 * @returns 0 or -1 for success or failure
 */
int load_stereo_calib_data(const std::string &cam0_data_dir,
                           const std::string &cam1_data_dir,
                           aprilgrids_t &cam0_aprilgrids,
                           aprilgrids_t &cam1_aprilgrids);

/**
 * Load preprocessed multi-camera calibration data, where each data path in
 * `data_dirs` are the preprocessed calibration data observed by each camera,
 * and the preprocessed calibration data will be loaded into `calib_data` where
 * the key is the camera index and the value is the detected aprilgrids. The
 * data in `calib_data` contains AprilGrids observed by all cameras at the same
 * timestamp.
 *
 * This function assumes:
 *
 * - Camera images are synchronized
 * - Images that are synchronized are expected to have the **same exact
 *   timestamp**
 *
 * @returns 0 or -1 for success or failure
 */
// int load_multicam_calib_data(const int nb_cams,
//                              const std::vector<std::string> &data_dirs,
//                              std::map<int, aprilgrids_t> &calib_data);

/**
 * Draw measured and projected pixel points.
 * @returns Image
 */
cv::Mat draw_calib_validation(const cv::Mat &image,
                              const vec2s_t &measured,
                              const vec2s_t &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color);

/**
 * `calib_target_t` to output stream.
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
