#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include <string>
#include <thread>

#include <ceres/ceres.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "core.hpp"
#include "aprilgrid.hpp"
#include "ceres_utils.hpp"
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
  std::map<int, aprilgrids_t> cam_grids;

  int nb_cams = 0;
  int nb_imus = 0;
  imu_params_t imu_params;
  std::map<int, camera_params_t> cam_params;
  mat2_t covar = I(2);

  id_t param_counter = 0;
  std::map<timestamp_t, pose_t> poses;
  std::map<int, extrinsics_t> cam_exts;
  std::map<int, extrinsics_t> imu_exts;

  calib_data_t() {
    // Problem
		prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
		prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
		prob_options.enable_fast_removal = true;
		problem = new ceres::Problem(prob_options);
  }

  calib_data_t(const std::string &config_file)
    : config{config_file},
      nb_cams{config_nb_cameras(config_file)},
      nb_imus{config_nb_imus(config_file)} {
    // Problem
		prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
		prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
		prob_options.enable_fast_removal = true;
		problem = new ceres::Problem(prob_options);

    // Calibration target
    if (calib_target_load(target, config_file, "calib_target") != 0) {
      FATAL("Failed to load calib target [%s]!", config_file.c_str());
    }

    // Load Imu
    if (nb_imus > 1) {
      FATAL("YAC currently does not support more than 1 IMU!");
    } else if (nb_imus == 1) {
      imu_params = load_imu_params(config, 0, true);
      imu_exts[0] = extrinsics_t{param_counter++};
    }

    // Load cameras
    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      cam_params[cam_idx] = load_cam_params(config, cam_idx, true);
      cam_exts[cam_idx] = load_cam_exts(config, cam_idx, true);
      vision_errors[cam_idx] = std::vector<double>();
    }
  }

  ~calib_data_t() {
	  if (problem) {
      delete problem;
      problem = nullptr;
    }
    if (loss) {
      delete loss;
      loss = nullptr;
    }
  }

  void add_calib_target(const calib_target_t &target_) {
    target = target_;
  }

  void add_imu(const imu_params_t &params) {
    imu_params = params;
  }

  void add_camera(const camera_params_t &params) {
    cam_params[params.cam_index] = params;
    nb_cams += 1;
    vision_errors[params.cam_index] = std::vector<double>();
  }

  void add_camera_extrinsics(const int cam_idx, const extrinsics_t &params) {
    cam_exts[cam_idx] = params;
  }

  void add_camera_extrinsics(const int cam_idx, const mat4_t &ext=I(4)) {
    cam_exts[cam_idx] = extrinsics_t{param_counter++, ext};
  }

  void add_imu_extrinsics(const int imu_idx, const extrinsics_t &params) {
    imu_exts[imu_idx] = params;
    nb_imus += 1;
  }

  void add_imu_extrinsics(const int imu_idx, const mat4_t &ext=I(4)) {
    imu_exts[imu_idx] = extrinsics_t{param_counter++, ext};
    nb_imus += 1;
  }

  void add_grids(const int cam_idx, const aprilgrids_t &grids) {
    cam_grids[cam_idx] = grids;
  }

  void add_pose(const timestamp_t &ts, const pose_t &pose) {
    poses[ts] = pose;
  }

  void preprocess_data() {
    const int rows = target.tag_rows;
    const int cols = target.tag_cols;
    const double size = target.tag_size;
    const double spacing = target.tag_spacing;

	  // Track aprilgrids for each camera and all timestamps
    std::set<timestamp_t> timestamps;
    std::map<int, std::map<timestamp_t, aprilgrid_t>> grid_tracker;
    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      for (auto &grid : cam_grids[cam_idx]) {
        grid_tracker[cam_idx][grid.timestamp] = grid;
        timestamps.insert(grid.timestamp);
      }
    }

    // Preprocess vision data
    std::map<int, aprilgrids_t> prep_grids;
    for (const auto &ts : timestamps) {
      for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
        if (grid_tracker[cam_idx].count(ts) == 0) {
          prep_grids[cam_idx].emplace_back(ts, rows, cols, size, spacing);
        } else {
          prep_grids[cam_idx].push_back(grid_tracker[cam_idx][ts]);
        }
      }
    }

    cam_grids.clear();
    cam_grids = prep_grids;
  }

  void check_data() {
    bool ok = true;

    auto nb_grids = cam_grids[0].size();
    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      if (nb_grids != cam_grids[cam_idx].size()) {
        FATAL("grids[%d].size() != nb_grids!", cam_idx);
        ok = false;
      }
    }

    std::vector<timestamp_t> timestamps;
    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      for (auto &grid : cam_grids[cam_idx]) {
        timestamps.push_back(grid.timestamp);
      }
    }

    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      for (size_t i = 0; i < cam_grids[cam_idx].size(); i++) {
        auto &grid = cam_grids[cam_idx][i];
        if (grid.timestamp != timestamps[i]) {
          LOG_ERROR("grid.timestamp != timestamps[i]!");
          ok = false;
          break;
        }

        if (grid.tag_rows != target.tag_rows) {
          LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
          LOG_ERROR("grid.tag_rows != target.tag_rows!");
          ok = false;
          break;
        } else if (grid.tag_cols != target.tag_cols) {
          LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
          LOG_ERROR("grid.tag_cols != target.tag_cols!");
          ok = false;
          break;
        } else if (grid.tag_size != target.tag_size) {
          LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
          LOG_ERROR("grid.tag_size != target.tag_size!");
          ok = false;
          break;
        } else if (grid.tag_spacing != target.tag_spacing) {
          LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
          LOG_ERROR("grid.tag_spacing != target.tag_spacing!");
          ok = false;
          break;
        }
      }
    }

    if (ok == false) {
      FATAL("Bad data!");
    }
  }

  static int config_nb_cameras(const std::string &config_file) {
    config_t config{config_file};

    const int max_cameras = 100;
    for (int i = 0; i < max_cameras; i++) {
      const std::string key = "cam" + std::to_string(i);
      if (yaml_has_key(config, key) == 0) {
        return i;
      }
    }

    return 0;
  }

  static int config_nb_imus(const std::string &config_file) {
    config_t config{config_file};

    const int max_imus = 100;
    for (int i = 0; i < max_imus; i++) {
      const std::string key = "imu" + std::to_string(i);
      if (yaml_has_key(config, key) == 0) {
        return i;
      }
    }

    return 0;
  }

  static void load_calib_target_params(const config_t &config,
                                       calib_target_t &params,
                                       bool verbose=false) {
    parse(config, "calib_target.target_type", params.target_type);
    parse(config, "calib_target.tag_rows", params.tag_rows);
    parse(config, "calib_target.tag_cols", params.tag_cols);
    parse(config, "calib_target.tag_size", params.tag_size);
    parse(config, "calib_target.tag_spacing", params.tag_spacing);

    if (verbose) {
      LOG_INFO("Calibration Target Parameters");
      LOG_INFO("----------------------------------------");
      LOG_INFO("target type: %s", params.target_type.c_str());
      LOG_INFO("tag rows: %d", params.tag_rows);
      LOG_INFO("tag cols: %d", params.tag_cols);
      LOG_INFO("tag size: %f", params.tag_size);
      LOG_INFO("tag spacing: %f", params.tag_spacing);
      LOG_INFO("");
    }
  }

  camera_params_t load_cam_params(const config_t &config,
                                  const int cam_idx,
                                  bool verbose=false) {
    // Load camera calibration
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    std::vector<int> cam_res;
    std::string proj_model;
    std::string dist_model;
    vec4_t proj_params = zeros(4, 1);
    vec4_t dist_params = zeros(4, 1);
    // -- Parse resolution, camera and distortion model
    parse(config, cam_str + ".resolution", cam_res);
    parse(config, cam_str + ".proj_model", proj_model);
    parse(config, cam_str + ".dist_model", dist_model);
    // -- Parse intrinsics
    if (yaml_has_key(config, cam_str + ".proj_params")) {
      parse(config, cam_str + ".proj_params", proj_params);
    }
    // -- Parse distortion
    if (yaml_has_key(config, cam_str + ".dist_params")) {
      parse(config, cam_str + ".dist_params", dist_params);
    }

    if (verbose) {
      LOG_INFO("Camera[%d] Parameters", cam_idx);
      LOG_INFO("----------------------------------------");
      LOG_INFO("proj_params: [%f, %f, %f, %f]",
               proj_params(0),
               proj_params(1),
               proj_params(2),
               proj_params(3));
      LOG_INFO("dist_params: [%f, %f, %f, %f]",
               dist_params(0),
               dist_params(1),
               dist_params(2),
               dist_params(3));
      LOG_INFO("");
    }

    return camera_params_t{param_counter++,
                           cam_idx, cam_res.data(),
                           proj_model, dist_model,
                           proj_params, dist_params};
  }

  static imu_params_t load_imu_params(const config_t &config,
                                      const int index,
                                      bool verbose=false) {
    imu_params_t imu_params;
    const std::string imu_key = "imu" + std::to_string(index);
    parse(config, imu_key + ".rate", imu_params.rate);
    parse(config, imu_key + ".a_max", imu_params.a_max);
    parse(config, imu_key + ".g_max", imu_params.g_max);
    parse(config, imu_key + ".sigma_g_c", imu_params.sigma_g_c);
    parse(config, imu_key + ".sigma_a_c", imu_params.sigma_a_c);
    parse(config, imu_key + ".sigma_gw_c", imu_params.sigma_gw_c);
    parse(config, imu_key + ".sigma_aw_c", imu_params.sigma_aw_c);
    parse(config, imu_key + ".g", imu_params.g);

    if (verbose) {
      LOG_INFO("IMU Parameters");
      LOG_INFO("----------------------------------------");
      LOG_INFO("rate: %f", imu_params.rate);
      LOG_INFO("a_max: %f", imu_params.a_max);
      LOG_INFO("g_max: %f", imu_params.g_max);
      LOG_INFO("sigma_g_c: %f", imu_params.sigma_g_c);
      LOG_INFO("sigma_a_c: %f", imu_params.sigma_a_c);
      LOG_INFO("sigma_gw_c: %f", imu_params.sigma_gw_c);
      LOG_INFO("sigma_aw_c: %f", imu_params.sigma_aw_c);
      LOG_INFO("sigma_ba: %f", imu_params.sigma_ba);
      LOG_INFO("sigma_bg: %f", imu_params.sigma_bg);
      LOG_INFO("g: %f", imu_params.g);
      LOG_INFO("");
    }

    return imu_params;
  }

  extrinsics_t load_cam_exts(const config_t &config,
                             const int cam_idx,
                             bool verbose=false) {
    const std::string cam_key = "cam" + std::to_string(cam_idx);
    const std::string key = "T_body0_" + cam_key;
    mat4_t exts = I(4);
    if (yaml_has_key(config, key)) {
      parse(config, key, exts);
    }

    if (verbose) {
      LOG_INFO("Camera Extrinsics: T_BC%d", cam_idx);
      LOG_INFO("----------------------------------------");
      LOG_INFO("[%f, %f, %f, %f,", exts(0, 0), exts(0, 1), exts(0, 2), exts(0, 3));
      LOG_INFO(" %f, %f, %f, %f,", exts(1, 0), exts(1, 1), exts(1, 2), exts(1, 3));
      LOG_INFO(" %f, %f, %f, %f,", exts(2, 0), exts(2, 1), exts(2, 2), exts(2, 3));
      LOG_INFO(" %f, %f, %f, %f]", exts(3, 0), exts(3, 1), exts(3, 2), exts(3, 3));
      LOG_INFO("");
    }

    return extrinsics_t{param_counter++, exts};
  }
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
