#ifndef YAC_CALIB_DATA_HPP
#define YAC_CALIB_DATA_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

/** Camera Data **/
typedef std::map<timestamp_t, std::map<int, aprilgrid_t>> camera_data_t;

// CALIBRATION TARGET //////////////////////////////////////////////////////////

/** Calibration target */
struct calib_target_t {
  std::string target_type = "aprilgrid";
  int tag_rows = 6;
  int tag_cols = 6;
  real_t tag_size = 0.088;
  real_t tag_spacing = 0.3;

  calib_target_t() = default;
  calib_target_t(const std::string target_type_,
                 const int tag_rows_,
                 const int tag_cols_,
                 const real_t tag_size_,
                 const real_t tag_spacing_);
  ~calib_target_t() = default;

  /**
   * Load calibration target.
   * @returns 0 or -1 for success or failure
   */
  int load(const std::string &target_file, const std::string &prefix = "");

  /** Print calibration target */
  void print();
};

// CALIBRATION CONFIG //////////////////////////////////////////////////////////

/** Calibration configuration */
struct calib_config_t {
  const config_t config;

  /** Constructor */
  calib_config_t(const std::string &config_file);

  /** Get number of cameras */
  int get_num_cams() const;

  /** Get number of imus */
  int get_num_imus() const;

  /** Get calibration target */
  calib_target_t get_calib_target(const std::string &prefix = "",
                                  bool verbose = true) const;

  /** Get imu parameters */
  imu_params_t get_imu_params(const int index, bool verbose = true) const;

  /** Get camera parameters */
  camera_params_t get_cam_params(const int cam_idx, bool verbose = true) const;

  /** Get camera extrinsics */
  extrinsics_t get_cam_exts(const int cam_idx, bool verbose = true) const;
};

// CALIBRATION DATA ////////////////////////////////////////////////////////////

/** Calibration data */
struct calib_data_t {
  bool ok = false;
  std::string node_name;

  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem = nullptr;
  ceres::LossFunction *loss = nullptr;
  PoseLocalParameterization pose_plus;
  std::map<int, std::vector<double>> vision_errors;

  // Params
  calib_target_t calib_target;
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, std::pair<int, aprilgrid_t>> cam_grids;

  int nb_cams = 0;
  int nb_imus = 0;
  imu_params_t imu_params;
  std::map<int, camera_geometry_t *> cam_geoms;
  std::map<int, camera_params_t> cam_params;
  mat2_t covar = I(2);

  // Data
  id_t param_counter = 0;
  std::map<timestamp_t, pose_t> poses;
  std::map<int, extrinsics_t> cam_exts;

  // Camera geometry
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  // Constructors & Destructors
  calib_data_t();
  calib_data_t(const std::string &config_file);
  ~calib_data_t();

  /** Add calibration target */
  void add_calib_target(const calib_target_t &target_);

  /** Add IMU */
  void add_imu(const imu_params_t &params);

  /** Add camera */
  void add_camera(const camera_params_t &params);

  /** Add camera extrinsics */
  void add_camera_extrinsics(const int cam_idx, const mat4_t &ext = I(4));

  /** Add grids */
  void add_grids(const int cam_idx, const aprilgrids_t &grids);

  /** Add pose */
  pose_t &add_pose(const timestamp_t &ts, const mat4_t &T);

  /** Check calibration data */
  // void check_data();

  /** Get number of aprilgrids */
  int get_number_of_grids(const timestamp_t &ts);

  /** Get aprilgrids at timestamp `ts` */
  std::vector<std::pair<int, aprilgrid_t>> get_grids(const timestamp_t &ts);

  /**
   * Preprocess camera image data and output AprilGrid detection data as
   * csv. Once the data is preprocessed the data is saved to `output_dir`.
   *
   * @returns 0 for Success, -1 for failure, and 1 where the output directory
   * contains data.
   */
  int preprocess_camera_data(const std::string &image_dir,
                             const std::string &output_dir,
                             const bool show_progress = true);

  /** Show results */
  void show_results();
};

// CALIBRATION FUNCTIONS ///////////////////////////////////////////////////////

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
 * Draw measured and projected pixel points.
 * @returns Image
 */
cv::Mat draw_calib_validation(const cv::Mat &image,
                              const vec2s_t &measured,
                              const vec2s_t &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
