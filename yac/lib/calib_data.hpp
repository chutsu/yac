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

/**
 * Calibration target.
 */
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

// /* Calib data */
// struct calib_data_t {
// 	ceres::Problem::Options prob_options;
// 	ceres::Problem *problem;
//   PoseLocalParameterization pose_plus;
//
//   aprilgrids_t grids;
//   std::map<int, camera_params_t> cam_params;
//   std::deque<pose_t> poses;
//   mat2_t covar = I(2);
//
// 	calib_data_t() {}
//
// 	calib_data_t(const aprilgrids_t &grids_,
// 							 const std::map<int, camera_params_t> &cam_params_,
// 					 	 	 const mat2_t covar_=I(2))
// 			: grids{grids_}, cam_params{cam_params_}, covar{covar_} {
// 		prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
// 		prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
// 		prob_options.enable_fast_removal = true;
// 		problem = new ceres::Problem(prob_options);
// 	}
//
// 	~calib_data_t() {
// 		delete problem;
// 	}
//
// 	void reset() {
//     delete problem;
// 		problem = new ceres::Problem(prob_options);
// 		poses.clear();
// 	}
// };

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

/**
 * Initialize focal lengths using aprilgrid. `axis` denotes the focal length
 * for x-axis or y-axis, 0 or 1 repsectively. The result is written to `focal`.
 * Returns 0 for success, -1 for failure.
 */
int focal_init(const aprilgrid_t &grid, const int axis, double &focal);

} // namespace yac
#endif // YAC_CALIB_DATA_HPP
