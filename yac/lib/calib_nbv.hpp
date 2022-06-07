#ifndef YAC_CALIB_NBV_HPP
#define YAC_CALIB_NBV_HPP

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_residuals.hpp"

namespace yac {

/**
 * Shannon Entropy
 * @param[in] covar Covariance matrix
 * @returns Shannon entropy
 */
double shannon_entropy(const matx_t &covar);

/**
 * Check if AprilGrid was fully observable
 * @param[in] cam_geom Camera geometry
 * @param[in] cam_params Camera parameters
 * @param[in] T_FC Fiducial-camera relative pose
 * @returns true or false
 */
bool check_fully_observable(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const mat4_t &T_FC);

/**
 * NBV Reached
 *
 * @param[in] nbv_target NBV target pose
 * @param[in] grid Current detected aprilgrid
 * @param[in] nbv_reproj_threshold NBV reprojection error threshold
 * @param[in] reproj_errors Reprojection Errors
 *
 * @returns True or false
 */
bool nbv_reached(const aprilgrid_t &nbv_target,
                 const aprilgrid_t &grid,
                 const real_t nbv_reproj_threshsold,
                 std::vector<double> &reproj_errors);

/**
 * Calibration target origin
 *
 * @param[in] T_FO Calibration target origin
 * @param[in] target Calibration target configuration
 * @param[in] cam_geom Camera Geometry
 * @param[in] cam_params Camera parameters
 * @param[in] target_scale Target scale
 *
 * @returns Calibration target origin
 */
int calib_target_origin(mat4_t &T_FO,
                        const calib_target_t &target,
                        const camera_geometry_t *cam_geom,
                        const camera_params_t *cam_params,
                        const double target_scale = 0.5);

/**
 * Calibration initial poses
 *
 * @param[out] poses Poses
 * @param[in] target Calibration target configuration
 * @param[in] cam_geom Camera Geometry
 * @param[in] cam_params Camera parameters
 *
 * @returns Calibration initial poses
 */
int calib_init_poses(mat4s_t &poses,
                     const calib_target_t &target,
                     const camera_geometry_t *cam_geom,
                     const camera_params_t *cam_params);

/**
 * Calibration NBV poses
 *
 * @param[out] poses Poses
 * @param[in] target Calibration target configuration
 * @param[in] cam_geom Camera Geometry
 * @param[in] cam_params Camera parameters
 * @param[in] range_x_size Range sizes in x-axis
 * @param[in] range_y_size Range sizes in y-axis
 * @param[in] range_z_size Range sizes in z-axis
 *
 * @returns Calibration NBV poses
 */
int calib_nbv_poses(mat4s_t &nbv_poses,
                    const calib_target_t &target,
                    const camera_geometry_t *cam_geom,
                    const camera_params_t *cam_params,
                    const int range_x_size = 6,
                    const int range_y_size = 6,
                    const int range_z_size = 1);

/**
 * Calibration NBV poses
 *
 * @param[out] poses Poses
 * @param[in] target Calibration target configuration
 * @param[in] cam_geoms Camera Geometry
 * @param[in] cam_params Camera parameters
 * @param[in] cam_exts Camera extrinsics
 * @param[in] range_x_size Range sizes in x-axis
 * @param[in] range_y_size Range sizes in y-axis
 * @param[in] range_z_size Range sizes in z-axis
 *
 * @returns Calibration NBV poses
 */
int calib_nbv_poses(std::map<int, mat4s_t> &nbv_poses,
                    const calib_target_t &target,
                    const CamIdx2Geometry &cam_geoms,
                    const CamIdx2Parameters &cam_params,
                    const CamIdx2Extrinsics &cam_exts,
                    const int range_x_size = 6,
                    const int range_y_size = 6,
                    const int range_z_size = 1);

/**
 * NBV Target Grid
 *
 * @param[in] target Calibration target configuration
 * @param[in] cam_geoms Camera Geometry
 * @param[in] cam_params Camera parameters
 * @param[in] nbv_ts NBV timestamp
 * @param[in] nbv_poses NBV poses
 *
 * @return AprilGrid
 */
aprilgrid_t nbv_target_grid(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const timestamp_t nbv_ts,
                            const mat4_t &nbv_pose);

/**
 * Draw NBV
 *
 * @param[in] target Calibration target configuration
 * @param[in] cam_geoms Camera Geometry
 * @param[in] cam_params Camera parameters
 * @param[in] T_FC Fiducial-camera relative pose
 * @param[in,out] image Image to draw on
 *
 * @returns Four corners of the AprilGrid in image coordinates
 */
vec2s_t nbv_draw(const calib_target_t &target,
                 const camera_geometry_t *cam_geom,
                 const camera_params_t *cam_params,
                 const mat4_t &T_FC,
                 cv::Mat &image);

} //  namespace yac
#endif // YAC_CALIB_NBV_HPP
