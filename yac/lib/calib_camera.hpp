#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"

namespace yac {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

/**
 * Pinhole Radial-tangential calibration residual
 */
struct pinhole_radtan4_residual_t {
  double z_[2] = {0.0, 0.0};        ///< Measurement
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  pinhole_radtan4_residual_t(const vec2_t &z, const vec3_t &p_F)
      : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~pinhole_radtan4_residual_t() {}

  /**
   * Calculate residual (auto diff version)
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
                  const T *const distortion_,
                  const T *const q_CF_,
                  const T *const r_CF_,
                  T *residuals_) const {
    // Map variables to Eigen
    Eigen::Matrix<T, 8, 1> cam_params;
    cam_params << intrinsics_[0], intrinsics_[1], intrinsics_[2], intrinsics_[3],
                  distortion_[0], distortion_[1], distortion_[2], distortion_[3];
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};

    // Form tf
    const Eigen::Quaternion<T> q_CF(q_CF_[3], q_CF_[0], q_CF_[1], q_CF_[2]);
    const Eigen::Matrix<T, 3, 3> C_CF = q_CF.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_CF{r_CF_[0], r_CF_[1], r_CF_[2]};
    Eigen::Matrix<T, 4, 4> T_CF = tf(C_CF, r_CF);

    // Transform and project point to image plane
    const Eigen::Matrix<T, 3, 1> p_C = (T_CF * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_hat;
    if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
      return false;
    }

    // Residual
    residuals_[0] = T(z_[0]) - z_hat(0);
    residuals_[1] = T(z_[1]) - z_hat(1);

    return true;
  }
};

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename RESIDUAL>
int calib_camera_stats(const aprilgrids_t &aprilgrids,
                       const double *intrinsics,
                       const double *distortion,
                       const mat4s_t &poses,
                       const std::string &output_path) {
  UNUSED(output_path);
  vec2s_t residuals;

  // Obtain residuals using optimized params
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    const quat_t q_CF = tf_quat(T_CF);
    const vec3_t r_CF = tf_trans(T_CF);

    // Iterate over all tags in AprilGrid
    for (const auto &tag_id : aprilgrid.ids) {
      // Get keypoints
      vec2s_t keypoints;
      if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
        LOG_ERROR("Failed to get AprilGrid keypoints!");
        return -1;
      }

      // Get object points
      vec3s_t object_points;
      if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
        LOG_ERROR("Failed to calculate AprilGrid object points!");
        return -1;
      }

      // Form residual and call the functor for four corners of the tag
      for (size_t j = 0; j < 4; j++) {
        const RESIDUAL residual{keypoints[j], object_points[j]};
        real_t res[2] = {0.0, 0.0};
        residual(intrinsics,
                 distortion,
                 q_CF.coeffs().data(),
                 r_CF.data(),
                 res);
        residuals.emplace_back(res[0], res[1]);
      }
    }
  }

  // Calculate RMSE reprojection error
  real_t err_sum = 0.0;
  for (auto &residual : residuals) {
    const real_t err = residual.norm();
    const real_t err_sq = err * err;
    err_sum += err_sq;
  }
  const real_t err_mean = err_sum / (real_t) residuals.size();
  const real_t rmse = sqrt(err_mean);
  std::cout << "nb_residuals: " << residuals.size() << std::endl;
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return 0;
}

/**
 * Generate camera poses for Next-Best-View.
 */
mat4s_t calib_generate_poses(const calib_target_t &target);

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_t *pose,
                             ceres::Problem &problem) {
  for (const auto &tag_id : aprilgrid.ids) {
    // Get keypoints
    vec2s_t keypoints;
    if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp = keypoints[i];
      const auto obj_pt = object_points[i];

      const auto residual = new pinhole_radtan4_residual_t{kp, obj_pt};
      const auto cost_func =
          new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t,
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_CF
                                          3  // Size of: r_CF
                                          >(residual);

      // const auto cost_func = new intrinsics_residual_t{kp, obj_pt};
      problem.AddResidualBlock(cost_func, // Cost function
                               NULL,      // Loss function
                               intrinsics,
                               distortion,
                               pose->q,
                               pose->r);
    }
  }

  return 0;
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename DM>
int calib_camera_solve(const aprilgrids_t &aprilgrids,
                       pinhole_t<DM> &cam,
                       mat4s_t &T_CF) {
  // Optimization variables
  std::vector<calib_pose_t> T_CF_params;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    T_CF_params.emplace_back(aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   cam.params.data(),
                                   cam.distortion.params.data(),
                                   &T_CF_params[i],
                                   problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem.SetParameterization(T_CF_params[i].q,
                                &quaternion_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Clean up
  T_CF.clear();
  for (auto pose_param : T_CF_params) {
    T_CF.push_back(pose_param.T());
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const vec2_t &resolution,
                        const pinhole_t<radtan4_t> &cam) {
  std::ofstream outfile(save_path);

  // Check if file is ok
  if (outfile.good() != true) {
    return -1;
  }

  // Save results
  const std::string indent = "  ";
  {
    const std::string res = vec2str(resolution);
    const std::string intrinsics = arr2str(cam.params.data(), 4);
    const std::string distortion = arr2str(cam.distortion.params.data(), 4);
    outfile << "cam0:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << res << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Finsh up
  outfile.close();

  return 0;
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target. This function assumes that the path to `config_file`
 * is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       imshow: true
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.085           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_camera_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
