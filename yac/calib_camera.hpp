#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "yac/core.hpp"
#include "yac/calib_data.hpp"

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

//
// // aprilgrid_t
// // nbv_create_aprilgrid(const calib_target_t &target,
// //                      const camera_geometry_t<pinhole_t, radtan4_t> &camera,
// //                      const vec2_t &resolution,
// //                      const mat4_t &T_CF);
// //
// // void nbv_draw_aprilgrid(const aprilgrid_t grid,
// //                         const pinhole_t &pinhole,
// //                         const radtan4_t &radtan,
// //                         const mat4_t &T_CT,
// //                         cv::Mat &frame);
// //
// // void nbv_find(const calib_target_t &target,
// //               const aprilgrids_t &aprilgrids,
// //               const pinhole_t &pinhole,
// //               const radtan4_t &radtan,
// //               mat4_t &nbv_pose,
// //               aprilgrid_t &nbv_grid);
//
// // real_t calib_camera_nbv_solve(aprilgrids_t &aprilgrids,
// //                               pinhole_t &pinhole,
// //                               radtan4_t &radtan,
// //                               mat4s_t &T_CF);
// //
// // int calib_camera_nbv(const std::string &target_path,
// //                      const size_t max_frames = 15);
//
// int calib_camera_batch(const std::string &target_path,
//                        const size_t max_frames = 15);

/*****************************************************************************
 *                             STEREO CAMERA
 ****************************************************************************/

/**
 * Stereo camera calibration residual
 */
struct stereo_residual_t {
  real_t z_C0_[2] = {0.0, 0.0};     ///< Measurement from cam0
  real_t z_C1_[2] = {0.0, 0.0};     ///< Measurement from cam1
  real_t p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  stereo_residual_t(const vec2_t &z_C0, const vec2_t &z_C1, const vec3_t &p_F)
      : z_C0_{z_C0(0), z_C0(1)},
        z_C1_{z_C1(0), z_C1(1)},
        p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~stereo_residual_t() {}

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const cam0_intrinsics,
                  const T *const cam0_distortion,
                  const T *const cam1_intrinsics,
                  const T *const cam1_distortion,
                  const T *const q_C0C1_,
                  const T *const r_C0C1_,
                  const T *const q_C0F_,
                  const T *const r_C0F_,
                  T *residual) const {
    // Map variables to Eigen
    // -- Fiducial point
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
    // -- cam0 intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam0;
    cam0 << cam0_intrinsics[0], cam0_intrinsics[1], cam0_intrinsics[2], cam0_intrinsics[3],
            cam0_distortion[0], cam0_distortion[1], cam0_distortion[2], cam0_distortion[3];
    // -- cam1 intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam1;
    cam1 << cam1_intrinsics[0], cam1_intrinsics[1], cam1_intrinsics[2], cam1_intrinsics[3],
            cam1_distortion[0], cam1_distortion[1], cam1_distortion[2], cam1_distortion[3];

    // Form transforms
    // clang-format off
    // -- Create transform between fiducial and cam0
    const Eigen::Quaternion<T> q_C0F(q_C0F_[3], q_C0F_[0], q_C0F_[1], q_C0F_[2]);
    const Eigen::Matrix<T, 3, 3> C_C0F = q_C0F.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0F{r_C0F_[0], r_C0F_[1], r_C0F_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0F = tf(C_C0F, r_C0F);
    // -- Create transform between cam0 and cam1
    const Eigen::Quaternion<T> q_C0C1(q_C0C1_[3], q_C0C1_[0], q_C0C1_[1], q_C0C1_[2]);
    const Eigen::Matrix<T, 3, 3> C_C0C1 = q_C0C1.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0C1{r_C0C1_[0], r_C0C1_[1], r_C0C1_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0C1 = tf(C_C0C1, r_C0C1);
    const Eigen::Matrix<T, 4, 4> T_C1C0 = T_C0C1.inverse();
    // clang-format on

    // Project
    // clang-format off
    // -- Project point observed from cam0 to cam0 image plane
    const Eigen::Matrix<T, 3, 1> p_C0 = (T_C0F * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C0_hat;
    if (pinhole_radtan4_project(cam0, p_C0, z_C0_hat) != 0) {
      return false;
    }
    // -- Project point observed from cam0 to cam1 image plane
    const Eigen::Matrix<T, 3, 1> p_C1 = (T_C1C0 * p_C0.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C1_hat;
    if (pinhole_radtan4_project(cam1, p_C1, z_C1_hat) != 0) {
      return false;
    }
    // clang-format on

    // Residual
    // -- cam0 residual
    residual[0] = T(z_C0_[0]) - z_C0_hat(0);
    residual[1] = T(z_C0_[1]) - z_C0_hat(1);
    // -- cam1 residual
    residual[2] = T(z_C1_[0]) - z_C1_hat(0);
    residual[3] = T(z_C1_[1]) - z_C1_hat(1);

    return true;
  }
};

static int process_aprilgrid(const aprilgrid_t &cam0_aprilgrid,
                             const aprilgrid_t &cam1_aprilgrid,
                             real_t *cam0_intrinsics,
                             real_t *cam0_distortion,
                             real_t *cam1_intrinsics,
                             real_t *cam1_distortion,
                             calib_pose_t *T_C0C1,
                             calib_pose_t *T_C0F,
                             ceres::Problem *problem) {
  for (const auto &tag_id : cam0_aprilgrid.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_get(cam0_aprilgrid, tag_id, cam0_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_get(cam1_aprilgrid, tag_id, cam1_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(cam0_aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp0 = cam0_keypoints[i];
      const auto kp1 = cam1_keypoints[i];
      const auto obj_pt = object_points[i];
      const auto residual = new stereo_residual_t{kp0, kp1, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<stereo_residual_t,
                                          4, // Size of: residual
                                          4, // Size of: cam0_intrinsics
                                          4, // Size of: cam0_distortion
                                          4, // Size of: cam1_intrinsics
                                          4, // Size of: cam1_distortion
                                          4, // Size of: q_C0C1
                                          3, // Size of: t_C0C1
                                          4, // Size of: q_C0F
                                          3  // Size of: t_C0F
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                cam0_intrinsics,
                                cam0_distortion,
                                cam1_intrinsics,
                                cam1_distortion,
                                T_C0C1->q,
                                T_C0C1->r,
                                T_C0F->q,
                                T_C0F->r);
    }
  }

  return 0;
}

template <typename DM>
static int save_results(const std::string &save_path,
                        const vec2_t &cam0_resolution,
                        const vec2_t &cam1_resolution,
                        const pinhole_t<DM> &cam0,
                        const pinhole_t<DM> &cam1,
                        const mat4_t &T_C0C1) {
  std::ofstream outfile(save_path);
  const std::string indent = "  ";

  // Check if file is ok
  if (outfile.good() != true) {
    return -1;
  }

  // Save cam0 results
  {
    const std::string resolution = vec2str(cam0_resolution);
    const std::string intrinsics = arr2str(cam0.params.data(), 4);
    const std::string distortion = arr2str(cam0.distortion.params.data(), 4);
    outfile << "cam0:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << resolution << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Save cam1 results
  {
    const std::string resolution = vec2str(cam1_resolution);
    const std::string intrinsics = arr2str(cam1.params.data(), 4);
    const std::string distortion = arr2str(cam1.distortion.params.data(), 4);
    outfile << "cam1:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << resolution << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Save camera extrinsics
  outfile << "T_C0C1:" << std::endl;
  outfile << indent << "rows: 4" << std::endl;
  outfile << indent << "cols: 4" << std::endl;
  outfile << indent << "data: [" << std::endl;
  outfile << mat2str(T_C0C1, indent + indent) << std::endl;
  outfile << indent << "]" << std::endl;
  outfile << std::endl;

  // Finsh up
  outfile.close();

  return 0;
}

template <typename DM>
int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_aprilgrids,
                       const std::vector<aprilgrid_t> &cam1_aprilgrids,
                       pinhole_t<DM> &cam0,
                       pinhole_t<DM> &cam1,
                       mat4_t &T_C0C1,
                       mat4s_t &T_C0F) {
  assert(cam0_aprilgrids.size() == cam1_aprilgrids.size());

  // Optimization variables
  calib_pose_t extrinsic_param{T_C0C1};
  std::vector<calib_pose_t> pose_params;
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    pose_params.emplace_back(cam0_aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  // clang-format off
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
  ceres::EigenQuaternionParameterization quaternion_parameterization;
  // clang-format on

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    int retval = process_aprilgrid(cam0_aprilgrids[i],
                                   cam1_aprilgrids[i],
                                   cam0.params.data(),
                                   cam0.distortion.params.data(),
                                   cam1.params.data(),
                                   cam1.distortion.params.data(),
                                   &extrinsic_param,
                                   &pose_params[i],
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(pose_params[i].q,
                                 &quaternion_parameterization);
  }
  problem->SetParameterization(extrinsic_param.q,
                               &quaternion_parameterization);

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Finish up
  T_C0C1 = extrinsic_param.T();
  for (auto pose_param : pose_params) {
    T_C0F.emplace_back(pose_param.T());
  }

  return 0;
}

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25[-]
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 *
 *     cam1:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 */
int calib_stereo_solve(const std::string &config_file);

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
