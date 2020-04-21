#ifndef YAC_CALIB_STEREO_HPP
#define YAC_CALIB_STEREO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"

namespace yac {


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
#endif // YAC_CALIB_STEREO_HPP
