#ifndef YAC_CALIB_MONO_HPP
#define YAC_CALIB_MONO_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "factor.hpp"

namespace yac {

/// Oplus matrix of a quaternion, i.e. q_AB*q_BC = oplus(q_BC)*q_AB.coeffs().
static mat4_t oplus(const quat_t & q_BC) {
  // clang-format off
  vec4_t q = q_BC.coeffs();
  mat4_t Q;
  Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
  Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
  Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  // clang-format on
  return Q;
}

static matx_t lift_quaternion(const quat_t &q) {
  // clang-format off
  mat_t<3, 4, Eigen::RowMajor> Jq_pinv;
  Jq_pinv << 2.0, 0.0, 0.0, 0.0,
             0.0, 2.0, 0.0, 0.0,
             0.0, 0.0, 2.0, 0.0;
  // clang-format on

  const Eigen::Quaterniond q_inv(q.w(), -q.x(), -q.y(), -q.z());
  return Jq_pinv * oplus(q_inv);
}

class PoseLocalParameterization : public ceres::LocalParameterization {
public:
  PoseLocalParameterization() {}
  virtual ~PoseLocalParameterization() {}

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
		// Form transform
		mat4_t T = tf(x);

		// Update rotation
		vec3_t dalpha{delta[0], delta[1], delta[2]};
		quat_t q = tf_quat(T);
		quat_t dq = quat_delta(dalpha);
		q = dq * q;
		q.normalize();

		// Update translation
		vec3_t dr{delta[3], delta[4], delta[5]};
		vec3_t r = tf_trans(T);
		r = r + dr;

		// Copy results to `x_plus_delta`
		x_plus_delta[0] = q.x();
		x_plus_delta[1] = q.y();
		x_plus_delta[2] = q.z();
		x_plus_delta[3] = q.w();

		x_plus_delta[4] = r(0);
		x_plus_delta[5] = r(1);
		x_plus_delta[6] = r(2);

		return true;
	}

  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
		Jp.setZero();
		Jp.bottomRightCorner<3, 3>().setIdentity();

    mat_t<4, 3> S = zeros(4, 3);
    S(0, 0) = 0.5;
    S(1, 1) = 0.5;
    S(2, 2) = 0.5;

    mat4_t T = tf(x);
    quat_t q = tf_quat(T);
    Jp.block<4, 3>(0, 0) = oplus(q) * S;

		return true;
	}

  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};

/**
 * Calibration mono residual
 */
template <typename CAMERA_TYPE>
struct calib_mono_residual_t : public ceres::SizedCostFunction<2, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_mono_residual_t(const int cam_res[2],
                        const vec3_t &r_FFi,
                        const vec2_t &z,
                        const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(2, 6)); // T_CF
    J_min.push_back(zeros(2, 8)); // cam params
  }

  ~calib_mono_residual_t() {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map camera-fiducial pose
    const mat4_t T_CF = tf(params[0]);
    const mat3_t C_CF = tf_rot(T_CF);
    const quat_t q_CF{C_CF};

    // Map camera params
    Eigen::Map<const vecx_t> cam_params(params[1], 8);

    // Transform and project point to image plane
    const vec3_t r_CFi = tf_point(T_CF, r_FFi_);
    const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
    mat_t<2, 3> Jh;
    matx_t J_params;
    vec2_t z_hat;
    bool valid = true;

    CAMERA_TYPE camera{cam_res_, cam_params};
    if (camera.project(r_CFi, z_hat, Jh) != 0) {
      valid = false;
    }
    J_params = camera.J_params(p);

    // Residuals
		const vec2_t r = sqrt_info_ * (z_ - z_hat);
    residuals[0] = r(0);
    residuals[1] = r(1);

    // Jacobians
    if (jacobians) {
      // Jacobians w.r.t T_CF
      if (jacobians[0]) {
        J_min[0].block(0, 0, 2, 3) = -1 * sqrt_info_ * Jh * -skew(C_CF * r_FFi_);
        J_min[0].block(0, 3, 2, 3) = -1 * sqrt_info_ * Jh * I(3);
        if (valid == false) {
          J_min[0].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        Eigen::Map<mat_t<2, 7, row_major_t>> J0(jacobians[0]);
        mat_t<6, 7, row_major_t> J_lift;
        J_lift.setZero();
        J_lift.topLeftCorner<3, 4>() = lift_quaternion(q_CF);
        J_lift.bottomRightCorner<3, 3>().setIdentity();
        J0 = J_min[0] * J_lift;
      }

      // Jacobians w.r.t camera params
      if (jacobians[1]) {
        J_min[1] = -1 * sqrt_info_ * J_params;
        if (valid == false) {
          J_min[1].setZero();
        }

        Eigen::Map<mat_t<2, 8, row_major_t>> J1(jacobians[1]);
        J1 = J_min[1];
      }
    }

    return true;
  }
};

template <typename T>
static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             const mat2_t &covar,
                             camera_params_t *cam_params,
                             pose_t *pose,
                             ceres::Problem *problem) {
  const int *cam_res = cam_params->resolution;

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
      vec2_t kp = keypoints[i];
      vec3_t obj_pt = object_points[i];
      auto cost_fn = new calib_mono_residual_t<T>{cam_res, obj_pt, kp, covar};
      problem->AddResidualBlock(cost_fn,
                                NULL,
                                pose->param.data(),
                                cam_params->param.data());
    }
  }

  return 0;
}

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename CAMERA_TYPE>
int calib_mono_stats(const aprilgrids_t &aprilgrids,
                     const camera_params_t &cam_params,
                     const mat4s_t &poses,
                     std::vector<double> *errors,
                     double *rmse,
                     double *mean_err) {
  // Obtain residuals using optimized params
  vec2s_t residuals;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    pose_t pose(i, i, T_CF);

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
        const auto z = keypoints[j];
        const auto r_FFi = object_points[j];

				vec_t<8> params;
				params << cam_params.proj_params(), cam_params.dist_params();
				CAMERA_TYPE camera(cam_params.resolution, params);

				vec2_t z_hat;
				const vec3_t r_CFi = tf_point(T_CF, r_FFi);
				camera.project(r_CFi, z_hat);
				vec2_t r = z - z_hat;

				residuals.push_back(r);
      }
    }
  }

  // Calculate RMSE reprojection error
  std::vector<double> r;
  real_t err_sum = 0.0;
  for (auto &residual : residuals) {
    r.push_back(residual(0));
    r.push_back(residual(1));

    const real_t err = residual.norm();
    const real_t err_sq = err * err;
    err_sum += err_sq;
    (*errors).push_back(err);
  }
  *mean_err = err_sum / (real_t) residuals.size();
  *rmse = sqrt(*mean_err);

  return 0;
}

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename CAMERA_TYPE>
int calib_mono_solve(const aprilgrids_t &grids,
                     const mat2_t &covar,
                     camera_params_t *cam_params,
                     mat4s_t *T_CF) {
  // Setup optimization variables
  std::vector<pose_t> poses;
  for (size_t i = 0; i < grids.size(); i++) {
    poses.emplace_back(i, i, grids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(prob_options);
  PoseLocalParameterization pose_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < grids.size(); i++) {
    int retval = process_aprilgrid<CAMERA_TYPE>(grids[i],
                                                covar,
                                                cam_params,
                                                &poses[i],
                                                &problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem.SetParameterization(poses[i].param.data(),
                                &pose_parameterization);
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

  // // Evaluate
	// auto eval = [&](ceres::ResidualBlockId res_id, vec2_t &r) {
	// 	const auto cost_fn = problem.GetCostFunctionForResidualBlock(res_id);
  //   r = zeros(cost_fn->num_residuals(), 1);
	// 	std::vector<double *> param_blocks;
	// 	problem.GetParameterBlocksForResidualBlock(res_id, &param_blocks);
  //   return cost_fn->Evaluate(param_blocks.data(), r.data(), nullptr);
  // };

	// // Get residual blocks
	// int inliers = 0;
	// int outliers = 0;
	// std::vector<ceres::ResidualBlockId> res_ids;
	// problem.GetResidualBlocks(&res_ids);
	// for (const auto res_id : res_ids) {
	// 	vec2_t r{0.0, 0.0};
	// 	eval(res_id, r);
  //
	// 	if (r.norm() > 3.0) {
	// 		problem.RemoveResidualBlock(res_id);
	// 		outliers++;
	// 	} else {
	// 		inliers++;
	// 	}
	// }
	// printf("\n");
	// printf("nb %d inliers\n", inliers);
	// printf("nb %d outliers\n", outliers);
	// printf("ratio of outliers/inliers: %f", (double) outliers / (double) inliers);
	// printf("\n");

	// // Solve the problem again
  // ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;

  // Clean up
  (*T_CF).clear();
  for (auto pose : poses) {
    (*T_CF).push_back(pose.tf());
  }

  // Show results
  std::vector<double> errs;
  double rmse = 0.0;
  double mean = 0.0;
  calib_mono_stats<CAMERA_TYPE>(grids, *cam_params, *T_CF, &errs, &rmse, &mean);

  printf("Optimization results:\n");
  printf("---------------------\n");
  print_vector("cam0.proj_params", (*cam_params).proj_params());
  print_vector("cam0.dist_params", (*cam_params).dist_params());
  printf("rms reproj error [px]: %f\n", rmse);
  printf("mean reproj error [px]: %f\n", mean);
  printf("\n");

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
 *       sigma_vision: 0.5
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
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const std::string &config_file);

// /** Cost function Spec **/
// struct cost_func_spec_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//   const int frame_idx = 0;
//   const std::vector<const param_t *> params;
//   size_t nb_res = 0;
//   size_t nb_params = 0;
//
//   bool status = false;
//   vecx_t r;
//   matxs_t J;
//
//   cost_func_spec_t(const int frame_idx_,
//                    const pose_t *pose_,
//                    const camera_params_t *cam_params_,
//                    const vec2_t &z,
//                    const vec3_t &obj_pt)
//       : frame_idx{frame_idx_},
//         params{pose_, cam_params_},
//         nb_res{2},
//         nb_params{2} {
//     // Instancitate cost function
//     calib_mono_residual_t cost_fn{*cam_params_, z, obj_pt};
//
//     // Allocate memory for residual vector
//     double *r_ = (double *) malloc(sizeof(double) * nb_res);
//
//     // Allocate memory for jacobians
//     double **J_ = (double **) malloc(sizeof(double *) * nb_params);
//     for (size_t i = 0; i < nb_params; i++) {
//       J_[i] = (double *) malloc(sizeof(double) * nb_res * params[i]->global_size);
//     }
//
//     // Evaluate
//     std::vector<const double *> cost_params;
//     for (const auto &param : params) {
//       cost_params.push_back(param->param.data());
//     }
//     status = cost_fn.Evaluate(cost_params.data(), r_, J_);
//
//     // Record results
//     r.resize(nb_res);
//     for (size_t i = 0; i < nb_res; i++) {
//       r(i) = r_[i];
//     }
//     for (size_t i = 0; i < nb_params; i++) {
//       J.push_back(cost_fn.J_min[i]);
//     }
//
//     // Free residual
//     if (r_) {
//       free(r_);
//     }
//
//     // Free Jacobians
//     if (J_) {
//       for (size_t i = 0; i < nb_params; i++) {
//         free(J_[i]);
//       }
//       free(J_);
//     }
//   }
//
//   cost_func_spec_t(const int frame_idx_, camera_params_t *cam_params_)
//       : frame_idx{frame_idx_}, params{cam_params_}, nb_res{8}, nb_params{1} {
//     const matx_t covar = 0.01 * I(8);
//     camera_params_factor_t cam_factor(0, covar, cam_params_);
//     cam_factor.eval();
//     r = cam_factor.residuals;
//     J = cam_factor.jacobians;
//   }
//
//   cost_func_spec_t(const int frame_idx_, pose_t *pose_)
//       : frame_idx{frame_idx_}, params{pose_}, nb_res{6}, nb_params{1} {
//     const matx_t covar = 0.01 * I(6);
//     pose_factor_t pose_factor(0, covar, pose_);
//     pose_factor.eval();
//     r = pose_factor.residuals;
//     J = pose_factor.jacobians;
//   }
// };

// /** Monocular Camera Calibration Covariance Estimation **/
// struct calib_mono_covar_est_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//   int frame_idx = 0;
//   bool pose_prior_set = false;
//
//   std::deque<cost_func_spec_t *> cost_specs;
//   camera_params_t *cam_params = nullptr;
//   std::deque<pose_t *> poses;
//
//   calib_mono_covar_est_t(const camera_params_t &params_,
//                          const aprilgrids_t &grids) {
//     const int cam_idx = params_.cam_index;
//     const int cam_res[2] = {params_.resolution[0], params_.resolution[1]};
//     const std::string &proj_model = params_.proj_model;
//     const std::string &dist_model = params_.dist_model;
//     const vecx_t &proj_params = params_.proj_params();
//     const vecx_t &dist_params = params_.dist_params();
//     cam_params = new camera_params_t{0, cam_idx, cam_res,
//                                      proj_model, dist_model,
//                                      proj_params, dist_params};
//
//     // Add camera params prior
//     const auto cam_prior = new cost_func_spec_t(frame_idx, cam_params);
//     cost_specs.push_back(cam_prior);
//
//     // for (size_t i = 0; i < grids.size(); i++) {
//     for (size_t i = 0; i < 10; i++) {
//       add(grids[i]);
//     }
//   }
//
//   virtual ~calib_mono_covar_est_t() {
//     for (auto &spec : cost_specs) {
//       delete spec;
//     }
//
//     delete cam_params;
//     for (auto &pose : poses) {
//       delete pose;
//     }
//   }
//
//   void add(const aprilgrid_t &grid) {
//     // Add new pose
//     auto T_CF = grid.T_CF;
//     const auto pose = new pose_t{frame_idx, frame_idx, grid.T_CF};
//     poses.push_back(pose);
//
//     // Add pose prior if not already added
//     if (pose_prior_set == false) {
//       const auto pose_prior = new cost_func_spec_t(frame_idx, pose);
//       cost_specs.push_back(pose_prior);
//       pose_prior_set = true;
//     }
//
//     // Add reprojection residuals
//     for (const auto &tag_id : grid.ids) {
//       // Get keypoints
//       vec2s_t keypoints;
//       if (aprilgrid_get(grid, tag_id, keypoints) != 0) {
//         FATAL("Failed to get AprilGrid keypoints!");
//       }
//
//       // Get object points
//       vec3s_t object_points;
//       if (aprilgrid_object_points(grid, tag_id, object_points) != 0) {
//         FATAL("Failed to calculate AprilGrid object points!");
//       }
//
//       // Form residual block
//       for (size_t i = 0; i < 4; i++) {
//         auto &kp = keypoints[i];
//         auto &obj_pt = object_points[i];
//         auto spec = new cost_func_spec_t{frame_idx, pose, cam_params, kp, obj_pt};
//         cost_specs.push_back(spec);
//       }
//     }
//
//     frame_idx++;
//   }
//
//   void remove_last() {
//     // Remove cost specs that belong to the last frame index
//     while (cost_specs.back()->frame_idx == (frame_idx-1)) {
//       auto last_spec = cost_specs.back();
//       cost_specs.pop_back();
//       delete last_spec;
//     }
//
//     // Remove last pose that belong to the last frame index
//     auto last_pose = poses.back();
//     poses.pop_back();
//     delete last_pose;
//
//     // Decrement frame index
//     frame_idx--;
//   }
//
//   int estimate(matx_t &covar) {
//     std::unordered_map<std::string, size_t> param_cs;
//     param_cs["pose_t"] = 0;
//     param_cs["camera_params_t"] = frame_idx * 6;
//
//     // Assign param global index
//     size_t params_size = 0;
//     std::vector<int> factor_ok;
//     std::unordered_map<const param_t *, size_t> param_index;
//
//     for (const auto &spec : cost_specs) {
//       for (size_t i = 0; i < spec->nb_params; i++) {
//         const auto param = spec->params[i];
//         const auto param_type = param->type;
//         const auto min_param_size = param->local_size;
//         if (param_index.count(param) > 0) {
//           continue; // Skip this param
//         }
//
//         param_index.insert({param, param_cs[param_type]});
//         param_cs[param_type] += min_param_size;
//         params_size += min_param_size;
//       }
//     }
//
//     // Form Hessian
//     const size_t H_size = frame_idx * 6 + 8;
//     matx_t H = zeros(H_size, H_size);
//
//     for (const auto &spec : cost_specs) {
//       // Form Hessian H
//       for (size_t i = 0; i < spec->nb_params; i++) {
//         const auto &param_i = spec->params[i];
//         const auto idx_i = param_index[param_i];
//         const auto size_i = param_i->local_size;
//         const matx_t &J_i = spec->J[i];
//
//         for (size_t j = i; j < spec->nb_params; j++) {
//           const auto &param_j = spec->params[j];
//           const auto idx_j = param_index[param_j];
//           const auto size_j = param_j->local_size;
//           const matx_t &J_j = spec->J[j];
//
//           if (i == j) {  // Diagonal
//             H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
//           } else {  // Off-diagonal
//             H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
//             H.block(idx_j, idx_i, size_j, size_i) =
//               H.block(idx_i, idx_j, size_i, size_j).transpose();
//           }
//         }
//       }
//     }
//
//     // Recover covariance matrix
//     // covar = pinv(H);
//     Eigen::FullPivLU<Eigen::MatrixXd> LU(H);
//     if (LU.rank() != H.rows()) { // Check full rank
//       return -1;
//     } else {
//       covar = H.llt().solve(I(H.rows()));
//       // covar = pinv(H);
//     }
//
//     return 0;
//   }
// };

// mat4s_t calib_generate_poses(const calib_target_t &target);
// mat4s_t generate_initial_poses(const calib_target_t &target);
// mat4s_t generate_nbv_poses(const calib_target_t &target);
//
// template <typename CAMERA>
// struct test_grid_t {
//   const int grid_rows = 5;
//   const int grid_cols = 5;
//   const double grid_depth = 5.0;
//   vec2s_t keypoints;
//   vec3s_t object_points;
//
//   test_grid_t(const camera_params_t &cam_params) {
//     const double dx = cam_params.resolution[0] / (grid_cols + 1);
//     const double dy = cam_params.resolution[1] / (grid_rows + 1);
//
//     auto cam_res = cam_params.resolution;
//     auto proj_params = cam_params.proj_params();
//     auto dist_params = cam_params.dist_params();
//     CAMERA camera{cam_res, proj_params, dist_params};
//
//     double kp_x = 0;
//     double kp_y = 0;
//     for (int i = 1; i < (grid_cols + 1); i++) {
//       kp_y += dy;
//       for (int j = 1; j < (grid_rows + 1); j++) {
//         kp_x += dx;
//
//         // Keypoint
//         const vec2_t kp{kp_x, kp_y};
//         keypoints.push_back(kp);
//
//         // Object point
//         Eigen::Vector3d ray;
//         camera.back_project(kp, ray);
//         object_points.push_back(ray * grid_depth);
//       }
//       kp_x = 0;
//     }
//   }
// };

// /** NBV for Monocular Camera Calibration **/
// struct nbv_t {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   calib_target_t calib_target;
//   calib_mono_covar_est_t covar_est;
//
//   nbv_t(const calib_target_t &target_,
//         const camera_params_t &cam_params_,
//         const aprilgrids_t &aprilgrids_)
//     : calib_target{target_}, covar_est{cam_params_, aprilgrids_} {}
//
//   virtual ~nbv_t() {}
//
//   int find_nbv(const calib_target_t &target) {
//     mat4s_t nbv_poses = generate_nbv_poses(target);
//     auto cam_params = covar_est.cam_params;
//
//     int cam_res[2] = {cam_params->resolution[0], cam_params->resolution[1]};
//     auto proj_params = cam_params->proj_params();
//     auto dist_params = cam_params->dist_params();
//     pinhole_radtan4_t camera{cam_res, proj_params, dist_params};
//
//     matx_t covar;
//     int retval = covar_est.estimate(covar);
//     if (retval == -1) {
//       LOG_WARN("Failed to recover covariance!");
//       return -1;
//     }
//     auto base_score = covar.trace();
//     printf("base score: %f\n", base_score);
//
//     real_t best_score = 0.0;
//     mat4_t best_pose = I(4);
//     for (const auto &pose : nbv_poses) {
//       aprilgrid_t grid;
//       aprilgrid_set_properties(grid,
//                                target.tag_rows,
//                                target.tag_cols,
//                                target.tag_size,
//                                target.tag_spacing);
//       grid.T_CF = pose;
//
//       for (int tag_id = 0; tag_id < 36; tag_id++) {
//         vec3s_t object_points;
//         vec3s_t test_points;
//         aprilgrid_object_points(grid, tag_id, object_points);
//         for (int i = 0; i < 4; i++) {
//           // auto test_point = object_points[i];
//           // test_point(2) = 5.0;
//           auto test_point = tf_point(pose, object_points[i]);
//           test_points.push_back(test_point);
//         }
//
//         vec2s_t keypoints;
//         for (int i = 0; i < 4; i++) {
//           vec2_t kp;
//           camera.project(test_points[i], kp);
//           keypoints.push_back(kp);
//         }
//
//         aprilgrid_add(grid, tag_id, keypoints);
//       }
//       covar_est.add(grid);
//
//       matx_t covar_nbv;
//       int retval = covar_est.estimate(covar_nbv);
//       if (retval == -1) {
//         LOG_WARN("Failed to recover covariance! Skipping this pose!");
//         continue;
//       }
//
//       auto score = covar_nbv.trace();
//       // auto score =  shannon_entropy(covar);
//
//       printf("score: %f\n", score);
//       printf("diff score: %f\n", base_score - score);
//       if (score > best_score) {
//         best_score = score;
//         best_pose = pose;
//       }
//       covar_est.remove_last();
//     }
//
//     printf("best score: %f\n", best_score);
//     printf("diff: %f\n", base_score - best_score);
//     print_matrix("best pose", best_pose);
//
//     return 0;
//   }
// };

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
