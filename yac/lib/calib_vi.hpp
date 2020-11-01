#ifndef YAC_CALIB_VI_HPP
#define YAC_CALIB_VI_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "core.hpp"
#include "calib_data.hpp"
#include "ceres_utils.hpp"
#include "calib_params.hpp"

namespace yac {

template <typename CAMERA_TYPE>
struct calib_reproj_residual_t
    : public ceres::SizedCostFunction<2, 7, 7, 7, 8> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_res_[2] = {0, 0};
  vec3_t r_FFi_{0.0, 0.0, 0.0};
  vec2_t z_{0.0, 0.0};

  const mat2_t covar_;
  const mat2_t info_;
  const mat2_t sqrt_info_;
  mutable matxs_t J_min;

  calib_reproj_residual_t(const int cam_res[2],
                          const vec3_t &r_FFi,
                          const vec2_t &z,
                          const mat2_t &covar)
      : cam_res_{cam_res[0], cam_res[1]},
        r_FFi_{r_FFi}, z_{z},
        covar_{covar},
        info_{covar.inverse()},
        sqrt_info_{info_.llt().matrixL().transpose()} {
    J_min.push_back(zeros(2, 6)); // T_WF
    J_min.push_back(zeros(2, 6)); // T_WS
    J_min.push_back(zeros(2, 6)); // T_SC
    J_min.push_back(zeros(2, 8)); // cam params
  }

  ~calib_reproj_residual_t() {}

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map fiducial pose T_WF
    const mat4_t T_WF = tf(params[0]);
    const mat3_t C_WF = tf_rot(T_WF);
    const quat_t q_WF{C_WF};

    // Map sensor pose T_WS
    const mat4_t T_WS = tf(params[1]);
    const mat4_t T_SW = T_WS.inverse();
    const mat3_t C_WS = tf_rot(T_WS);
    const quat_t q_WS{C_WS};

    // Map sensor-camera pose T_SC
    const mat4_t T_SC = tf(params[2]);
    const mat4_t T_CS = T_SC.inverse();
    const mat3_t C_SC = tf_rot(T_SC);
    const quat_t q_SC{C_SC};

    // Map camera params
    Eigen::Map<const vecx_t> cam_params(params[3], 8);

    // Transform and project point to image plane
    const vec3_t r_CFi = tf_point(T_CS * T_SW * T_WF, r_FFi_);
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
    const matx_t weighted_Jh = -1 * sqrt_info_ * Jh;
		const mat3_t C_CW = tf_rot(T_CS * T_SW);
		const mat3_t C_CS = C_SC.transpose();

    if (jacobians != NULL) {
      // Jacobians w.r.t. T_WF
      if (jacobians[0] != NULL) {
        J_min[0].block(0, 0, 2, 3) = weighted_Jh * -C_CW * -skew(C_WF * r_FFi_);
        J_min[0].block(0, 3, 2, 3) = weighted_Jh * -C_CW * I(3);
        if (valid == false) {
          J_min[0].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[0], q_WF, jacobians[0]);
      }

      // Jacobians w.r.t T_WS
      if (jacobians[1] != NULL) {
        J_min[1].block(0, 0, 2, 3) = weighted_Jh * -C_CW * -skew(C_WS * r_CFi);
        J_min[1].block(0, 3, 2, 3) = weighted_Jh * -C_CW * I(3);
        if (valid == false) {
          J_min[1].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[1], q_WS, jacobians[1]);
      }

      // Jacobians w.r.t T_SC
      if (jacobians[2] != NULL) {
        J_min[2].block(0, 0, 2, 3) = weighted_Jh * C_CS * skew(C_SC * r_CFi);
        J_min[2].block(0, 3, 2, 3) = weighted_Jh * -C_CS * I(3);
        if (valid == false) {
          J_min[2].setZero();
        }

        // Convert from minimial jacobians to local jacobian
        lift_pose_jacobian(J_min[2], q_SC, jacobians[2]);
      }

      // Jacobians w.r.t. camera parameters
      if (jacobians[3] != NULL) {
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

/** Inertial residual */
struct calib_imu_residual_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	const int imu_index = 0;
  const timestamps_t imu_ts;
  const vec3s_t imu_accel;
  const vec3s_t imu_gyro;
  const vec3_t g{0.0, 0.0, -9.81};

  mat_t<15, 15> P = zeros(15, 15);  // Covariance matrix
  mat_t<12, 12> Q = zeros(12, 12);  // noise matrix
  mat_t<15, 15> F = zeros(15, 15);  // Transition matrix
  mutable matxs_t J_min;

	// Parameter indicies
	const int ROT_IDX = 0;
	const int POS_IDX = 3;
	const int VEL_IDX = 6;
	const int BA_IDX = 9;
	const int BG_IDX = 12;

  // Delta position, velocity and rotation between timestep i and j
  // (i.e start and end of imu measurements)
  vec3_t dp{0.0, 0.0, 0.0};
  vec3_t dv{0.0, 0.0, 0.0};
  quat_t dq{1.0, 0.0, 0.0, 0.0};

  // Accelerometer and gyroscope biases
  vec3_t bg{0.0, 0.0, 0.0};
  vec3_t ba{0.0, 0.0, 0.0};

  calib_imu_residual_t(const int imu_index_,
				 	 	 	 	 	 		 const timestamps_t imu_ts_,
                       const vec3s_t imu_accel_,
                       const vec3s_t imu_gyro_ )
      : imu_index{imu_index_},
			 	imu_ts{imu_ts_},
        imu_accel{imu_accel_},
        imu_gyro{imu_gyro_} {
    J_min.push_back(zeros(15, 6));  // T_WS at timestep i
    J_min.push_back(zeros(15, 9));  // Speed and bias at timestep i
    J_min.push_back(zeros(15, 6));  // T_WS at timestep j
    J_min.push_back(zeros(15, 9));  // Speed and bias at timestep j
    propagate(imu_ts_, imu_accel_, imu_gyro_);
  }

  void reset() {
    P = zeros(15, 15);
    F = zeros(15, 15);

    dp = zeros(3);
    dv = zeros(3);
    dq = quat_t{1.0, 0.0, 0.0, 0.0};
    ba = zeros(3);
    bg = zeros(3);
  }

  void propagate(const timestamps_t &ts,
                 const vec3s_t &a_m,
                 const vec3s_t &w_m) {
		assert(ts.size() > 2);
    assert(ts.size() == a_m.size());
    assert(w_m.size() == a_m.size());

		real_t dt = 0.0;
    real_t dt_prev = ns2sec(ts[1] - ts[0]);
    for (size_t i = 0; i < w_m.size(); i++) {
      // Calculate dt
      if ((i + 1) < w_m.size()) {
        dt = ns2sec(ts[i + 1] - ts[i]);
        dt_prev = dt;
      } else {
        dt = dt_prev;
      }

      // Update relative position and velocity
      dp = dp + dv * dt + 0.5 * (dq * (a_m[i] - ba)) * dt * dt;
      dv = dv + (dq * (a_m[i] - ba)) * dt;

      // Update relative rotation
      const real_t scalar = 1.0;
      const vec3_t vector = 0.5 * (w_m[i] - bg) * dt;
      const quat_t dq_i{scalar, vector(0), vector(1), vector(2)};
      dq = dq * dq_i;

      // Transition matrix F
      const mat3_t C_ji = dq.toRotationMatrix();
      mat_t<15, 15> F_i = zeros(15, 15);
      F_i.block<3, 3>(ROT_IDX, ROT_IDX) = -skew(w_m[i] - bg);
      F_i.block<3, 3>(ROT_IDX, BG_IDX) = -I(3);
      F_i.block<3, 3>(POS_IDX, VEL_IDX) = I(3);
      F_i.block<3, 3>(VEL_IDX, ROT_IDX) = -C_ji * skew(a_m[i] - ba);
      F_i.block<3, 3>(VEL_IDX, BA_IDX) = -C_ji;

      // Input matrix G
      mat_t<15, 12> G_i = zeros(15, 12);
      G_i.block<3, 3>(ROT_IDX, BG_IDX) = -I(3);
      G_i.block<3, 3>(VEL_IDX, BA_IDX) = -C_ji;
      G_i.block<3, 3>(BA_IDX, BA_IDX) = I(3);
      G_i.block<3, 3>(BG_IDX, BG_IDX) = I(3);

      // Update jacobian and covariance matrix
      const mat_t<15, 15> I_Fi_dt = I(15) + dt * F;
      const mat_t<15, 12> Gi_dt = G_i * dt;
      F = I_Fi_dt * F;
      P = I_Fi_dt * P * I_Fi_dt.transpose() + Gi_dt * Q * Gi_dt.transpose();
    }
  }

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map out parameters
    // -- Sensor pose at timestep i
    const mat4_t T_i = tf(params[0]);
    const mat3_t C_i = tf_rot(T_i);
    const mat3_t C_i_inv = C_i.transpose();
    const quat_t q_i = tf_quat(T_i);
    const vec3_t r_i = tf_trans(T_i);
    // -- Speed and bias at timestamp i
    const vec_t<9> sb_i{params[1]};
    const vec3_t v_i = sb_i.segment<3>(0);
    const vec3_t ba_i = sb_i.segment<3>(3);
    const vec3_t bg_i = sb_i.segment<3>(6);
    // -- Sensor pose at timestep j
    const mat4_t T_j = tf(params[2]);
    const quat_t q_j = tf_quat(T_j);
    const vec3_t r_j = tf_trans(T_j);
    // -- Speed and bias at timestep j
    const vec_t<9> sb_j{params[3]};
    const vec3_t v_j = sb_j.segment<3>(0);
    const vec3_t ba_j = sb_j.segment<3>(3);
    const vec3_t bg_j = sb_j.segment<3>(6);

    // Obtain Jacobians for gyro and accel bias
    const mat3_t dp_dba = F.block<3, 3>(POS_IDX, BA_IDX);
    const mat3_t dp_dbg = F.block<3, 3>(POS_IDX, BG_IDX);
    const mat3_t dv_dba = F.block<3, 3>(VEL_IDX, BA_IDX);
    const mat3_t dv_dbg = F.block<3, 3>(VEL_IDX, BG_IDX);
    const mat3_t dq_dbg = F.block<3, 3>(ROT_IDX, BG_IDX);

		// Calculate square root info
		const matx_t info{P.inverse()};
    const matx_t sqrt_info{info.llt().matrixL().transpose()};

    // Calculate residuals
    const real_t dt_ij = ns2sec(imu_ts.back() - imu_ts.front());
    const real_t dt_ij_sq = dt_ij * dt_ij;
    const vec3_t dbg = bg_i - bg;
    const vec3_t dba = ba_i - ba;
    const vec3_t alpha = dp + dp_dbg * dbg + dp_dba * dba;
    const vec3_t beta = dv + dv_dbg * dbg + dv_dba * dba;
    const quat_t gamma = dq * quat_delta(dq_dbg * dbg);

    const quat_t q_i_inv = q_i.inverse();
    const quat_t q_j_inv = q_j.inverse();
    const quat_t gamma_inv = gamma.inverse();

    // clang-format off
		const vec3_t err_rot = 2.0 * (gamma_inv * (q_i_inv * q_j)).vec();
		const vec3_t err_trans = C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha;
		const vec3_t err_vel = C_i_inv * (v_j - v_i + g * dt_ij) - beta;
		const vec3_t err_ba = ba_j - ba_i;
		const vec3_t err_bg = bg_j - bg_i;
    Eigen::Map<vec_t<15>> r(residuals);
		r << err_rot, err_trans, err_vel, err_ba, err_bg;
		r = sqrt_info * r;
    // clang-format on

		// Calculate Jacobians
		if (jacobians) {
			// Jacobian w.r.t. pose_i
			if (jacobians[0]) {
				J_min[0] = zeros(15, 6);
				J_min[0].block<3, 3>(ROT_IDX, ROT_IDX) = -quat_mat_xyz(quat_lmul(q_j_inv * q_i) * quat_rmul(gamma));
				J_min[0].block<3, 3>(POS_IDX, ROT_IDX) = skew(C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq));
				J_min[0].block<3, 3>(POS_IDX, POS_IDX) = -C_i_inv;
				J_min[0].block<3, 3>(VEL_IDX, ROT_IDX) = skew(C_i_inv * (v_j - v_i + g * dt_ij));
				J_min[0] = sqrt_info * J_min[0];
			}

			// Jacobian w.r.t. sb_i
			if (jacobians[1]) {
				J_min[1] = zeros(15, 9);
				J_min[1].block<3, 3>(POS_IDX, VEL_IDX - VEL_IDX) = -C_i_inv * dt_ij;
				J_min[1].block<3, 3>(POS_IDX, BA_IDX - VEL_IDX) = -dp_dba;
				J_min[1].block<3, 3>(POS_IDX, BG_IDX - VEL_IDX) = -dp_dbg;
				J_min[1].block<3, 3>(VEL_IDX, VEL_IDX - VEL_IDX) = -C_i_inv;
				J_min[1].block<3, 3>(VEL_IDX, BA_IDX - VEL_IDX) = -dv_dba;
				J_min[1].block<3, 3>(VEL_IDX, BG_IDX - VEL_IDX) = -dv_dbg;
				J_min[1].block<3, 3>(BA_IDX, BA_IDX - VEL_IDX) = -I(3);
				J_min[1].block<3, 3>(BG_IDX, BG_IDX - VEL_IDX) = -I(3);
				J_min[1] = sqrt_info * J_min[1];
			}

			// Jacobian w.r.t. pose_j
			if (jacobians[2]) {
				J_min[2] = zeros(15, 6);
				J_min[2].block<3, 3>(ROT_IDX, ROT_IDX) = quat_lmul_xyz(gamma_inv * q_i_inv * q_j_inv);
				J_min[2].block<3, 3>(POS_IDX, POS_IDX) = C_i_inv;
				J_min[2] = sqrt_info * J_min[2];
			}

			// Jacobian w.r.t. sb_j
			if (jacobians[3]) {
				// -- Speed and bias at j Jacobian
				J_min[3] = zeros(15, 9);
				J_min[3].block<3, 3>(3, 0) = C_i_inv;
				J_min[3].block<3, 3>(9, 3) = I(3);
				J_min[3] = sqrt_info * J_min[3];
			}
		}

    return true;
  }
};

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 */
template <typename CAMERA_TYPE>
int calib_vi_solve(const std::map<int, aprilgrid_t> &cam_grids,
                   const mat2_t &covar,
                   std::map<int, camera_params_t> &cam_params,
                   std::map<int, extrinsic_t> &extrinsics) {
  // Setup optimization problem
  ceres::Problem::Options prob_options;
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(prob_options));
  PoseLocalParameterization pose_parameterization;

  // // Process all aprilgrid data
  // for (size_t i = 0; i < cam0_grids.size(); i++) {
  //   int retval = process_aprilgrid<CAMERA_TYPE>(cam0_grids[i],
  //                                               cam1_grids[i],
  //                                               covar,
  //                                               &rel_poses[i],
  //                                               &extrinsic_param,
  //                                               cam0,
  //                                               cam1,
  //                                               problem.get());
  //   if (retval != 0) {
  //     LOG_ERROR("Failed to add AprilGrid measurements to problem!");
  //     return -1;
  //   }
  //
  //   problem->SetParameterization(rel_poses[i].param.data(),
  //                                &pose_parameterization);
  // }

  // Set solver options
  // ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  // options.max_num_iterations = 100;
  // options.check_gradients = true;

  // // Solve
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, problem.get(), &summary);
  // std::cout << summary.FullReport() << std::endl;

  // // Show results
  // std::vector<double> cam0_errs, cam1_errs;
  // double cam0_rmse, cam1_rmse = 0.0;
  // double cam0_mean, cam1_mean = 0.0;
  // calib_mono_stats<CAMERA_TYPE>(cam0_grids, *cam0, *T_C0F, &cam0_errs, &cam0_rmse, &cam0_mean);
  // calib_mono_stats<CAMERA_TYPE>(cam1_grids, *cam1, *T_C1F, &cam1_errs, &cam1_rmse, &cam1_mean);

  // printf("Optimization results:\n");
  // printf("---------------------\n");
  // print_vector("cam0.proj_params", (*cam0).proj_params());
  // print_vector("cam0.dist_params", (*cam0).dist_params());
  // print_vector("cam1.proj_params", (*cam1).proj_params());
  // print_vector("cam1.dist_params", (*cam1).dist_params());
  // printf("\n");
  // print_matrix("T_C1C0", *T_C1C0);
  // printf("cam0 rms reproj error [px]: %f\n", cam0_rmse);
  // printf("cam1 rms reproj error [px]: %f\n", cam1_rmse);
  // printf("cam0 mean reproj error [px]: %f\n", cam0_mean);
  // printf("cam1 mean reproj error [px]: %f\n", cam1_mean);
  // printf("\n");

  return 0;
}

} //  namespace yac
#endif // YAC_CALIB_VI_HPP
