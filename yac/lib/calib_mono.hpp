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

/**
 * Calibration mono residual
 */
struct calib_mono_residual_t
		: public ceres::SizedCostFunction<2, 7, 4, 4> {
  int resolution_[2] = {0, 0};
  std::string proj_model_ = "pinhole";
  std::string dist_model_ = "radtan4";
  vec2_t z_{0.0, 0.0};        ///< Measurement
  vec3_t r_FFi_{0.0, 0.0, 0.0}; ///< Object point

  calib_mono_residual_t(const calib_params_t &calib_params,
                        const vec2_t &z,
                        const vec3_t &r_FFi)
      : resolution_{calib_params.img_w, calib_params.img_h},
        proj_model_{calib_params.proj_model},
        dist_model_{calib_params.dist_model},
        z_{z(0), z(1)},
        r_FFi_{r_FFi(0), r_FFi(1), r_FFi(2)} {}

  ~calib_mono_residual_t() {}

  bool Evaluate(double const * const *parameters,
                double *residuals,
                double **jacobians,
                double **jacobians_min) const {
    // Map camera-fiducial pose
    const mat4_t T_CF = tf(parameters[0]);
    const mat3_t C_CF = tf_rot(T_CF);
    const quat_t q_CF{C_CF};

    // Map camera parameters
    vecx_t proj_params;
		proj_params.resize(4);
    proj_params << parameters[1][0],
                   parameters[1][1],
                   parameters[1][2],
                   parameters[1][3];

    vecx_t dist_params;
		dist_params.resize(4);
    dist_params << parameters[2][0],
                   parameters[2][1],
                   parameters[2][2],
                   parameters[2][3];

    // Transform and project point to image plane
    const vec3_t r_CFi = tf_point(T_CF, r_FFi_);
    const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
    mat_t<2, 3> Jh;
    matx_t J_params;
    vec2_t z_hat;
		bool valid = true;
    if (proj_model_ == "pinhole" && dist_model_ == "radtan4") {
      pinhole_radtan4_t camera{resolution_, proj_params, dist_params};
      if (camera.project(r_CFi, z_hat, Jh) != 0) {
				valid = false;
        // return true;
      }
			J_params = camera.J_params(p);

    } else if (proj_model_ == "pinhole" && dist_model_ == "equi4") {
      pinhole_equi4_t camera{resolution_, proj_params, dist_params};
      if (camera.project(r_CFi, z_hat, Jh) != 0) {
				valid = false;
        // return true;
      }
    } else {
      FATAL("Unsupported [%s-%s] projection distortion combination!",
            proj_model_.c_str(), dist_model_.c_str());
    }

    // Residuals
    residuals[0] = z_(0) - z_hat(0);
    residuals[1] = z_(1) - z_hat(1);

    // Jacobians
    if (jacobians) {
      // Jacobians w.r.t q_CF
      if (jacobians[0]) {
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_min;
        J0_min.block(0, 0, 2, 3) = -1 * Jh * -skew(C_CF * r_FFi_);
        J0_min.block(0, 3, 2, 3) = -1 * Jh * I(3);
				if (valid == false) {
					J0_min.setZero();
				}

        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        J_lift.setZero();
        J_lift.topLeftCorner<3, 4>() = lift_quaternion(q_CF);
        J_lift.bottomRightCorner<3, 3>().setIdentity();

        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = J0_min * J_lift;

        if (jacobians_min && jacobians_min[0]) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians_min[0]);
          J = J0_min;
        }
      }

      // Jacobians w.r.t proj parameters
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = -1 * J_params.block(0, 0, 2, 4);
				if (valid == false) {
					J1.setZero();
				}

        if (jacobians_min && jacobians_min[1]) {
          Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians_min[1]);
          J = J1;
        }
      }

      // Jacobians w.r.t dist parameters
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = -1 * J_params.block(0, 4, 2, 4);
				if (valid == false) {
					J2.setZero();
				}

        if (jacobians_min && jacobians_min[2]) {
          Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians_min[2]);
          J = J2;
        }
      }

      // // Jacobians w.r.t q_CF
      // if (jacobians[0]) {
      //   Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J0_min;
      //   J0_min = -1 * Jh * -skew(C_CF * r_FFi_);
			// 	if (valid == false) {
			// 		J0_min.setZero();
			// 	}
      //
      //   Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J0(jacobians[0]);
      //   J0 = J0_min * lift_quaternion(q_CF);
      //
      //   if (jacobians_min && jacobians_min[0]) {
      //     Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians_min[0]);
      //     J = J0_min;
      //   }
      // }
      //
      // // Jacobians w.r.t r_CF
      // if (jacobians[1]) {
      //   Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J1(jacobians[1]);
      //   J1 = -1 * Jh * I(3);
			// 	if (valid == false) {
			// 		J1.setZero();
			// 	}
      //
      //   if (jacobians_min && jacobians_min[1]) {
      //     Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians_min[1]);
      //     J = J1;
      //   }
      // }

      // // Jacobians w.r.t proj parameters
      // if (jacobians[2]) {
      //   Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J2(jacobians[2]);
      //   J2 = -1 * J_params.block(0, 0, 2, 4);
			// 	if (valid == false) {
			// 		J2.setZero();
			// 	}
      //
      //   if (jacobians_min && jacobians_min[2]) {
      //     Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians_min[2]);
      //     J = J2;
      //   }
      // }
      //
      // // Jacobians w.r.t dist parameters
      // if (jacobians[3]) {
      //   Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J3(jacobians[3]);
      //   J3 = -1 * J_params.block(0, 4, 2, 4);
			// 	if (valid == false) {
			// 		J3.setZero();
			// 	}
      //
      //   if (jacobians_min && jacobians_min[3]) {
      //     Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians_min[3]);
      //     J = J3;
      //   }
      // }
    }

    return true;


  }

  bool Evaluate(double const * const *parameters,
                double *residuals,
                double **jacobians) const {
    return Evaluate(parameters, residuals, jacobians, nullptr);
  }
};

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const aprilgrids_t &aprilgrids,
                     calib_params_t &calib_params,
                     mat4s_t &T_CF);

/**
 * Estimate the covariance of the monocular camera calibration problem.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_estimate_covariance(const aprilgrids_t &aprilgrids,
                                   calib_params_t &calib_params,
                                   mat4s_t &T_CF);

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
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_solve(const std::string &config_file);

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_mono_stats(const aprilgrids_t &aprilgrids,
                     const calib_params_t &calib_params,
                     const mat4s_t &poses);

/**
 * Generate poses
 */
mat4s_t calib_generate_poses(const calib_target_t &target);

mat4s_t generate_initial_poses(const calib_target_t &target);
mat4s_t generate_nbv_poses(const calib_target_t &target);

} //  namespace yac
#endif // YAC_CALIB_MONO_HPP
