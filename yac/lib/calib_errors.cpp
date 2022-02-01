#include "calib_errors.hpp"

namespace yac {

// CALIB ERROR /////////////////////////////////////////////////////////////////

bool calib_error_t::eval() {
  // Pre-allocate memory for residuals
  residuals = zeros(num_residuals(), 1);

  // Pre-allocate memory for jacobians
  std::vector<matx_row_major_t> jacobians;
  jacobians.resize(param_blocks.size());

  std::vector<double *> param_ptrs;
  std::vector<double *> jacobian_ptrs;
  for (size_t i = 0; i < param_blocks.size(); i++) {
    const auto &param = param_blocks[i];
    param_ptrs.push_back(param->param.data());
    jacobians[i].resize(residuals.size(), param->global_size);
    jacobian_ptrs.push_back(jacobians[i].data());
  }

  // Evaluate cost function to obtain the jacobians and residuals
  Evaluate(param_ptrs.data(), residuals.data(), jacobian_ptrs.data());

  // Scale jacobians and residuals if using loss function
  // following ceres-sovler in `internal/ceres/corrector.cc`
  if (loss_fn) {
    double residual_scaling = 0.0;
    double alpha_sq_norm = 0.0;
    double rho[3] = {0.0};

    double sq_norm = residuals.squaredNorm();
    loss_fn->Evaluate(sq_norm, rho);
    double sqrt_rho1 = sqrt(rho[1]);

    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling = sqrt_rho1;
      alpha_sq_norm = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - sqrt(D);
      residual_scaling = sqrt_rho1 / (1 - alpha);
      alpha_sq_norm = alpha / sq_norm;
    }

    for (size_t i = 0; i < param_blocks.size(); i++) {
      // clang-format off
      jacobians[i] = sqrt_rho1 * (
        jacobians[i] - alpha_sq_norm * residuals *
        (residuals.transpose() * jacobians[i])
      );
      // clang-format on
    }

    residuals *= residual_scaling;
  }
}

bool calib_error_t::check_jacs(const int param_idx,
                               const std::string &jac_name,
                               const double step,
                               const double tol) {
  // Setup
  param_t *param = param_blocks[param_idx];
  const int residual_size = num_residuals();
  const int local_size = param->local_size;
  const int global_size = param->global_size;

  // Params
  std::vector<double *> param_ptrs;
  for (size_t i = 0; i < param_blocks.size(); i++) {
    param_ptrs.push_back(param_blocks[i]->data());
  }

  // Jacobians
  double **jacobian_ptrs = new double *[param_blocks.size()];
  std::vector<matx_row_major_t> jacobians;
  jacobians.resize(param_blocks.size());
  for (size_t i = 0; i < param_blocks.size(); i++) {
    jacobians[i].resize(residual_size, param_blocks[i]->global_size);
    jacobian_ptrs[i] = jacobians[i].data();
  }

  // Base-line
  vecx_t r = zeros(residual_size);
  Evaluate(param_ptrs.data(), r.data(), jacobian_ptrs);
  free(jacobian_ptrs);

  // Finite difference
  matx_t fdiff = zeros(residual_size, local_size);
  vecx_t r_fd = zeros(residual_size);
  for (int i = 0; i < local_size; i++) {
    param->perturb(i, step);
    Evaluate(param_ptrs.data(), r_fd.data(), nullptr);
    fdiff.col(i) = (r_fd - r) / step;
    param->perturb(i, -step);
  }

  int retval = check_jacobian(jac_name, fdiff, J_min[param_idx], tol, true);
  return (retval == 0) ? true : false;
}

// POSE ERROR //////////////////////////////////////////////////////////////////

pose_error_t::pose_error_t(const pose_t &pose, const matx_t &covar)
    : pose_meas_{pose}, covar_{covar}, info_{covar.inverse()},
      sqrt_info_{info_.llt().matrixL().transpose()} {}

bool pose_error_t::Evaluate(double const *const *params,
                            double *residuals,
                            double **jacobians) const {
  // Parameters
  mat4_t pose_meas = pose_meas_.tf();
  mat4_t pose_est = tf(params[0]);

  // Error
  const mat4_t dpose = pose_meas * pose_est.inverse();
  const vec3_t dr = tf_trans(pose_meas) - tf_trans(pose_est);
  const mat3_t dC = tf_rot(dpose);
  const quat_t dq{dC};
  const vec3_t dtheta = 2 * vec3_t{dq.x(), dq.y(), dq.z()};
  vec6_t error;
  error.head<3>() = dr;
  error.tail<3>() = dtheta;

  // Residuals
  Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
  r = sqrt_info_ * error;

  // Jacobians
  if (jacobians != NULL && jacobians[0] != NULL) {
    mat_t<6, 6> J_min;
    J_min.setIdentity();
    J_min *= -1.0;
    J_min.block<3, 3>(3, 3) = -plus(dq).topLeftCorner<3, 3>();
    J_min = (sqrt_info_ * J_min).eval();

    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[0]);
    J.setZero();
    J.block(0, 0, 6, 6) = J_min;
  }

  return true;
}

// REPROJECTION ERROR //////////////////////////////////////////////////////////

reproj_error_t::reproj_error_t(camera_geometry_t *cam_geom_,
                               camera_params_t *cam_params_,
                               pose_t *T_BCi_,
                               pose_t *T_C0F_,
                               fiducial_corner_t *p_FFi_,
                               const vec2_t &z_,
                               const mat2_t &covar_)
    : cam_geom{cam_geom_}, cam_params{cam_params_}, T_BCi{T_BCi_},
      T_C0F{T_C0F_}, p_FFi{p_FFi_}, z{z_}, covar(covar_), info(covar.inverse()),
      sqrt_info(info.llt().matrixU()) {
  // Data
  param_blocks.push_back(T_BCi_);
  param_blocks.push_back(T_C0F_);
  param_blocks.push_back(p_FFi_);
  param_blocks.push_back(cam_params_);
  residuals = zeros(2, 1);
  J_min.push_back(zeros(2, 6));
  J_min.push_back(zeros(2, 6));
  J_min.push_back(zeros(2, 3));
  J_min.push_back(zeros(2, 8));

  // Ceres-Solver
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(7); // Camera-camera extrinsics
  block_sizes->push_back(7); // Camera-fiducial relative pose
  block_sizes->push_back(3); // Fiducial corner parameter
  block_sizes->push_back(8); // Camera parameters
}

int reproj_error_t::get_residual(vec2_t &z_hat, vec2_t &r) const {
  assert(T_BCi != nullptr);
  assert(T_C0F != nullptr);

  // Map parameters out
  const mat4_t T_C0Ci_ = T_BCi->tf();
  const mat4_t T_C0F_ = T_C0F->tf();
  const vec3_t p_FFi_ = p_FFi->param;

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0_ = T_C0Ci_.inverse();
  const vec3_t p_CiFi = tf_point(T_CiC0_ * T_C0F_, p_FFi_);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  auto param = cam_params->param;
  if (cam_geom->project(res, param, p_CiFi, z_hat) != 0) {
    return -1;
  }
  // -- Residual
  r = z - z_hat;

  return 0;
}

int reproj_error_t::get_residual(vec2_t &r) const {
  assert(T_BCi != nullptr);
  assert(T_C0F != nullptr);

  vec2_t z_hat = zeros(2, 1);
  if (get_residual(z_hat, r) != 0) {
    return -1;
  }

  return 0;
}

int reproj_error_t::get_reproj_error(real_t &error) const {
  vec2_t r;
  if (get_residual(r) != 0) {
    return -1;
  }
  error = r.norm();
  return 0;
}

bool reproj_error_t::Evaluate(double const *const *params,
                              double *residuals,
                              double **jacobians) const {
  // Map parameters out
  const mat4_t T_C0Ci = tf(params[0]);
  const mat4_t T_C0F = tf(params[1]);
  Eigen::Map<const vecx_t> p_FFi(params[2], 3);
  Eigen::Map<const vecx_t> param(params[3], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0 = T_C0Ci.inverse();
  const vec3_t p_CiFi = tf_point(T_CiC0 * T_C0F, p_FFi);
  // -- Project point from camera frame to image plane
  auto res = cam_params->resolution;
  vec2_t z_hat;
  bool valid = true;
  if (cam_geom->project(res, param, p_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residual
  Eigen::Map<vec2_t> r(residuals);
  r = sqrt_info * (z - z_hat);

  // Jacobians
  const matx_t Jh = cam_geom->project_jacobian(param, p_CiFi);
  const matx_t Jh_weighted = -1 * sqrt_info * Jh;

  if (jacobians) {
    // Jacobians w.r.t T_C0Ci
    if (jacobians[0]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0Ci = C_CiC0.transpose();
        const vec3_t p_CiFi = tf_point(T_CiC0 * T_C0F, p_FFi);

        // clang-format off
        J_min[0].block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
        J_min[0].block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * p_CiFi);
        // clang-format on

        J = J_min[0] * lift_pose_jacobian(T_C0Ci);
      }
    }

    // Jacobians w.r.t T_C0F
    if (jacobians[1]) {
      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J.setZero();

      if (valid) {
        const mat3_t C_CiC0 = tf_rot(T_CiC0);
        const mat3_t C_C0F = tf_rot(T_C0F);
        J_min[1].block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
        J_min[1].block(0, 3, 2, 3) =
            Jh_weighted * C_CiC0 * -skew(C_C0F * p_FFi);
        J = J_min[1] * lift_pose_jacobian(T_C0F);
      }
    }

    // Jacobians w.r.t fiducial corner
    if (jacobians[2]) {
      Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[2]);
      J.setZero();
      if (valid) {
        const mat4_t T_CiF = T_CiC0 * T_C0F;
        const mat3_t C_CiF = tf_rot(T_CiF);
        J_min[2] = Jh_weighted * C_CiF;
        J.block(0, 0, 2, 3) = J_min[2];
      }
    }

    // Jacobians w.r.t cam params
    if (jacobians[3]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[3]);
      J.setZero();
      if (valid) {
        const matx_t J_cam = cam_geom->params_jacobian(param, p_CiFi);
        J_min[3] = -1 * sqrt_info * J_cam;
        J = J_min[3];
      }
    }
  }

  return true;
}

// FIDUCIAL ERROR //////////////////////////////////////////////////////////////

fiducial_error_t::fiducial_error_t(const timestamp_t &ts,
                                   camera_geometry_t *cam_geom,
                                   camera_params_t *cam_params,
                                   extrinsics_t *cam_exts,
                                   extrinsics_t *imu_exts,
                                   fiducial_t *fiducial,
                                   pose_t *pose,
                                   const int tag_id,
                                   const int corner_idx,
                                   const vec3_t &r_FFi,
                                   const vec2_t &z,
                                   const mat4_t &T_WF,
                                   const mat2_t &covar)
    : ts_{ts}, cam_geom_{cam_geom},
      cam_params_{cam_params}, cam_exts_{cam_exts}, imu_exts_{imu_exts},
      fiducial_{fiducial}, pose_{pose}, tag_id_{tag_id},
      corner_idx_{corner_idx}, r_FFi_{r_FFi}, z_{z}, T_WF_{T_WF}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {
  set_num_residuals(2);
  auto block_sizes = mutable_parameter_block_sizes();
  block_sizes->push_back(fiducial_->param.size()); // T_WF
  block_sizes->push_back(7);                       // T_WS
  block_sizes->push_back(7);                       // T_BS
  block_sizes->push_back(7);                       // T_BCi
  block_sizes->push_back(8);                       // camera parameters
}

int fiducial_error_t::get_residual(vec2_t &r) const {
  // Map parameters
  const mat4_t T_WF = fiducial_->estimate();
  const mat4_t T_WS = tf(pose_->param.data());
  const mat4_t T_BS = tf(imu_exts_->param.data());
  const mat4_t T_BCi = tf(cam_exts_->param.data());
  Eigen::Map<const vecx_t> params(cam_params_->param.data(), 8);

  // Transform and project point to image plane
  const auto res = cam_params_->resolution;
  const mat4_t T_CiB = T_BCi.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
  vec2_t z_hat;
  if (cam_geom_->project(res, params, r_CiFi, z_hat) != 0) {
    return -1;
  }

  // Calculate residual
  r = z_ - z_hat;

  return 0;
}

int fiducial_error_t::get_reproj_error(real_t &error) const {
  vec2_t r;
  if (get_residual(r) != 0) {
    return -1;
  }
  error = r.norm();
  return 0;
}

bool fiducial_error_t::Evaluate(double const *const *params,
                                double *residuals,
                                double **jacobians) const {
  // Map parameters
#if FIDUCIAL_PARAMS_SIZE == 2
  const double roll = params[0][0];
  const double pitch = params[0][1];
  const double yaw = quat2euler(tf_quat(T_WF_))(2);
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF_);
  const mat4_t T_WF = tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 3
  const double roll = params[0][0];
  const double pitch = params[0][1];
  const double yaw = params[0][2];
  const vec3_t rpy{roll, pitch, yaw};
  const mat3_t C_WF = euler321(rpy);
  const vec3_t r_WF = tf_trans(T_WF_);
  const mat4_t T_WF = tf(C_WF, r_WF);
#elif FIDUCIAL_PARAMS_SIZE == 7
  const mat4_t T_WF = tf(params[0]);
#endif
  const mat4_t T_WS = tf(params[1]);
  const mat4_t T_BS = tf(params[2]);
  const mat4_t T_BCi = tf(params[3]);
  Eigen::Map<const vecx_t> cam_params(params[4], 8);

  // Transform and project point to image plane
  bool valid = true;
  const auto cam_res = cam_params_->resolution;
  const mat4_t T_CiB = T_BCi.inverse();
  const mat4_t T_SW = T_WS.inverse();
  const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);
  vec2_t z_hat;
  if (cam_geom_->project(cam_res, cam_params, r_CiFi, z_hat) != 0) {
    valid = false;
  }

  // Residuals
  const vec2_t r = sqrt_info_ * (z_ - z_hat);
  residuals[0] = r.x();
  residuals[1] = r.y();

  // Jacobians
  const matx_t Jh = cam_geom_->project_jacobian(cam_params, r_CiFi);
  const matx_t Jh_weighted = sqrt_info_ * Jh;

  if (jacobians) {
    // Jacobians w.r.t. T_WF
    if (jacobians[0]) {
#if FIDUCIAL_PARAMS_SIZE == 2 || FIDUCIAL_PARAMS_SIZE == 3
      const double cphi = cos(roll);
      const double sphi = sin(roll);
      const double ctheta = cos(pitch);
      const double stheta = sin(pitch);
      const double cpsi = cos(yaw);
      const double spsi = sin(yaw);

      const double x = r_FFi_(0);
      const double y = r_FFi_(1);
      const double z = r_FFi_(2);

      const vec3_t J_x{y * (sphi * spsi + stheta * cphi * cpsi) +
                           z * (-sphi * stheta * cpsi + spsi * cphi),
                       y * (-sphi * cpsi + spsi * stheta * cphi) +
                           z * (-sphi * spsi * stheta - cphi * cpsi),
                       y * cphi * ctheta - z * sphi * ctheta};
      const vec3_t J_y{-x * stheta * cpsi + y * sphi * cpsi * ctheta +
                           z * cphi * cpsi * ctheta,
                       -x * spsi * stheta + y * sphi * spsi * ctheta +
                           z * spsi * cphi * ctheta,
                       -x * ctheta - y * sphi * stheta - z * stheta * cphi};
      const vec3_t J_z{-x * spsi * ctheta +
                           y * (-sphi * spsi * stheta - cphi * cpsi) +
                           z * (sphi * cpsi - spsi * stheta * cphi),
                       x * cpsi * ctheta +
                           y * (sphi * stheta * cpsi - spsi * cphi) +
                           z * (sphi * spsi + stheta * cphi * cpsi),
                       0};
#endif

      // Fill Jacobian
      const mat3_t C_CiW = tf_rot(T_CiB * T_BS * T_SW);
#if FIDUCIAL_PARAMS_SIZE == 2
      Eigen::Map<mat_t<2, 2, row_major_t>> J(jacobians[0]);
      J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
#elif FIDUCIAL_PARAMS_SIZE == 3
      Eigen::Map<mat_t<2, 3, row_major_t>> J(jacobians[0]);
      J.block(0, 0, 2, 1) = -1 * Jh_weighted * C_CiW * J_x;
      J.block(0, 1, 2, 1) = -1 * Jh_weighted * C_CiW * J_y;
      J.block(0, 2, 2, 1) = -1 * Jh_weighted * C_CiW * J_z;
#elif FIDUCIAL_PARAMS_SIZE == 7
      const mat3_t C_WF = tf_rot(T_WF);
      mat_t<2, 6, row_major_t> J_min;
      J_min.setZero();
      J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiW * I(3);
      J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiW * -skew(C_WF * r_FFi_);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
      J = J_min * lift_pose_jacobian(T_WF);
#endif
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_WS
    if (jacobians[1]) {
      const mat3_t C_CiS = tf_rot(T_CiB * T_BS);
      const mat3_t C_WS = tf_rot(T_WS);
      const mat3_t C_SW = C_WS.transpose();
      const mat3_t C_CiW = C_CiS * C_SW;
      const mat4_t T_SW = T_WS.inverse();
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      mat_t<2, 6, row_major_t> J_min;
      J_min.setZero();
      J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiW * I(3);
      J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiW * -skew(C_WS * r_SFi);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
      J = J_min * lift_pose_jacobian(T_WS);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BS
    if (jacobians[2]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BS = tf_rot(T_BS);
      const vec3_t r_SFi = tf_point(T_SW * T_WF, r_FFi_);

      mat_t<2, 6, row_major_t> J_min;
      J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiB * I(3);
      J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiB * -skew(C_BS * r_SFi);

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[2]);
      J = J_min * lift_pose_jacobian(T_BS);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t T_BCi
    if (jacobians[3]) {
      const mat3_t C_CiB = tf_rot(T_CiB);
      const mat3_t C_BCi = C_CiB.transpose();
      const vec3_t r_CiFi = tf_point(T_CiB * T_BS * T_SW * T_WF, r_FFi_);

      // clang-format off
      mat_t<2, 6, row_major_t> J_min;
      J_min.block(0, 0, 2, 3) = -1 * Jh_weighted * -C_CiB * I(3);
      J_min.block(0, 3, 2, 3) = -1 * Jh_weighted * -C_CiB * -skew(C_BCi * r_CiFi);
      // clang-format on

      Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[3]);
      J = J_min * lift_pose_jacobian(T_BCi);
      if (valid == false) {
        J.setZero();
      }
    }

    // Jacobians w.r.t. camera parameters
    if (jacobians[4]) {
      const matx_t J_cam = cam_geom_->params_jacobian(cam_params, r_CiFi);
      Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[4]);
      J.block(0, 0, 2, 8) = -1 * sqrt_info_ * J_cam;
      if (valid == false) {
        J.setZero();
      }
    }

    // // Jacobians w.r.t. time delay
    // if (jacobians[5]) {
    //   Eigen::Map<vec2_t> J(jacobians[5]);
    //   J = sqrt_info_ * v_ij_;
    //   if (valid == false) {
    //     J.setZero();
    //   }
    // }
  }

  return true;
}

// INERTIAL ERROR //////////////////////////////////////////////////////////////

ImuError::ImuError(const imu_data_t &imu_data,
                   const imu_params_t &imu_params,
                   const timestamp_t &t0,
                   const timestamp_t &t1) {
  imu_data_ = imu_data;
  imu_params_ = imu_params;
  t0_ = t0;
  t1_ = t1;
}

int ImuError::propagation(const imu_data_t &imu_data,
                          const imu_params_t &imu_params,
                          mat4_t &T_WS,
                          vec_t<9> &sb,
                          const timestamp_t &t_start,
                          const timestamp_t &t_end,
                          covariance_t *covariance,
                          jacobian_t *jacobian) {
  // now the propagation
  // timestamp_t time = t_start;
  // timestamp_t end = t_end;
  // printf("time_start: [%ld]\n", time.toNSec());
  // printf("time_end:   [%ld]\n", end.toNSec());
  // printf("imu front:  [%ld]\n", imu_data.front().timeStamp.toNSec());
  // printf("imu back:   [%ld]\n", imu_data.back().timeStamp.toNSec());

  // sanity check:
  assert(imu_data.timestamps.front() <= t_start);
  assert(imu_data.timestamps.back() >= t_end);
  // if (!(imu_data.back().timeStamp >= end))
  //   return -1; // nothing to do...

  // initial condition
  vec3_t r_0 = tf_trans(T_WS);
  quat_t q_WS_0 = tf_quat(T_WS);
  mat3_t C_WS_0 = tf_rot(T_WS);

  // increments (initialise with identity)
  quat_t Delta_q(1, 0, 0, 0);
  mat3_t C_integral = mat3_t::Zero();
  mat3_t C_doubleintegral = mat3_t::Zero();
  vec3_t acc_integral = vec3_t::Zero();
  vec3_t acc_doubleintegral = vec3_t::Zero();

  // cross matrix accumulatrion
  mat3_t cross = mat3_t::Zero();

  // sub-Jacobians
  mat3_t dalpha_db_g = mat3_t::Zero();
  mat3_t dv_db_g = mat3_t::Zero();
  mat3_t dp_db_g = mat3_t::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double, 15, 15> P_delta = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  timestamp_t ts_k = t_start;
  for (size_t k = 0; k < (imu_data.size() - 1); k++) {
    vec3_t omega_S_0 = imu_data.gyro[k];
    vec3_t acc_S_0 = imu_data.accel[k];
    vec3_t omega_S_1 = imu_data.gyro[k + 1];
    vec3_t acc_S_1 = imu_data.accel[k + 1];
    // print_vector("omega_S_0", omega_S_0);
    // print_vector("omega_S_1", omega_S_1);
    // print_vector("acc_S_0", acc_S_0);
    // print_vector("acc_S_1", acc_S_1);

    // Time delta
    timestamp_t ts_kp1;
    if ((k + 1) == imu_data.size()) {
      ts_kp1 = t_end;
    } else {
      ts_kp1 = imu_data.timestamps[k + 1];
    }
    // printf("ts_kp1: %ld\n", ts_kp1);
    // printf("ts_k:     %ld\n", ts_k);
    // printf("diff:     %ld\n", ts_kp1 - ts_k);
    if ((ts_kp1 - ts_k) < 0) {
      continue;
    }
    double dt = ns2sec(ts_kp1 - ts_k);
    // printf("dt: %f\n", dt);

    // If next imu measurement timestamp is after t_end, interpolate gyro and
    // accel at t_end
    if (ts_kp1 > t_end) {
      double interval = ns2sec(ts_kp1 - imu_data.timestamps[k]);
      ts_kp1 = t_end;
      dt = ns2sec(ts_kp1 - ts_k);
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;
    // printf("Delta_t: %f\n", Delta_t);

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(ts_kp1 - ts_k);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_params.sigma_g_c;
    double sigma_a_c = imu_params.sigma_a_c;

    // if (fabs(omega_S_0[0]) > imuParams.g_max ||
    //     fabs(omega_S_0[1]) > imuParams.g_max ||
    //     fabs(omega_S_0[2]) > imuParams.g_max ||
    //     fabs(omega_S_1[0]) > imuParams.g_max ||
    //     fabs(omega_S_1[1]) > imuParams.g_max ||
    //     fabs(omega_S_1[2]) > imuParams.g_max) {
    //   sigma_g_c *= 100;
    //   LOG(WARNING) << "gyr saturation";
    // }
    //
    // if (fabs(acc_S_0[0]) > imuParams.a_max ||
    //     fabs(acc_S_0[1]) > imuParams.a_max ||
    //     fabs(acc_S_0[2]) > imuParams.a_max ||
    //     fabs(acc_S_1[0]) > imuParams.a_max ||
    //     fabs(acc_S_1[1]) > imuParams.a_max ||
    //     fabs(acc_S_1[2]) > imuParams.a_max) {
    //   sigma_a_c *= 100;
    //   LOG(WARNING) << "acc saturation";
    // }

    // actual propagation
    // clang-format off
      // orientation:
      quat_t dq;
      const vec3_t omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - sb.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
      dq.w() = cos_theta_half;
      quat_t Delta_q_1 = Delta_q * dq;
      // rotation matrix integral:
      const mat3_t C = Delta_q.toRotationMatrix();
      const mat3_t C_1 = Delta_q_1.toRotationMatrix();
      const vec3_t acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - sb.segment<3>(6));
      const mat3_t C_integral_1 = C_integral + 0.5 * (C + C_1) * dt;
      const vec3_t acc_integral_1 = acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
      // rotation matrix double integral:
      C_doubleintegral += C_integral * dt + 0.25 * (C + C_1) * dt * dt;
      acc_doubleintegral += acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

      // Jacobian parts
      dalpha_db_g += dt * C_1;
      const mat3_t cross_1 = dq.inverse().toRotationMatrix() * cross + rightJacobian(omega_S_true * dt) * dt;
      const mat3_t acc_S_x = skew(acc_S_true);
      mat3_t dv_db_g_1 = dv_db_g + 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      dp_db_g += dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
    // clang-format on

    // covariance propagation
    // clang-format off
      if (covariance) {
        Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
        // transform
        F_delta.block<3, 3>(0, 3) = -skew( acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
        F_delta.block<3, 3>(0, 6) = I(3) * dt;
        F_delta.block<3, 3>(0, 9) = dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(0, 12) = -C_integral * dt + 0.25 * (C + C_1) * dt * dt;
        F_delta.block<3, 3>(3, 9) = -dt * C_1;
        F_delta.block<3, 3>(6, 3) = -skew(0.5 * (C + C_1) * acc_S_true * dt);
        F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
        P_delta = F_delta * P_delta * F_delta.transpose();
        // add noise. Note that transformations with rotation matrices can be
        // ignored, since the noise is isotropic.
        // F_tot = F_delta*F_tot;
        const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
        P_delta(3, 3) += sigma2_dalpha;
        P_delta(4, 4) += sigma2_dalpha;
        P_delta(5, 5) += sigma2_dalpha;
        const double sigma2_v = dt * sigma_a_c * imu_params.sigma_a_c;
        P_delta(6, 6) += sigma2_v;
        P_delta(7, 7) += sigma2_v;
        P_delta(8, 8) += sigma2_v;
        const double sigma2_p = 0.5 * dt * dt * sigma2_v;
        P_delta(0, 0) += sigma2_p;
        P_delta(1, 1) += sigma2_p;
        P_delta(2, 2) += sigma2_p;
        const double sigma2_b_g = dt * imu_params.sigma_gw_c * imu_params.sigma_gw_c;
        P_delta(9, 9) += sigma2_b_g;
        P_delta(10, 10) += sigma2_b_g;
        P_delta(11, 11) += sigma2_b_g;
        const double sigma2_b_a = dt * imu_params.sigma_aw_c * imu_params.sigma_aw_c;
        P_delta(12, 12) += sigma2_b_a;
        P_delta(13, 13) += sigma2_b_a;
        P_delta(14, 14) += sigma2_b_a;
      }
    // clang-format on

    // memory shift
    Delta_q = Delta_q_1;
    C_integral = C_integral_1;
    acc_integral = acc_integral_1;
    cross = cross_1;
    dv_db_g = dv_db_g_1;
    ts_k = ts_kp1;

    ++i;

    if (ts_kp1 == t_end)
      break;
  }
  // printf("Delta_t: %f\n", Delta_t);
  // print_vector("acc_doubleintegral", acc_doubleintegral);

  // actual propagation output:
  const vec3_t g_W = imu_params.g * vec3_t(0, 0, 6371009).normalized();
  const vec3_t trans_update =
      r_0 + sb.head<3>() * Delta_t +
      C_WS_0 * (acc_doubleintegral /*-C_doubleintegral*sb.segment<3>(6)*/) -
      0.5 * g_W * Delta_t * Delta_t;
  const quat_t quat_update = q_WS_0 * Delta_q;
  T_WS = tf(quat_update, trans_update);
  sb.head<3>() +=
      C_WS_0 * (acc_integral /*-C_integral*sb.segment<3>(6)*/) - g_W * Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double, 15, 15> &F = *jacobian;
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g,
                     // d/db_a
    F.block<3, 3>(0, 3) = -skew(C_WS_0 * acc_doubleintegral);
    F.block<3, 3>(0, 6) = I(3) * Delta_t;
    F.block<3, 3>(0, 9) = C_WS_0 * dp_db_g;
    F.block<3, 3>(0, 12) = -C_WS_0 * C_doubleintegral;
    F.block<3, 3>(3, 9) = -C_WS_0 * dalpha_db_g;
    F.block<3, 3>(6, 3) = -skew(C_WS_0 * acc_integral);
    F.block<3, 3>(6, 9) = C_WS_0 * dv_db_g;
    F.block<3, 3>(6, 12) = -C_WS_0 * C_integral;
  }

  // overall covariance, if requested
  if (covariance) {
    Eigen::Matrix<double, 15, 15> &P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double, 15, 15> T = Eigen::Matrix<double, 15, 15>::Identity();
    T.topLeftCorner<3, 3>() = C_WS_0;
    T.block<3, 3>(3, 3) = C_WS_0;
    T.block<3, 3>(6, 6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }

  return i;
}

int ImuError::redoPreintegration(const mat4_t & /*T_WS*/,
                                 const vec_t<9> &sb,
                                 timestamp_t time,
                                 timestamp_t end) const {
  // Ensure unique access
  std::lock_guard<std::mutex> lock(preintegrationMutex_);

  // Sanity check:
  assert(imu_data_.timestamps.front() <= time);
  if (!(imu_data_.timestamps.back() >= end))
    return -1; // nothing to do...

  // Increments (initialise with identity)
  Delta_q_ = quat_t(1, 0, 0, 0);
  C_integral_ = mat3_t::Zero();
  C_doubleintegral_ = mat3_t::Zero();
  acc_integral_ = vec3_t::Zero();
  acc_doubleintegral_ = vec3_t::Zero();

  // Cross matrix accumulatrion
  cross_ = mat3_t::Zero();

  // Sub-Jacobians
  dalpha_db_g_ = mat3_t::Zero();
  dv_db_g_ = mat3_t::Zero();
  dp_db_g_ = mat3_t::Zero();

  // Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  for (size_t k = 0; k < imu_data_.size(); k++) {
    vec3_t omega_S_0 = imu_data_.gyro[k];
    vec3_t acc_S_0 = imu_data_.accel[k];
    vec3_t omega_S_1 = imu_data_.gyro[k + 1];
    vec3_t acc_S_1 = imu_data_.accel[k + 1];

    // Time delta
    timestamp_t nexttime;
    if ((k + 1) == imu_data_.size()) {
      nexttime = end;
    } else {
      nexttime = imu_data_.timestamps[k + 1];
    }
    if ((nexttime - time) < 0) {
      continue;
    }
    double dt = ns2sec(nexttime - time);
    // dt = 0.005;
    // printf("dt: %f\n", dt);

    // Interpolate IMU measurements that are beyond the image timestamp
    if (end < nexttime) {
      double interval = ns2sec(nexttime - imu_data_.timestamps[k]);
      nexttime = end;
      dt = ns2sec(nexttime - time);
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    // Check dt is larger than 0
    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    // Interpolate the first IMU measurements
    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / ns2sec(nexttime - imu_data_.timestamps[k]);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }
    // std::cout << "-- " << i << " [" << it->timeStamp << "]" << std::endl;

    // Ensure measurement integrity
    double sigma_g_c = imu_params_.sigma_g_c;
    double sigma_a_c = imu_params_.sigma_a_c;
    // if (fabs(omega_S_0[0]) > imu_params_.g_max ||
    //     fabs(omega_S_0[1]) > imu_params_.g_max ||
    //     fabs(omega_S_0[2]) > imu_params_.g_max ||
    //     fabs(omega_S_1[0]) > imu_params_.g_max ||
    //     fabs(omega_S_1[1]) > imu_params_.g_max ||
    //     fabs(omega_S_1[2]) > imu_params_.g_max) {
    //   sigma_g_c *= 100;
    //   LOG(WARNING) << "gyr saturation";
    // }
    // if (fabs(acc_S_0[0]) > imu_params_.a_max ||
    //     fabs(acc_S_0[1]) > imu_params_.a_max ||
    //     fabs(acc_S_0[2]) > imu_params_.a_max ||
    //     fabs(acc_S_1[0]) > imu_params_.a_max ||
    //     fabs(acc_S_1[1]) > imu_params_.a_max ||
    //     fabs(acc_S_1[2]) > imu_params_.a_max) {
    //   sigma_a_c *= 100;
    //   LOG(WARNING) << "acc saturation";
    // }

    // Actual propagation
    // clang-format off
      // -- Orientation:
      quat_t dq;
      const vec3_t omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - sb.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
      dq.w() = cos_theta_half;
      quat_t Delta_q_1 = Delta_q_ * dq;
      // -- Rotation matrix integral:
      const mat3_t C = Delta_q_.toRotationMatrix();
      const mat3_t C_1 = Delta_q_1.toRotationMatrix();
      const vec3_t acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - sb.segment<3>(6));
      const mat3_t C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
      const vec3_t acc_integral_1 = acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt;
      // -- Rotation matrix double integral:
      C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
      acc_doubleintegral_ += acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;
    // clang-format on

    // Jacobian parts
    // clang-format off
      dalpha_db_g_ += C_1 * rightJacobian(omega_S_true * dt) * dt;
      const mat3_t cross_1 = dq.inverse().toRotationMatrix() * cross_ + rightJacobian(omega_S_true * dt) * dt;
      const mat3_t acc_S_x = skew(acc_S_true);
      mat3_t dv_db_g_1 = dv_db_g_ + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      dp_db_g_ += dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    // clang-format on

    // Covariance propagation
    // clang-format off
      Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
      // Transform
      F_delta.block<3, 3>(0, 3) = -skew(acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
      F_delta.block<3, 3>(0, 6) = I(3) * dt;
      F_delta.block<3, 3>(0, 9) = dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(0, 12) = -C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
      F_delta.block<3, 3>(3, 9) = -dt * C_1;
      F_delta.block<3, 3>(6, 3) = -skew(0.5 * (C + C_1) * acc_S_true * dt);
      F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
      P_delta_ = F_delta * P_delta_ * F_delta.transpose();
      // Add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      // F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta_(3, 3) += sigma2_dalpha;
      P_delta_(4, 4) += sigma2_dalpha;
      P_delta_(5, 5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * sigma_a_c;
      P_delta_(6, 6) += sigma2_v;
      P_delta_(7, 7) += sigma2_v;
      P_delta_(8, 8) += sigma2_v;
      const double sigma2_p = 0.5 * dt * dt * sigma2_v;
      P_delta_(0, 0) += sigma2_p;
      P_delta_(1, 1) += sigma2_p;
      P_delta_(2, 2) += sigma2_p;
      const double sigma2_b_g = dt * imu_params_.sigma_gw_c * imu_params_.sigma_gw_c;
      P_delta_(9, 9) += sigma2_b_g;
      P_delta_(10, 10) += sigma2_b_g;
      P_delta_(11, 11) += sigma2_b_g;
      const double sigma2_b_a = dt * imu_params_.sigma_aw_c * imu_params_.sigma_aw_c;
      P_delta_(12, 12) += sigma2_b_a;
      P_delta_(13, 13) += sigma2_b_a;
      P_delta_(14, 14) += sigma2_b_a;
    // clang-format on

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;
    cross_ = cross_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == end)
      break;
  }

  // store the reference (linearisation) point
  sb_ref_ = sb;

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();
  // proto::print_matrix("P_delta", P_delta_);

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();

  // std::cout << squareRootInformation_ << std::endl;
  // exit(0);

  return i;
}

bool ImuError::Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

bool ImuError::EvaluateWithMinimalJacobians(double const *const *params,
                                            double *residuals,
                                            double **jacobians,
                                            double **jacobiansMinimal) const {
  // Map T_WS_0
  const vec3_t r_WS_0{params[0][0], params[0][1], params[0][2]};
  const quat_t q_WS_0{params[0][6], params[0][3], params[0][4], params[0][5]};
  const mat4_t T_WS_0 = tf(q_WS_0, r_WS_0);

  // Map T_WS_1
  const vec3_t r_WS_1{params[2][0], params[2][1], params[2][2]};
  const quat_t q_WS_1{params[2][6], params[2][3], params[2][4], params[2][5]};
  const mat4_t T_WS_1 = tf(q_WS_1, r_WS_1);

  // Map speed and bias
  vec_t<9> sb0;
  vec_t<9> sb1;
  for (size_t i = 0; i < 9; ++i) {
    sb0(i) = params[1][i];
    sb1(i) = params[3][i];
  }

  // Keep track of evaulation values
  T_WS_0_last_ = T_WS_0;
  T_WS_1_last_ = T_WS_1;
  sb0_last_ = sb0;
  sb1_last_ = sb1;
  timestamp_t time_start = t0_;
  timestamp_t time_end = t1_;

  // this will NOT be changed:
  const mat3_t C_WS_0 = tf_rot(T_WS_0);
  const mat3_t C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double Delta_t = ns2sec(t1_ - t0_);
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_);
    Delta_b = sb0.tail<6>() - sb_ref_.tail<6>();
  }
  int nb_meas = redoPreintegration(T_WS_0, sb0, time_start, time_end);
  bool valid = true;
  if (nb_meas == 0) {
    valid = false;
  }
  Delta_b.setZero();

  // Actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_); // this is a bit
                                                            // stupid, but
                                                            // shared read-locks
                                                            // only come in
                                                            // C++14
                                                            // clang-format off
      const vec3_t g_W = imu_params_.g * vec3_t(0, 0, 6371009).normalized();
                                                            // clang-format on

    // Assign Jacobian w.r.t. x0
    // clang-format off
      Eigen::Matrix<double, 15, 15> F0 = Eigen::Matrix<double, 15, 15>::Identity(); // holds for d/db_g, d/db_a
      const vec3_t delta_p_est_W = tf_trans(T_WS_0) - tf_trans(T_WS_1) + sb0.head<3>() * Delta_t - 0.5 * g_W * Delta_t * Delta_t;
      const vec3_t delta_v_est_W = sb0.head<3>() - sb1.head<3>() - g_W * Delta_t;
      const quat_t Dq = deltaQ(-dalpha_db_g_ * Delta_b.head<3>()) * Delta_q_;
      F0.block<3, 3>(0, 0) = C_S0_W;
      F0.block<3, 3>(0, 3) = C_S0_W * skew(delta_p_est_W);
      F0.block<3, 3>(0, 6) = C_S0_W * I(3) * Delta_t;
      F0.block<3, 3>(0, 9) = dp_db_g_;
      F0.block<3, 3>(0, 12) = -C_doubleintegral_;
      F0.block<3, 3>(3, 3) = (plus(Dq * tf_quat(T_WS_1).inverse()) * oplus(tf_quat(T_WS_0))) .topLeftCorner<3, 3>();
      F0.block<3, 3>(3, 9) = (oplus(tf_quat(T_WS_1).inverse() * tf_quat(T_WS_0)) * oplus(Dq)) .topLeftCorner<3, 3>() * (-dalpha_db_g_);
      F0.block<3, 3>(6, 3) = C_S0_W * skew(delta_v_est_W);
      F0.block<3, 3>(6, 6) = C_S0_W;
      F0.block<3, 3>(6, 9) = dv_db_g_;
      F0.block<3, 3>(6, 12) = -C_integral_;
    // clang-format on

    // assign Jacobian w.r.t. x1
    // clang-format off
      Eigen::Matrix<double, 15, 15> F1 = -Eigen::Matrix<double, 15, 15>::Identity(); // holds for the biases
      F1.block<3, 3>(0, 0) = -C_S0_W;
      F1.block<3, 3>(3, 3) = -(plus(Dq) * oplus(tf_quat(T_WS_0)) * plus(tf_quat(T_WS_1).inverse())) .topLeftCorner<3, 3>();
      F1.block<3, 3>(6, 6) = -C_S0_W;
    // clang-format on

    // Overall error vector
    // clang-format off
      Eigen::Matrix<double, 15, 1> error;
      // -- Position
      error.segment<3>(0) = C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3, 6>(0, 9) * Delta_b;
      // -- Orientation
      error.segment<3>(3) = 2 * (Dq * (tf_quat(T_WS_1).inverse() * tf_quat(T_WS_0))) .vec(); // 2*T_WS_0.q()*Dq*T_WS_1.q().inverse();//
      // -- Velocity
      error.segment<3>(6) = C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3, 6>(6, 9) * Delta_b;
      // -- Biases
      error.tail<6>() = sb0.tail<6>() - sb1.tail<6>();
    // clang-format on

    // Error weighting
    // clang-format off
      Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error(residuals);
      weighted_error = squareRootInformation_ * error;

      // Get the Jacobians
      if (jacobians != NULL) {
        // Sensor pose 0
        // clang-format off
        if (jacobians[0] != NULL) {
          // Jacobian w.r.t. minimal perturbance
          Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_ * F0.block<15, 6>(0, 0);

          Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
          // J0.setZero();
          // J0.block(0, 0, 15, 6) = J0_minimal;
          J0 = J0_minimal * lift_pose_jacobian(T_WS_0);
          if (valid == false) {
            J0.setZero();
          }

          // if requested, provide minimal Jacobians
          if (jacobiansMinimal != NULL) {
            if (jacobiansMinimal[0] != NULL) {
              Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J0_minimal_mapped(jacobiansMinimal[0]);
              J0_minimal_mapped = J0_minimal;
            }
          }
        }
      // clang-format on

      // Speed and Bias 0
      if (jacobians[1] != NULL) {
        // clang-format off
          Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1(jacobians[1]);
          J1.setZero();
          J1 = squareRootInformation_ * F0.block<15, 9>(0, 6);
          if (valid == false) {
            J1.setZero();
          }

          // if requested, provide minimal Jacobians
          if (jacobiansMinimal != NULL) {
            if (jacobiansMinimal[1] != NULL) {
              Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1_minimal_mapped(jacobiansMinimal[1]);
              J1_minimal_mapped = J1;
            }
          }
        }
      // clang-format on

      // Sensor pose 1
      // clang-format off
        if (jacobians[2] != NULL) {
          // Jacobian w.r.t. minimal perturbance
          Eigen::Matrix<double, 15, 6> J2_minimal = squareRootInformation_ * F1.block<15, 6>(0, 0);

          Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J2(jacobians[2]);
          // J2.setZero();
          // J2.block(0, 0, 15, 6) = J2_minimal;
          J2 = J2_minimal * lift_pose_jacobian(T_WS_1);
          if (valid == false) {
            J2.setZero();
          }

          // if requested, provide minimal Jacobians
          if (jacobiansMinimal != NULL) {
            if (jacobiansMinimal[2] != NULL) {
              Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J2_minimal_mapped(jacobiansMinimal[2]);
              J2_minimal_mapped = J2_minimal;
            }
          }
        }
      // clang-format on

      // Speed and Bias 1
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3(
            jacobians[3]);
        J3.setZero();
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);
        if (valid == false) {
          J3.setZero();
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                J3_minimal_mapped(jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }

    } // jacobian
  }   // mutex

  // eval = true;
  return true;
}

} //  namespace yac
