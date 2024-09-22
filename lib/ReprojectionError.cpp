#include "ReprojectionError.hpp"

namespace yac {

ReprojectionError::ReprojectionError(
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const vec2_t &z,
    const mat2_t &covar)
    : ResidualBlock{"ReprojectionError", param_ptrs, param_types, 2},
      camera_geometry_{camera_geometry}, z_{z}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<ReprojectionError>
ReprojectionError::create(const std::shared_ptr<CameraGeometry> &camera,
                          double *T_C0F,
                          double *p_FFi,
                          const vec2_t &z,
                          const mat2_t &covar) {
  std::vector<double *> param_ptrs = {p_FFi,
                                      T_C0F,
                                      camera->getExtrinsicPtr(),
                                      camera->getIntrinsicPtr()};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POINT,
                                               ParamBlock::POSE,
                                               ParamBlock::EXTRINSIC,
                                               ParamBlock::INTRINSIC8};

  return std::make_shared<ReprojectionError>(camera,
                                             param_ptrs,
                                             param_types,
                                             z,
                                             covar);
}

bool ReprojectionError::getResiduals(vec2_t &r) const {
  r = residuals_;
  return valid_;
}

bool ReprojectionError::getReprojError(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool ReprojectionError::EvaluateWithMinimalJacobians(
    double const *const *params,
    double *res,
    double **jacs,
    double **min_jacs) const {
  // Map parameters out
  Eigen::Map<const vec3_t> p_FFi(params[0], 3);
  const mat4_t T_C0F = tf(params[1]);
  const mat4_t T_C0Ci = tf(params[2]);
  Eigen::Map<const vecx_t> intrinsic(params[3], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const mat4_t T_CiC0 = tf_inv(T_C0Ci);
  const mat4_t T_CiF = T_CiC0 * T_C0F;
  const vec3_t p_Ci = tf_point(T_CiF, p_FFi);
  // -- Project point from camera frame to image plane
  const auto camera_model = camera_geometry_->getCameraModel();
  const vec2i_t resolution = camera_geometry_->getResolution();

  vec2_t z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
  }

  // Residual
  Eigen::Map<vec2_t> r(res);
  r = sqrt_info_ * (z_ - z_hat);
  residuals_ = z_ - z_hat;

  // Jacobians
  const matx_t Jh = camera_model->projectJacobian(intrinsic, p_Ci);
  const matx_t Jh_weighted = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr) {
    return true;
  }

  // Jacobians w.r.t p_FFi
  if (jacs[0]) {
    const mat3_t C_CiF = tf_rot(T_CiF);
    matx_t J_min = Jh_weighted * C_CiF;

    Eigen::Map<mat_t<2, 3, row_major_t>> J(jacs[0]);
    J = (valid_) ? J_min : zeros(2, 3);

    if (min_jacs && min_jacs[0]) {
      Eigen::Map<mat_t<2, 3, row_major_t>> min_J(min_jacs[0]);
      min_J = (valid_) ? J_min : zeros(2, 3);
    }
  }

  // Jacobians w.r.t T_C0F
  if (jacs[1]) {
    // clang-format off
      const mat3_t C_CiC0 = tf_rot(T_CiC0);
      const mat3_t C_C0F = tf_rot(T_C0F);
      matx_t J_min = zeros(2, 6);
      J_min.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
      J_min.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -C_C0F * skew(p_FFi);
    // clang-format on

    Eigen::Map<mat_t<2, 7, row_major_t>> J(jacs[1]);
    J.setZero();
    if (valid_) {
      J.block<2, 6>(0, 0) = J_min;
    }

    if (min_jacs && min_jacs[1]) {
      Eigen::Map<mat_t<2, 6, row_major_t>> min_J(min_jacs[1]);
      min_J = (valid_) ? J_min : zeros(2, 6);
    }
  }

  // Jacobians w.r.t T_C0Ci
  if (jacs[2]) {
    // clang-format off
      const mat3_t C_C0Ci = tf_rot(T_C0Ci);
      const vec3_t p_C0Fi = tf_point(T_C0F, p_FFi);
      const vec3_t p_C0Ci = tf_trans(T_C0Ci);
      matx_t J_min = zeros(2, 6);
      J_min.block(0, 0, 2, 3) = Jh_weighted * -C_C0Ci;
      J_min.block(0, 3, 2, 3) = Jh_weighted * -C_C0Ci * skew(p_C0Fi - p_C0Ci) * -C_C0Ci;
    // clang-format on

    Eigen::Map<mat_t<2, 7, row_major_t>> J(jacs[2]);
    J.setZero();
    if (valid_) {
      J.block<2, 6>(0, 0) = J_min;
    }

    if (min_jacs && min_jacs[2]) {
      Eigen::Map<mat_t<2, 6, row_major_t>> min_J(min_jacs[2]);
      min_J = (valid_) ? J_min : zeros(2, 6);
    }
  }

  // Jacobians w.r.t intrinsic
  if (jacs[3]) {
    Eigen::Map<mat_t<2, 8, row_major_t>> J(jacs[3]);
    const matx_t J_cam = camera_model->paramsJacobian(intrinsic, p_Ci);
    const matx_t J_min = -1 * sqrt_info_ * J_cam;
    J = (valid_) ? J_min : zeros(2, 8);

    if (min_jacs && min_jacs[3]) {
      Eigen::Map<mat_t<2, 8, row_major_t>> min_J(min_jacs[3]);
      min_J = (valid_) ? J_min : zeros(2, 8);
    }
  }

  return true;
}

} // namespace yac
