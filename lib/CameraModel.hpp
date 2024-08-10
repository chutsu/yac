#pragma once

#include "Core.hpp"

namespace yac {

/********************************* RADTAN4 ************************************/

vec2_t radtan4_distort(const vec4_t &dist_params, const vec2_t &p);
vec2_t radtan4_undistort(const vec4_t &dist_params, const vec2_t &p0);
mat2_t radtan4_point_jacobian(const vec4_t &dist_params, const vec2_t &p);
matx_t radtan4_paramsJacobian(const vec4_t &dist_params, const vec2_t &p);

/********************************** EQUI4 *************************************/

vec2_t equi4_distort(const vec4_t &dist_params, const vec2_t &p);
vec2_t equi4_undistort(const vec4_t &dist_params, const vec2_t &p0);
mat2_t equi4_point_jacobian(const vec4_t &dist_params, const vec2_t &p);
matx_t equi4_paramsJacobian(const vec4_t &dist_params, const vec2_t &p);

/********************************* PROJECT ************************************/

vec2_t project_point(const vec3_t &p_C);
mat_t<2, 3> projectJacobian(const vec3_t &p_C);

/********************************* PINHOLE ************************************/

double pinhole_focal(const int image_size, const double fov);
mat3_t
pinhole_K(const double fx, const double fy, const double cx, const double cy);
int pinhole_project(const int res[2],
                    const vec4_t &proj_params,
                    const vec3_t &p,
                    vec2_t &z_hat);
mat2_t pinhole_point_jacobian(const vec4_t &proj_params);
mat_t<2, 4> pinhole_paramsJacobian(const vec4_t &proj_params, const vec2_t &p);

/****************************** PINHOLE-RADTAN4 *******************************/

int pinhole_radtan4_project(const int res[2],
                            const vecx_t &params,
                            const vec3_t &p_C,
                            vec2_t &z_hat);
matx_t pinhole_radtan4_projectJacobian(const vecx_t &params, const vec3_t &p_C);
matx_t pinhole_radtan4_paramsJacobian(const vecx_t &params, const vec3_t &p_C);
int pinhole_radtan4_backProject(const vecx_t &params,
                                const vec2_t &x,
                                vec3_t &ray);
vec2_t pinhole_radtan4_undistort(const vecx_t &params, const vec2_t &z);

/******************************* PINHOLE-EQUI4 ********************************/

int pinhole_equi4_project(const int res[2],
                          const vecx_t &params,
                          const vec3_t &p_C,
                          vec2_t &z_hat);
matx_t pinhole_equi4_projectJacobian(const vecx_t &params, const vec3_t &p_C);
matx_t pinhole_equi4_paramsJacobian(const vecx_t &params, const vec3_t &p_C);
int pinhole_equi4_backProject(const vecx_t &params,
                              const vec2_t &x,
                              vec3_t &ray);
vec2_t pinhole_equi4_undistort(const vecx_t &params, const vec2_t &z);

/****************************** CAMERA GEOMETRY *******************************/

struct CameraModel {
  std::string type;

  CameraModel(const std::string &type_) : type{type_} {}
  virtual ~CameraModel() = default;

  virtual int project(const int res[2],
                      const vecx_t &params,
                      const vec3_t &p_C,
                      vec2_t &z_hat) const = 0;

  virtual matx_t projectJacobian(const vecx_t &params,
                                 const vec3_t &p_C) const = 0;

  virtual matx_t paramsJacobian(const vecx_t &params,
                                const vec3_t &p_C) const = 0;

  virtual int backProject(const vecx_t &params,
                          const vec2_t &x,
                          vec3_t &ray) const = 0;

  virtual vec2_t undistort(const vecx_t &params, const vec2_t &z) const = 0;
};

struct PinholeRadtan4 : CameraModel {
  PinholeRadtan4() : CameraModel{"pinhole-radtan4"} {}

  int project(const int res[2],
              const vecx_t &params,
              const vec3_t &p_C,
              vec2_t &z_hat) const override {
    return pinhole_radtan4_project(res, params, p_C, z_hat);
  }

  matx_t projectJacobian(const vecx_t &params,
                         const vec3_t &p_C) const override {
    return pinhole_radtan4_projectJacobian(params, p_C);
  }

  matx_t paramsJacobian(const vecx_t &params,
                        const vec3_t &p_C) const override {
    return pinhole_radtan4_paramsJacobian(params, p_C);
  }

  int backProject(const vecx_t &params,
                  const vec2_t &x,
                  vec3_t &ray) const override {
    return pinhole_radtan4_backProject(params, x, ray);
  }

  vec2_t undistort(const vecx_t &params, const vec2_t &z) const override {
    return pinhole_radtan4_undistort(params, z);
  }
};

struct PinholeEqui4 : CameraModel {
  PinholeEqui4() : CameraModel{"pinhole-equi4"} {}

  int project(const int res[2],
              const vecx_t &params,
              const vec3_t &p_C,
              vec2_t &z_hat) const override {
    return pinhole_equi4_project(res, params, p_C, z_hat);
  }

  matx_t projectJacobian(const vecx_t &params,
                         const vec3_t &p_C) const override {
    return pinhole_equi4_projectJacobian(params, p_C);
  }

  matx_t paramsJacobian(const vecx_t &params,
                        const vec3_t &p_C) const override {
    return pinhole_equi4_paramsJacobian(params, p_C);
  }

  int backProject(const vecx_t &params,
                  const vec2_t &x,
                  vec3_t &ray) const override {
    return pinhole_equi4_backProject(params, x, ray);
  }

  vec2_t undistort(const vecx_t &params, const vec2_t &z) const override {
    return pinhole_equi4_undistort(params, z);
  }
};

/*********************************** MISC *************************************/

int solvepnp(const CameraModel *cam,
             const int cam_res[2],
             const vecx_t &cam_params,
             const vec2s_t &keypoints,
             const vec3s_t &object_points,
             mat4_t &T_CF);

} // namespace yac
