#ifndef YAC_CAMERA_GEOMETRY_HPP
#define YAC_CAMERA_GEOMETRY_HPP

#include "PinholeCamera.hpp"
#include "RadialTangentialDistortion.hpp"
#include "EquidistantDistortion.hpp"

typedef yac::camera::PinholeCamera<yac::camera::RadialTangentialDistortion> PinholeRadtan;
typedef yac::camera::PinholeCamera<yac::camera::EquidistantDistortion> PinholeEqui;

#endif // YAC_CAMERA_GEOMETRY_HPP
