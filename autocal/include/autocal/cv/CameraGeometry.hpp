#ifndef AUTOCAL_CV_CAMERA_GEOMETRY_HPP
#define AUTOCAL_CV_CAMERA_GEOMETRY_HPP

#include "autocal/cv/PinholeCamera.hpp"
#include "autocal/cv/RadialTangentialDistortion.hpp"
#include "autocal/cv/EquidistantDistortion.hpp"

typedef autocal::camera::PinholeCamera<autocal::camera::RadialTangentialDistortion> PinholeRadtan;
typedef autocal::camera::PinholeCamera<autocal::camera::EquidistantDistortion> PinholeEqui;

#endif // AUTOCAL_CV_CAMERA_GEOMETRY_HPP
