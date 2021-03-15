#ifndef AUTOCAL_PARAMETERS_HPP
#define AUTOCAL_PARAMETERS_HPP

#include <deque>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Dense>
#include "./Duration.hpp"
#include "./Transformation.hpp"

namespace autocal {

/**
 * Properties of an IMU
 */
struct ImuParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  autocal::Transformation T_BS;  ///< Transformation from Body frame to IMU (sensor frame S).
  double a_max;     ///< Accelerometer saturation. [m/s^2]
  double g_max;     ///< Gyroscope saturation. [rad/s]
  double sigma_g_c; ///< Gyroscope noise density.
  double sigma_bg;  ///< Initial gyroscope bias.
  double sigma_a_c; ///< Accelerometer noise density.
  double sigma_ba;  ///< Initial accelerometer bias
  double sigma_gw_c;  ///< Gyroscope drift noise density.
  double sigma_aw_c;  ///< Accelerometer drift noise density.
  double tau;         ///< Reversion time constant of accerometer bias. [s]
  double g;           ///< Earth acceleration.
  Eigen::Vector3d a0; ///< Mean of the prior accelerometer bias.
  int rate;           ///< IMU rate in Hz.
};

} // namespace autocal
#endif // AUTOCAL_PARAMETERS_HPP
