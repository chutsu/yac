#ifndef YAC_VARIABLES_HPP
#define YAC_VARIABLES_HPP

#include <Eigen/Core>

namespace yac {

typedef Eigen::Matrix<double, 9, 1> SpeedAndBiases;
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;
typedef Eigen::Matrix<double, 3, 1> Speed;
typedef Eigen::Matrix<double, 3, 1> GyroBias;
typedef Eigen::Matrix<double, 3, 1> AccBias;
typedef Eigen::Matrix<double, 6, 1> ImuBias;
typedef Eigen::Matrix<double, 3, 1> MagnetometerBias;
typedef Eigen::Matrix<double, 1, 1> MagnetometerWorldZBias;
typedef Eigen::Matrix<double, 3, 1> Wind;
typedef Eigen::Matrix<double, 1, 1> Qff;

} // namespace yac
#endif /* YAC_VARIABLES_HPP */
