#ifndef YAC_CV_EQUIDISTANTDISTORTION_HPP
#define YAC_CV_EQUIDISTANTDISTORTION_HPP

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "DistortionBase.hpp"

namespace yac {
namespace camera {

/// \brief Implements equidistant distortion.
class EquidistantDistortion : public DistortionBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The default constructor with all zero ki
  EquidistantDistortion();

  /// \brief Constructor initialising ki
  /// @param[in] k1 equidistant parameter 1
  /// @param[in] k2 equidistant parameter 2
  /// @param[in] k3 equidistant parameter 3
  /// @param[in] k4 equidistant parameter 4
  EquidistantDistortion(double k1, double k2, double k3, double k4);
  //////////////////////////////////////////////////////////////
  /// \name Methods related to generic parameters
  /// @{

  /// \brief set the generic parameters
  /// @param[in] parameters Parameter vector -- length must correspond
  /// numDistortionIntrinsics().
  /// @return    True if the requirements were followed.
  bool setParameters(const Eigen::VectorXd &parameters);

  /// \brief Obtain the generic parameters.
  bool getParameters(Eigen::VectorXd &parameters) const {
    parameters = parameters_;
    return true;
  }
  /// \brief The class type.
  std::string type() const { return "EquidistantDistortion"; }

  /// \brief Number of distortion parameters.
  int numDistortionIntrinsics() const { return NumDistortionIntrinsics; }

  static const int NumDistortionIntrinsics =
      4; ///< The Number of distortion parameters.
  /// @}

  /// \brief Unit test support -- create a test distortion object
  static std::shared_ptr<DistortionBase> createTestObject() {
    return std::shared_ptr<DistortionBase>(
        new EquidistantDistortion(-0.21, 0.14, 0.0006, 0.0003));
  }
  /// \brief Unit test support -- create a test distortion object
  static EquidistantDistortion testObject() {
    return EquidistantDistortion(-0.21, 0.14, 0.0006, 0.0003);
  }

  //////////////////////////////////////////////////////////////
  /// \name Distortion functions
  /// @{

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  bool distort(const Eigen::Vector2d &pointUndistorted,
               Eigen::Vector2d *pointDistorted) const;

  /// \brief Distortion and Jacobians.
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image
  /// point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the
  /// intrinsics vector.
  /// @return     True on success (no singularity)
  bool distort(const Eigen::Vector2d &pointUndistorted,
               Eigen::Vector2d *pointDistorted,
               Eigen::Matrix2d *pointJacobian,
               Eigen::Matrix2Xd *parameterJacobian = NULL) const;

  /// \brief Distortion and Jacobians using external distortion intrinsics
  /// parameters.
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[in]  parameters        The distortion intrinsics vector.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image
  /// point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the
  /// intrinsics vector.
  /// @return     True on success (no singularity)
  bool distortWithExternalParameters(
      const Eigen::Vector2d &pointUndistorted,
      const Eigen::VectorXd &parameters,
      Eigen::Vector2d *pointDistorted,
      Eigen::Matrix2d *pointJacobian = NULL,
      Eigen::Matrix2Xd *parameterJacobian = NULL) const;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistortion functions
  /// @{

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @return     True on success (no singularity)
  bool undistort(const Eigen::Vector2d &pointDistorted,
                 Eigen::Vector2d *pointUndistorted) const;

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointJacobian    The Jacobian w.r.t. changes on the image
  /// point.
  /// @return     True on success (no singularity)
  bool undistort(const Eigen::Vector2d &pointDistorted,
                 Eigen::Vector2d *pointUndistorted,
                 Eigen::Matrix2d *pointJacobian) const;
  /// @}

protected:
  Eigen::Matrix<double, NumDistortionIntrinsics, 1>
      parameters_; ///< all distortion parameters

  double k1_; ///< radial parameter 1
  double k2_; ///< radial parameter 2
  double k3_; ///< tangential parameter 1
  double k4_; ///< tangential parameter 2
};

} // namespace camera
} // namespace yac
#endif /* YAC_CV_EQUIDISTANTDISTORTION_HPP */
