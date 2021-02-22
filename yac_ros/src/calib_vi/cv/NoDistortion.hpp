#ifndef YAC_CV_NODISTORTION_HPP
#define YAC_CV_NODISTORTION_HPP

#include <memory>
#include <Eigen/Core>
#include "DistortionBase.hpp"

namespace yac {
namespace camera {

/// \class NoDistortion
/// \brief This trivially doesn't do anything in terms of distortion.
/// This is useful for testing, or working with pre-undistorted images.
class NoDistortion : public DistortionBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Destructor, not doing anything
  ~NoDistortion() {}

  //////////////////////////////////////////////////////////////
  /// \name Methods related to generic parameters
  /// @{

  /// \brief set the generic parameters
  /// @param[in] parameters Parameter vector -- length must correspond
  /// numDistortionIntrinsics().
  /// @return    True if the requirements were followed.
  bool setParameters(const Eigen::VectorXd &parameters) {
    (void) parameters;
    return true;
  }

  /// \brief Obtain the generic parameters.
  bool getParameters(Eigen::VectorXd &parameters) const {
    parameters.resize(0);
    return true;
  }

  /// \brief The class type.
  std::string type() const { return "NoDistortion"; }

  /// \brief Number of derived class distortion parameters
  int numDistortionIntrinsics() const { return 0; }

  static const int NumDistortionIntrinsics =
      0; ///< THis class does not contain any distortion intrinsics.
  /// @}

  /// \brief Unit test support -- create a test distortion object
  static std::shared_ptr<DistortionBase> createTestObject() {
    return std::shared_ptr<DistortionBase>(new NoDistortion());
  }
  /// \brief Unit test support -- create a test distortion object
  static NoDistortion testObject() { return NoDistortion(); }

  //////////////////////////////////////////////////////////////
  /// \name Distortion functions
  /// @{

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  bool distort(const Eigen::Vector2d &pointUndistorted,
               Eigen::Vector2d *pointDistorted) const {
    *pointDistorted = pointUndistorted;
    return true;
  }

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
               Eigen::Matrix2Xd *parameterJacobian = NULL) const {
    *pointDistorted = pointUndistorted;
    *pointJacobian = Eigen::Matrix2d::Identity();
    if (parameterJacobian) {
      parameterJacobian->resize(2, 0);
    }
    return true;
  }

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
      Eigen::Matrix2Xd *parameterJacobian = NULL) const {
    (void) parameters;
    *pointDistorted = pointUndistorted;
    if (pointJacobian) {
      *pointJacobian = Eigen::Matrix2d::Identity();
    }
    if (parameterJacobian) {
      parameterJacobian->resize(2, 0);
    }
    return true;
  }
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistortion functions
  /// @{

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @return     True on success (no singularity)
  bool undistort(const Eigen::Vector2d &pointDistorted,
                 Eigen::Vector2d *pointUndistorted) const {
    *pointUndistorted = pointDistorted;
    return true;
  }

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointJacobian    The Jacobian w.r.t. changes on the image
  /// point.
  /// @return     True on success (no singularity)
  bool undistort(const Eigen::Vector2d &pointDistorted,
                 Eigen::Vector2d *pointUndistorted,
                 Eigen::Matrix2d *pointJacobian) const {
    *pointUndistorted = pointDistorted;
    *pointJacobian = Eigen::Matrix2d::Identity();
    return true;
  }
  /// @}
};

} // namespace camera
} // namespace yac
#endif /* AUTOCAL_CAMERAS_NODISTORTION_HPP */
