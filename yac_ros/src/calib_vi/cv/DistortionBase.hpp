#ifndef YAC_CV_DISTORTIONBASE_HPP
#define YAC_CV_DISTORTIONBASE_HPP

#include <Eigen/Core>

namespace yac {
namespace camera {

/// \brief Base class for all distortion models.
class DistortionBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Destructor -- not doing anything
  virtual ~DistortionBase() {}

  //////////////////////////////////////////////////////////////
  /// \name Methods related to generic parameters
  /// @{

  /// \brief set the generic parameters
  /// @param[in] parameters Parameter vector -- length must correspond
  /// numDistortionIntrinsics().
  /// @return    True if the requirements were followed.
  virtual bool setParameters(const Eigen::VectorXd &parameters) = 0;

  /// \brief Obtain the generic parameters.
  virtual bool getParameters(Eigen::VectorXd &parameters) const = 0;

  /// \brief The derived class type.
  virtual std::string type() const = 0;

  /// \brief Number of derived class distortion parameters
  virtual int numDistortionIntrinsics() const = 0;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Distortion functions
  /// @{

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  virtual bool distort(const Eigen::Vector2d &pointUndistorted,
                       Eigen::Vector2d *pointDistorted) const = 0;

  /// \brief Distortion and Jacobians.
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image
  /// point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the
  /// intrinsics vector.
  /// @return     True on success (no singularity)
  virtual bool distort(const Eigen::Vector2d &pointUndistorted,
                       Eigen::Vector2d *pointDistorted,
                       Eigen::Matrix2d *pointJacobian,
                       Eigen::Matrix2Xd *parameterJacobian = NULL) const = 0;

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
  virtual bool distortWithExternalParameters(
      const Eigen::Vector2d &pointUndistorted,
      const Eigen::VectorXd &parameters,
      Eigen::Vector2d *pointDistorted,
      Eigen::Matrix2d *pointJacobian = NULL,
      Eigen::Matrix2Xd *parameterJacobian = NULL) const = 0;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistortion functions
  /// @{

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @return     True on success (no singularity)
  virtual bool undistort(const Eigen::Vector2d &pointDistorted,
                         Eigen::Vector2d *pointUndistorted) const = 0;

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointJacobian    The Jacobian w.r.t. changes on the image
  /// point.
  /// @return     True on success (no singularity)
  virtual bool undistort(const Eigen::Vector2d &pointDistorted,
                         Eigen::Vector2d *pointUndistorted,
                         Eigen::Matrix2d *pointJacobian) const = 0;
  /// @}
};

} // namespace camera
} // namespace yac

#endif /* YAC_CV_DISTORTIONBASE_HPP */
