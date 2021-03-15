#ifndef AUTOCAL_CAMERAS_PINHOLECAMERA_HPP
#define AUTOCAL_CAMERAS_PINHOLECAMERA_HPP

#include <vector>
#include <memory>
#include <stdint.h>
#include <Eigen/Core>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include "autocal/cv/CameraBase.hpp"
// #include "autocal/cv/DistortionBase.hpp"
#include "autocal/cv/NoDistortion.hpp"

namespace autocal {
namespace camera {

template <class DISTORTION_T>
class PinholeCamera; // forward declaration

/// \class PinholeCameraBase
/// \brief This is an interface for all the different distortion versions,
/// allowing generic undistortion.
class PinholeCameraBase : public CameraBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PinholeCameraBase() {}

  /// \brief Constructor for width, height and Id
  PinholeCameraBase(int imageWidth, int imageHeight, uint64_t id = 0)
      : CameraBase(imageWidth, imageHeight, id) {}

  /// \brief Destructor.
  virtual ~PinholeCameraBase() {}

  /// \brief Initialise undistort maps to defaults, i.e.
  /// undistortedFocalLengh = 0.5 * (focalLengthU() + focalLengthV()) (same for
  /// U and V),
  /// same image dimensions and center in the middle, i.e
  /// undistortedImageCenterU() = 0.5 * imageWith() + 0.5.
  /// \return True on success.
  virtual bool initialiseUndistortMaps() = 0;

  /// \brief Initialise undistort maps, provide custom parameters for the
  /// undistorted cam.
  /// @param[in] undistortedImageWidth The width in pixels.
  /// @param[in] undistortedImageHeight The height in pixels.
  /// @param[in] undistortedFocalLengthU The horizontal focal length in pixels.
  /// @param[in] undistortedFocalLengthV The vertical focal length in pixels.
  /// @param[in] undistortedImageCenterU The horizontal centre in pixels.
  /// @param[in] undistortedImageCenterV The vertical centre in pixels.
  /// \return True on success.
  virtual bool initialiseUndistortMaps(int undistortedImageWidth,
                                       int undistortedImageHeight,
                                       double undistortedFocalLengthU,
                                       double undistortedFocalLengthV,
                                       double undistortedImageCenterU,
                                       double undistortedImageCenterV) = 0;

  /// \brief Get undistorted image -- assumes initialiseUndistortMaps was called
  /// @param[in] srcImg The distorted input image.
  /// @param[out] dstImg The undistorted output image.
  /// \return True on success.
  virtual bool undistortImage(const cv::Mat &srcImg,
                              cv::Mat &destImg) const = 0;

  /// \brief Get the model of the undistorted camera.
  /// \return The PinholeCamera without distortion associated with the
  /// undistorted image.
  virtual PinholeCamera<NoDistortion> undistortedPinholeCamera() const = 0;

  /// \brief Get the focal length along the u-dimension.
  /// \return The horizontal focal length in pixels.
  virtual double focalLengthU() const = 0;

  /// \brief Get the focal length along the v-dimension.
  /// \return The vertical focal length in pixels.
  virtual double focalLengthV() const = 0;

  /// \brief Get the image centre along the u-dimension.
  /// \return The horizontal centre in pixels.
  virtual double imageCenterU() const = 0;

  /// \brief Get the focal image centre along the v-dimension.
  /// \return The vertical centre in pixels.
  virtual double imageCenterV() const = 0;
};

/// \class PinholeCamera<DISTORTION_T>
/// \brief This implements a standard pinhole camera projection model.
/// \tparam DISTORTION_T the distortion type, e.g.
/// autocal::camera::RadialTangentialDistortion
template <class DISTORTION_T>
class PinholeCamera : public PinholeCameraBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DISTORTION_T distortion_t; ///< Makes the distortion type accessible.

  PinholeCamera() {}

  /// \brief Constructor that will figure out the type of distortion
  /// @param[in] imageWidth The width in pixels.
  /// @param[in] imageHeight The height in pixels.
  /// @param[in] focalLengthU The horizontal focal length in pixels.
  /// @param[in] focalLengthV The vertical focal length in pixels.
  /// @param[in] imageCenterU The horizontal centre in pixels.
  /// @param[in] imageCenterV The vertical centre in pixels.
  /// @param[in] distortion The distortion object to be used.
  /// @param[in] id Assign a generic ID, if desired.
  PinholeCamera(int imageWidth,
                int imageHeight,
                double focalLengthU,
                double focalLengthV,
                double imageCenterU,
                double imageCenterV,
                const distortion_t &distortion,
                uint64_t id = -1);

  /// \brief Destructor.
  virtual ~PinholeCamera() {}

  static const int NumProjectionIntrinsics =
      4; ///< optimisable projection intrinsics
  static const int NumIntrinsics =
      NumProjectionIntrinsics +
      distortion_t::NumDistortionIntrinsics; ///< total number of intrinsics

  /// \brief Get the focal length along the u-dimension.
  /// \return The horizontal focal length in pixels.
  virtual double focalLengthU() const { return fu_; }

  /// \brief Get the focal length along the v-dimension.
  /// \return The vertical focal length in pixels.
  virtual double focalLengthV() const { return fv_; }

  /// \brief Get the image centre along the u-dimension.
  /// \return The horizontal centre in pixels.
  virtual double imageCenterU() const { return cu_; }

  /// \brief Get the focal image centre along the v-dimension.
  /// \return The vertical centre in pixels.
  virtual double imageCenterV() const { return cv_; }

  /// \brief Get the intrinsics as a concatenated vector.
  /// \return The intrinsics as a concatenated vector.
  void getIntrinsics(Eigen::VectorXd &intrinsics) const;

  /// \brief overwrite all intrinsics - use with caution !
  /// \param[in] intrinsics The intrinsics as a concatenated vector.
  bool setIntrinsics(const Eigen::VectorXd &intrinsics);

  /// \brief Get the total number of intrinsics.
  /// \return Number of intrinsics parameters.
  int noIntrinsicsParameters() const { return NumIntrinsics; }

  /// \brief Initialise undistort maps to defaults, i.e.
  /// undistortedFocalLengh = 0.5 * (focalLengthU() + focalLengthV()) (same for
  /// U and V),
  /// same image dimensions and center in the middle, i.e
  /// undistortedImageCenterU() = 0.5 * imageWith() + 0.5.
  /// \return True on success.
  virtual bool initialiseUndistortMaps();

  /// \brief Initialise undistort maps, provide custom parameters for the
  /// undistorted cam.
  /// @param[in] undistortedImageWidth The width in pixels.
  /// @param[in] undistortedImageHeight The height in pixels.
  /// @param[in] undistortedFocalLengthU The horizontal focal length in pixels.
  /// @param[in] undistortedFocalLengthV The vertical focal length in pixels.
  /// @param[in] undistortedImageCenterU The horizontal centre in pixels.
  /// @param[in] undistortedImageCenterV The vertical centre in pixels.
  /// \return True on success.
  virtual bool initialiseUndistortMaps(int undistortedImageWidth,
                                       int undistortedImageHeight,
                                       double undistortedFocalLengthU,
                                       double undistortedFocalLengthV,
                                       double undistortedImageCenterU,
                                       double undistortedImageCenterV);

  /// \brief Get the model of the undistorted camera.
  /// \return The PinholeCamera without distortion associated with the
  /// undistorted image.
  virtual PinholeCamera<NoDistortion> undistortedPinholeCamera() const;

  /// \brief Get undistorted image -- assumes initialiseUndistortMaps was called
  /// @param[in] srcImg The distorted input image.
  /// @param[out] dstImg The undistorted output image.
  /// \return True on success.
  virtual bool undistortImage(const cv::Mat &srcImg, cv::Mat &destImg) const;

  //////////////////////////////////////////////////////////////
  /// \name Methods to project points
  /// @{

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Euclidean coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus project(const Eigen::Vector3d &point,
                                       Eigen::Vector2d *imagePoint) const;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function
  /// w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function
  /// w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus
  project(const Eigen::Vector3d &point,
          Eigen::Vector2d *imagePoint,
          Eigen::Matrix<double, 2, 3> *pointJacobian,
          Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[in]  parameters         The intrinsics.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function
  /// w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function
  /// w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus projectWithExternalParameters(
      const Eigen::Vector3d &point,
      const Eigen::VectorXd &parameters,
      Eigen::Vector2d *imagePoint,
      Eigen::Matrix<double, 2, 3> *pointJacobian,
      Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const;

  /// \brief Projects Euclidean points to 2d image points (projection) in a
  /// batch.
  ///        Uses projection including distortion models.
  /// @param[in]  points      The points in Euclidean coordinates (one point per
  /// column).
  /// @param[out] imagePoints The image points (one point per column).
  /// @param[out] stati       Get information about the success of the
  /// projections. See
  ///                         \ref ProjectionStatus for more information.
  void projectBatch(const Eigen::Matrix3Xd &points,
                    Eigen::Matrix2Xd *imagePoints,
                    std::vector<CameraBase::ProjectionStatus> *stati) const;

  /// \brief Projects a point in homogenous coordinates to a 2d image point
  /// (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Homogeneous coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus projectHomogeneous(
      const Eigen::Vector4d &point, Eigen::Vector2d *imagePoint) const;

  /// \brief Projects a point in homogenous coordinates to a 2d image point
  /// (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Homogeneous coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function
  /// w.r.t. the point.
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function
  /// w.r.t. the intrinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus
  projectHomogeneous(const Eigen::Vector4d &point,
                     Eigen::Vector2d *imagePoint,
                     Eigen::Matrix<double, 2, 4> *pointJacobian,
                     Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const;

  /// \brief Projects a point in homogenous coordinates to a 2d image point
  /// (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Homogeneous coordinates.
  /// @param[in]  parameters         The intrinsics.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function
  /// w.r.t. the point.
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function
  /// w.r.t. the intrinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  CameraBase::ProjectionStatus projectHomogeneousWithExternalParameters(
      const Eigen::Vector4d &point,
      const Eigen::VectorXd &parameters,
      Eigen::Vector2d *imagePoint,
      Eigen::Matrix<double, 2, 4> *pointJacobian = NULL,
      Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const;

  /// \brief Projects points in homogenous coordinates to 2d image points
  /// (projection) in a batch.
  ///        Uses projection including distortion models.
  /// @param[in]  points      The points in homogeneous coordinates (one point
  /// per column).
  /// @param[out] imagePoints The image points (one point per column).
  /// @param[out] stati       Get information about the success of the
  /// projections. See
  ///                         \ref ProjectionStatus for more information.
  void projectHomogeneousBatch(
      const Eigen::Matrix4Xd &points,
      Eigen::Matrix2Xd *imagePoints,
      std::vector<CameraBase::ProjectionStatus> *stati) const;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to backproject points
  /// @{

  /// \brief Back-project a 2d image point into Euclidean space (direction
  /// vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The Euclidean direction vector.
  /// @return     true on success.
  bool backProject(const Eigen::Vector2d &imagePoint,
                   Eigen::Vector3d *direction) const;

  /// \brief Back-project a 2d image point into Euclidean space (direction
  /// vector).
  /// @param[in]  imagePoint         The image point.
  /// @param[out] direction          The Euclidean direction vector.
  /// @param[out] pointJacobian      Jacobian of the back-projection function
  /// w.r.t. the point.
  /// @return     true on success.
  bool backProject(const Eigen::Vector2d &imagePoint,
                   Eigen::Vector3d *direction,
                   Eigen::Matrix<double, 3, 2> *pointJacobian) const;

  /// \brief Back-project 2d image points into Euclidean space (direction
  /// vectors).
  /// @param[in]  imagePoints The image points (one point per column).
  /// @param[out] directions  The Euclidean direction vectors (one point per
  /// column).
  /// @param[out] success     Success of each of the back-projection
  bool backProjectBatch(const Eigen::Matrix2Xd &imagePoints,
                        Eigen::Matrix3Xd *directions,
                        std::vector<bool> *success) const;

  /// \brief Back-project a 2d image point into homogeneous point (direction
  /// vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The homogeneous point as direction vector.
  /// @return     true on success.
  bool backProjectHomogeneous(const Eigen::Vector2d &imagePoint,
                              Eigen::Vector4d *direction) const;

  /// \brief Back-project a 2d image point into homogeneous point (direction
  /// vector).
  /// @param[in]  imagePoint         The image point.
  /// @param[out] direction          The homogeneous point as direction vector.
  /// @param[out] pointJacobian      Jacobian of the back-projection function.
  /// @return     true on success.
  bool backProjectHomogeneous(const Eigen::Vector2d &imagePoint,
                              Eigen::Vector4d *direction,
                              Eigen::Matrix<double, 4, 2> *pointJacobian) const;

  /// \brief Back-project 2d image points into homogeneous points (direction
  /// vectors).
  /// @param[in]  imagePoints The image points (one point per column).
  /// @param[out] directions  The homogeneous points as direction vectors (one
  /// point per column).
  /// @param[out] success     Success of each of the back-projection
  bool backProjectHomogeneousBatch(const Eigen::Matrix2Xd &imagePoints,
                                   Eigen::Matrix4Xd *directions,
                                   std::vector<bool> *success) const;
  /// @}

  /// \brief get a test instance
  static std::shared_ptr<CameraBase> createTestObject() {
    return std::shared_ptr<CameraBase>(
        new PinholeCamera(752,
                          480,
                          350,
                          360,
                          378,
                          238,
                          distortion_t::testObject()));
  }
  /// \brief get a test instance
  static PinholeCamera testObject() {
    return PinholeCamera(752,
                         480,
                         350,
                         360,
                         378,
                         238,
                         distortion_t::testObject());
  }

  /// \brief Obtain the projection type
  std::string type() const {
    return "PinholeCamera<" + distortion_.type() + ">";
  }

  /// \brief Obtain the projection type
  const std::string distortionType() const { return distortion_.type(); }

  /// \brief No default constructor.
  // PinholeCamera() = delete;

  distortion_t distortion_; ///< the distortion to be used

  Eigen::Matrix<double, NumIntrinsics, 1>
      intrinsics_;     ///< summary of all intrinsics parameters
  double fu_;          ///< focalLengthU
  double fv_;          ///< focalLengthV
  double cu_;          ///< imageCenterU
  double cv_;          ///< imageCenterV
  double one_over_fu_; ///< 1.0 / fu_
  double one_over_fv_; ///< 1.0 / fv_
  double fu_over_fv_;  ///< fu_ / fv_

  cv::Mat map_x_fast_; ///< OpenCV undistort fast map x-coordinates
  cv::Mat map_y_fast_; ///< OpenCV undistort fast map x-coordinates

  int undistortedImageWidth_ =
      0; ///< undistortedImageWidth The width in pixels.
  int undistortedImageHeight_ =
      0; ///< undistortedImageHeight The height in pixels.
  double undistortedFocalLengthU_ =
      0.0; ///< undistortedFocalLengthU The horizontal focal length in pixels.
  double undistortedFocalLengthV_ =
      0.0; ///< undistortedFocalLengthV The vertical focal length in pixels.
  double undistortedImageCenterU_ =
      0.0; ///< undistortedImageCenterU The horizontal centre in pixels.
  double undistortedImageCenterV_ =
      0.0; ///< undistortedImageCenterV The vertical centre in pixels.
};

} // namespace camera
} // namespace autocal
#include "PinholeCameraImpl.hpp"
#endif /* AUTOCAL_CAMERAS_PINHOLECAMERA_HPP */
