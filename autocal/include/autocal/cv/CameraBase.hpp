#ifndef AUTOCAL_CAMEREA_CAMERA_BASE_HPP
#define AUTOCAL_CAMEREA_CAMERA_BASE_HPP

#include <vector>
#include <memory>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
// #include "autocal/cv/DistortionBase.hpp"

namespace autocal {

/**
 * Base class for all camera models.
 */
class CameraBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Indicates what happened when applying any of the project functions.
   */
  enum class ProjectionStatus {
    Successful,
    OutsideImage,
    Masked,
    Behind,
    Invalid
  };

  CameraBase() : imageWidth_(0), imageHeight_(0), id_(0) {}
  CameraBase(int imageWidth, int imageHeight, uint64_t id = 0)
      : imageWidth_(imageWidth), imageHeight_(imageHeight), id_(id) {}
  virtual ~CameraBase() {}

  /****************************************************************************
   * @name Masking
   * @{
   ***************************************************************************/
  /**
   * Set the mask. It must be the same size as the image and comply with
   * OpenCV: 0 == masked, nonzero == valid.  Type must be CV_8U1C.
   *
   * @param mask The actual mask.
   * @return True if the requirements were followed.
   */
  bool setMask(const cv::Mat &mask);

  /**
   * Using a nonzero mask?
   */
  bool hasMask() const;

  /**
   * Remove mask
   */
  bool removeMask();

  /**
   * Return the mask
   */
  const cv::Mat &mask() const;
  /****************************************************************************
   * @} End of Masking
   ***************************************************************************/

  /****************************************************************************
   * @name Camera id
   * @{
   ***************************************************************************/
  /**
   * Set camera id
   */
  void setId(uint64_t id) { id_ = id; }

  /**
   * Obtain camera id
   */
  uint64_t id() const { return id_; }
  /****************************************************************************
   * @} End of Camera id
   ***************************************************************************/

  /**
   * Return of the image width in pixels
   */
  uint32_t imageWidth() const { return imageWidth_; }

  /**
   * Return of the image height in pixels
   */
  uint32_t imageHeight() const { return imageHeight_; }

  /**
   * Get intrinsics
   */
  virtual void getIntrinsics(Eigen::VectorXd &intrinsics) const = 0;

  /**
   * Set intrinsics
   */
  virtual bool setIntrinsics(const Eigen::VectorXd &intrinsics) = 0;

  /****************************************************************************
   * @name Project Points
   * @{
   ***************************************************************************/
  /**
   * Projects a Euclidean point to a 2d image point (projection). Uses
   * projection including distortion models.
   *
   * @param point Point in Euclidean coordinates.
   * @param imagePoint Image point.
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus project(const Eigen::Vector3d &point,
                                   Eigen::Vector2d *imagePoint) const = 0;

  /**
   * Projects a Euclidean point to a 2d image point (projection). Uses
   * projection including distortion models.
   *
   * @param point Point in Euclidean coordinates
   * @param imagePoint Image point
   * @param pointJacobian Jacobian of the projection function w.r.t. the point
   * @param intrinsicsJacobian Jacobian of the projection function w.r.t. the
   * intinsics.
   *
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus
  project(const Eigen::Vector3d &point,
          Eigen::Vector2d *imagePoint,
          Eigen::Matrix<double, 2, 3> *pointJacobian,
          Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const = 0;

  /**
   * Projects a Euclidean point to a 2d image point (projection). Uses
   * projection including distortion models.
   *
   * @param point Point in Euclidean coordinates
   * @param intrinsics Camera intrinsics
   * @param imagePoint Image point
   * @param pointJacobian Jacobian of the projection function w.r.t. the point
   * @param intrinsicsJacobian The Jacobian of the projection function w.r.t.
   * the intinsics
   *
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus projectWithExternalParameters(
      const Eigen::Vector3d &point,
      const Eigen::VectorXd &intrinsics,
      Eigen::Vector2d *imagePoint,
      Eigen::Matrix<double, 2, 3> *pointJacobian = NULL,
      Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const = 0;

  /**
   * Projects a bunch of Euclidean points to 2d image points (projection)
   *
   * Uses projection including distortion models.
   * @param points Points in Euclidean coordinates (one point per column).
   * @param imagePoints Image points (one point per column).
   * @param status Projection status. See \ref ProjectionStatus for more
   * information.
   */
  virtual void projectBatch(const Eigen::Matrix3Xd &points,
                            Eigen::Matrix2Xd *imagePoints,
                            std::vector<ProjectionStatus> *stati) const = 0;

  /**
   * Projects a point in homogenous coordinates to a 2d image point
   * (projection). Uses projection including distortion models.
   *
   * @param point Point in Homogeneous coordinates
   * @param imagePoint Image point
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus projectHomogeneous(
      const Eigen::Vector4d &point, Eigen::Vector2d *imagePoint) const = 0;

  /**
   * Projects a point in homogenous coordinates to a 2d image point
   * (projection). Uses projection including distortion models.
   *
   * @param point Point in Homogeneous coordinates
   * @param imagePoint Image point
   * @param pointJacobian Jacobian of the projection function w.r.t. the point
   * @param intrinsicsJacobian Jacobian of the projection function w.r.t. the
   * intrinsics.
   *
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus
  projectHomogeneous(const Eigen::Vector4d &point,
                     Eigen::Vector2d *imagePoint,
                     Eigen::Matrix<double, 2, 4> *pointJacobian,
                     Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const = 0;

  /**
   * Projects a point in homogenous coordinates to a 2d image point
   * (projection). Uses projection including distortion models.
   *
   * @param point Point in Homogeneous coordinates
   * @param parameters Camera intrinsics
   * @param imagePoint Image point
   * @param pointJacobian Jacobian of the projection function w.r.t. the point
   * @param intrinsicsJacobian The Jacobian of the projection function w.r.t.
   * the intrinsics
   *
   * @return Projection status. See \ref ProjectionStatus for more information.
   */
  virtual ProjectionStatus projectHomogeneousWithExternalParameters(
      const Eigen::Vector4d &point,
      const Eigen::VectorXd &parameters,
      Eigen::Vector2d *imagePoint,
      Eigen::Matrix<double, 2, 4> *pointJacobian = NULL,
      Eigen::Matrix2Xd *intrinsicsJacobian = NULL) const = 0;

  /**
   * Projects points in homogenous coordinates to 2d image points (projection)
   * in a batch. Uses projection including distortion models.
   *
   * @param points Points in homogeneous coordinates (one point per column)
   * @param imagePoints The image points (one point per column)
   * @param status Projection status. See \ref ProjectionStatus for more
   * information
   */
  virtual void
  projectHomogeneousBatch(const Eigen::Matrix4Xd &points,
                          Eigen::Matrix2Xd *imagePoints,
                          std::vector<ProjectionStatus> *stati) const = 0;
  /****************************************************************************
   * @} End of Project Points
   ***************************************************************************/

  /****************************************************************************
   * @name Back Project Points
   * @{
   ***************************************************************************/
  /**
   * Back-project a 2d image point into Euclidean space (direction vector)
   *
   * @param imagePoint Image point
   * @param direction Euclidean direction vector
   * @return True or false for success or failure
   */
  virtual bool backProject(const Eigen::Vector2d &imagePoint,
                           Eigen::Vector3d *direction) const = 0;

  /**
   * Back-project a 2d image point into Euclidean space (direction vector)
   *
   * @param imagePoint Image point
   * @param direction Euclidean direction vector
   * @param pointJacobian Jacobian of the back-projection function w.r.t. the
   * point
   *
   * @return True or false for success or failure
   */
  virtual bool
  backProject(const Eigen::Vector2d &imagePoint,
              Eigen::Vector3d *direction,
              Eigen::Matrix<double, 3, 2> *pointJacobian) const = 0;

  /**
   * Back-project 2d image points into Euclidean space (direction vectors)
   *
   * @param imagePoints The image points (one point per column)
   * @param directions The Euclidean direction vectors (one point per column)
   * @return True or false for success or failure
   */
  virtual bool backProjectBatch(const Eigen::Matrix2Xd &imagePoints,
                                Eigen::Matrix3Xd *directions,
                                std::vector<bool> *success) const = 0;

  /**
   * Back-project a 2d image point into homogeneous point (direction vector)
   *
   * @param  imagePoint The image point.
   * @param direction  The homogeneous point as direction vector.
   * @return True or false for success or failure
   */
  virtual bool backProjectHomogeneous(const Eigen::Vector2d &imagePoint,
                                      Eigen::Vector4d *direction) const = 0;

  /**
   * Back-project a 2d image point into homogeneous point (direction vector)
   *
   * @param imagePoint Image point
   * @param direction Homogeneous point as direction vector
   * @param pointJacobian Jacobian of the back-projection function.
   *
   * @return True or false for success or failure
   */
  virtual bool
  backProjectHomogeneous(const Eigen::Vector2d &imagePoint,
                         Eigen::Vector4d *direction,
                         Eigen::Matrix<double, 4, 2> *pointJacobian) const = 0;

  /**
   * Back-project 2d image points into homogeneous points (direction vectors)
   *
   * @param imagePoints Image points (one point per column)
   * @param directions Homogeneous points as direction vectors (one point per
   * column)
   * @return True or false for success or failure
   */
  virtual bool
  backProjectHomogeneousBatch(const Eigen::Matrix2Xd &imagePoints,
                              Eigen::Matrix4Xd *directions,
                              std::vector<bool> *success) const = 0;
  /****************************************************************************
   * @} End of Back Project Points
   ***************************************************************************/

  /****************************************************************************
   * @name Unit Testing
   * @{
   ****************************************************************************/
  /**
   * Creates a random (uniform distribution) image point
   *
   * @return A random image point
   */
  virtual Eigen::Vector2d createRandomImagePoint() const;

  /**
   * Creates a random visible point in Euclidean coordinates.
   *
   * @param minDist Minimal distance of this point
   * @param maxDist Maximum distance of this point
   * @return A random Euclidean point
   */
  virtual Eigen::Vector3d createRandomVisiblePoint(double minDist = 0.0,
                                                   double maxDist = 10.0) const;

  /**
   * Creates a random visible point in homogeneous coordinates
   *
   * @param minDist Minimal distance of this point
   * @param maxDist Maximum distance of this point
   *
   * @return A random homogeneous point
   */
  virtual Eigen::Vector4d createRandomVisibleHomogeneousPoint(
      double minDist = 0.0, double maxDist = 10.0) const;
  /****************************************************************************
   * @} End of Unit Testing
   ***************************************************************************/

  /**
   * Obtain the number of intrinsics parameters.
   */
  virtual int noIntrinsicsParameters() const = 0;

  /**
   * Obtain the type
   */
  virtual std::string type() const = 0;

  /**
   * Obtain the projection type
   */
  virtual const std::string distortionType() const = 0;

  /**
   * Check if the keypoint is masked.
   */
  bool isMasked(const Eigen::Vector2d &imagePoint) const;

  /**
   * Check if the keypoint is in the image.
   */
  bool isInImage(const Eigen::Vector2d &imagePoint) const;

  cv::Mat mask_; ///< Mask -- empty by default

  int imageWidth_;  ///< image width in pixels
  int imageHeight_; ///< image height in pixels

  uint64_t id_; ///< an Id
};

} // namespace autocal
#endif /* AUTOCAL_CAMEREA_CAMERA_BASE_HPP */
