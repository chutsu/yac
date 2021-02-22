#include "CameraBase.hpp"

namespace yac {

Eigen::Vector2d CameraBase::createRandomImagePoint() const {
  // Uniform random sample in image coordinates.
  // Add safety boundary for later inaccurate backprojection
  Eigen::Vector2d outPoint = Eigen::Vector2d::Random();
  outPoint += Eigen::Vector2d::Ones();
  outPoint *= 0.5;
  outPoint[0] *= double(imageWidth_ - 0.022);
  outPoint[0] += 0.011;
  outPoint[1] *= double(imageHeight_ - 0.022);
  outPoint[1] += 0.011;
  return outPoint;
}

Eigen::Vector3d CameraBase::createRandomVisiblePoint(double minDist,
                                                     double maxDist) const {
  // Random image point first:
  Eigen::Vector2d imagePoint = createRandomImagePoint();

  // Now sample random depth:
  Eigen::Vector2d depth = Eigen::Vector2d::Random();
  Eigen::Vector3d ray;
  backProject(imagePoint, &ray);
  ray.normalize();
  ray *= (0.5 * (maxDist - minDist) * (depth[0] + 1.0) +
          minDist); // rescale and offset
  return ray;
}

Eigen::Vector4d CameraBase::createRandomVisibleHomogeneousPoint(
    double minDist, double maxDist) const {
  Eigen::Vector3d point = createRandomVisiblePoint(minDist, maxDist);
  return Eigen::Vector4d(point[0], point[1], point[2], 1.0);
}

bool CameraBase::setMask(const cv::Mat &mask) {
  // check type
  if (mask.type() != CV_8UC1) {
    return false;
  }
  // check size
  if (mask.rows != imageHeight_) {
    return false;
  }
  if (mask.cols != imageWidth_) {
    return false;
  }
  mask_ = mask;
  return true;
}

bool CameraBase::removeMask() {
  mask_.resize(0);
  return true;
}

bool CameraBase::hasMask() const { return (mask_.data); }

const cv::Mat &CameraBase::mask() const { return mask_; }

bool CameraBase::isMasked(const Eigen::Vector2d &imagePoint) const {
  if (!isInImage(imagePoint)) {
    return true;
  }
  if (!hasMask()) {
    return false;
  }
  return mask_.at<uchar>(int(imagePoint[1]), int(imagePoint[0]));
}

bool CameraBase::isInImage(const Eigen::Vector2d &imagePoint) const {
  if (imagePoint[0] < 0.0 || imagePoint[1] < 0.0) {
    return false;
  }
  if (imagePoint[0] >= imageWidth_ || imagePoint[1] >= imageHeight_) {
    return false;
  }
  return true;
}

} // namespace yac
