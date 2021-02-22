#ifndef YAC_COMMON_MEASUREMENTS_HPP
#define YAC_COMMON_MEASUREMENTS_HPP

#include <deque>
#include <vector>
#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#include <Eigen/Dense>
#include "Time.hpp"
// #include "yac/kinematics/Transformation.hpp"

namespace yac {

/**
 * Generic measurement
 *
 * A measurement always come with a timestamp so that an asynchronous operation
 * can be performed.
 *
 * @tparam MEASUREMENT_T Measurement data type.
 */
template <class MEASUREMENT_T>
struct Measurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  yac::Time timeStamp;   ///< Measurement timestamp
  MEASUREMENT_T measurement; ///< Actual measurement.
  int sensorId = -1; ///< Sensor ID. E.g. camera index in a multicamera setup

  Measurement() : timeStamp(0.0) {}
  Measurement(const yac::Time &timeStamp,
              const MEASUREMENT_T &measurement,
              const int sensorId = -1)
      : timeStamp{timeStamp}, measurement{measurement}, sensorId{sensorId} {}
};

/**
 * IMU measurements. For now assume they are synchronized:
 */
struct ImuSensorReadings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d gyroscopes;     ///< Gyroscope measurement.
  Eigen::Vector3d accelerometers; ///< Accelerometer measurement.

  ImuSensorReadings() : gyroscopes{}, accelerometers{} {}
  ImuSensorReadings(Eigen::Vector3d gyroscopes, Eigen::Vector3d accelerometers)
      : gyroscopes{gyroscopes}, accelerometers{accelerometers} {}
};

/**
 * Depth camera measurements. For now assume they are synchronized:
 */
struct DepthCameraData {
  cv::Mat image;                       ///< Grayscale/RGB image.
  cv::Mat depthImage;                  ///< Depth image.
  std::vector<cv::KeyPoint> keypoints; ///< Keypoints if available.
  bool deliversKeypoints;              ///< Keypoints delievered to measurement?
};

/**
 * Position measurement.
 */
struct PositionReading {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position;           ///< Position measurement.
  Eigen::Matrix3d positionCovariance; ///< Measurement covariance.
};

/**
 * GPS position measurement.
 */
struct GpsPositionReading {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double lat_wgs84; ///< Latitude in WGS84 coordinate system [deg]
  double lon_wgs84; ///< Longitude in WGS84 coordiante system [deg]
  double alt_wgs84; ///< Altitude in WGS84 coordinate system [m]
  Eigen::Matrix3d positionCovarianceENU; ///< Measurement covariance (ENU)
};

/**
 * Magnetometer measurement.
 */
struct MagnetometerReading {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ///< The magnetic flux density measurement. [uT]
  Eigen::Vector3d fluxDensity;
};

/**
 * Barometer measurement.
 */
struct BarometerReading {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double pressure;    ///< Pressure measurement. [Pa]
  double temperature; ///< Temperature. [K]
};

/// Differential pressure sensor measurement.
struct DifferentialPressureReading {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double pressure;                ///< Pressure measurement. [Pa]
  Eigen::Vector3d acceleration_B; ///< Acceleration in B-frame.
};

// Store raw measurements before more advanced filling into data structures
// happens:
typedef Measurement<ImuSensorReadings> ImuMeasurement;
typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>>
    ImuMeasurementDeque;

/**
 * Camera measurement.
 */
struct CameraData {
  cv::Mat image;                       ///< Image
  cv::Mat depthImage;                  ///< Image
  std::vector<cv::KeyPoint> keypoints; ///< Keypoints if available
  bool deliversKeypoints;              ///< Keypoints delivered?
};

/**
 * Keypoint measurement
 */
struct KeypointData {
  std::vector<cv::KeyPoint> keypoints;        ///< Keypoints
  std::vector<long unsigned int> landmarkIds; ///< Associated landmark IDs
  cv::Mat descriptors;                        ///< Keypoint descriptors
};

/**
 * Frame measurement
 */
struct FrameData {
  typedef std::shared_ptr<yac::FrameData> Ptr;
  CameraData image;       ///< Camera measurement, i.e., image
  KeypointData keypoints; ///< Keypoints
};

// TYPE-DEFS
// clang-format off
typedef Measurement<CameraData> CameraMeasurement;
typedef Measurement<FrameData> FrameMeasurement;
typedef Measurement<DepthCameraData> DepthCameraMeasurement;
typedef Measurement<PositionReading> PositionMeasurement;

typedef std::deque<PositionMeasurement, Eigen::aligned_allocator<PositionMeasurement>> PositionMeasurementDeque;
typedef Measurement<GpsPositionReading> GpsPositionMeasurement;

typedef std::deque<GpsPositionMeasurement, Eigen::aligned_allocator<GpsPositionMeasurement>> GpsPositionMeasurementDeque;
typedef Measurement<MagnetometerReading> MagnetometerMeasurement;

typedef std::deque<MagnetometerMeasurement, Eigen::aligned_allocator<MagnetometerMeasurement>> MagnetometerMeasurementDeque;
typedef Measurement<BarometerReading> BarometerMeasurement;

typedef Measurement<DifferentialPressureReading> DifferentialPressureMeasurement;
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;
// clang-format on

} // namespace yac
#endif // YAC_COMMON_MEASUREMENTS_HPP
