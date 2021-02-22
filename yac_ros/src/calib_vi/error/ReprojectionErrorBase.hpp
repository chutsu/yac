#ifndef YAC_REPROJECTION_ERROR_BASE_HPP
#define YAC_REPROJECTION_ERROR_BASE_HPP

#include <type_traits>
#include <ceres/ceres.h>

// #include "yac/ceres/error/ErrorInterface.hpp"
#include "ErrorInterface.hpp"

namespace yac {

/// \brief Reprojection error base class.
template <int... T>
class ReprojectionErrorBase_ : public ::ceres::SizedCostFunction<T...>,
                               public ErrorInterface {
public:
  typedef ::ceres::SizedCostFunction<T...> ceres_base_t;

  /// \brief Camera ID.
  uint64_t cameraId() const { return cameraId_; }

  /// \brief Set camera ID.
  /// @param[in] cameraId ID of the camera.
  void setCameraId(uint64_t cameraId) { cameraId_ = cameraId; }

  uint64_t cameraId_; ///< ID of the camera.
};

/// \brief 2D keypoint reprojection error base class.
template <int... T>
class ReprojectionError2dBase_ : public ReprojectionErrorBase_<T...> {
public:
  typedef ReprojectionErrorBase_<T...> base_t;
  typedef typename ReprojectionErrorBase_<T...>::ceres_base_t ceres_base_t;

  /// \brief Measurement type (2D).
  typedef Eigen::Vector2d measurement_t;

  /// \brief Covariance / information matrix type (2x2).
  typedef Eigen::Matrix2d covariance_t;

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement vector (2d).
  virtual void setMeasurement(const measurement_t &measurement) = 0;

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix (2x2).
  virtual void setInformation(const covariance_t &information) = 0;

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector (2d).
  virtual const measurement_t &measurement() const = 0;

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix (2x2).
  virtual const covariance_t &information() const = 0;

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t &covariance() const = 0;
};

/// \brief Intrinsics and extrinsics reprojection error base class.
template <int... T>
class IntrinsicsExtrinsicsErrorBase_ : public ReprojectionErrorBase_<T...> {
public:
  typedef ReprojectionErrorBase_<T...> base_t;
  typedef typename ReprojectionErrorBase_<T...>::ceres_base_t ceres_base_t;

  /// \brief Measurement type (2D).
  typedef Eigen::Vector2d measurement_t;

  /// \brief Covariance / information matrix type (2x2).
  typedef Eigen::Matrix2d covariance_t;

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement vector (2d).
  virtual void setMeasurement(const measurement_t &measurement) = 0;

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix (2x2).
  virtual void setInformation(const covariance_t &information) = 0;

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector (2d).
  virtual const measurement_t &measurement() const = 0;

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix (2x2).
  virtual const covariance_t &information() const = 0;

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t &covariance() const = 0;
};

/// \brief Calibration reprojection error base class.
template <int... T>
class CalibReprojErrorBase_ : public ReprojectionErrorBase_<T...> {
public:
  typedef ReprojectionErrorBase_<T...> base_t;
  typedef typename ReprojectionErrorBase_<T...>::ceres_base_t ceres_base_t;

  /// \brief Measurement type (2D).
  typedef Eigen::Vector2d measurement_t;

  /// \brief Covariance / information matrix type (2x2).
  typedef Eigen::Matrix2d covariance_t;

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement vector (2d).
  virtual void setMeasurement(const measurement_t &measurement) = 0;

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix (2x2).
  virtual void setInformation(const covariance_t &information) = 0;

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector (2d).
  virtual const measurement_t &measurement() const = 0;

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix (2x2).
  virtual const covariance_t &information() const = 0;

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t &covariance() const = 0;
};

/// \brief Calibration reprojection error base class.
template <int... T>
class CalibReprojError2Base_ : public ReprojectionErrorBase_<T...> {
public:
  typedef ReprojectionErrorBase_<T...> base_t;
  typedef typename ReprojectionErrorBase_<T...>::ceres_base_t ceres_base_t;

  /// \brief Measurement type (2D).
  typedef Eigen::Vector2d measurement_t;

  /// \brief Covariance / information matrix type (2x2).
  typedef Eigen::Matrix2d covariance_t;

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement vector (2d).
  virtual void setMeasurement(const measurement_t &measurement) = 0;

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix (2x2).
  virtual void setInformation(const covariance_t &information) = 0;

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector (2d).
  virtual const measurement_t &measurement() const = 0;

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix (2x2).
  virtual const covariance_t &information() const = 0;

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t &covariance() const = 0;
};

typedef ReprojectionErrorBase_<
    2 /* number of residuals */,
    7 /* size of first parameter */,
    4 /* size of second parameter */,
    7 /* size of third parameter (camera extrinsics) */>
    ReprojectionErrorBase;

typedef ReprojectionError2dBase_<
    2 /* number of residuals */,
    7 /* size of first parameter */,
    4 /* size of second parameter */,
    7 /* size of third parameter (camera extrinsics) */>
    ReprojectionError2dBase;

typedef IntrinsicsExtrinsicsErrorBase_<2 /* number of residuals */,
                                       7 /* size of T_WS */,
                                       7 /* size of T_WF */,
                                       7 /* size of T_SC */,
                                       8 /* size of cam params */>
    IntrinsicsExtrinsicsErrorBase;

typedef CalibReprojErrorBase_<2 /* number of residuals */,
                              7 /* size of T_WS */,
                              2 /* size of T_WF */,
                              7 /* size of T_SC */,
                              8 /* size of cam params */>
    CalibReprojErrorBase;

typedef CalibReprojError2Base_<2 /* number of residuals */,
                               7 /* size of T_WS */,
                               7 /* size of T_WF */,
                               7 /* size of T_SC */,
                               8 /* size of cam params */>
    CalibReprojError2Base;

} // namespace yac
#endif /* YAC_REPROJECTION_ERROR_BASE_HPP */
