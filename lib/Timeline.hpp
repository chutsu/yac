#pragma once
#include "Core.hpp"
#include "CalibTarget.hpp"

namespace yac {

/*****************************************************************************
 * TimelineEvent
 ****************************************************************************/

/** Timeline Event **/
struct TimelineEvent {
  const std::string type = "";
  const timestamp_t ts = 0.0;

  TimelineEvent(const std::string &type_, const timestamp_t &ts_);
  virtual ~TimelineEvent() = default;
};

/*****************************************************************************
 * CameraEvent
 ****************************************************************************/

/** Camera Event **/
struct CameraEvent : TimelineEvent {
  const int camera_index = -1;
  const std::string image_path;
  const cv::Mat frame;

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const std::string &image_path_);

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const cv::Mat &frame_);
};

/*****************************************************************************
 * ImuEvent
 ****************************************************************************/

/** Imu Event **/
struct ImuEvent : TimelineEvent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  const vec3_t acc = zeros(3, 1);
  const vec3_t gyr = zeros(3, 1);

  ImuEvent(const timestamp_t ts_, const vec3_t &acc_, const vec3_t &gyr_);
};

/*****************************************************************************
 * CalibTargetEvent
 ****************************************************************************/

/** Calibration Target Event **/
struct CalibTargetEvent : TimelineEvent {
  const int camera_index;
  const std::shared_ptr<CalibTarget> calib_target;

  CalibTargetEvent(const timestamp_t ts_,
                   const int camera_index_,
                   const std::shared_ptr<CalibTarget> &calib_target_);
};

/*****************************************************************************
 * Timeline
 ****************************************************************************/

/** Timeline **/
struct Timeline {
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, TimelineEvent *> data;

  Timeline() = default;

  virtual ~Timeline();

  /** Add camera event **/
  void add(const timestamp_t &ts,
           const int camera_index,
           const std::string &image_path);

  /** Add camera event **/
  void add(const timestamp_t &ts, const int camera_index, const cv::Mat &image);

  /** Add imu event **/
  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr);

  /** Add calibration target event **/
  void add(const timestamp_t &ts,
           const int camera_index,
           const std::shared_ptr<CalibTarget> &calib_target);

  /** Get number of events **/
  int getNumEvents(const timestamp_t &ts);

  /** Get events **/
  std::vector<TimelineEvent *> getEvents(const timestamp_t &ts);
};

} // namespace yac
