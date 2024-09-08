#pragma once
#include "Core.hpp"

namespace yac {

/** Timeline Event **/
struct TimelineEvent {
  const std::string type = "";
  const timestamp_t ts = 0.0;

  TimelineEvent(const std::string &type_, const timestamp_t &ts_)
      : type{type_}, ts{ts_} {}

  virtual ~TimelineEvent() = default;
};

/** Camera Event **/
struct CameraEvent : TimelineEvent {
  const int camera_index = -1;
  const std::string image_path;
  const cv::Mat frame;

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const std::string &image_path_)
      : TimelineEvent{"CameraEvent", ts_}, camera_index{camera_index_},
        image_path{image_path_} {}

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const cv::Mat &frame_)
      : TimelineEvent{"CameraEvent", ts_},
        camera_index{camera_index_}, frame{frame_} {}
};

/** Imu Event **/
struct ImuEvent : TimelineEvent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  const vec3_t acc = zeros(3, 1);
  const vec3_t gyr = zeros(3, 1);

  ImuEvent(const timestamp_t ts_, const vec3_t &acc_, const vec3_t &gyr_)
      : TimelineEvent{"ImuEvent", ts_}, acc{acc_}, gyr{gyr_} {}
};

/** Calibration Target Event **/
struct CalibTargetEvent : TimelineEvent {
  const int camera_index;
  const std::shared_ptr<CalibTarget> calib_target;

  CalibTargetEvent(const timestamp_t ts_,
                   const int camera_index_,
                   const std::shared_ptr<CalibTarget> &calib_target_)
      : TimelineEvent{"CalibTargetEvent", ts_}, camera_index{camera_index_},
        calib_target{calib_target_} {}
};

/** Timeline **/
struct Timeline {
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, TimelineEvent *> data;

  Timeline() = default;

  virtual ~Timeline() {
    for (auto &kv : data) {
      delete kv.second;
    }
  }

  /** Add camera event **/
  void add(const timestamp_t &ts,
           const int camera_index,
           const std::string &image_path) {
    timestamps.insert(ts);
    data.insert({ts, new CameraEvent{ts, camera_index, image_path}});
  }

  /** Add camera event **/
  void add(const timestamp_t &ts,
           const int camera_index,
           const cv::Mat &image) {
    timestamps.insert(ts);
    data.insert({ts, new CameraEvent{ts, camera_index, image}});
  }

  /** Add imu event **/
  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr) {
    timestamps.insert(ts);
    data.insert({ts, new ImuEvent{ts, acc, gyr}});
  }

  /** Add calibration target event **/
  void add(const timestamp_t &ts,
           const int camera_index,
           const std::shared_ptr<CalibTarget> &calib_target) {
    timestamps.insert(ts);
    data.insert({ts, new CalibTargetEvent{ts, camera_index, calib_target}});
  }

  /** Get number of events **/
  int getNumEvents(const timestamp_t &ts) {
    const auto range = data.equal_range(ts);
    return std::distance(range.first, range.second);
  }

  /** Get events **/
  std::vector<TimelineEvent *> getEvents(const timestamp_t &ts) {
    std::vector<TimelineEvent *> events;

    const auto range = data.equal_range(ts);
    for (auto it = range.first; it != range.second; it++) {
      events.push_back(it->second);
    }

    return events;
  }
};

} // namespace yac
