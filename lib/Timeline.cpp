#include "Timeline.hpp"

namespace yac {

/*****************************************************************************
 * TimelineEvent
 ****************************************************************************/

TimelineEvent::TimelineEvent(const std::string &type_, const timestamp_t &ts_)
    : type{type_}, ts{ts_} {}

/*****************************************************************************
 * CameraEvent
 ****************************************************************************/

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_index_,
                         const std::string &image_path_)
    : TimelineEvent{"CameraEvent", ts_}, camera_index{camera_index_},
      image_path{image_path_} {}

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_index_,
                         const cv::Mat &frame_)
    : TimelineEvent{"CameraEvent", ts_},
      camera_index{camera_index_}, frame{frame_} {}

/*****************************************************************************
 * ImuEvent
 ****************************************************************************/

ImuEvent::ImuEvent(const timestamp_t ts_,
                   const vec3_t &acc_,
                   const vec3_t &gyr_)
    : TimelineEvent{"ImuEvent", ts_}, acc{acc_}, gyr{gyr_} {}

/*****************************************************************************
 * CalibTargetEvent
 ****************************************************************************/

CalibTargetEvent::CalibTargetEvent(
    const timestamp_t ts_,
    const int camera_index_,
    const std::shared_ptr<CalibTarget> &calib_target_)
    : TimelineEvent{"CalibTargetEvent", ts_}, camera_index{camera_index_},
      calib_target{calib_target_} {}

/*****************************************************************************
 * Timeline
 ****************************************************************************/

Timeline::~Timeline() {
  for (auto &kv : data) {
    delete kv.second;
  }
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const std::string &image_path) {
  timestamps.insert(ts);
  data.insert({ts, new CameraEvent{ts, camera_index, image_path}});
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const cv::Mat &image) {
  timestamps.insert(ts);
  data.insert({ts, new CameraEvent{ts, camera_index, image}});
}

void Timeline::add(const timestamp_t &ts,
                   const vec3_t &acc,
                   const vec3_t &gyr) {
  timestamps.insert(ts);
  data.insert({ts, new ImuEvent{ts, acc, gyr}});
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const std::shared_ptr<CalibTarget> &calib_target) {
  timestamps.insert(ts);
  data.insert({ts, new CalibTargetEvent{ts, camera_index, calib_target}});
}

int Timeline::getNumEvents(const timestamp_t &ts) {
  const auto range = data.equal_range(ts);
  return std::distance(range.first, range.second);
}

std::vector<TimelineEvent *> Timeline::getEvents(const timestamp_t &ts) {
  std::vector<TimelineEvent *> events;

  const auto range = data.equal_range(ts);
  for (auto it = range.first; it != range.second; it++) {
    events.push_back(it->second);
  }

  return events;
}

} // namespace yac
