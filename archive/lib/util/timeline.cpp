#include "timeline.hpp"

namespace yac {

timeline_t::timeline_t() {}

timeline_t::~timeline_t() {
  for (auto &kv : data) {
    delete kv.second;
  }
}

void timeline_t::add(const timestamp_t &ts,
                     const int cam_idx,
                     const std::string &img_path) {
  timestamps.insert(ts);
  data.insert({ts, new camera_event_t{ts, cam_idx, img_path}});
}

void timeline_t::add(const timestamp_t &ts,
                     const int cam_idx,
                     const cv::Mat &image) {
  timestamps.insert(ts);
  data.insert({ts, new camera_event_t{ts, cam_idx, image}});
}

void timeline_t::add(const timestamp_t &ts,
                     const vec3_t &acc,
                     const vec3_t &gyr) {
  timestamps.insert(ts);
  data.insert({ts, new imu_event_t{ts, acc, gyr}});
}

void timeline_t::add(const timestamp_t &ts,
                     const int cam_idx,
                     const aprilgrid_t &grid) {
  timestamps.insert(ts);
  data.insert({ts, new aprilgrid_event_t{ts, cam_idx, grid}});
}

void timeline_t::add(const timestamp_t &ts,
                     const std::string &object_name,
                     const quat_t &q_WM,
                     const vec3_t &r_WM) {
  timestamps.insert(ts);
  data.insert({ts, new mocap_event_t{ts, object_name, q_WM, r_WM}});
}

int timeline_t::nb_events(const timestamp_t &ts) {
  const auto range = data.equal_range(ts);
  return std::distance(range.first, range.second);
}

std::vector<timeline_event_t *> timeline_t::get_events(const timestamp_t &ts) {
  std::vector<timeline_event_t *>events;

  const auto range = data.equal_range(ts);
  for (auto it = range.first; it != range.second; it++) {
    events.push_back(it->second);
  }

  return events;
}

} //  namespace yac
