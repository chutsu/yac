#ifndef YAC_TIMELINE_HPP
#define YAC_TIMELINE_HPP

#include "core.hpp"

namespace yac {

/* Timeline type */
#define NOT_SET 0
#define IMU_EVENT 1
#define CAMERA_EVENT 2
#define VICON_EVENT 3

struct timeline_event_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // General
  int type = 0;
  timestamp_t ts = 0.0;

  // IMU data
  vec3_t a_m = zeros(3, 1);
  vec3_t w_m = zeros(3, 1);

  // Camera data
  int camera_index = -1;
  std::string image_path;

  // MOCAP data
  std::string object_name;
  vec3_t r_WM = zeros(3, 1);
  quat_t q_WM = quat_t{1.0, 0.0, 0.0, 0.0};

  timeline_event_t() {}

  // IMU data constructor
  timeline_event_t(const timestamp_t ts_,
                   const vec3_t &a_m_,
                   const vec3_t &w_m_)
    : type{IMU_EVENT}, ts{ts_}, a_m{a_m_}, w_m{w_m_} {}

  // Camera data constructor
  timeline_event_t(const timestamp_t ts_,
                   const int camera_index_,
                   const std::string &image_path_)
      : type{CAMERA_EVENT}, ts{ts_}, camera_index{camera_index_},
        image_path{image_path_} {}

  // MOCAP data constructor
  timeline_event_t(const timestamp_t ts_,
                  const std::string &object_name_,
                  const vec3_t &r_WM_,
                  const quat_t &q_WM_)
      : type{VICON_EVENT}, ts{ts_},
        object_name{object_name_}, r_WM{r_WM_}, q_WM{q_WM_} {}

  ~timeline_event_t() {}
};

struct timeline_t {
  std::multimap<timestamp_t, timeline_event_t> data;
  std::set<timestamp_t> timestamps;

  timeline_t() {}
  ~timeline_t() {}

  void add(const timeline_event_t &event) {
    const auto &ts = event.ts;
    data.insert({ts, event});
    timestamps.insert(ts);
  }
};

} //  namespace yac
#endif // YAC_TIMELINE_HPP
