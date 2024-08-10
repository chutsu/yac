#ifndef YAC_TIMELINE_HPP
#define YAC_TIMELINE_HPP

#include "core.hpp"
#include "aprilgrid.hpp"

namespace yac {

struct timeline_event_t {
  const std::string type = "";
  const timestamp_t ts = 0.0;

  timeline_event_t(const std::string &type_, const timestamp_t &ts_)
    : type{type_}, ts{ts_} {}

  virtual ~timeline_event_t() {}
};

struct camera_event_t : timeline_event_t {
  const int cam_idx = -1;
  const std::string img_path;
  const cv::Mat frame;

  camera_event_t(const timestamp_t ts_,
                 const int cam_idx_,
                 const std::string &img_path_)
      : timeline_event_t{"camera_event_t", ts_},
        cam_idx{cam_idx_},
        img_path{img_path_} {}

  camera_event_t(const timestamp_t ts_,
                 const int cam_idx_,
                 const cv::Mat &frame_)
      : timeline_event_t{"camera_event_t", ts_},
        cam_idx{cam_idx_},
        frame{frame_} {}
};

struct imu_event_t : timeline_event_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  const vec3_t acc = zeros(3, 1);
  const vec3_t gyr = zeros(3, 1);

  imu_event_t(const timestamp_t ts_,
              const vec3_t &acc_,
              const vec3_t &gyr_)
    : timeline_event_t{"imu_event_t", ts_},
      acc{acc_},
      gyr{gyr_} {}
};

struct aprilgrid_event_t : timeline_event_t {
  const int cam_idx;
  const aprilgrid_t grid;

  aprilgrid_event_t(const timestamp_t ts_,
                   const int cam_idx_,
                   const aprilgrid_t &grid_)
    : timeline_event_t{"aprilgrid_event_t", ts_},
      cam_idx{cam_idx_},
      grid{grid_} {}
};

struct mocap_event_t : timeline_event_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  const std::string object_name;
  const quat_t q_WM;
  const vec3_t r_WM;

  mocap_event_t(const timestamp_t ts_,
                const std::string &object_name_,
                const quat_t &q_WM_,
                const vec3_t &r_WM_)
      : timeline_event_t{"mocap_event_t", ts_},
        object_name{object_name_},
        q_WM{q_WM_},
        r_WM{r_WM_} {}
};

struct timeline_t {
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, timeline_event_t *> data;

  timeline_t();
  virtual ~timeline_t();

  void add(const timestamp_t &ts,
           const int cam_idx,
           const std::string &img_path);
  void add(const timestamp_t &ts, const int cam_idx, const cv::Mat &image);
  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr);
  void add(const timestamp_t &ts, const int cam_idx, const aprilgrid_t &grid);
  void add(const timestamp_t &ts,
           const std::string &object_name,
           const quat_t &q_WM,
           const vec3_t &r_WM);

  int nb_events(const timestamp_t &ts);
  std::vector<timeline_event_t *> get_events(const timestamp_t &ts);
};

} //  namespace yac
#endif // YAC_TIMELINE_HPP
