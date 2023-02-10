#include "../munit.hpp"
#include "util/timeline.hpp"

namespace yac {

int test_timeline_event() {
  timeline_event_t event("event_t", 0);
  return 0;
}

int test_camera_event() {
  const timestamp_t ts = 0;
  const int camera_index = 0;
  const std::string image_path = "";
  camera_event_t event(ts, camera_index, image_path);
  return 0;
}

int test_imu_event() {
  const timestamp_t ts = 0;
  const vec3_t acc{0.0, 0.0, 0.0};
  const vec3_t gyr{0.0, 0.0, 0.0};
  imu_event_t event(ts, acc, gyr);
  return 0;
}

int test_aprilgrid_event() {
  const timestamp_t ts = 0;
  const int camera_index = 0;
  const aprilgrid_t grid;
  aprilgrid_event_t event(ts, camera_index, grid);
  return 0;
}

int test_mocap_event() {
  const timestamp_t ts = 0;
  const std::string object_name = "drone";
  const quat_t q_WM{1.0, 0.0, 0.0, 0.0};
  const vec3_t r_WM{0.0, 0.0, 0.0};
  mocap_event_t event(ts, object_name, q_WM, r_WM);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_timeline_event);
  MU_ADD_TEST(test_camera_event);
  MU_ADD_TEST(test_imu_event);
  MU_ADD_TEST(test_aprilgrid_event);
  MU_ADD_TEST(test_mocap_event);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
