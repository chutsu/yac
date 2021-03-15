#include <gtest/gtest.h>
#include <glog/logging.h>

#include "autocal/common/Measurements.hpp"
#include "autocal/core/core.hpp"
#include "autocal/ceres/common.hpp"
#include "autocal/ceres/error/ImuError.hpp"
#include "autocal/ceres/param/TimeDelayParameterBlock.hpp"
// #include "autocal/ceres/param/FiducialParameterBlock.hpp"

#include "EuRoCTestSetup.hpp"

#define TEST_DATA "/data/euroc_mav/imu_april"

using namespace autocal;

TEST(ImuError, checkJacobians) {
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  // Loop through timeline
  autocal::ImuMeasurementDeque imu_data;
  std::deque<aprilgrid_t> cam0_grids;
  bool imu0_ready = false;
  printf("Checking jacobians!\n");

  for (const auto &ts : test_data.timeline.timestamps) {
    const auto result = test_data.timeline.data.equal_range(ts);

    // Process camera and inertial measurements
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;
      const bool aprilgrid_event = (event.type == APRILGRID_EVENT);
      const bool cam0_event = (event.camera_index == 0);
      const bool imu_event = (event.type == IMU_EVENT);

      // AprilGrid event handler
      if (imu0_ready && aprilgrid_event && cam0_event) {
        cam0_grids.push_back(event.grid);

      // IMU event handler
      } else if (imu_event) {
        imu0_ready = true;
        const auto timestamp = autocal::Time(ns2sec(ts));
        const auto accel = event.a_m;
        const auto gyro = event.w_m;
        const auto measurement = autocal::ImuSensorReadings{gyro, accel};
        imu_data.emplace_back(timestamp, measurement);
      }
    }

    // Do we have two aprilgrids?
    bool grids_ok = (cam0_grids.size() == 2);
    if (grids_ok == false) {
      continue;
    }

    // De we have enough imu data?
    const auto grid_last_ts = cam0_grids.back().timestamp;
    const auto imu_last_ts = imu_data.back().timeStamp.toNSec();
    bool imu_ok = imu_last_ts > grid_last_ts;
    if (imu_ok) {
      // Check ImuError jacobians
      auto t0 = cam0_grids[0].timestamp;
      auto t1 = cam0_grids[1].timestamp;
      auto T_FC0_0 = cam0_grids[0].T_CF.inverse();
      auto T_FC0_1 = cam0_grids[1].T_CF.inverse();
      const auto T_C0S = test_data.T_SC0.inverse();
      const auto T_WS_0 = test_data.T_WF * T_FC0_0 * T_C0S;
      const auto T_WS_1 = test_data.T_WF * T_FC0_1 * T_C0S;

      double speed_biases_0[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double speed_biases_1[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double time_delay = 0.01;

      int param_id = 0;
      auto pose0 = create_pose_block(T_WS_0, param_id);
      auto state0 = create_speed_bias_block(t0, speed_biases_0, param_id);
      auto pose1 = create_pose_block(T_WS_1, param_id);
      auto state1 = create_speed_bias_block(t1, speed_biases_1, param_id);
      auto td = create_time_delay_block(time_delay, param_id);

      std::vector<autocal::ceres::ParameterBlock *> param_blocks;
      param_blocks.push_back(&pose0);
      param_blocks.push_back(&state0);
      param_blocks.push_back(&pose1);
      param_blocks.push_back(&state1);
      param_blocks.push_back(&td);

      // Form residual block
      const autocal::Time time_start{ns2sec(t0)};
      const autocal::Time time_end{ns2sec(t1)};
      autocal::ceres::ImuError error(imu_data, test_data.imu0_params, time_start, time_end);

      // Check Jacobian
      bool retval = autocal::check_jacobians(&error, param_blocks, 1e-4);
      ASSERT_TRUE(retval);
      printf(".");
      fflush(stdout);

      // Pop front of cam grids
      cam0_grids.pop_front();
      imu_data.clear();
    }
  }
  printf("\n");
}
