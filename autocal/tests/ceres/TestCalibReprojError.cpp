#include <memory>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "autocal/cv/CameraGeometry.hpp"
#include "autocal/cv/PinholeCamera.hpp"
#include "autocal/cv/RadialTangentialDistortion.hpp"
#include "autocal/cv/EquidistantDistortion.hpp"
#include "autocal/ceres/common.hpp"
#include "autocal/ceres/param/CameraParameterBlock.hpp"
#include "autocal/ceres/param/FiducialParameterBlock.hpp"
#include "autocal/ceres/param/PoseParameterBlock.hpp"
#include "autocal/ceres/error/CalibReprojError.hpp"
#include "autocal/kinematics/Transformation.hpp"
#include "autocal/time/Time.hpp"
#include "autocal/common/FrameTypedefs.hpp"
#include "autocal/util/assert_macros.hpp"

#include "EuRoCTestSetup.hpp"

#define TEST_DATA "/data/euroc_mav/imu_april"

using namespace autocal;

static int camera_event_handler(
    const timeline_event_t<timestamp_t> &event,
    const mat4_t T_WS,
    const mat4_t T_WF,
    autocal::ceres::FiducialParameterBlock *fiducial_block,
    autocal::ceres::PoseParameterBlock *extrinsic_block,
    autocal::ceres::CameraParameterBlock *camera_block,
    int &param_id) {
  // Create pose block
  auto pose_block = create_pose_block(T_WS, param_id);

  // Create residual block
  // -- Loop over every tag seen
  for (const auto tag_id : event.grid.ids) {
    vec2s_t keypoints;
    aprilgrid_get(event.grid, tag_id, keypoints);

    // -- Loop over every corner seen
    for (size_t j = 0; j < 4; j++) {
      const int corner_id = j;
      vec3_t p_F;
      aprilgrid_object_point(event.grid, tag_id, corner_id, p_F);

      // Form residual block
      autocal::ceres::CalibReprojError<PinholeEqui> error(
        0,
        tag_id,
        corner_id,
        p_F,
        T_WF,
        keypoints[j],
        I(2)
      );

      std::vector<autocal::ceres::ParameterBlock *> param_blocks;
      param_blocks.push_back(&pose_block);
      param_blocks.push_back(fiducial_block);
      param_blocks.push_back(extrinsic_block);
      param_blocks.push_back(camera_block);

      // Check Jacobian
      if (autocal::check_jacobians(&error, param_blocks, 1e-4) != true) {
        return -1;
      }

    }
  }

  return 0;
}

TEST(CalibReprojError, checkJacobians) {
  euroc_test_data_t test_data = load_euroc_test_data(TEST_DATA);

  // Build the problem.
  int param_id = 0;
  auto cam0_block = create_cam_block(test_data.cam0_params, param_id);
  auto extrinsic_block = create_pose_block(test_data.T_SC0, param_id);
  auto fiducial_block = create_fiducial_block(test_data.T_WF, param_id);

  // Loop through timeline and check jacobians
  printf("Checking jacobians!\n");
  int pose_id = 0;
  for (const auto &ts : test_data.timeline.timestamps) {
    const auto result = test_data.timeline.data.equal_range(ts);
    // -- Loop through events at timestamp ts since there could be two events
    // at the same timestamp
    for (auto it = result.first; it != result.second; it++) {
      const auto event = it->second;
      if (event.type == APRILGRID_EVENT && event.camera_index == 0) {
        int retval = camera_event_handler(event,
                                          test_data.T_WS[pose_id],
                                          test_data.T_WF,
                                          &fiducial_block,
                                          &extrinsic_block,
                                          &cam0_block,
                                          param_id);
        ASSERT_EQ(0, retval);
        printf(".");
        fflush(stdout);
      }
    }
  }
  printf("\n");
}
