#include <gtest/gtest.h>

#include "CalibData.hpp"
#include "ReprojectionError.hpp"
#include "SolvePnp.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(ReprojectionError, evaluate) {
  // Load test data
  CalibData calib_data{TEST_CONFIG};

  // Get 1 detected aprilgrid
  CameraData camera_data = calib_data.getCameraData(0);
  CalibTargetPtr calib_target;
  for (const auto &[ts, target] : camera_data) {
    const int tag_rows = target->getTagRows();
    const int tag_cols = target->getTagCols();
    const int total_tags = tag_rows * tag_cols;
    if (target->getNumDetected() == total_tags) {
      calib_target = target;
      break;
    }
  }

  // Form reprojection residual
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  calib_target->getMeasurements(tag_ids,
                                corner_indicies,
                                keypoints,
                                object_points);

  // Create camera geometry
  // clang-format off
  const int camera_index = 0;
  const vec2i_t resolution{752, 480};
  const std::string camera_model = "pinhole-radtan4";
  vecx_t intrinsic;
  intrinsic.resize(8);
  intrinsic << 458.654, 457.296, 367.215, 248.375,
               -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
  vecx_t extrinsic;
  extrinsic.resize(7);
  extrinsic << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  auto camera_geometry = std::make_shared<CameraGeometry>(camera_index,
                                                          camera_model,
                                                          resolution,
                                                          intrinsic,
                                                          extrinsic);
  // clang-format on

  // Estimate relative pose
  mat4_t T_CiF;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiF);
  if (status != 0) {
    FATAL("Failed to estimate relative pose!");
  }

  // Create residual block
  mat2_t covar = I(2);
  vecx_t relpose = tf_vec(T_CiF);
  auto residual_block = ReprojectionError::create(camera_geometry,
                                                  relpose.data(),
                                                  object_points[0].data(),
                                                  keypoints[0],
                                                  covar);
  // Check residual size and parameter block sizes
  auto block_sizes = residual_block->parameter_block_sizes();
  ASSERT_EQ(residual_block->num_residuals(), 2);
  ASSERT_EQ(block_sizes[0], 3);
  ASSERT_EQ(block_sizes[1], 7);
  ASSERT_EQ(block_sizes[2], 7);
  ASSERT_EQ(block_sizes[3], 8);

  // Check param pointers
  auto param_ptrs = residual_block->getParamPtrs();
  ASSERT_EQ(param_ptrs[0], object_points[0].data());
  ASSERT_EQ(param_ptrs[1], relpose.data());
  ASSERT_EQ(param_ptrs[2], camera_geometry->getExtrinsicPtr());
  ASSERT_EQ(param_ptrs[3], camera_geometry->getIntrinsicPtr());


  // Check Jacobians
  ASSERT_TRUE(residual_block->checkJacobian(0, "J_point"));
  ASSERT_TRUE(residual_block->checkJacobian(1, "J_relpose"));
  ASSERT_TRUE(residual_block->checkJacobian(2, "J_extrinsic"));
  ASSERT_TRUE(residual_block->checkJacobian(3, "J_camera"));
}

} // namespace yac
