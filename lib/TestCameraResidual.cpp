#include <gtest/gtest.h>

#include "CalibData.hpp"
#include "CameraResidual.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

TEST(CameraResidual, evaluate) {
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

  mat4_t T_CiF;
  int status = solvepnp(camera_geometry->getCameraModel().get(),
                        resolution,
                        camera_geometry->getIntrinsic(),
                        keypoints,
                        object_points,
                        T_CiF);
  if (status != 0) {
    FATAL("Failed to estimate relative pose!");
  }
  std::cout << T_CiF << std::endl;

  // const calib_target_t calib_target;
  // mat4_t T_C0Ci = I(4);
  // mat4_t T_C0F = T_C0Ci * T_CiF;
  // pose_t cam_exts{grid.timestamp, T_C0Ci};
  // pose_t rel_pose{grid.timestamp, T_C0F}; // Fiducial corners
  // fiducial_corners_t corners{calib_target};
  // auto corner = corners.get_corner(tag_ids[0], corner_indicies[0]);

  // CameraResidual residual(&cam_geom,
  //                         &cam_params,
  //                         &cam_exts,
  //                         &rel_pose,
  //                         corner,
  //                         keypoints[0],
  //                         I(2));

  // ASSERT_TRUE(r.check_jacs(0, "J_cam_exts"));
  // ASSERT_TRUE(r.check_jacs(1, "J_rel_pose"));
  // ASSERT_TRUE(r.check_jacs(2, "J_cam"));
}

} // namespace yac
