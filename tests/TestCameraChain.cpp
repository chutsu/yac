#include <gtest/gtest.h>

#include "CameraChain.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace yac {

static std::shared_ptr<CameraGeometry> setup_euroc_camera0() {
  const int camera_index = 0;
  const vec2i_t resolution{752, 480};
  const std::string camera_model = "pinhole-radtan4";

  vecx_t intrinsic;
  intrinsic.resize(8);
  intrinsic(0) = 458.654;
  intrinsic(1) = 457.296;
  intrinsic(2) = 367.215;
  intrinsic(3) = 248.375;
  intrinsic(4) = -0.28340811;
  intrinsic(5) = 0.07395907;
  intrinsic(6) = 0.00019359;
  intrinsic(7) = 1.76187114e-05;

  vecx_t extrinsic;
  extrinsic.resize(7);
  extrinsic.setZero();
  extrinsic(6) = 1.0;

  return std::make_shared<CameraGeometry>(camera_index,
                                          camera_model,
                                          resolution,
                                          intrinsic,
                                          extrinsic);
}

static std::shared_ptr<CameraGeometry> setup_euroc_camera1() {
  const int camera_index = 1;
  const vec2i_t resolution{752, 480};
  const std::string camera_model = "pinhole-radtan4";

  vecx_t intrinsic;
  intrinsic.resize(8);
  intrinsic(0) = 457.587;
  intrinsic(1) = 456.134;
  intrinsic(2) = 379.999;
  intrinsic(3) = 255.238;
  intrinsic(4) = -0.28368365;
  intrinsic(5) = 0.07451284;
  intrinsic(6) = -0.00010473;
  intrinsic(7) = -3.555e-05;

  vecx_t extrinsic;
  extrinsic.resize(7);
  extrinsic.setZero();
  extrinsic(6) = 1.0;

  return std::make_shared<CameraGeometry>(camera_index,
                                          camera_model,
                                          resolution,
                                          intrinsic,
                                          extrinsic);
}

TEST(CameraChain, construct) {
  // Load data
  CalibData calib_data{TEST_CONFIG};
  auto camera_data = calib_data.getAllCameraData();

  // Setup camera geometries
  auto camera0 = setup_euroc_camera0();
  auto camera1 = setup_euroc_camera1();
  std::map<int, CameraGeometryPtr> camera_geometries;
  camera_geometries[0] = camera0;
  camera_geometries[1] = camera1;

  // Construct camera chain
  mat4_t T_CiCj;
  CameraChain camchain{camera_geometries, camera_data};
  ASSERT_EQ(camchain.find(0, 1, T_CiCj), 0);

  print_matrix("T_CiCj", T_CiCj);
}

} // namespace yac
