#include <iostream>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "autocal/core/core.hpp"
#include "autocal/cv/PinholeCamera.hpp"
#include "autocal/cv/CameraGeometry.hpp"
#include "autocal/cv/NoDistortion.hpp"
#include "autocal/cv/RadialTangentialDistortion.hpp"
#include "autocal/cv/EquidistantDistortion.hpp"

using namespace autocal;
using namespace autocal::camera;

TEST(PinholeCamera, functions)
{
  const size_t NUM_POINTS = 100;

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<CameraBase>> cameras;
  // cameras.push_back(PinholeRadtan::createTestObject());
  cameras.push_back(PinholeEqui::createTestObject());

  for (size_t c = 0; c < cameras.size(); ++c) {
    // try quite a lot of points:
    for (size_t i = 0; i < NUM_POINTS; ++i) {
      // create a random point in the field of view:
      Eigen::Vector2d imagePoint = cameras.at(c)->createRandomImagePoint();

      // backProject
      Eigen::Vector3d ray;
      ASSERT_TRUE(cameras.at(c)->backProject(imagePoint, &ray)) <<
                        "unsuccessful back projection";

      // randomise distance
      ray.normalize();
      ray *= (0.2 + 8 * (Eigen::Vector2d::Random()[0] + 1.0));

      // Get camera parameters
      vecx_t cam_params;
      cameras[0]->getIntrinsics(cam_params);
      // vecx_t proj_params = cam_params.head(4);
      // vecx_t dist_params = cam_params.tail(4);

      // project
      Eigen::Vector2d imagePoint2;
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Matrix2Xd J_intrinsics;
      CameraBase::ProjectionStatus status;
      status = cameras.at(c)->projectWithExternalParameters(
        ray,
        cam_params,
        &imagePoint2,
        &J,
        &J_intrinsics
      );

      ASSERT_TRUE(status == CameraBase::ProjectionStatus::Successful) << "unsuccessful projection";

      // check they are the same
      ASSERT_TRUE((imagePoint2 - imagePoint).norm() < 0.01) <<
                        "project/unproject failure";

      // check point Jacobian vs. NumDiff
      const double dp = 1.0e-7;
      Eigen::Matrix<double, 2, 3> J_numDiff;
      for (size_t d = 0; d < 3; ++d) {
        Eigen::Vector3d point_p = ray
            + Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector3d point_m = ray
            - Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        cameras.at(c)->projectWithExternalParameters(point_p, cam_params, &imagePoint_p);
        cameras.at(c)->projectWithExternalParameters(point_m, cam_params, &imagePoint_m);
        J_numDiff.col(d) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      // print_matrix("J_numDiff", J_numDiff);
      // print_matrix("J", J);
      // std::cout << std::endl;
      ASSERT_TRUE((J_numDiff - J).norm() < 0.0001) <<
                        "Jacobian Verification failed";

      // check intrinsics Jacobian
      const int numIntrinsics = cameras.at(c)->noIntrinsicsParameters();
      Eigen::VectorXd intrinsics;
      cameras.at(c)->getIntrinsics(intrinsics);
      Eigen::Matrix2Xd J_numDiff_intrinsics;
      J_numDiff_intrinsics.resize(2,numIntrinsics);
      for (int d = 0; d < numIntrinsics; ++d) {
        Eigen::VectorXd di;
        di.resize(numIntrinsics);
        di.setZero();
        di[d] = dp;
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        Eigen::VectorXd intrinsics_p = intrinsics+di;
        Eigen::VectorXd intrinsics_m = intrinsics-di;
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_p, &imagePoint_p);
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_m, &imagePoint_m);
        J_numDiff_intrinsics.col(d) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }

      ASSERT_TRUE((J_numDiff_intrinsics - J_intrinsics).norm() < 0.0001) <<
          "Jacobian verification failed";
    }
  }
}

