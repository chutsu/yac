CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(proto_ros)

# DEPENDENCIES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
SET(DEPS yaml-cpp ceres apriltags ${OpenCV_LIBS})

# BUILD LIBRARY
INCLUDE_DIRECTORIES(.)
ADD_LIBRARY(
  yac
  yac/aprilgrid.cpp
  yac/calib_camera.cpp
  yac/calib_data.cpp
  yac/core.cpp
)

ADD_EXECUTABLE(calib_camera yac/apps/calib_camera.cpp)
TARGET_LINK_LIBRARIES(calib_camera yac ${DEPS})

ADD_EXECUTABLE(calib_stereo yac/apps/calib_stereo.cpp)
TARGET_LINK_LIBRARIES(calib_stereo yac ${DEPS})
