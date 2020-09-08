CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(yac)

# DEPENDENCIES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
SET(DEPS yaml-cpp ceres apriltags ${OpenCV_LIBS})

FIND_PACKAGE(catkin REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
CATKIN_PACKAGE(INCLUDE_DIRS lib LIBRARIES yac)
INSTALL(DIRECTORY lib/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN ".h" PATTERN ".hpp"
)

# LIBRARY
INCLUDE_DIRECTORIES(lib)
ADD_LIBRARY(
  yac
  lib/core.cpp
  lib/aprilgrid.cpp
  lib/calib_data.cpp
  lib/calib_mono.cpp
  lib/calib_stereo.cpp
  # lib/calib_mocap_marker.cpp
)

# TESTS
SET(TEST_BIN_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
FILE(COPY tests/test_data DESTINATION ${TEST_BIN_PATH})
ADD_DEFINITIONS(-DTEST_PATH="${TEST_BIN_PATH}")

ADD_EXECUTABLE(test_aprilgrid tests/test_aprilgrid.cpp)
TARGET_LINK_LIBRARIES(test_aprilgrid yac ${DEPS})

ADD_EXECUTABLE(test_calib_mono tests/test_calib_mono.cpp)
TARGET_LINK_LIBRARIES(test_calib_mono yac ${DEPS})

ADD_EXECUTABLE(test_calib_data tests/test_calib_data.cpp)
TARGET_LINK_LIBRARIES(test_calib_data yac ${DEPS})

ADD_EXECUTABLE(test_calib_stereo tests/test_calib_stereo.cpp)
TARGET_LINK_LIBRARIES(test_calib_stereo yac ${DEPS})
