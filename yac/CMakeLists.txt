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
  lib/calib_camera.cpp
  lib/calib_stereo.cpp
)

# APPS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/apps)
ADD_EXECUTABLE(calib_camera apps/calib_camera.cpp)
TARGET_LINK_LIBRARIES(calib_camera yac ${DEPS})

ADD_EXECUTABLE(calib_stereo apps/calib_stereo.cpp)
TARGET_LINK_LIBRARIES(calib_stereo yac ${DEPS})

# TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(test_aprilgrid tests/test_aprilgrid.cpp)
TARGET_LINK_LIBRARIES(test_aprilgrid yac ${DEPS})

ADD_EXECUTABLE(test_calib_camera tests/test_calib_camera.cpp)
TARGET_LINK_LIBRARIES(test_calib_camera yac ${DEPS})

ADD_EXECUTABLE(test_calib_data tests/test_calib_data.cpp)
TARGET_LINK_LIBRARIES(test_calib_data yac ${DEPS})

ADD_EXECUTABLE(test_calib_stereo tests/test_calib_stereo.cpp)
TARGET_LINK_LIBRARIES(test_calib_stereo yac ${DEPS})
