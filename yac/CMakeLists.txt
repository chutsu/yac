CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(yac)

# Tell cmake where to find custom AprilTag3 and Ceres-Solver
SET(apriltag3_DIR "../deps/share/apriltag3/cmake")
# SET(Ceres_DIR "../deps/lib/cmake/Ceres")

# DEPENDENCIES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES("../deps/include")
LINK_DIRECTORIES("../deps/lib")
SET(DEPS yaml-cpp glog ceres apriltag3 apriltags ${OpenCV_LIBS})

FIND_PACKAGE(OpenMP REQUIRED)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  ADD_DEFINITIONS(-DUSE_OPENMP=1)
ENDIF()

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
  lib/calib_data.cpp
  lib/calib_mocap.cpp
  lib/calib_mono.cpp
  lib/calib_params.cpp
  lib/calib_stereo.cpp
  lib/ceres_utils.cpp
  lib/core.cpp
)

# TESTS
SET(TEST_BIN_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
FILE(COPY tests/test_data DESTINATION ${TEST_BIN_PATH})
FILE(COPY tests/test_data/mocap_data DESTINATION /tmp)
ADD_DEFINITIONS(-DTEST_PATH="${TEST_BIN_PATH}")

ADD_EXECUTABLE(test_aprilgrid tests/test_aprilgrid.cpp)
TARGET_LINK_LIBRARIES(test_aprilgrid yac ${DEPS})

ADD_EXECUTABLE(test_calib_mocap tests/test_calib_mocap.cpp)
TARGET_LINK_LIBRARIES(test_calib_mocap yac ${DEPS})

ADD_EXECUTABLE(test_calib_mono tests/test_calib_mono.cpp)
TARGET_LINK_LIBRARIES(test_calib_mono yac ${DEPS})

ADD_EXECUTABLE(test_calib_data tests/test_calib_data.cpp)
TARGET_LINK_LIBRARIES(test_calib_data yac ${DEPS})

ADD_EXECUTABLE(test_calib_stereo tests/test_calib_stereo.cpp)
TARGET_LINK_LIBRARIES(test_calib_stereo yac ${DEPS})

ADD_EXECUTABLE(test_calib_stereo_inc tests/test_calib_stereo_inc.cpp)
TARGET_LINK_LIBRARIES(test_calib_stereo_inc yac ${DEPS})

ADD_EXECUTABLE(test_calib_vi tests/test_calib_vi.cpp)
TARGET_LINK_LIBRARIES(test_calib_vi yac ${DEPS})

ADD_EXECUTABLE(test_calib_nbv tests/test_calib_nbv.cpp)
TARGET_LINK_LIBRARIES(test_calib_nbv yac ${DEPS})

ADD_EXECUTABLE(test_marg_error tests/test_marg_error.cpp)
TARGET_LINK_LIBRARIES(test_marg_error yac ${DEPS})
