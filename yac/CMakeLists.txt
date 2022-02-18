CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (POLICY CMP0048)
  CMAKE_POLICY(SET CMP0048 NEW)
ENDIF (POLICY CMP0048)
PROJECT(yac)

# GENERAL SETTINGS
SET(USE_OPENMP ON)
SET(apriltag3_DIR "../deps/share/apriltag3/cmake")  # Use custom AprilTag3

# COMPILATION SETTINGS
SET(CMAKE_CXX_STANDARD 17)
# ADD_COMPILE_OPTIONS(
#   -Wall
#   -Wextra
#   -fsanitize=address
#   -fno-omit-frame-pointer
# )
# LINK_LIBRARIES(
#   -fsanitize=address
#   -static-libasan # Use -static-libsan for Clang
# )

# DEPENDENCIES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES("../deps/include")
LINK_DIRECTORIES("../deps/lib")
SET(DEPS yaml-cpp glog ceres apriltag3 apriltag ${OpenCV_LIBS})

IF(USE_OPENMP)
  FIND_PACKAGE(OpenMP REQUIRED)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    ADD_DEFINITIONS(-DUSE_OPENMP=1)
  ENDIF(OPENMP_FOUND)
ENDIF(USE_OPENMP)

FIND_PACKAGE(catkin REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
CATKIN_PACKAGE(INCLUDE_DIRS lib LIBRARIES yac yac_util)
INSTALL(DIRECTORY lib/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN ".h" PATTERN ".hpp"
)

# LIBRARY
INCLUDE_DIRECTORIES(lib)
ADD_LIBRARY(
  yac_util
  lib/util/aprilgrid.cpp
  lib/util/ceres_utils.cpp
  lib/util/config.cpp
  lib/util/core.cpp
  lib/util/cv.cpp
  lib/util/data.cpp
  lib/util/fs.cpp
  lib/util/net.cpp
  lib/util/sim.cpp
  lib/util/timeline.cpp
)
ADD_LIBRARY(
  yac
  lib/calib_camera.cpp
  lib/calib_data.cpp
  lib/calib_errors.cpp
  # lib/calib_mocap.cpp
  lib/calib_nbt.cpp
  lib/calib_nbv.cpp
  lib/calib_params.cpp
  lib/calib_vi.cpp
)

# TESTS
SET(TEST_BIN_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
FILE(COPY tests/test_data DESTINATION ${TEST_BIN_PATH})
FILE(COPY tests/test_data/mocap_data DESTINATION /tmp)
ADD_DEFINITIONS(-DTEST_PATH="${TEST_BIN_PATH}")

ADD_EXECUTABLE(test_util_aprilgrid tests/util/test_aprilgrid.cpp)
TARGET_LINK_LIBRARIES(test_util_aprilgrid yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_config tests/util/test_config.cpp)
TARGET_LINK_LIBRARIES(test_util_config yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_core tests/util/test_core.cpp)
TARGET_LINK_LIBRARIES(test_util_core yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_cv tests/util/test_cv.cpp)
TARGET_LINK_LIBRARIES(test_util_cv yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_data tests/util/test_data.cpp)
TARGET_LINK_LIBRARIES(test_util_data yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_fs tests/util/test_fs.cpp)
TARGET_LINK_LIBRARIES(test_util_fs yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_net tests/util/test_net.cpp)
TARGET_LINK_LIBRARIES(test_util_net yac yac_util ${DEPS})

ADD_EXECUTABLE(test_util_timeline tests/util/test_timeline.cpp)
TARGET_LINK_LIBRARIES(test_util_timeline yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_camera tests/test_calib_camera.cpp)
TARGET_LINK_LIBRARIES(test_calib_camera yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_data tests/test_calib_data.cpp)
TARGET_LINK_LIBRARIES(test_calib_data yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_errors tests/test_calib_errors.cpp)
TARGET_LINK_LIBRARIES(test_calib_errors yac yac_util ${DEPS})

# ADD_EXECUTABLE(test_calib_mocap tests/test_calib_mocap.cpp)
# TARGET_LINK_LIBRARIES(test_calib_mocap yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_nbt tests/test_calib_nbt.cpp)
TARGET_LINK_LIBRARIES(test_calib_nbt yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_nbv tests/test_calib_nbv.cpp)
TARGET_LINK_LIBRARIES(test_calib_nbv yac yac_util ${DEPS})

ADD_EXECUTABLE(test_calib_vi tests/test_calib_vi.cpp)
TARGET_LINK_LIBRARIES(test_calib_vi yac yac_util ${DEPS})

# DEMO
ADD_EXECUTABLE(calib_euroc demo/calib_euroc.cpp)
TARGET_LINK_LIBRARIES(calib_euroc yac yac_util ${DEPS})

ADD_EXECUTABLE(calib_inspect demo/calib_inspect.cpp)
TARGET_LINK_LIBRARIES(calib_inspect yac yac_util ${DEPS})
