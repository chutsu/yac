CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (POLICY CMP0048)
  CMAKE_POLICY(SET CMP0048 NEW)
ENDIF (POLICY CMP0048)
PROJECT(yac)

# COMPILATION SETTINGS
SET(USE_ADDRESS_SANITIZER OFF)
SET(CMAKE_CXX_STANDARD 17)
ADD_COMPILE_OPTIONS(-Wall -Wextra)

# ADDRESS SANITIZER
IF (USE_ADDRESS_SANITIZER)
  MESSAGE(AUTHOR_WARNING "Building with address sanitizer - This will make executables 2x slower!")
  ADD_COMPILE_OPTIONS(
    -fsanitize=address
    -fno-omit-frame-pointer
  )
  LINK_LIBRARIES(
    -fsanitize=address
    -static-libasan # Use -static-libsan for Clang
  )
ENDIF()

# DEPENDENCIES
LIST(APPEND CMAKE_MODULE_PATH "../cmake")

IF (NOT DEFINED apriltag3_DIR)
  SET(apriltag3_DIR "/opt/yac/share/apriltag3/cmake")  # Use custom AprilTag3
ENDIF()

IF (NOT DEFINED USE_OPENMP)
  SET(USE_OPENMP ON)
ENDIF()

IF(USE_OPENMP)
  FIND_PACKAGE(OpenMP REQUIRED)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    ADD_DEFINITIONS(-DUSE_OPENMP=1)
  ENDIF(OPENMP_FOUND)
ENDIF(USE_OPENMP)

FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(TBB REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED CHOLMOD SPQR)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SUITESPARSEQR_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SUITESPARSE_CONFIG_INCLUDE_DIR})
INCLUDE_DIRECTORIES("/opt/yac/include")
LINK_DIRECTORIES("/opt/yac/lib")
SET(DEPS
  yaml-cpp
  glog
  ceres
  apriltag3
  apriltag
  ${OpenCV_LIBS}
	${CHOLMOD_LIBRARY}
	${SUITESPARSEQR_LIBRARY}
	${SUITESPARSE_CONFIG_LIBRARY}
	${TBB_LIBRARY}
)

# LIBRARY
INCLUDE_DIRECTORIES(lib)
ADD_LIBRARY(
  yac
  SHARED
  lib/util/aprilgrid.cpp
  lib/util/ceres_utils.cpp
  lib/util/config.cpp
  lib/util/core.cpp
  lib/util/cv.cpp
  lib/util/data.cpp
  lib/util/fs.cpp
  lib/util/net.cpp
  lib/util/timeline.cpp
  lib/calib_camera.cpp
  lib/calib_data.cpp
  lib/calib_loss.cpp
  lib/calib_residuals.cpp
  lib/calib_mocap.cpp
  lib/calib_nbt.cpp
  lib/calib_nbv.cpp
  lib/calib_params.cpp
  lib/calib_vi.cpp
  lib/solver.cpp
  lib/suitesparse.cpp
)
INSTALL(TARGETS yac
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/lib/
        DESTINATION include/yac
        FILES_MATCHING PATTERN "*.h*")

# TESTS
SET(TEST_BIN_PATH ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY tests/test_data DESTINATION ${TEST_BIN_PATH})
ADD_DEFINITIONS(-DTEST_PATH="${TEST_BIN_PATH}")

ADD_EXECUTABLE(test_util_aprilgrid tests/util/test_aprilgrid.cpp)
TARGET_LINK_LIBRARIES(test_util_aprilgrid yac ${DEPS})

ADD_EXECUTABLE(test_util_config tests/util/test_config.cpp)
TARGET_LINK_LIBRARIES(test_util_config yac ${DEPS})

ADD_EXECUTABLE(test_util_core tests/util/test_core.cpp)
TARGET_LINK_LIBRARIES(test_util_core yac ${DEPS})

ADD_EXECUTABLE(test_util_cv tests/util/test_cv.cpp)
TARGET_LINK_LIBRARIES(test_util_cv yac ${DEPS})

ADD_EXECUTABLE(test_util_data tests/util/test_data.cpp)
TARGET_LINK_LIBRARIES(test_util_data yac ${DEPS})

ADD_EXECUTABLE(test_util_fs tests/util/test_fs.cpp)
TARGET_LINK_LIBRARIES(test_util_fs yac ${DEPS})

ADD_EXECUTABLE(test_util_net tests/util/test_net.cpp)
TARGET_LINK_LIBRARIES(test_util_net yac ${DEPS})

ADD_EXECUTABLE(test_util_timeline tests/util/test_timeline.cpp)
TARGET_LINK_LIBRARIES(test_util_timeline yac ${DEPS})

ADD_EXECUTABLE(test_calib_camera tests/test_calib_camera.cpp)
TARGET_LINK_LIBRARIES(test_calib_camera yac ${DEPS})

ADD_EXECUTABLE(test_calib_data tests/test_calib_data.cpp)
TARGET_LINK_LIBRARIES(test_calib_data yac ${DEPS})

ADD_EXECUTABLE(test_calib_residuals tests/test_calib_residuals.cpp)
TARGET_LINK_LIBRARIES(test_calib_residuals yac ${DEPS})

ADD_EXECUTABLE(test_calib_mocap tests/test_calib_mocap.cpp)
TARGET_LINK_LIBRARIES(test_calib_mocap yac ${DEPS})

ADD_EXECUTABLE(test_calib_nbt tests/test_calib_nbt.cpp)
TARGET_LINK_LIBRARIES(test_calib_nbt yac ${DEPS})

ADD_EXECUTABLE(test_calib_nbv tests/test_calib_nbv.cpp)
TARGET_LINK_LIBRARIES(test_calib_nbv yac ${DEPS})

ADD_EXECUTABLE(test_calib_vi tests/test_calib_vi.cpp)
TARGET_LINK_LIBRARIES(test_calib_vi yac ${DEPS})

ADD_EXECUTABLE(test_solver tests/test_solver.cpp)
TARGET_LINK_LIBRARIES(test_solver yac ${DEPS})

# DEMO
ADD_EXECUTABLE(calib_camera demo/calib_camera.cpp)
TARGET_LINK_LIBRARIES(calib_camera yac ${DEPS})
INSTALL(TARGETS calib_camera)

ADD_EXECUTABLE(calib_euroc demo/calib_euroc.cpp)
TARGET_LINK_LIBRARIES(calib_euroc yac ${DEPS})
INSTALL(TARGETS calib_euroc)

ADD_EXECUTABLE(calib_info demo/calib_info.cpp)
TARGET_LINK_LIBRARIES(calib_info yac ${DEPS})
INSTALL(TARGETS calib_info)

ADD_EXECUTABLE(calib_inspect demo/calib_inspect.cpp)
TARGET_LINK_LIBRARIES(calib_inspect yac ${DEPS})
INSTALL(TARGETS calib_inspect)

ADD_EXECUTABLE(calib_mocap demo/calib_mocap.cpp)
TARGET_LINK_LIBRARIES(calib_mocap yac ${DEPS})
INSTALL(TARGETS calib_mocap)

ADD_EXECUTABLE(calib_vi demo/calib_vi.cpp)
TARGET_LINK_LIBRARIES(calib_vi yac ${DEPS})
INSTALL(TARGETS calib_vi)
