CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(autocal)

# C++ COMPILER SETTINGS
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(BUILD_SHARED_LIBS ON)

# Tell cmake where to find custom
# - AprilTag3
SET(apriltag_DIR "../deps/share/apriltag3/cmake")

# DEPENDENCIES
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
# FIND_PACKAGE(apriltag REQUIRED)
FIND_PACKAGE(GTest REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${apriltag_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

FIND_PACKAGE(OpenMP REQUIRED)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  ADD_DEFINITIONS(-DUSE_OPENMP=1)
ENDIF()

FIND_PROGRAM(CCACHE_FOUND ccache)
IF(CCACHE_FOUND)
  SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
ENDIF(CCACHE_FOUND)

# CATKIN
FIND_PACKAGE(catkin REQUIRED yac)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
CATKIN_PACKAGE(
	CATKIN_DEPENDS yac
	INCLUDE_DIRS include
	LIBRARIES yac ${PROJECT_NAME}
)

# LIBRARY
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES("../deps/include")
LINK_DIRECTORIES("../deps/lib")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_LIBRARY(
  ${PROJECT_NAME}
  # -- cv
	src/cv/CameraBase.cpp
	src/cv/EquidistantDistortion.cpp
  src/cv/RadialTangentialDistortion.cpp
  # -- common
  # src/common/AprilGrid.cpp
  # src/common/CalibData.cpp
  src/common/Common.cpp
  # src/common/Core.cpp
  src/common/Calibrator.cpp
  src/common/Duration.cpp
  # src/common/EuRoC.cpp
  # src/common/NBT.cpp
  src/common/Time.cpp
  src/common/Transformation.cpp
  # ---- error
  src/error/ImuError.cpp
  src/error/PoseError.cpp
  src/error/SpeedAndBiasError.cpp
  src/error/MarginalizationError.cpp
  # ---- param
  src/param/CameraParameterBlock.cpp
  src/param/LocalParamizationAdditionalInterfaces.cpp
  src/param/PoseLocalParameterization.cpp
  src/param/PoseParameterBlock.cpp
  src/param/SpeedAndBiasParameterBlock.cpp
  src/param/TimeDelayParameterBlock.cpp
  # -- Estimator
  src/Estimator.cpp
  src/Map.cpp
)
SET(AUTOCAL_DEPS
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  ${CERES_LIBRARIES}
  apriltag3
  apriltags
  yaml-cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${AUTOCAL_DEPS})
INSTALL(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # TESTS
# # -- UNIT TESTS
# FILE(COPY tests/test_data
#      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
# SET(
# 	UNITTESTS
#   # common-TestAprilGrid
#   # ceres-TestCalibrator
#   ceres-TestEstimator
#   # ceres-TestImuError
#   # ceres-TestCalibReprojError
#   # ceres-TestNBT
#   # cv-TestPinhole
# )
# FOREACH(TEST ${UNITTESTS})
#   STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
#   ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp tests/test_runner.cpp)
# 	TARGET_LINK_LIBRARIES(
# 		${TEST}
# 		${PROJECT_NAME}
#     ${AUTOCAL_DEPS}
# 		${GTEST_BOTH_LIBRARIES}
# 		glog
# 		pthread
# 	)
# ENDFOREACH(TEST)
