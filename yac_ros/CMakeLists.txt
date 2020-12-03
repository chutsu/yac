CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(yac_ros)

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
SET(YAC_DEPS yaml-cpp ceres apriltag3 apriltags ${OpenCV_LIBS})

FIND_PACKAGE(OpenMP REQUIRED)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  ADD_DEFINITIONS(-DUSE_OPENMP=1)
ENDIF()

# CATKIN DEPENDENCIES
CMAKE_POLICY(SET CMP0054 NEW)
FIND_PACKAGE(
  catkin
  REQUIRED
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
    yac
)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# LINK
SET(
  DEPS
  yac_ros
  ${catkin_LIBRARIES}
  ${YAC_DEPS}
)

# CATKING PACKAGE
CATKIN_PACKAGE(
  CATKIN_DEPENDS
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
  LIBRARIES
    yac_ros
)

# YAC ROS
ADD_LIBRARY(yac_ros ros.cpp)
INSTALL(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ROS NODES
ADD_EXECUTABLE(calib_capture_node calib_capture_node.cpp)
TARGET_LINK_LIBRARIES(calib_capture_node ${DEPS})

ADD_EXECUTABLE(calib_mono_node calib_mono_node.cpp)
TARGET_LINK_LIBRARIES(calib_mono_node ${DEPS})

# ADD_EXECUTABLE(calib_mono_nbv_node calib_mono_nbv_node.cpp)
# TARGET_LINK_LIBRARIES(calib_mono_nbv_node ${DEPS})

# ADD_EXECUTABLE(calib_stereo_node calib_stereo_node.cpp)
# TARGET_LINK_LIBRARIES(calib_stereo_node ${DEPS})

ADD_EXECUTABLE(calib_mocap_capture_node calib_mocap_capture_node.cpp)
TARGET_LINK_LIBRARIES(calib_mocap_capture_node ${DEPS})

ADD_EXECUTABLE(calib_mocap_node calib_mocap_node.cpp)
TARGET_LINK_LIBRARIES(calib_mocap_node ${DEPS})
