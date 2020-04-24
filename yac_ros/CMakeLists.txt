CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(yac_ros)

# DEPENDENCIES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
SET(YAC_DEPS yaml-cpp ceres apriltags ${OpenCV_LIBS})

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
ADD_EXECUTABLE(calib_mono_node calib_mono_node.cpp)
TARGET_LINK_LIBRARIES(calib_mono_node ${DEPS})

# ADD_EXECUTABLE(calib_mono_validate_node calib_mono_validate_node.cpp)
# TARGET_LINK_LIBRARIES(calib_mono_validate_node ${DEPS})

ADD_EXECUTABLE(calib_stereo_node calib_stereo_node.cpp)
TARGET_LINK_LIBRARIES(calib_stereo_node ${DEPS})

# ADD_EXECUTABLE(calib_stereo_validate_node calib_stereo_validate_node.cpp)
# TARGET_LINK_LIBRARIES(calib_stereo_validate_node ${DEPS})

ADD_EXECUTABLE(calib_vicon_capture_node calib_vicon_capture_node.cpp)
TARGET_LINK_LIBRARIES(calib_vicon_capture_node ${DEPS})

ADD_EXECUTABLE(calib_vicon_node calib_vicon_node.cpp)
TARGET_LINK_LIBRARIES(calib_vicon_node ${DEPS})
