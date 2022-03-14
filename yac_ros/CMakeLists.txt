CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (POLICY CMP0048)
  CMAKE_POLICY(SET CMP0048 NEW)
ENDIF (POLICY CMP0048)
PROJECT(yac_ros)

# COMPILATION SETTINGS
SET(CMAKE_CXX_STANDARD 17)
ADD_COMPILE_OPTIONS(
  -Wall
  -Wextra
  # -fsanitize=address
  # -fno-omit-frame-pointer
)
# LINK_LIBRARIES(
#   -fsanitize=address
#   -static-libasan # Use -static-libsan for Clang
# )

# DEPENDENCIES
IF (NOT DEFINED Eigen3_DIR)
  SET(Eigen3_DIR "/opt/yac/share/eigen3/cmake")
ENDIF()

IF (NOT DEFINED apriltag3_DIR)
  SET(apriltag3_DIR "/opt/yac/share/apriltag3/cmake")  # Use custom AprilTag3
ENDIF()

IF (NOT DEFINED Ceres_DIR OR "${Ceres_DIR}" STREQUAL "Ceres_DIR-NOTFOUND")
  SET(Ceres_DIR "/opt/yac/lib/cmake/Ceres")
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
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES("/opt/yac/include")
LINK_DIRECTORIES("/opt/yac/lib")

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
		tf2
		tf2_ros
		tf2_geometry_msgs
)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

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
		tf2
		tf2_ros
		tf2_geometry_msgs
  LIBRARIES
    yac_ros
)

# LINK
SET(YAC_LIBRARIES yaml-cpp glog ceres apriltag3 apriltag ${OpenCV_LIBS} cholmod spqr)
SET(DEPS yac_ros yac ${catkin_LIBRARIES} ${YAC_LIBRARIES})

# YAC ROS
ADD_LIBRARY(
  yac_ros
  src/ros_calib.cpp
  src/ros_utils.cpp
)
INSTALL(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ROS NODES
ADD_EXECUTABLE(calib_camera_node src/nodes/calib_camera_node.cpp)
TARGET_LINK_LIBRARIES(calib_camera_node ${DEPS})

ADD_EXECUTABLE(calib_imucam_node src/nodes/calib_imucam_node.cpp)
TARGET_LINK_LIBRARIES(calib_imucam_node ${DEPS})

# ADD_EXECUTABLE(calib_mocap_node src/nodes/calib_mocap_node.cpp)
# TARGET_LINK_LIBRARIES(calib_mocap_node ${DEPS})

ADD_EXECUTABLE(calib_nbt_node src/nodes/calib_nbt_node.cpp)
TARGET_LINK_LIBRARIES(calib_nbt_node ${DEPS})

ADD_EXECUTABLE(calib_nbv_node src/nodes/calib_nbv_node.cpp)
TARGET_LINK_LIBRARIES(calib_nbv_node ${DEPS})

ADD_EXECUTABLE(record_camera_node src/nodes/record_camera_node.cpp)
TARGET_LINK_LIBRARIES(record_camera_node ${DEPS})

ADD_EXECUTABLE(record_mocap_node src/nodes/record_mocap_node.cpp)
TARGET_LINK_LIBRARIES(record_mocap_node ${DEPS})
