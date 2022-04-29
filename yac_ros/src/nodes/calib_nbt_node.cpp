#include "calib_nbv_node.hpp"
#include "calib_nbt_node.hpp"

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Calibrate cameras
  { yac::calib_nbv_t nbv{node_name}; }
  // IMPORTANT NOTE: The brackets around calib_nbt_t() are necessary to trigger
  // the destructor on finish.

  // Calibrate imu-camera
  yac::calib_nbt_t nbt{node_name};

  return 0;
}
