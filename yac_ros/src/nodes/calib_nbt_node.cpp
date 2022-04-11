#include "calib_nbv_node.hpp"
#include "calib_nbt_node.hpp"

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start calibrating
  {
    yac::calib_nbv_t nbv{node_name};
    nbv.loop();
  }

  // Start calibrating
  {
    yac::calib_nbt_t nbt{node_name};
    nbt.loop();
  }

  return 0;
}
