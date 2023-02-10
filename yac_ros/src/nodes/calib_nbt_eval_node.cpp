#include "calib_nbt_eval_node.hpp"

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  yac::calib_nbt_eval_t node{node_name};

  return 0;
}
