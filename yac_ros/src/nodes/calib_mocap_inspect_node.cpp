#include "calib_mocap_inspect_node.hpp"

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Run ROS node
  ros::NodeHandle nh;
  std::string calib_file;
  std::string camera_topic;
  std::string mocap_topic;
  ROS_PARAM(nh, node_name + "/calib_file", calib_file);
  ROS_PARAM(nh, node_name + "/camera_topic", camera_topic);
  ROS_PARAM(nh, node_name + "/mocap_topic", mocap_topic);
  calib_mocap_inspector_t node{calib_file, camera_topic, mocap_topic};
  node.loop();

  return 0;
}
