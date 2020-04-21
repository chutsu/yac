#include "proto/proto.hpp"
#include "proto/ros/ros.hpp"

using namespace proto;

void process_rosbag(const std::string &rosbag_path,
                    const std::string &cam0_topic,
                    const std::string &out_path) {
  // Check output dir
  if (dir_exists(out_path) == false) {
    if (dir_create(out_path) != 0) {
      FATAL("Failed to create dir [%s]", out_path.c_str());
    }
  }

  // Prepare data files
  const auto cam0_output_path = out_path + "/cam0";
  auto cam0_csv = camera_init_output_file(cam0_output_path);

  // Open ROS bag
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  // Process ROS bag
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  rosbag::View bag_view(bag);
  size_t msg_idx = 0;
  for (const auto &msg : bag_view) {
    // Print progress
    print_progress((double) msg_idx / bag_view.size());
    msg_idx++;

    // Process camera data
    if (msg.getTopic() == cam0_topic) {
      image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);
    }
  }

  // Clean up rosbag
  print_progress(1.0);
  bag.close();
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string config_file;
  std::string rosbag_path;
  std::string cam0_topic;
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
  ROS_PARAM(ros_nh, node_name + "/rosbag", rosbag_path);
  ROS_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);

  // Parse config file
  std::string data_path;
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);

  // Process rosbag
  process_rosbag(rosbag_path, cam0_topic, data_path);

  // Calibrate camera intrinsics
  if (calib_camera_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  return 0;
}
