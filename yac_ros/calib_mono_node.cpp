#include "yac.hpp"
#include "ros.hpp"

using namespace yac;

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
    if (msg.getTopic() == cam0_topic) {
      // Handle image message
      image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);

      // Print progress
      if (msg_idx % 10 == 0) {
        printf(".");
      }
      msg_idx++;
    }
  }
  printf("\n");

  // Clean up rosbag
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
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

  // Parse config file
  std::string bag_path;
  std::string cam0_topic;
  std::string data_path;
  config_t config{config_file};
  parse(config, "ros.bag", bag_path);
  parse(config, "ros.cam0_topic", cam0_topic);
  parse(config, "settings.data_path", data_path);

  // Process rosbag
  process_rosbag(bag_path, cam0_topic, data_path);

  // Calibrate camera intrinsics
  if (calib_mono_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  return 0;
}
