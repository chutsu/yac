#include "yac/yac.hpp"
#include "yac_ros/ros.hpp"

void process_rosbag(const std::string &rosbag_path,
                    const std::string &cam0_topic,
                    const std::string &cam1_topic,
                    const std::string &out_path) {
  // Check output dir
  if (yac::dir_exists(out_path) == false) {
    if (yac::dir_create(out_path) != 0) {
      FATAL("Failed to create dir [%s]", out_path.c_str());
    }
  }

  // Prepare data files
  const auto cam0_output_path = out_path + "/cam0";
  const auto cam1_output_path = out_path + "/cam1";
  auto cam0_csv = yac::camera_init_output_file(cam0_output_path);
  auto cam1_csv = yac::camera_init_output_file(cam1_output_path);

  // Open ROS bag
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  // Process ROS bag
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  rosbag::View bag_view(bag);
  size_t msg_idx = 0;
  for (const auto &msg : bag_view) {
    // Print progress
    yac::print_progress((double) msg_idx / bag_view.size());
    msg_idx++;

    // Process cam0 data
    if (msg.getTopic() == cam0_topic) {
      yac::image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);
    }

    // Process cam1 data
    if (msg.getTopic() == cam1_topic) {
      yac::image_message_handler(msg, cam1_output_path + "/data/", cam1_csv);
    }
  }

  // Clean up rosbag
  yac::print_progress(1.0);
  bag.close();
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string config_file;
  std::string rosbag_path;
  std::string cam0_topic;
  std::string cam1_topic;
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
  ROS_PARAM(ros_nh, node_name + "/rosbag", rosbag_path);
  ROS_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(ros_nh, node_name + "/cam1_topic", cam1_topic);

  // Parse config file
  std::string data_path;
  yac::config_t config{config_file};
  yac::parse(config, "settings.data_path", data_path);

  // Process rosbag
  process_rosbag(rosbag_path, cam0_topic, cam1_topic, data_path);

  // Calibrate camera intrinsics
  if (yac::calib_stereo_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  return 0;
}
