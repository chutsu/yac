#include "yac/yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;

void process_rosbag(const std::string &config_file) {
  // Parse config file
  config_t config{config_file};

  std::string rosbag_path;
  std::map<int, std::string> cam_topics;
  std::string data_path;
  parse(config, "ros.bag", rosbag_path);
  parse_camera_topics(config, cam_topics);
  parse(config, "settings.data_path", data_path);

  // Check output dir
  if (dir_exists(data_path) == false) {
    if (dir_create(data_path) != 0) {
      FATAL("Failed to create dir [%s]", data_path.c_str());
    }
  }

  // Prepare data files and directories
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  std::map<int, std::ofstream> cam_csvs;
  for (const auto [cam_idx, cam_topic] : cam_topics) {
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    const auto cam_path = data_path + "/" + cam_str;
    if (dir_create(cam_path) != 0) {
      FATAL("Failed to create dir [%s]", cam_path.c_str());
    }

    cam_csvs[cam_idx] = camera_init_output_file(cam_path);
    LOG_INFO("- %s topic [%s]", cam_str.c_str(), cam_topic.c_str());
  }

  // Open ROS bag
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  // Process ROS bag
  rosbag::View bag_view(bag);
  size_t msg_idx = 0;
  for (const auto &msg : bag_view) {
    // Handle camera mesages
    for (const auto [cam_idx, cam_topic] : cam_topics) {
      if (msg.getTopic() == cam_topic) {
        const std::string cam_str = "cam" + std::to_string(cam_idx);
        const std::string cam_path = data_path + "/" + cam_str + "/data/";
        image_message_handler(msg, cam_path, cam_csvs[cam_idx]);
      }
    }

    // Print progress
    msg_idx++;
    if ((msg_idx % 10) == 0) {
      printf(".");
    }
  }
  printf("\n");

  // Clean up rosbag
  bag.close();
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string config_file;
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

  // Process rosbag
  process_rosbag(config_file);

  // Calibrate camera intrinsics
  calib_camera_t calib{config_file};
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");
  calib.save_stats("/tmp/calib-stats.csv");

  return 0;
}
