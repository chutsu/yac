#include "yac.hpp"
#include "ros.hpp"

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
  LOG_INFO("- cam0 topic [%s]", cam0_topic.c_str());
  LOG_INFO("- cam1 topic [%s]", cam1_topic.c_str());
  rosbag::View bag_view(bag);
  size_t msg_idx = 0;
  bool cam_event = false;
  for (const auto &msg : bag_view) {
    // Process cam0 data
    if (msg.getTopic() == cam0_topic) {
      yac::image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);
      cam_event = true;
    }

    // Process cam1 data
    if (msg.getTopic() == cam1_topic) {
      yac::image_message_handler(msg, cam1_output_path + "/data/", cam1_csv);
      cam_event = true;
    }

    // Print progress
    if (cam_event) {
      if ((msg_idx % 10) == 0) {
        printf(".");
      }
      msg_idx++;
      cam_event = false;
    }
  }
  printf("\n");

  // Clean up rosbag
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
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

  // Parse config file
  std::string bag_path;
  std::string cam0_topic;
  std::string cam1_topic;
  std::string data_path;
  yac::config_t config{config_file};
  yac::parse(config, "ros.bag", bag_path);
  yac::parse(config, "ros.cam0_topic", cam0_topic);
  yac::parse(config, "ros.cam1_topic", cam1_topic);
  yac::parse(config, "settings.data_path", data_path);

  // Process rosbag
  process_rosbag(bag_path, cam0_topic, cam1_topic, data_path);

  // Calibrate camera intrinsics
  if (yac::calib_stereo_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  return 0;
}
