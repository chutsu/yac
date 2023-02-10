#include "yac/yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;

void process_rosbag(const std::string &config_file, calib_camera_t &calib) {
  // Parse config file
  config_t config{config_file};

  std::string rosbag_path;
  std::map<int, std::string> cam_topics;
  std::string data_path;
  parse(config, "ros.bag", rosbag_path);
  parse_camera_topics(config, cam_topics);

  // Open ROS bag
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  // Process ROS bag
  rosbag::View bag_view(bag);
  size_t msg_idx = 0;
  std::map<int, std::map<timestamp_t, aprilgrid_t>> cam_data;

  for (const auto &msg : bag_view) {
    // Handle camera mesages
    for (const auto [cam_idx, cam_topic] : cam_topics) {
      if (msg.getTopic() == cam_topic) {
        const auto image_msg = msg.instantiate<sensor_msgs::Image>();
        const auto image_ptr = cv_bridge::toCvCopy(image_msg, "mono8");
        const timestamp_t ts = ros::Time(image_msg->header.stamp).toNSec();
        const cv::Mat cam_img = image_ptr->image;
        cam_data[cam_idx][ts] = calib.detector->detect(ts, cam_img);
      }
    }

    // Print progress
    msg_idx++;
    if ((msg_idx % 10) == 0) {
      printf(".");
      fflush(stdout);
    }
  }
  printf("\n");

  // Add camera data to calibrator
  calib.add_camera_data(cam_data);

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

  // Calibrate camera intrinsics
  calib_camera_t calib{config_file};
  process_rosbag(config_file, calib);
  calib.solve();
  calib.save_results("/tmp/calib-results.yaml");

  return 0;
}
