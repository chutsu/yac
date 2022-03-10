#include "yac/yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;

void process_rosbag(const std::string &config_file,
                    std::string &data_path,
                    std::string &results_path) {
  // Parse config file
  config_t config{config_file};

  std::string rosbag_path;
  std::map<int, std::string> cam_topics;
  std::string imu_topic;
  parse(config, "ros.bag", rosbag_path);
  parse_camera_topics(config, cam_topics);
  parse(config, "ros.imu0_topic", imu_topic);
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_path", results_path);

  // Check output dir
  if (dir_exists(data_path) == false) {
    if (dir_create(data_path) != 0) {
      FATAL("Failed to create dir [%s]", data_path.c_str());
    }
  }

  // Prepare data files and directories
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  std::map<int, std::ofstream> cam_csvs;
  std::ofstream imu_csv;
  // -- Prepare camera csv files
  for (const auto [cam_idx, cam_topic] : cam_topics) {
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    const auto cam_path = data_path + "/" + cam_str;
    if (dir_create(cam_path) != 0) {
      FATAL("Failed to create dir [%s]", cam_path.c_str());
    }

    cam_csvs[cam_idx] = camera_init_output_file(cam_path);
    LOG_INFO("- %s topic [%s]", cam_str.c_str(), cam_topic.c_str());
  }
  // -- Prepare imu csv file
  const std::string imu_save_path = data_path + "/" + "imu0";
  imu_csv = imu_init_output_file(imu_save_path);
  LOG_INFO("- imu0 topic [%s]", imu_topic.c_str());

  // Open ROS bag
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  // Process ROS bag
  rosbag::View bag_view(bag);
  std::map<int, size_t> nb_cam_msgs;
  size_t nb_imu_msgs = 0;

  progressbar bar(bag_view.size());
  for (const auto &msg : bag_view) {
    // Handle camera mesages
    for (const auto [cam_idx, cam_topic] : cam_topics) {
      if (msg.getTopic() == cam_topic) {
        const std::string cam_str = "cam" + std::to_string(cam_idx);
        const std::string cam_path = data_path + "/" + cam_str + "/data/";
        image_message_handler(msg, cam_path, cam_csvs[cam_idx]);
        nb_cam_msgs[cam_idx]++;
      }
    }

    // Handle imu messsages
    if (msg.getTopic() == imu_topic) {
      imu_message_handler(msg, imu_csv);
      nb_imu_msgs++;
    }

    // Print progress
    bar.update();
  }
  printf("\n");

  // Summary
  LOG_INFO("Processed:");
  // -- Camera messages
  for (const auto [cam_idx, cam_topic] : cam_topics) {
    if (nb_cam_msgs[cam_idx]) {
      LOG_INFO("- cam%d msgs: %ld", cam_idx, nb_cam_msgs[cam_idx]);
    } else {
      LOG_WARN("No cam%d msgs were processed!", cam_idx);
    }
  }
  // -- IMU messages
  if (nb_imu_msgs) {
    LOG_INFO("- imu0 msgs: %d", nb_imu_msgs);
  } else {
    LOG_WARN("No imu msgs were processed!");
  }

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
  std::string data_path;
  std::string results_path;
  process_rosbag(config_file, data_path, results_path);

  // Calibrate
  calib_vi_t calib{config_file};
  calib.load_data(data_path);
  calib.solve();
  calib.save_results(results_path);

  return 0;
}
