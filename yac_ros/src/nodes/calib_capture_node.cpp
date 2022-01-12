#include <signal.h>
#include <thread>
#include <termios.h>

#include "yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;

// GLOBAL VARS
bool keep_running = true;
bool capture_event = false;
int save_counter = 0;
aprilgrid_detector_t detector(6, 6, 0.088, 0.3);
rosbag::Bag bag;
bool imshow = true;
bool show_detection = true;
std::map<int, std::string> cam_topics;

static void signal_handler(int sig) {
  UNUSED(sig);
  bag.close();
  ros::shutdown();
}

static void image_cb(const sensor_msgs::ImageConstPtr &msg) {
  // Convert ROS message to cv::Mat
  const cv::Mat image = msg_convert(msg);
  const auto ts = msg->header.stamp;

  if (imshow) {
    auto vis_image = gray2rgb(image);

    if (show_detection) {
      auto grid = detector.detect(ts.toNSec(), image, true);
      cv::Scalar color(0, 0, 255);
      for (const auto &kp : grid.keypoints()) {
        cv::circle(vis_image, cv::Point(kp(0), kp(1)), 1.0, color, 2, 8);
      }
    }

    cv::imshow("Camera View", vis_image);
    char key = (char)cv::waitKey(1);
    if (key == 'c') {
      capture_event = true;
    } else if (key == 'q') {
      keep_running = false;
    }
  }

  if (capture_event) {
    printf("Capturing cam0 image [%d]\n", save_counter);
    bag.write(cam_topics[0], ts, msg);
    capture_event = false;
    save_counter++;
  }
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  signal(SIGINT, signal_handler);
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  ros::NodeHandle nh;
  std::string rosbag_path;
  ROS_PARAM(nh, node_name + "/imshow", imshow);
  ROS_PARAM(nh, node_name + "/show_detection", show_detection);
  ROS_PARAM(nh, node_name + "/rosbag_path", rosbag_path);
  ROS_PARAM(nh, node_name + "/cam0_topic", cam_topics[0]);
  // ROS_OPTIONAL_PARAM(nh, node_name + "/cam1_topic", cam_topics[1], "");

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

  // Setup subscribers
  // -- Camera subscribers
  std::map<int, ros::Subscriber> cam_subs;
  for (size_t i = 0; i < cam_topics.size(); i++) {
    if (cam_topics[i].empty()) {
      continue;
    }

    // Subscribe to camera topic
    cam_subs[i] = nh.subscribe(cam_topics[i], i, image_cb);

    // Spin for 2 seconds then check if cam0_topic exists
    for (int i = 0; i < 2; i++) {
      sleep(1);
      ros::spinOnce();
    }

    // Check
    if (cam_subs[i].getNumPublishers() == 0) {
      ROS_FATAL("Topic [%s] not found!", cam_topics[i].c_str());
      return -1;
    }
  }
  if (cam_subs.size() == 0) {
    ROS_FATAL("Not subscribing to any camera?");
  }

  // Non-blocking keyboard handler
  struct termios term_config;
  struct termios term_config_orig;
  tcgetattr(0, &term_config);
  term_config_orig = term_config;
  term_config.c_lflag &= ~ICANON;
  term_config.c_lflag &= ~ECHO;
  term_config.c_lflag &= ~ISIG;
  term_config.c_cc[VMIN] = 0;
  term_config.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &term_config);

  // Capture thread
  std::thread keyboard_thread([&]() {
    printf("Press 'c' to capture, 'q' to stop!\n");
    while (keep_running) {
      int n = getchar();
      if (n != EOF) {
        int key = n;
        // std::cout << "key: " << key << std::endl;

        switch (key) {
          case 113: // 'q' key
            keep_running = false;
            break;
          case 126: // Presentation clicker up / down key
          case 99:  // 'c' key
            capture_event = true;
            break;
        }
      }
    }
  });

  // Loop
  while (keep_running) {
    ros::spinOnce();
  }
  keyboard_thread.join();

  // Clean up
  tcsetattr(0, TCSANOW, &term_config_orig);
  bag.close();
  printf("\x1B[1;32mROS bag saved to [%s]\x1B[1;0m\n", rosbag_path.c_str());
  printf("\x1B[1;32m- cam0 msgs recorded: %d\x1B[1;0m\n", save_counter);

  return 0;
}
