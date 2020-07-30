#include <signal.h>
#include <thread>
#include <termios.h>

#include "yac.hpp"
#include "ros.hpp"

using namespace yac;

// GLOBAL VARS
bool keep_running = true;
bool capture_event = false;
rosbag::Bag bag;
bool imshow;
std::string cam0_topic;

int cam_counter = 0;

static void signal_handler(int sig) {
  UNUSED(sig);
  bag.close();
  ros::shutdown();
}

static std::string keyboard_event() {
  std::string event;
  std::cin >> event;
  return event;
}

static void image_cb(const sensor_msgs::ImageConstPtr &msg) {
  // Convert ROS message to cv::Mat
  const cv::Mat image = msg_convert(msg);
  const auto ts = msg->header.stamp;
  if (imshow) {
    cv::imshow("Camera View", image);
    char key = (char) cv::waitKey(1);
    if (key == 'c') {
      capture_event = true;
    } else if (key == 'q') {
      keep_running = false;
    }
  }

  if (capture_event) {
    printf("Capturing cam0 image [%d]\n", cam_counter);
    bag.write("/cam0/image", ts, msg);
    capture_event = false;
    cam_counter++;
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
  ROS_PARAM(nh, node_name + "/rosbag_path", rosbag_path);
  ROS_PARAM(nh, node_name + "/cam0_topic", cam0_topic);

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

  // Setup subscribers
  // -- Camera subscriber
  const auto cam0_sub = nh.subscribe(cam0_topic, 100, image_cb);

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

  std::thread keyboard_thread([&](){
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
  printf("ROS bag saved to [%s]\n", rosbag_path.c_str());
  printf("- cam0 msgs recorded: %d\n", cam_counter);

  return 0;
}
