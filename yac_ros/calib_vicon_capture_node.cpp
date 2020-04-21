#include <signal.h>
#include <thread>
#include <termios.h>

#include "yac/yac.hpp"
#include "yac_ros/ros.hpp"

using namespace yac;

// GLOBAL VARS
bool keep_running = true;
bool capture_event = false;
rosbag::Bag bag;
bool imshow;
std::string cam0_topic;
std::string body0_topic;
std::string target0_topic;

int cam_counter = 0;
int body_counter = 0;
int target_counter = 0;

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

static void body_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/body0/pose", msg->header.stamp, msg);
  body_counter++;
}

static void body_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
  body_counter++;
}

static void body_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
  body_counter++;
}

static void target_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/target0/pose", msg->header.stamp, msg);
  target_counter++;
}

static void target_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
  target_counter++;
}

static void target_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
  target_counter++;
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
  std::string body0_topic_type;
  std::string target0_topic_type;
  ROS_PARAM(nh, node_name + "/rosbag_path", rosbag_path);
  ROS_PARAM(nh, node_name + "/imshow", imshow);
  ROS_PARAM(nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(nh, node_name + "/body0_topic", body0_topic);
  ROS_PARAM(nh, node_name + "/body0_topic_type", body0_topic_type);
  ROS_PARAM(nh, node_name + "/target0_topic", target0_topic);
  ROS_PARAM(nh, node_name + "/target0_topic_type", target0_topic_type);

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

  // Setup subscribers
  // -- Camera subscriber
  const auto cam0_sub = nh.subscribe(cam0_topic, 100, image_cb);
  // -- Mocap body subscriber
  ros::Subscriber body0_sub;
  if (body0_topic_type == "geometry_msgs/PoseStamped") {
    body0_sub = nh.subscribe(body0_topic, 100, body_pose_cb);
  } else if (body0_topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    body0_sub = nh.subscribe(body0_topic, 100, body_pose_covar_cb);
  } else if (body0_topic_type == "nav_msgs/Odometry") {
    body0_sub = nh.subscribe(body0_topic, 100, body_odom_cb);
  } else {
    FATAL("Unsupported body0_topic_type [%s]!", body0_topic_type.c_str());
  }
  // -- Mocap target subscriber
  ros::Subscriber target0_sub;
  if (target0_topic_type == "geometry_msgs/PoseStamped") {
    target0_sub = nh.subscribe(target0_topic, 100, target_pose_cb);
  } else if (target0_topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    target0_sub = nh.subscribe(target0_topic, 100, target_pose_covar_cb);
  } else if (target0_topic_type == "nav_msgs/Odometry") {
    target0_sub = nh.subscribe(target0_topic, 100, target_odom_cb);
  } else {
    FATAL("Unsupported target0_topic_type [%s]!", target0_topic_type.c_str());
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
  printf("- body0 msgs recorded: %d\n", body_counter);
  printf("- target0 msgs recorded: %d\n", target_counter);

  return 0;
}
