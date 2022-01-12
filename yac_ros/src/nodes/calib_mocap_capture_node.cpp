#include <signal.h>
#include <thread>
#include <termios.h>

#include "yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;

// GLOBAL VARS
bool keep_running = true;
bool capture_event = false;
rosbag::Bag bag;
bool imshow;
std::string cam0_topic;
std::string body0_topic;
std::string target0_topic;
uint64_t cam0_first_ts = 0;
uint64_t body0_first_ts = 0;
uint64_t target0_first_ts = 0;
uint64_t cam0_last_ts = 0;
uint64_t body0_last_ts = 0;
uint64_t target0_last_ts = 0;

int cam_counter = 0;
int body_counter = 0;
int target_counter = 0;

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
    cv::imshow("Camera View", image);
    char key = (char)cv::waitKey(1);
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

  if (cam0_first_ts == 0) {
    cam0_first_ts = ts.toNSec();
  }
  cam0_last_ts = ts.toNSec();
}

static void body_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/body0/pose", msg->header.stamp, msg);
  body_counter++;

  const auto ts = msg->header.stamp;
  if (body0_first_ts == 0) {
    body0_first_ts = ts.toNSec();
  }
  body0_last_ts = ts.toNSec();
}

static void body_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
  body_counter++;

  const auto ts = msg->header.stamp;
  if (body0_first_ts == 0) {
    body0_first_ts = ts.toNSec();
  }
  body0_last_ts = ts.toNSec();
}

static void body_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
  body_counter++;

  const auto ts = msg->header.stamp;
  if (body0_first_ts == 0) {
    body0_first_ts = ts.toNSec();
  }
  body0_last_ts = ts.toNSec();
}

static void target_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/target0/pose", msg->header.stamp, msg);
  target_counter++;

  const auto ts = msg->header.stamp;
  if (target0_first_ts == 0) {
    target0_first_ts = ts.toNSec();
  }
  target0_last_ts = ts.toNSec();
}

static void target_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
  target_counter++;

  const auto ts = msg->header.stamp;
  if (target0_first_ts == 0) {
    target0_first_ts = ts.toNSec();
  }
  target0_last_ts = ts.toNSec();
}

static void target_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
  target_counter++;

  const auto ts = msg->header.stamp;
  if (target0_first_ts == 0) {
    target0_first_ts = ts.toNSec();
  }
  target0_last_ts = ts.toNSec();
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

  // -- Double check subscribers are getting data
  ros_topic_subscribed(cam0_sub, cam0_topic);
  ros_topic_subscribed(body0_sub, body0_topic);
  ros_topic_subscribed(target0_sub, target0_topic);

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

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
  LOG_INFO("ROS bag saved to [%s]", rosbag_path.c_str());
  LOG_INFO("- cam0 msgs recorded: %d", cam_counter);
  LOG_INFO("- body0 msgs recorded: %d", body_counter);
  LOG_INFO("- target0 msgs recorded: %d", target_counter);

  // Check data integrity
  bool data_valid = true;
  if (cam0_first_ts < body0_first_ts) {
    LOG_ERROR("cam0 first timestamp < body0 first timestamp!");
    data_valid = false;
  }
  if (cam0_first_ts < target0_first_ts) {
    LOG_ERROR("cam0 first timestamp < target0 first timestamp!");
    data_valid = false;
  }
  if (cam0_last_ts > body0_last_ts) {
    LOG_ERROR("cam0 last timestamp > body0 last timestamp!");
    data_valid = false;
  }
  if (cam0_last_ts > target0_last_ts) {
    LOG_ERROR("cam0 last timestamp > target0 last timestamp!");
    data_valid = false;
  }
  if (data_valid == false) {
    LOG_ERROR("Are the computers time synchronized?");
    LOG_ERROR("ROS bag is invalid for calibration, deleting...");
    if (remove(rosbag_path.c_str()) == 0) {
      LOG_ERROR("ROS bag [%s] deleted!", rosbag_path.c_str());
    } else {
      LOG_ERROR("Failed to delete ROS bag [%s]!", rosbag_path.c_str());
    }
  }

  return 0;
}
