#ifndef YAC_ROS_UTILS_HPP
#define YAC_ROS_UTILS_HPP

#include <functional>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>

#include "yac.hpp"

namespace yac {

/*****************************************************************************
 *                                  MSG
 ****************************************************************************/

std_msgs::UInt8 msg_build(const uint8_t i);
std_msgs::Bool msg_build(const bool b);
std_msgs::String msg_build(const std::string &s);
std_msgs::Float64 msg_build(const double d);

geometry_msgs::Vector3 msg_build(const yac::vec3_t &vec);
geometry_msgs::Quaternion msg_build(const yac::quat_t &q);
geometry_msgs::PoseStamped msg_build(const size_t seq,
                                     const ros::Time &time,
                                     const std::string &frame_id,
                                     const yac::mat4_t &pose);
geometry_msgs::TwistStamped msg_build(const size_t seq,
                                      const ros::Time &time,
                                      const std::string &frame_id,
                                      const yac::vec3_t &linear_velocity,
                                      const yac::vec3_t &angular_velocity);
geometry_msgs::TransformStamped build_msg(const ros::Time &ts,
                                          const std::string &frame_id,
                                          const std::string &child_frame_id,
                                          const yac::mat4_t &T);

void msg_convert(const std_msgs::Header &msg,
                 size_t seq,
                 yac::timestamp_t &ts,
                 std::string &frame_id);
uint8_t msg_convert(const std_msgs::UInt8 &msg);
bool msg_convert(const std_msgs::Bool &msg);
float msg_convert(const std_msgs::Float64 &msg);
std::string msg_convert(const std_msgs::String &msg);

yac::vec3_t msg_convert(const geometry_msgs::Vector3 &msg);
yac::vec3_t msg_convert(const geometry_msgs::Point &msg);
yac::quat_t msg_convert(const geometry_msgs::Quaternion &msg);
cv::Mat msg_convert(const sensor_msgs::ImageConstPtr &msg);

/*****************************************************************************
 *                                  BAG
 *****************************************************************************/

bool check_ros_topics(const std::string &rosbag_path,
                      const std::vector<std::string> &target_topics);

std::ofstream pose_init_output_file(const std::string &output_path);
std::ofstream camera_init_output_file(const std::string &output_path);
std::ofstream imu_init_output_file(const std::string &output_path);
std::ofstream accel_init_output_file(const std::string &output_path);
std::ofstream gyro_init_output_file(const std::string &output_path);

void load_imu_data(const std::string &csv_file,
                   timestamps_t &timestamps,
                   vec3s_t &gyro,
                   vec3s_t &accel);
void pose_message_handler(const rosbag::MessageInstance &msg,
                          const std::string &output_path,
                          std::ofstream &pose_data);
void image_message_handler(const rosbag::MessageInstance &msg,
                           const std::string &output_path,
                           std::ofstream &camera_data);
void imu_message_handler(const rosbag::MessageInstance &msg,
                         std::ofstream &imu_data);
void accel_message_handler(const rosbag::MessageInstance &msg,
                           std::ofstream &accel_csv,
                           timestamps_t &accel_ts,
                           vec3s_t &accel_data);
void gyro_message_handler(const rosbag::MessageInstance &msg,
                          std::ofstream &gyro_csv,
                          timestamps_t &gyro_ts,
                          vec3s_t &gyro_data);

/*****************************************************************************
 *                                NODE
 ****************************************************************************/

#define ROS_PARAM(NH, X, Y)                                                    \
  if (NH.getParam(X, Y) == false) {                                            \
    ROS_FATAL_STREAM("Failed to get ROS param [" << X << "]!");                \
    exit(0);                                                                   \
  }

#define ROS_PTR_PARAM(NH, X, Y)                                                \
  if (NH->getParam(X, Y) == false) {                                           \
    ROS_FATAL_STREAM("Failed to get ROS param [" << X << "]!");                \
    exit(0);                                                                   \
  }

#define ROS_OPTIONAL_PARAM(NH, X, Y, DEFAULT)                                  \
  if (NH.getParam(X, Y) == false) {                                            \
    Y = DEFAULT;                                                               \
  }

#define ROS_PTR_OPTIONAL_PARAM(NH, X, Y, DEFAULT)                              \
  if (NH->getParam(X, Y) == false) {                                           \
    Y = DEFAULT;                                                               \
  }

#define RUN_ROS_NODE(NODE_CLASS)                                               \
  int main(int argc, char **argv) {                                            \
    std::string node_name;                                                     \
    if (ros::isInitialized() == false) {                                       \
      node_name = yac::ros_node_name(argc, argv);                            \
      ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);    \
    }                                                                          \
                                                                               \
    NODE_CLASS node;                                                           \
    node.node_name_ = node_name;                                               \
    if (node.configure() != 0) {                                               \
      ROS_ERROR("Failed to configure [%s]!", #NODE_CLASS);                     \
      return -1;                                                               \
    }                                                                          \
    node.loop();                                                               \
    return 0;                                                                  \
  }

#define RUN_ROS_NODE_RATE(NODE_CLASS, NODE_RATE)                               \
  int main(int argc, char **argv) {                                            \
    std::string node_name;                                                     \
    if (ros::isInitialized() == false) {                                       \
      node_name = yac::ros_node_name(argc, argv);                            \
      ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);    \
    }                                                                          \
                                                                               \
    NODE_CLASS node;                                                           \
    node.node_name_ = node_name;                                               \
    if (node.configure(NODE_RATE) != 0) {                                      \
      ROS_ERROR("Failed to configure [%s]!", #NODE_CLASS);                     \
      return -1;                                                               \
    }                                                                          \
    node.loop();                                                               \
    return 0;                                                                  \
  }

std::string ros_node_name(int argc, char *argv[]);
void ros_topic_subscribed(const ros::Subscriber &sub,
                          const std::string &topic);

struct ros_node_t {
  bool configured_ = false;
  bool debug_mode_ = false;
  bool sim_mode_ = false;

  int argc_ = 0;
  char **argv_ = nullptr;

  std::string node_name_;
  size_t ros_seq_ = 0;
  ros::NodeHandle ros_nh_;
  ros::Rate *ros_rate_ = nullptr;
  ros::Time ros_last_updated_;

  std::map<std::string, ros::Publisher> ros_pubs_;
  std::map<std::string, ros::Subscriber> ros_subs_;
  std::map<std::string, ros::ServiceServer> ros_services_;
  std::map<std::string, ros::ServiceClient> ros_clients_;

  std::map<std::string, image_transport::Publisher> img_pubs_;
  std::map<std::string, image_transport::Subscriber> img_subs_;
  std::function<int()> loop_cb_;

  ros_node_t();
  ~ros_node_t();

  int configure();
  int configure(const int hz);
  void shutdown_callback(const std_msgs::Bool &msg);
  int add_shutdown_subscriber(const std::string &topic);
  int add_image_publisher(const std::string &topic);
  int add_loop_callback(std::function<int()> cb);

  template <typename M, typename T>
  int add_image_subscriber(const std::string &topic,
                           void (T::*fp)(M),
                           T *obj,
                           uint32_t queue_size = 1) {
    // pre-check
    if (configured_ == false) {
      return -1;
    }

    // image transport
    image_transport::ImageTransport it(ros_nh_);
    img_subs_[topic] = it.subscribe(topic, queue_size, fp, obj);

    return 0;
  }

  template <typename M>
  int add_publisher(const std::string &topic,
                    uint32_t queue_size = 1,
                    bool latch = false) {
    ros::Publisher publisher;

    // pre-check
    if (configured_ == false) {
      return -1;
    }

    // add publisher
    publisher = ros_nh_.advertise<M>(topic, queue_size, latch);
    ros_pubs_[topic] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int add_subscriber(const std::string &topic,
                     void (T::*fp)(M),
                     T *obj,
                     uint32_t queue_size = 1) {
    ros::Subscriber subscriber;

    // pre-check
    if (configured_ == false) {
      return -1;
    }

    // add subscriber
    subscriber = ros_nh_.subscribe(topic, queue_size, fp, obj);
    ros_subs_[topic] = subscriber;

    return 0;
  }

  template <class T, class MReq, class MRes>
  int add_service(const std::string &service_topic,
                  bool (T::*fp)(MReq &, MRes &),
                  T *obj) {
    ros::ServiceServer server;

    // pre-check
    if (configured_ == false) {
      return -1;
    }

    // register service server
    server = ros_nh_.advertiseService(service_topic, fp, obj);
    ros_services_[service_topic] = server;

    return 0;
  }

  template <typename M>
  int addClient(const std::string &service_topic) {
    ros::ServiceClient client;

    // pre-check
    if (configured_ == false) {
      return -1;
    }

    // register service server
    client = ros_nh_.serviceClient<M>(service_topic);
    ros_clients_[service_topic] = client;

    return 0;
  }

  int loop();
};

} // namespace yac
#endif // YAC_ROS_UTILS_HPP
