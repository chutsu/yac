#include "proto/ros/ros.hpp"

// NODE SETTINGS
static const double NODE_RATE = 1.0;

// PUBLISH TOPIC
static const std::string SAY_TOPIC = "/example/say";

namespace proto {

struct example_node_t : ros_node_t {
  example_node_t() : ros_node_t() {}

  int configure(const int hz) {
    ros_node_t::configure(hz);
    add_publisher<std_msgs::String>(SAY_TOPIC);
    add_subscriber(SAY_TOPIC, &example_node_t::say_callback, this);
    add_loop_callback(std::bind(&example_node_t::loop_callback, this));
    return 0;
  }

  void say_callback(const std_msgs::String &msg) {
    std::cout << msg.data << std::endl;
  }

  int loop_callback() {
    std_msgs::String msg;
    msg.data = "Hello";
    ros_pubs_[SAY_TOPIC].publish(msg);
    return 0;
  }
};

} // namespace proto
RUN_ROS_NODE_RATE(proto::example_node_t, NODE_RATE);
