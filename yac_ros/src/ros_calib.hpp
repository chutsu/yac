#ifndef YAC_ROS_CALIB_HPP
#define YAC_ROS_CALIB_HPP

#include <signal.h>
#include <thread>
#include <termios.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

#include "yac/yac.hpp"
#include "ros_utils.hpp"

namespace yac {

void draw_hcentered_text(const std::string &text,
                         const float text_scale,
                         const int text_thickness,
                         const int text_ypos,
                         cv::Mat &image);
void draw_camera_index(const int cam_idx, cv::Mat &image);
void draw_nbv_reproj_error(const real_t nbv_reproj_error, cv::Mat &image);
void draw_status_text(const std::string &text, cv::Mat &image);
void draw_detected(const aprilgrid_t &grid, cv::Mat &image);
void draw_nbv(const mat4_t &T_FC0, cv::Mat &image);
bool tf_ok(const mat4_t &pose);
void update_aprilgrid_model(const ros::Time &ts,
                            const calib_target_t &target,
                            ros::Publisher &rviz_pub,
                            bool remove = false);

void publish_fiducial_tf(const ros::Time &ts,
                         const calib_target_t &target,
                         const mat4_t &T_WF,
                         tf2_ros::TransformBroadcaster &tf_br,
                         ros::Publisher rviz_pub,
                         bool remove = false);
void publish_tf(const ros::Time &ts,
                const std::string &pose_name,
                const mat4_t &pose,
                tf2_ros::TransformBroadcaster &tf_br,
                bool remove = false);

/* ROS Calibration Configuration */
struct ros_config_t {
  ros::NodeHandle *ros_nh = nullptr;
  std::string node_name;

  bool publish_tfs = true;
  std::string image_format = "bgr8";
  std::string calib_file;
  std::string cam0_topic;
  std::string cam1_topic;
  std::string imu0_topic;

  ros_config_t(int argc, char **argv) {
    node_name = yac::ros_node_name(argc, argv);
    if (ros::isInitialized() == false) {
      ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    }
    ros_nh = new ros::NodeHandle();

    ROS_PTR_PARAM(ros_nh, node_name + "/publish_tfs", publish_tfs);
    ROS_PTR_PARAM(ros_nh, node_name + "/image_format", image_format);
    ROS_PTR_PARAM(ros_nh, node_name + "/calib_file", calib_file);
    ROS_PTR_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);
    ROS_PTR_PARAM(ros_nh, node_name + "/cam1_topic", cam1_topic);
    ROS_PTR_OPTIONAL_PARAM(ros_nh, node_name + "/imu0_topic", imu0_topic, "");
  }

  ~ros_config_t() {
    if (ros_nh) {
      delete ros_nh;
    }
  }
};

} // namespace yac
#endif // YAC_ROS_CALIB_HPP
