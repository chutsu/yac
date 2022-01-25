#include "ros_utils.hpp"

namespace yac {

/****************************************************************************
 *                                MSG
 ****************************************************************************/

std_msgs::Bool msg_build(const bool b) {
  std_msgs::Bool msg;
  msg.data = b;
  return msg;
}

std_msgs::String msg_build(const std::string &s) {
  std_msgs::String msg;
  msg.data = s;
  return msg;
}

std_msgs::Float64 msg_build(double d) {
  std_msgs::Float64 msg;
  msg.data = d;
  return msg;
}

geometry_msgs::Vector3 msg_build(yac::vec3_t &vec) {
  geometry_msgs::Vector3 msg;
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
  return msg;
}

geometry_msgs::Vector3 msg_build(const yac::vec3_t &vec) {
  geometry_msgs::Vector3 msg;
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
  return msg;
}

geometry_msgs::Quaternion msg_build(const yac::quat_t &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::PoseStamped msg_build(const size_t seq,
                                     const ros::Time &time,
                                     const std::string &frame_id,
                                     const yac::mat4_t &pose) {
  geometry_msgs::PoseStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;

  yac::vec3_t position = yac::tf_trans(pose);
  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);

  yac::quat_t orientation = yac::tf_quat(pose);
  msg.pose.orientation.w = orientation.w();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();

  return msg;
}

geometry_msgs::TwistStamped msg_build(const size_t seq,
                                      const ros::Time &time,
                                      const std::string &frame_id,
                                      const yac::vec3_t &linear_velocity,
                                      const yac::vec3_t &angular_velocity) {
  geometry_msgs::TwistStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;

  msg.twist.linear.x = linear_velocity(0);
  msg.twist.linear.y = linear_velocity(1);
  msg.twist.linear.z = linear_velocity(2);

  msg.twist.angular.x = angular_velocity(0);
  msg.twist.angular.y = angular_velocity(1);
  msg.twist.angular.z = angular_velocity(2);

  return msg;
}

geometry_msgs::TransformStamped build_msg(const ros::Time &ts,
                                          const std::string &frame_id,
                                          const std::string &child_frame_id,
                                          const yac::mat4_t &T) {
  const auto r = yac::tf_trans(T);
  const auto q = yac::tf_quat(T);

  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ts;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.transform.translation.x = r(0);
  msg.transform.translation.y = r(1);
  msg.transform.translation.z = r(2);
  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();

  return msg;
}

void msg_convert(const std_msgs::Header &msg,
                 size_t &seq,
                 yac::timestamp_t &ts,
                 std::string &frame_id) {
  seq = msg.seq;
  ts = msg.stamp.toNSec();
  frame_id = msg.frame_id;
}

uint8_t msg_convert(const std_msgs::UInt8 &msg) { return msg.data; }

bool msg_convert(const std_msgs::Bool &msg) { return msg.data; }

float msg_convert(const std_msgs::Float64 &msg) { return msg.data; }

std::string msg_convert(const std_msgs::String &msg) { return msg.data; }

yac::vec3_t msg_convert(const geometry_msgs::Vector3 &msg) {
  return yac::vec3_t{msg.x, msg.y, msg.z};
}

yac::vec3_t msg_convert(const geometry_msgs::Point &msg) {
  return yac::vec3_t{msg.x, msg.y, msg.z};
}

yac::quat_t msg_convert(const geometry_msgs::Quaternion &msg) {
  return yac::quat_t{msg.w, msg.x, msg.y, msg.z};
}

cv::Mat msg_convert(const sensor_msgs::ImageConstPtr &msg) {
  const auto image_ptr = cv_bridge::toCvCopy(msg);
  return image_ptr->image;
}

/*****************************************************************************
 *                                  BAG
 ****************************************************************************/

bool check_ros_topics(const std::string &rosbag_path,
                      const std::vector<std::string> &target_topics) {
  // Get all ros topics in bag
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);
  rosbag::View bag_view(bag);
  std::set<std::string> topics;
  for (const rosbag::ConnectionInfo *info : bag_view.getConnections()) {
    if (topics.find(info->topic) == topics.end()) {
      topics.insert(info->topic);
    }
  }

  // Make sure all target topics exist in bag
  for (const auto &target_topic : target_topics) {
    if (topics.find(target_topic) == topics.end()) {
      LOG_ERROR("Topic [%s] does not exist in ros bag [%s]!",
                target_topic.c_str(),
                rosbag_path.c_str());
      return false;
    }
  }

  return true;
}

std::ofstream pose_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "x,";
  data_file << "y,";
  data_file << "z,";
  data_file << "qx,";
  data_file << "qy,";
  data_file << "qz,";
  data_file << "qw" << std::endl;

  return data_file;
}

std::ofstream camera_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],filename" << std::endl;

  return data_file;
}

std::ofstream imu_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "a_RS_S_x [m s^-2],";
  data_file << "a_RS_S_y [m s^-2],";
  data_file << "a_RS_S_z [m s^-2],";
  data_file << "w_RS_S_x [rad s^-1],";
  data_file << "w_RS_S_y [rad s^-1],";
  data_file << "w_RS_S_z [rad s^-1]" << std::endl;

  return data_file;
}

std::ofstream accel_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "a_RS_S_x [m s^-2],";
  data_file << "a_RS_S_y [m s^-2],";
  data_file << "a_RS_S_z [m s^-2]" << std::endl;

  return data_file;
}

std::ofstream gyro_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "w_RS_S_x [rad s^-1],";
  data_file << "w_RS_S_y [rad s^-1],";
  data_file << "w_RS_S_z [rad s^-1]" << std::endl;

  return data_file;
}

void load_imu_data(const std::string &csv_file,
                   timestamps_t &timestamps,
                   vec3s_t &gyro,
                   vec3s_t &accel) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(csv_file, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", csv_file.c_str());
    return;
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    double a_x, a_y, a_z = 0.0;
    const int retval = fscanf(fp,
                              "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                              &ts,
                              &a_x,
                              &a_y,
                              &a_z,
                              &w_x,
                              &w_y,
                              &w_z);
    if (retval != 6) {
      LOG_ERROR("Failed to parse line [%d] in [%s]!", i, csv_file.c_str());
      return;
    }
    timestamps.push_back(ts);
    gyro.emplace_back(w_x, w_y, w_z);
    accel.emplace_back(a_x, a_y, a_z);
  }
  fclose(fp);
}

void pose_message_handler(const rosbag::MessageInstance &msg,
                          std::ofstream &pose_data) {
  const auto pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
  const auto ts = ros::Time(pose_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Save pose to data.csv
  pose_data << ts.toNSec() << ",";
  pose_data << pose_msg->pose.position.x << ",";
  pose_data << pose_msg->pose.position.y << ",";
  pose_data << pose_msg->pose.position.z << ",";
  pose_data << pose_msg->pose.orientation.x << ",";
  pose_data << pose_msg->pose.orientation.y << ",";
  pose_data << pose_msg->pose.orientation.z << ",";
  pose_data << pose_msg->pose.orientation.w << std::endl;
}

void image_message_handler(const rosbag::MessageInstance &msg,
                           const std::string &output_path,
                           std::ofstream &camera_data) {
  const auto image_msg = msg.instantiate<sensor_msgs::Image>();
  const auto ts = ros::Time(image_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());
  const std::string save_path{output_path + ts_str + ".png"};

  // Check save dir
  if (dir_exists(output_path) == false) {
    if (dir_create(output_path) != 0) {
      FATAL("Failed to create dir [%s]", output_path.c_str());
    }
  }

  // Convert ROS message to cv image
  cv_bridge::CvImagePtr bridge;
  bridge = cv_bridge::toCvCopy(image_msg, "bgr8");

  // Save image to file
  if (file_exists(save_path) == false) {
    if (cv::imwrite(save_path, bridge->image) == false) {
      FATAL("Failed to save image to [%s]", save_path.c_str());
    }
  }

  // Save image file to data.csv
  camera_data << ts.toNSec() << "," << ts.toNSec() << ".png" << std::endl;
  camera_data.flush();
}

void imu_message_handler(const rosbag::MessageInstance &msg,
                         std::ofstream &imu_data) {
  const auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(imu_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const vec3_t gyro = msg_convert(imu_msg->angular_velocity);
  const vec3_t accel = msg_convert(imu_msg->linear_acceleration);

  // Save imu measurement to file
  // -- Timestamp [ns]
  imu_data << ts.toNSec() << ",";
  // -- Accelerometer [m s^-2]
  imu_data << accel(0) << ",";
  imu_data << accel(1) << ",";
  imu_data << accel(2) << ",";
  // -- Angular velocity [rad s^-1]
  imu_data << gyro(0) << ",";
  imu_data << gyro(1) << ",";
  imu_data << gyro(2) << std::endl;
}

void accel_message_handler(const rosbag::MessageInstance &msg,
                           std::ofstream &accel_csv,
                           timestamps_t &accel_ts,
                           vec3s_t &accel_data) {
  const auto accel_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(accel_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const vec3_t accel = msg_convert(accel_msg->linear_acceleration);

  // Save imu measurement to file
  // -- Timestamp [ns]
  accel_csv << ts.toNSec() << ",";
  // -- Accelerometer [m s^-2]
  accel_csv << accel(0) << ",";
  accel_csv << accel(1) << ",";
  accel_csv << accel(2) << std::endl;

  // Keep track of accel data
  accel_ts.push_back(ts.toNSec());
  accel_data.push_back(accel);
}

void gyro_message_handler(const rosbag::MessageInstance &msg,
                          std::ofstream &gyro_csv,
                          timestamps_t &gyro_ts,
                          vec3s_t &gyro_data) {
  const auto gyro_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(gyro_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const vec3_t gyro = msg_convert(gyro_msg->angular_velocity);

  // Save imu measurement to file
  // -- Timestamp [ns]
  gyro_csv << ts.toNSec() << ",";
  // -- Angular velocity [rad s^-1]
  gyro_csv << gyro(0) << ",";
  gyro_csv << gyro(1) << ",";
  gyro_csv << gyro(2) << std::endl;

  // Keep track of gyro data
  gyro_ts.push_back(ts.toNSec());
  gyro_data.push_back(gyro);
}

/*****************************************************************************
 *                                NODE
 ****************************************************************************/

std::string ros_node_name(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg.find("__name:=") != std::string::npos) {
      return arg.substr(8);
    }
  }
  FATAL("Failed to find node name?");
}

void ros_topic_subscribed(const ros::Subscriber &sub,
                          const std::string &topic) {
  // First check if subscriber connected with any publishers.
  if (sub.getNumPublishers() > 0) {
    return;
  }

  // Spin for 2 seconds
  for (int i = 0; i < 2; i++) {
    sleep(1);
    ros::spinOnce();
  }

  // Check again
  if (sub.getNumPublishers() == 0) {
    FATAL("No data detected in ros topic [%s]!", topic.c_str());
  } else {
    LOG_INFO("Subscribed to ros topic [%s]", topic.c_str());
  }
}

ros_node_t::ros_node_t() {}

ros_node_t::~ros_node_t() {
  ros::shutdown();
  if (ros_rate_) {
    delete ros_rate_;
  }
}

int ros_node_t::configure() {
  ros_nh_.getParam("/debug_mode", debug_mode_);
  ros_nh_.getParam("/sim_mode", sim_mode_);
  configured_ = true;

  return 0;
}

int ros_node_t::configure(const int hz) {
  configure();
  ros_rate_ = new ros::Rate(hz);
  return 0;
}

void ros_node_t::shutdown_callback(const std_msgs::Bool &msg) {
  if (msg.data) {
    ros::shutdown();
  }
}

int ros_node_t::add_shutdown_subscriber(const std::string &topic) {
  ros::Subscriber sub;

  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Register subscriber
  sub = ros_nh_.subscribe(topic, 1, &ros_node_t::shutdown_callback, this);
  ros_subs_[topic] = sub;

  return 0;
}

int ros_node_t::add_image_publisher(const std::string &topic) {
  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Image transport
  image_transport::ImageTransport it(ros_nh_);
  img_pubs_[topic] = it.advertise(topic, 1);

  return 0;
}

int ros_node_t::add_loop_callback(std::function<int()> cb) {
  loop_cb_ = cb;
  return 0;
}

int ros_node_t::loop() {
  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Loop
  ROS_INFO("ROS node [%s] is running!", node_name_.c_str());
  while (ros::ok()) {
    // Loop callback
    if (loop_cb_ != nullptr) {
      int retval = loop_cb_();
      if (retval != 0) {
        return retval;
      }
    }

    // Update
    ros::spinOnce();
    ros_seq_++;
    ros_last_updated_ = ros::Time::now();
    if (ros_rate_) {
      ros_rate_->sleep();
    }
  }

  return 0;
}

} // namespace yac
