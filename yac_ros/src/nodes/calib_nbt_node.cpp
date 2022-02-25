#include <signal.h>
#include <thread>
#include <termios.h>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "yac.hpp"
#include "../ros_calib.hpp"
#include "../ros_utils.hpp"

namespace yac {

struct calib_nbt_t {
  // Calibration state
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBT = 1,
    BATCH = 2,
  };

  // Flags
  int state = INITIALIZE;
  std::mutex calib_mutex;
  bool keep_running = true;
  bool find_nbv_event = false;

  // Settings
  bool use_apriltags3 = false;
  int min_init_views = 5;

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  // -- IMU
  std::string imu0_topic;
  ros::Subscriber imu0_sub;
  // -- Cameras
  image_transport::ImageTransport img_trans{ros_nh};
  std::map<int, std::string> mcam_topics;
  std::map<int, image_transport::Subscriber> mcam_subs;

  // Calibration
  std::unique_ptr<calib_vi_t> calib;

  // Data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  std::map<int, aprilgrid_t> grid_buffer;

  /* Constructor */
  calib_nbt_t() = delete;

  /* Constructor */
  calib_nbt_t(const std::string &node_name_) : node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Setup calibrator
    calib = std::make_unique<calib_vi_t>(config_file);
    calib->max_iter = 5;

    // Setup ROS
    setup_ros(config_file);
  }

  /* Destructor */
  ~calib_nbt_t() {}

  /** Setup ROS */
  void setup_ros(const std::string &config_file) {
    // Parse ros topics
    config_t config{config_file};
    // -- IMU topic
    parse(config, "ros.imu0_topic", imu0_topic);
    // -- Camera topics
    parse_camera_topics(config, mcam_topics);
    if (mcam_topics.size() == 0) {
      FATAL("No camera topics found in [%s]!", config_file.c_str());
    }

    // Subscribe
    // -- IMU
    LOG_INFO("Subscribing to imu0 @ [%s]", imu0_topic.c_str());
    imu0_sub =
        ros_nh.subscribe(imu0_topic, 1000, &calib_nbt_t::imu0_callback, this);
    // -- Cameras
    for (const auto [cam_idx, topic] : mcam_topics) {
      LOG_INFO("Subscribing to cam%d @ [%s]", cam_idx, topic.c_str());
      auto _1 = std::placeholders::_1;
      auto cb = std::bind(&calib_nbt_t::image_callback, this, _1, cam_idx);
      mcam_subs[cam_idx] = img_trans.subscribe(topic, 1, cb);
    }
  }

  /**
   * Update image buffer
   * @param[in] cam_idx Camera index
   * @param[in] msg Image messsage
   * @returns True or False if all camera images have arrived
   */
  bool update_image_buffer(const int cam_idx,
                           const sensor_msgs::ImageConstPtr &msg) {
    // Convert message to image and add to image buffer
    const timestamp_t ts_k = msg->header.stamp.toNSec();
    img_buffer[cam_idx] = {ts_k, msg_convert(msg)};

    // Make sure timestamps in in image buffer all the same
    bool ready = true;
    for (auto &[cam_idx, data] : img_buffer) {
      const auto img_ts = data.first;
      const auto &img = data.second;
      if (ts_k > img_ts) {
        ready = false;
      }
    }

    return ready;
  }

  /** Detect AprilGrids */
  void detect_aprilgrid() {
    // Clear previous detected grids
    grid_buffer.clear();

    // Make a copy of camera indices
    std::vector<int> cam_indices;
    for (const auto &[cam_idx, _] : img_buffer) {
      cam_indices.push_back(cam_idx);
    }

    // Detect aprilgrid in each camera
    for (size_t i = 0; i < cam_indices.size(); i++) {
      const auto cam_idx = cam_indices[i];
      const auto &data = img_buffer[cam_idx];

      const auto img_ts = data.first;
      const auto &img = data.second;
      const auto grid = calib->detector->detect(img_ts, img, use_apriltags3);
      if (grid.detected) {
        grid_buffer[cam_idx] = grid;
      }
    }
  }

  /** Event Keyboard Handler */
  void event_handler(int key) {
    if (key != EOF) {
      switch (key) {
        case 113: // 'q' key
          LOG_INFO("User requested program termination!");
          LOG_INFO("Exiting ...");
          keep_running = false;
          break;
      }
    }
  }

  void visualize() {
    // Visualize Initialization mode
    cv::Mat viz;
    for (auto [cam_idx, data] : img_buffer) {
      // Convert image gray to rgb
      auto img_gray = data.second;
      auto img = gray2rgb(img_gray);

      // Draw "detected" if AprilGrid was observed
      if (grid_buffer.count(cam_idx)) {
        draw_detected(grid_buffer[cam_idx], img);
      }

      // Stack the images up
      if (viz.empty()) {
        viz = img;
        continue;
      }
      cv::hconcat(viz, img, viz);
    }
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** Initialize Intrinsics + Extrinsics Mode */
  void mode_init() {
    // Visualize
    // visualize();

    // Pre-check
    if (grid_buffer.size() == 0) {
      return;
    }

    // Add to camera data to calibrator
    for (const auto &[cam_idx, grid] : grid_buffer) {
      calib->add_measurement(cam_idx, grid);
    }

    // Check if we have enough grids for all cameras
    const int views_left = min_init_views - calib->nb_views();
    if (views_left > 0) {
      return;
    }

    // Solve initial views
    calib->solve();
    calib->enable_marginalization = true;
    calib->window_size = 5;

    // Transition to NBT mode
    LOG_INFO("Transition to NBT mode");
    state = NBT;
    find_nbv_event = true;
  }

  void mode_nbt() {
    for (const auto &[cam_idx, grid] : grid_buffer) {
      calib->add_measurement(cam_idx, grid);
    }
    calib->show_results();
  }

  /**
   * IMU Callback
   * @param[in] msg Imu message
   */
  void imu0_callback(const sensor_msgs::ImuConstPtr &msg) {
    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    const timestamp_t ts = msg->header.stamp.toNSec();
    calib->add_measurement(ts, a_m, w_m);
  }

  /**
   * Camera Image Callback
   * @param[in] msg Image message
   * @param[in] cam_idx Camera index
   */
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const int cam_idx) {
    // Add image to buffer
    if (update_image_buffer(cam_idx, msg) == false) {
      return;
    }

    // Detect
    detect_aprilgrid();

    // States
    switch (state) {
      case INITIALIZE:
        mode_init();
        break;
      case NBT:
        mode_nbt();
        break;
      default:
        FATAL("Implementation Error!");
        break;
    }
  }

  void loop() {
    while (keep_running) {
      ros::spinOnce();
    }
  }
};

} // namespace yac

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start calibrating
  yac::calib_nbt_t calib{node_name};
  calib.loop();

  return 0;
}
