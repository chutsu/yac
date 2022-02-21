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
    NBT = 2,
    BATCH = 3,
  };
  int state = INITIALIZE;

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  image_transport::ImageTransport img_trans{ros_nh};
  std::map<int, std::string> mcam_topics;
  std::map<int, image_transport::Subscriber> mcam_subs;

  // Flags
  std::mutex calib_mutex;
  bool keep_running = true;
  bool find_nbv_event = false;
  // struct termios term_config_orig;

  // Calibration
  calib_target_t calib_target;
  std::unique_ptr<aprilgrid_detector_t> detector;
  std::unique_ptr<calib_vi_t> calib;

  // Data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  std::map<int, aprilgrid_t> grid_buffer;
  std::map<int, aprilgrids_t> cam_grids;

  // NBT Data
  std::map<int, mat4s_t> nbv_poses;
  int nbv_cam_idx = 0;
  int nbv_idx = 0;
  aprilgrid_t nbv_target;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // NBT Settings
  int min_intrinsics_views = 5;
  real_t min_intrinsics_view_diff = 10.0;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;

  /* Constructor */
  calib_nbt_t() = delete;

  /* Constructor */
  calib_nbt_t(const std::string &node_name_) : node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
    setup_calib_target(config_file);
    setup_calibrator(config_file);
    setup_aprilgrid_detector();
    setup_ros_topics(config_file);
  }

  /* Destructor */
  ~calib_nbt_t() = default;

  /** Setup Calibration Target */
  void setup_calib_target(const std::string &config_file) {
    config_t config{config_file};
    if (calib_target.load(config_file, "calib_target") != 0) {
      FATAL("Failed to parse calib_target in [%s]!", config_file.c_str());
    }
  }

  /** Setup Camera Calibrator */
  void setup_calibrator(const std::string &config_file) {}

  /** Setup AprilGrid detector */
  void setup_aprilgrid_detector() {
    detector = std::make_unique<aprilgrid_detector_t>(calib_target.tag_rows,
                                                      calib_target.tag_cols,
                                                      calib_target.tag_size,
                                                      calib_target.tag_spacing);
  }

  /** Setup ROS Topics */
  void setup_ros_topics(const std::string &config_file) {
    // Parse camera ros topics
    config_t config{config_file};
    parse_camera_topics(config, mcam_topics);
    if (mcam_topics.size() == 0) {
      FATAL("No camera topics found in [%s]!", config_file.c_str());
    }

    // Subscribe
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
  void detect() {
    grid_buffer.clear();

    for (auto &[cam_idx, data] : img_buffer) {
      const auto img_ts = data.first;
      const auto &img = data.second;
      const auto grid = detector->detect(img_ts, img);
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

  /**
   * Camera Image Callback
   * @param[in] msg Image message
   * @param[in] cam_idx Camera index
   */
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const int cam_idx) {
    // Update image buffer
    std::lock_guard<std::mutex> guard(calib_mutex);
    if (update_image_buffer(cam_idx, msg) == false) {
      return;
    }

    // Detect Calibration Target
    detect();

    // // States
    // switch (state) {
    //   case INITIALIZE:
    //     mode_init();
    //     break;
    //   case NBT:
    //     mode_nbv();
    //     break;
    //   default:
    //     FATAL("Implementation Error!");
    //     break;
    // }
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
