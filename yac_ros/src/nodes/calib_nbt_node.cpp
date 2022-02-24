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
  int state = INITIALIZE;

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  image_transport::ImageTransport img_trans{ros_nh};
  std::string imu_topic;
  std::map<int, std::string> mcam_topics;
  std::map<int, image_transport::Subscriber> mcam_subs;

  // Flags
  std::mutex calib_mutex;
  bool keep_running = true;
  bool find_nbv_event = false;

  // Calibration
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
  int min_init_views = 100;

  /* Constructor */
  calib_nbt_t() = delete;

  /* Constructor */
  calib_nbt_t(const std::string &node_name_) : node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Setup Calibrator
    calib = std::make_unique<calib_vi_t>(config_file);

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

  /* Destructor */
  ~calib_nbt_t() = default;

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

    // Make a copy of camera indices
    std::vector<int> cam_indices;
    for (const auto &[cam_idx, _] : img_buffer) {
      cam_indices.push_back(cam_idx);
    }

    // Detect aprilgrid in each camera
#pragma omp parallel for shared(grid_buffer)
    for (size_t i = 0; i < cam_indices.size(); i++) {
      const auto cam_idx = cam_indices[i];
      const auto &data = img_buffer[cam_idx];

      const auto img_ts = data.first;
      const auto &img = data.second;
      const auto grid = calib->detector->detect(img_ts, img);
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

  // /** Draw NBT */
  // void draw_nbv(cv::Mat &img) {
  //   // Draw NBT
  //   const auto cam_geom = calib->cam_geoms[nbv_cam_idx];
  //   const auto cam_params = calib->cam_params[nbv_cam_idx];
  //   const mat4_t T_FC0 = nbv_poses.at(nbv_cam_idx).at(nbv_idx);
  //   const mat4_t T_C0Ci = calib->cam_exts[nbv_cam_idx]->tf();
  //   const mat4_t T_FCi = T_FC0 * T_C0Ci;
  //   nbv_draw(calib_target, cam_geom, cam_params, T_FCi, img);
  //
  //   // Show NBT Reproj Error
  //   draw_nbv_reproj_error(nbv_reproj_err, img);
  //
  //   // Show NBT status
  //   if (nbv_reproj_err < (nbv_reproj_error_threshold * 1.5)) {
  //     std::string text = "Nearly There!";
  //     if (nbv_reproj_err <= nbv_reproj_error_threshold) {
  //       text = "HOLD IT!";
  //     }
  //     draw_status_text(text, img);
  //   }
  // }

  /** Initialize Intrinsics + Extrinsics Mode */
  int mode_init() {
    // Pre-check
    if (grid_buffer.size() != 0) {
      for (auto &[cam_idx, data] : img_buffer) {
        // Check if aprilgrid is detected
        if (grid_buffer.count(cam_idx) == 0) {
          continue;
        }
        const auto &grid_k = grid_buffer[cam_idx];
        if (grid_k.detected == false) {
          continue;
        }

        // Add camera data
        calib->add_measurement(cam_idx, grid_k);
      }
    }

    // Check if we have enough grids for all cameras
    const int views_left = min_init_views - calib->nb_views();
    if (views_left > 0) {
      LOG_INFO("Still need %d views", views_left);
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
      return 0;
    }

    // Transition to NBT mode
    calib->solve();
    state = NBT;
    find_nbv_event = true;

    return 0;
  }

  /**
   * IMU Callback
   * @param[in] msg Imu message
   */
  void imu_callback(const sensor_msgs::ImuConstPtr &msg) {
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
    // Update image buffer
    std::lock_guard<std::mutex> guard(calib_mutex);
    if (update_image_buffer(cam_idx, msg) == false) {
      return;
    }

    // Detect Calibration Target
    detect();

    // States
    switch (state) {
      case INITIALIZE:
        mode_init();
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
