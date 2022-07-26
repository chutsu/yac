#pragma once
#include "yac/yac.hpp"
#include "../ros_calib.hpp"
#include "../ros_utils.hpp"

#include <yac_ros/CalibTargetMeasurementSet.h>

namespace yac {

struct calib_nbv_eval_t {
  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  ros::Publisher finish_pub;
  ros::Subscriber nbt_data_sub;
  ros::Subscriber reset_sub;

  // Setting
  std::string config_file;
  std::string data_path;

  // Calibration
  int views_added = 0;
  std::unique_ptr<calib_camera_t> calib;

  // Data
  std::map<int, std::map<timestamp_t, cv::Mat>> cam_images_store;
  std::map<int, std::map<timestamp_t, aprilgrid_t>> cam_grids_store;

  /* Constructor */
  calib_nbv_eval_t() = delete;

  /* Constructor */
  calib_nbv_eval_t(const std::string &node_name_) : node_name{node_name_} {
    // Get camera calibration file
    ROS_PARAM(ros_nh, node_name_ + "/config_file", config_file);
    config_t config{config_file};
    parse(config, "ros.data_path", data_path);

    // Publishers
    finish_pub = ros_nh.advertise<std_msgs::Bool>("/yac_ros/nbv_finish", 0);

    // Subscribers
    // -- NBT data topic
    const std::string nbt_data_topic = "/yac_ros/nbv_data";
    LOG_INFO("Subscribing to nbt_data @ [%s]", nbt_data_topic.c_str());
    nbt_data_sub = ros_nh.subscribe(nbt_data_topic,
                                    1,
                                    &calib_nbv_eval_t::eval_callback,
                                    this);
    // -- Reset topic
    const std::string reset_topic = "/yac_ros/reset";
    reset_sub = ros_nh.subscribe(reset_topic,
                                 1,
                                 &calib_nbv_eval_t::reset_callback,
                                 this);

    // Setup Calibrator
    LOG_INFO("Setting up camera calibrator ...");
    calib = std::make_unique<calib_camera_t>(config_file);

    // Loop
    loop();
  }

  /* Destructor */
  ~calib_nbv_eval_t() = default;

  /** Add view to calibrator */
  void add_view(const std::map<int, cv::Mat> &cam_imgs,
                const std::map<int, aprilgrid_t> &cam_grids) {
    // Add view
    if (calib->add_nbv_view(cam_grids)) {
      LOG_INFO("Add View! [Number of NBVs: %d]", views_added++);
      if (calib->nb_views() >= calib->min_nbv_views) {
        calib->_remove_outliers(false);
      }

      // Keep track of aprilgrids and images
      for (const auto &[cam_idx, cam_grid] : cam_grids) {
        const auto ts = cam_grid.timestamp;
        const auto &img = cam_imgs[cam_idx];
        const auto &grid = calib->calib_views[ts]->grids[cam_idx];

        // Keep track of all aprilgrids and images
        cam_images_store[cam_idx][ts] = img;
        cam_grids_store[cam_idx][ts] = grid;
      }

      // Marginalize oldest view
      if (calib->nb_views() > calib->sliding_window_size) {
        calib->marginalize();
      }

    } else {
      LOG_INFO("Reject View!");
    }

    // Solve
    calib->verbose = false;
    calib->enable_nbv = false;
    calib->enable_outlier_filter = false;
    calib->solver->solve(30);
  }

  /** Evaluation callback */
  void eval_callback(const yac_ros::CalibTargetMeasurementSet &msg) {}

  /** Reset callback */
  void reset_callback(const std_msgs::Bool &msg) {
    // Pre-check
    if (msg.data == false) {
      return;
    }
  }

  void loop() {
    while (1) {
      ros::spinOnce();
    }
  }
};

} // namespace yac
