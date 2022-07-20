#pragma once
#include "yac/yac.hpp"
#include "../ros_calib.hpp"
#include "../ros_utils.hpp"

#include <yac_ros/CalibVI.h>

namespace yac {

void publish_nbt(const lissajous_traj_t &traj, ros::Publisher &pub) {
  auto ts = ros::Time::now();
  auto frame_id = "map";

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ts;
  path_msg.header.frame_id = frame_id;

  const timestamp_t ts_end = traj.ts_start + sec2ts(traj.T);
  const timestamp_t dt = (ts_end - traj.ts_start) / 100.0;
  timestamp_t ts_k = traj.ts_start;
  while (ts_k < ts_end) {
    const mat4_t pose = traj.get_pose(ts_k);
    path_msg.poses.push_back(msg_build(0, ts, frame_id, pose));
    ts_k += dt;
  }

  pub.publish(path_msg);
}

struct calib_nbt_eval_t {
  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  ros::Publisher nbt_pub;
  ros::Publisher finish_pub;
  ros::Subscriber nbt_data_sub;
  ros::Subscriber reset_sub;

  // Setting
  std::string config_file;
  std::string data_path;
  real_t nbt_threshold = 0.2;

  // Data
  std::map<timestamp_t, real_t> calib_info;
  std::map<timestamp_t, real_t> calib_info_predict;

  /* Constructor */
  calib_nbt_eval_t() = delete;

  /* Constructor */
  calib_nbt_eval_t(const std::string &node_name_) : node_name{node_name_} {
    // Get camera calibration file
    ROS_PARAM(ros_nh, node_name_ + "/config_file", config_file);
    config_t config{config_file};
    parse(config, "ros.data_path", data_path);

    // Publishers
    nbt_pub = ros_nh.advertise<nav_msgs::Path>("/yac_ros/nbt", 0);
    finish_pub = ros_nh.advertise<std_msgs::Bool>("/yac_ros/nbt_finish", 0);

    // Subscribers
    // -- NBT data topic
    const std::string nbt_data_topic = "/yac_ros/nbt_data";
    LOG_INFO("Subscribing to nbt_data @ [%s]", nbt_data_topic.c_str());
    nbt_data_sub = ros_nh.subscribe(nbt_data_topic,
                                    1,
                                    &calib_nbt_eval_t::eval_callback,
                                    this);
    // -- Reset topic
    const std::string reset_topic = "/yac_ros/reset";
    reset_sub = ros_nh.subscribe(reset_topic,
                                 1,
                                 &calib_nbt_eval_t::reset_callback,
                                 this);

    // Loop
    loop();
  }

  /* Destructor */
  ~calib_nbt_eval_t() = default;

  /** Evaluation callback */
  void eval_callback(const yac_ros::CalibVI &msg) {
    // Send empty path message to delete previous one
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    nbt_pub.publish(path_msg);

    // NBT Data
    nbt_data_t nbt_data;

    // Timestamp
    const timestamp_t ts_now = msg.header.stamp.toNSec();

    // Calibration target
    nbt_data.calib_target.target_type = msg.calib_target.target_type;
    nbt_data.calib_target.tag_rows = msg.calib_target.tag_rows;
    nbt_data.calib_target.tag_cols = msg.calib_target.tag_cols;
    nbt_data.calib_target.tag_size = msg.calib_target.tag_size;
    nbt_data.calib_target.tag_spacing = msg.calib_target.tag_spacing;

    // Imu parameters
    // -- IMU rate
    nbt_data.imu_rate = msg.imu_rate;
    // -- IMU params
    nbt_data.imu_params.rate = msg.imu_params.rate;
    nbt_data.imu_params.a_max = msg.imu_params.a_max;
    nbt_data.imu_params.g_max = msg.imu_params.g_max;
    nbt_data.imu_params.sigma_g_c = msg.imu_params.sigma_g_c;
    nbt_data.imu_params.sigma_a_c = msg.imu_params.sigma_a_c;
    nbt_data.imu_params.sigma_gw_c = msg.imu_params.sigma_gw_c;
    nbt_data.imu_params.sigma_aw_c = msg.imu_params.sigma_aw_c;
    nbt_data.imu_params.sigma_bg = msg.imu_params.sigma_bg;
    nbt_data.imu_params.sigma_ba = msg.imu_params.sigma_ba;
    nbt_data.imu_params.g = msg.imu_params.g;
    // -- Imu extrinsics
    vecx_t imu_ext = zeros(7, 1);
    imu_ext[0] = msg.imu_exts.pose[0];
    imu_ext[1] = msg.imu_exts.pose[1];
    imu_ext[2] = msg.imu_exts.pose[2];
    imu_ext[3] = msg.imu_exts.pose[3];
    imu_ext[4] = msg.imu_exts.pose[4];
    imu_ext[5] = msg.imu_exts.pose[5];
    imu_ext[6] = msg.imu_exts.pose[6];
    const mat4_t T_BS = tf(imu_ext);
    nbt_data.imu_exts = std::make_unique<extrinsics_t>(T_BS);
    nbt_data.time_delay = std::make_unique<time_delay_t>(0.0);

    // Camera parameters
    // -- Camera rate
    nbt_data.cam_rate = msg.cam_rate;
    // -- Camera params and extrinsics
    for (size_t cam_idx = 0; cam_idx < msg.cam_params.size(); cam_idx++) {
      const auto &cam = msg.cam_params[cam_idx];
      const auto proj_model = cam.proj_model;
      const auto dist_model = cam.dist_model;

      // Camera geometry
      if (proj_model == "pinhole" && dist_model == "radtan4") {
        nbt_data.cam_geoms[cam_idx] = std::make_shared<pinhole_radtan4_t>();
      } else if (proj_model == "pinhole" && dist_model == "equi4") {
        nbt_data.cam_geoms[cam_idx] = std::make_shared<pinhole_equi4_t>();
      } else {
        FATAL("[%s-%s] Not implemented!",
              proj_model.c_str(),
              dist_model.c_str());
      }

      // Camera params
      const int cam_res[2] = {(int)cam.resolution[0], (int)cam.resolution[1]};

      vecx_t proj_params = zeros(cam.proj_params.size(), 1);
      for (size_t i = 0; i < cam.proj_params.size(); i++) {
        proj_params(i) = cam.proj_params[i];
      }

      vecx_t dist_params = zeros(cam.dist_params.size(), 1);
      for (size_t i = 0; i < cam.dist_params.size(); i++) {
        dist_params(i) = cam.dist_params[i];
      }

      nbt_data.cam_params[cam_idx] =
          std::make_shared<camera_params_t>(cam_idx,
                                            cam_res,
                                            proj_model,
                                            dist_model,
                                            proj_params,
                                            dist_params,
                                            true);

      // Camera extrinsics
      vecx_t cam_ext = zeros(7, 1);
      cam_ext[0] = msg.cam_exts[cam_idx].pose[0];
      cam_ext[1] = msg.cam_exts[cam_idx].pose[1];
      cam_ext[2] = msg.cam_exts[cam_idx].pose[2];
      cam_ext[3] = msg.cam_exts[cam_idx].pose[3];
      cam_ext[4] = msg.cam_exts[cam_idx].pose[4];
      cam_ext[5] = msg.cam_exts[cam_idx].pose[5];
      cam_ext[6] = msg.cam_exts[cam_idx].pose[6];
      const mat4_t T_BCi = tf(cam_ext);
      nbt_data.cam_exts[cam_idx] = std::make_shared<extrinsics_t>(T_BCi, true);
    }

    // Fiducial pose
    vecx_t fiducial_pose;
    fiducial_pose = zeros(7, 1);
    fiducial_pose[0] = msg.fiducial_pose[0];
    fiducial_pose[1] = msg.fiducial_pose[1];
    fiducial_pose[2] = msg.fiducial_pose[2];
    fiducial_pose[3] = msg.fiducial_pose[3];
    fiducial_pose[4] = msg.fiducial_pose[4];
    fiducial_pose[5] = msg.fiducial_pose[5];
    fiducial_pose[6] = msg.fiducial_pose[6];
    nbt_data.T_WF = tf(fiducial_pose);

    // Calibration info
    matx_t H = zeros(6, 6);
    size_t H_idx = 0;
    for (size_t i = 0; i < 6; i++) {
      for (size_t j = 0; j < 6; j++) {
        H(i, j) = msg.calib_info[H_idx++];
      }
    }

    // Generate NBTs
    LOG_INFO("Generate NBTs");
    const real_t cam_dt = 1.0 / nbt_data.cam_rate;
    const timestamp_t ts_start = ts_now + cam_dt;
    const timestamp_t ts_end = ts_start + sec2ts(3.0);
    lissajous_trajs_t trajs;
    nbt_lissajous_trajs(ts_start,
                        ts_end,
                        nbt_data.calib_target,
                        nbt_data.T_WF,
                        trajs);

    // Evaluate NBT trajectories
    LOG_INFO("Evaluate NBTs");
    real_t info_k = 0.0;
    real_t info_kp1 = 0.0;
    const int idx = nbt_find(trajs, nbt_data, H, true, &info_k, &info_kp1);

    // Track info
    calib_info[ts_now] = info_k;
    calib_info_predict[ts_now] = info_kp1;

    // Check if info gain is below threshold
    const auto info_gain = 0.5 * (info_k - info_kp1);
    if (info_gain < nbt_threshold) {
      // Publish finish message
      LOG_INFO("Info Gain Threshold Met!");
      std_msgs::Bool msg;
      msg.data = true;
      finish_pub.publish(msg);

      // Save info
      const std::string info_path = data_path + "/calib_imu/calib_info.csv";
      FILE *info_csv = fopen(info_path.c_str(), "w");
      fprintf(info_csv, "#ts,info_current,info_nbt_predict\n"); // Header
      for (const auto &[ts, info] : calib_info) {
        fprintf(info_csv, "%ld,", ts);
        fprintf(info_csv, "%f,", info);
        fprintf(info_csv, "%f\n", calib_info_predict[ts]);
      }
      fclose(info_csv);
    } else if (idx >= 0) {
      // Publish NBT
      publish_nbt(trajs[idx], nbt_pub);
    }
  }

  /** Reset callback */
  void reset_callback(const std_msgs::Bool &msg) {
    // Pre-check
    if (msg.data == false) {
      return;
    }

    // Clear calibration info
    calib_info.clear();
    calib_info_predict.clear();
  }

  void loop() {
    while (1) {
      ros::spinOnce();
    }
  }
};

} // namespace yac
