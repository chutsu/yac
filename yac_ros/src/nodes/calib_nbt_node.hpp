#pragma once
#include <signal.h>
#include <thread>
#include <termios.h>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "yac/yac.hpp"
#include "../ros_calib.hpp"
#include "../ros_utils.hpp"

namespace yac {

void publish_nbt(const ctraj_t &traj, ros::Publisher &pub) {
  auto ts = ros::Time::now();
  auto frame_id = "map";

  nav_msgs::Path path_msg;
  path_msg.header.seq = 0;
  path_msg.header.stamp = ts;
  path_msg.header.frame_id = frame_id;

  for (size_t i = 0; i < traj.orientations.size(); i++) {
    auto pose = tf(traj.orientations[i], traj.positions[i]);
    auto pose_stamped = msg_build(0, ts, frame_id, pose);
    path_msg.poses.push_back(pose_stamped);
  }

  pub.publish(path_msg);
}

void publish_nbt(const lissajous_traj_t &traj, ros::Publisher &pub) {
  auto ts = ros::Time::now();
  auto frame_id = "map";

  nav_msgs::Path path_msg;
  path_msg.header.seq = 0;
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

struct calib_nbt_t {
  // Calibration state
  enum CALIB_STATE {
    INITIALIZE = 1,
    NBT = 2,
    BATCH = 3,
  };

  // Flags
  int state = INITIALIZE;
  std::mutex mtx;
  bool keep_running = true;
  bool finding_nbt = false;

  // Settings
  std::string config_file;
  std::string calib_file;
  std::string data_path;
  std::string imu_dir;
  std::map<int, std::string> cam_dirs;
  bool use_apriltags3 = true;
  int min_init_views = 10;

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
  // -- TF broadcaster
  tf2_ros::TransformBroadcaster tf_br;
  // -- Rviz marker publisher
  ros::Publisher rviz_pub;
  // -- NBT publisher
  ros::Publisher nbt_pub;

  // Calibration
  std::unique_ptr<calib_vi_t> calib;

  // Data
  size_t frame_idx = 0;
  profiler_t prof;
  std::map<int, FILE *> cam_files;
  FILE *imu_file;

  // Threads
  std::thread nbt_thread;

  /* Constructor */
  calib_nbt_t() = delete;

  /* Constructor */
  calib_nbt_t(const std::string &node_name_) : node_name{node_name_} {
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Get camera calibration file
    config_t config{config_file};
    parse(config, "ros.data_path", data_path);
    calib_file = data_path + "/calib_camera/calib-results.yaml";

    // Setup calibrator
    calib = std::make_unique<calib_vi_t>(calib_file);
    calib->max_iter = 30;

    // Setup output directories
    if (prep_output() != 0) {
      LOG_WARN("Failed to setup output directories!");
      LOG_WARN("Stopping NBT node...");
      return;
    }

    // Setup ROS
    setup_ros(config_file);

    // Loop
    loop();
  }

  /* Destructor */
  ~calib_nbt_t() {
    for (auto &[cam_idx, cam_file] : cam_files) {
      if (cam_file) {
        fclose(cam_file);
      }
    }

    if (imu_file) {
      fclose(imu_file);
    }
  }

  /** Prepare output directories */
  int prep_output() {
    // Parse data path
    config_t config{config_file};
    parse(config, "ros.data_path", data_path);
    data_path = data_path + "/calib_imu";

    // Create calib data directory
    // -- Check to see if directory already exists
    if (opendir(data_path.c_str())) {
      LOG_WARN("Output directory [%s] already exists!", data_path.c_str());
      return -1;
    }
    // -- Create Calibration data directory
    LOG_INFO("Creating dir [%s]", data_path.c_str());
    if (system(("mkdir -p " + data_path).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", data_path.c_str());
    }
    // -- Create Imu directory and csv file
    imu_dir = data_path + "/imu0";
    LOG_INFO("Creating dir [%s]", imu_dir.c_str());
    if (system(("mkdir -p " + imu_dir).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", imu_dir.c_str());
    }
    imu_file = fopen((imu_dir + "/data.csv").c_str(), "w");
    fprintf(imu_file, "ts,gx,gy,gz,ax,ay,az\n");
    // -- Create Camera directories and image index file
    for (const auto &cam_idx : calib->get_camera_indices()) {
      const auto cam_dir = data_path + "/cam" + std::to_string(cam_idx);
      LOG_INFO("Creating dir [%s]", cam_dir.c_str());

      // Create camera directory
      cam_dirs[cam_idx] = cam_dir;
      if (system(("mkdir -p " + cam_dir + "/data").c_str()) != 0) {
        FATAL("Failed to create dir [%s]", cam_dir.c_str());
      }

      // Create camera index file
      const std::string csv_path = cam_dir + "/data.csv";
      cam_files[cam_idx] = fopen(csv_path.c_str(), "w");
    }
    // -- Copy config file to root of calib data directory
    auto src = config_file;
    auto dst = data_path + "/" + parse_fname(config_file);
    if (file_copy(src, dst) != 0) {
      FATAL("Failed to copy file from [%s] to [%s]!",
            config_file.c_str(),
            data_path.c_str());
    }

    return 0;
  }

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

    // Publishers
    // clang-format off
    rviz_pub = ros_nh.advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);
    nbt_pub = ros_nh.advertise<nav_msgs::Path>("/yac_ros/nbt", 0);
    // clang-format on

    // Subscribers
    // clang-format off
    // -- IMU
    LOG_INFO("Subscribing to imu0 @ [%s]", imu0_topic.c_str());
    imu0_sub = ros_nh.subscribe(imu0_topic, 1000, &calib_nbt_t::imu0_callback, this);
    // -- Cameras
    for (const auto [cam_idx, topic] : mcam_topics) {
      LOG_INFO("Subscribing to cam%d @ [%s]", cam_idx, topic.c_str());
      auto _1 = std::placeholders::_1;
      auto cb = std::bind(&calib_nbt_t::image_callback, this, _1, cam_idx);
      mcam_subs[cam_idx] = img_trans.subscribe(topic, 10, cb);
    }
    // clang-format on
  }

  /** Event Keyboard Handler */
  void event_handler(int key) {
    if (key != EOF) {
      switch (key) {
        case 'q':
          LOG_INFO("User requested program termination!");
          LOG_INFO("Exiting ...");
          keep_running = false;
          break;
        case 'f':
          LOG_INFO("Finding NBT...");
          find_nbt();
          break;
        case 'e': {
          matx_t covar;
          calib->recover_calib_covar(covar);
          printf("shannon entropy: %f\n", shannon_entropy(covar));
          break;
        }
        case 'r':
          LOG_INFO("Resetting ...");
          state = INITIALIZE;
          break;
      }
    }
  }

  /** Visualize */
  void visualize() {
    // Pre-check
    const auto cam_idx = 0;
    if (calib->img_buf.count(cam_idx) == 0) {
      return;
    }

    // Convert image gray to rgb
    const auto &img_gray = calib->img_buf[cam_idx].second;
    auto viz = gray2rgb(img_gray);

    // Draw "detected" if AprilGrid was observed
    if (calib->grid_buf.count(cam_idx)) {
      draw_detected(calib->grid_buf[cam_idx], viz);
    }

    // Show
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** Find NBT thread */
  void find_nbt_thread() {
    finding_nbt = true;

    // Pre-check
    if (calib->running == false) {
      return;
    }

    // Copy calibrator
    std::unique_lock<std::mutex> guard(mtx);
    calib_vi_t calib_copy{*calib.get()};
    // calib_copy.solve();
    // matx_t covar;
    // matx_t covar_copy;
    // calib->recover_calib_covar(covar);
    // calib_copy.recover_calib_covar(covar_copy);
    // printf("shannon_entropy: %f\n", shannon_entropy(covar));
    // printf("shannon_entropy copy: %f\n", shannon_entropy(covar_copy));
    guard.unlock();

    // Generate NBTs
    LOG_INFO("Generate NBTs");
    printf("nb_views: %ld\n", calib_copy.calib_views.size());
    const int cam_idx = 0;
    const timestamp_t ts_start = calib_copy.calib_views.back()->ts + 1;
    const timestamp_t ts_end = ts_start + sec2ts(2.0);
    mat4_t T_FO;
    calib_target_origin(T_FO,
                        calib_copy.calib_target,
                        calib_copy.cam_geoms[cam_idx],
                        calib_copy.cam_params[cam_idx]);

    lissajous_trajs_t trajs;
    nbt_lissajous_trajs(ts_start,
                        ts_end,
                        calib_copy.calib_target,
                        calib_copy.cam_geoms[cam_idx],
                        calib_copy.cam_params[cam_idx],
                        calib_copy.imu_exts,
                        calib_copy.get_fiducial_pose(),
                        T_FO,
                        trajs);

    // Evaluate NBT trajectories
    LOG_INFO("Evaluate NBTs");
    prof.start("find_nbt");
    const int best_index = nbt_find(trajs, calib_copy, true);
    prof.stop("find_nbt");
    prof.print("find_nbt");
    if (best_index >= 0) {
      publish_nbt(trajs[best_index], nbt_pub);
    }

    finding_nbt = false;
  }

  /** Find NBT */
  void find_nbt() {
    if (finding_nbt == false) {
      if (nbt_thread.joinable()) {
        nbt_thread.join();
      }
      nbt_thread = std::thread(&calib_nbt_t::find_nbt_thread, this);
    } else {
      LOG_WARN("Already finding NBT!");
    }
  }

  /** Initialize Intrinsics + Extrinsics Mode */
  void mode_init() {
    // Visualize
    visualize();

    // Pre-check
    if (calib->grid_buf.size() == 0) {
      return;
    }

    // Check if we have enough grids for all cameras
    const int views_left = min_init_views - calib->nb_views();
    if (views_left > 0) {
      return;
    }

    // Solve initial views
    calib->solve();
    calib->show_results();

    // Change settings for online execution
    calib->enable_marginalization = true;
    calib->max_iter = 5;
    calib->window_size = 5;
    calib->reset();

    // Transition to NBT mode
    LOG_INFO("Transition to NBT mode");
    state = NBT;
    // find_nbt_event = true;
  }

  /** NBT Mode **/
  void mode_nbt() {
    // Visualize
    visualize();
  }

  /** Publish Estimates */
  void publish_estimates(const timestamp_t ts) {
    // Pre-check
    if (calib->nb_views() < calib->window_size) {
      return;
    }

    // Publish
    ros::Time ros_ts;
    ros_ts.fromNSec(ts);
    // -- Fiducial frame
    const mat4_t T_WF = calib->get_fiducial_pose();
    publish_fiducial_tf(ros_ts, calib->calib_target, T_WF, tf_br, rviz_pub);
    // -- IMU frame
    const mat4_t T_WS = calib->get_imu_pose();
    publish_tf(ros_ts, "T_WS", T_WS, tf_br);
    // -- Camera frames
    const mat4_t T_BS = calib->get_imu_extrinsics();
    const mat4_t T_SB = tf_inv(T_BS);
    for (const auto cam_idx : calib->get_camera_indices()) {
      const mat4_t T_BCi = calib->get_camera_extrinsics(cam_idx);
      const mat4_t T_WCi = T_WS * T_SB * T_BCi;
      publish_tf(ros_ts, "T_WC" + std::to_string(cam_idx), T_WCi, tf_br);
    }
  }

  /**
   * IMU Callback
   * @param[in] msg Imu message
   */
  void imu0_callback(const sensor_msgs::ImuConstPtr &msg) {
    std::lock_guard<std::mutex> guard(mtx);
    const timestamp_t ts = msg->header.stamp.toNSec();
    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    calib->add_measurement(ts, a_m, w_m);

    fprintf(imu_file, "%ld,", ts);
    fprintf(imu_file, "%f,%f,%f,", gyr.x, gyr.y, gyr.z);
    fprintf(imu_file, "%f,%f,%f\n", acc.x, acc.y, acc.z);
    fflush(imu_file);
  }

  /**
   * Camera Image Callback
   * @param[in] msg Image message
   * @param[in] cam_idx Camera index
   */
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const int cam_idx) {
    // Setup
    std::lock_guard<std::mutex> guard(mtx);
    const timestamp_t ts = msg->header.stamp.toNSec();
    const cv::Mat cam_img = msg_convert(msg);

    // Write image measurement to disk
    const std::string img_fname = std::to_string(ts) + ".png";
    const std::string img_path = cam_dirs[cam_idx] + "/data/" + img_fname;
    cv::imwrite(img_path.c_str(), cam_img);
    fprintf(cam_files[cam_idx], "%ld,%ld.png\n", ts, ts);
    fflush(cam_files[cam_idx]);

    // Add camera image to calibrator
    const bool ready = calib->add_measurement(ts, cam_idx, cam_img);
    if (ready == false) {
      return;
    }

    // printf("\nts: %ld\n", ts);
    // calib->prof.print("detection");
    // calib->prof.print("solve");
    // calib->prof.print("marginalize");

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

    // Publish estimates
    publish_estimates(ts);
  }

  void loop() {
    while (keep_running) {
      ros::spinOnce();
    }
  }
};

} // namespace yac
