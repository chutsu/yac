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

#include <yac_ros/CalibVI.h>

namespace yac {

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

void publish_nbt_data(const calib_vi_t *calib, ros::Publisher &pub) {
  yac_ros::CalibVI msg;

  // Header
  msg.header.stamp = ros::Time::now();

  // Calibration Target
  msg.calib_target.target_type = calib->calib_target.target_type;
  msg.calib_target.tag_rows = calib->calib_target.tag_rows;
  msg.calib_target.tag_cols = calib->calib_target.tag_cols;
  msg.calib_target.tag_size = calib->calib_target.tag_size;
  msg.calib_target.tag_spacing = calib->calib_target.tag_spacing;

  // IMU
  // -- Imu rate
  msg.imu_rate = calib->get_imu_rate();
  // -- Imu params
  msg.imu_params.rate = calib->imu_params.rate;
  msg.imu_params.a_max = calib->imu_params.a_max;
  msg.imu_params.g_max = calib->imu_params.g_max;
  msg.imu_params.sigma_g_c = calib->imu_params.sigma_g_c;
  msg.imu_params.sigma_a_c = calib->imu_params.sigma_a_c;
  msg.imu_params.sigma_gw_c = calib->imu_params.sigma_gw_c;
  msg.imu_params.sigma_aw_c = calib->imu_params.sigma_aw_c;
  msg.imu_params.sigma_bg = calib->imu_params.sigma_bg;
  msg.imu_params.sigma_ba = calib->imu_params.sigma_ba;
  msg.imu_params.g = calib->imu_params.g;
  // -- Imu extrinsics
  const vecx_t imu_exts = tf_vec(calib->imu_exts->tf());
  msg.imu_exts.name = "imu0";
  msg.imu_exts.pose[0] = imu_exts[0];
  msg.imu_exts.pose[1] = imu_exts[1];
  msg.imu_exts.pose[2] = imu_exts[2];
  msg.imu_exts.pose[3] = imu_exts[3];
  msg.imu_exts.pose[4] = imu_exts[4];
  msg.imu_exts.pose[5] = imu_exts[5];
  msg.imu_exts.pose[6] = imu_exts[6];

  // Cameras
  // --- Camera rate
  msg.cam_rate = calib->get_camera_rate();
  // --- Camera params
  for (const auto &[cam_idx, cam] : calib->cam_params) {
    yac_ros::CameraParams cam_msg;
    cam_msg.cam_idx = cam_idx;
    cam_msg.proj_model = cam->proj_model;
    cam_msg.dist_model = cam->dist_model;
    cam_msg.resolution[0] = cam->resolution[0];
    cam_msg.resolution[1] = cam->resolution[1];
    for (int i = 0; i < cam->proj_params().size(); i++) {
      cam_msg.proj_params.push_back(cam->proj_params()[i]);
    }
    for (int i = 0; i < cam->dist_params().size(); i++) {
      cam_msg.dist_params.push_back(cam->dist_params()[i]);
    }
    msg.cam_params.push_back(cam_msg);
  }
  // --- Camera extrinsics
  for (const auto &[cam_idx, cam_exts] : calib->cam_exts) {
    const vecx_t exts = tf_vec(cam_exts->tf());
    yac_ros::Extrinsics exts_msg;
    exts_msg.name = "cam" + std::to_string(cam_idx);
    exts_msg.pose[0] = exts[0];
    exts_msg.pose[1] = exts[1];
    exts_msg.pose[2] = exts[2];
    exts_msg.pose[3] = exts[3];
    exts_msg.pose[4] = exts[4];
    exts_msg.pose[5] = exts[5];
    exts_msg.pose[6] = exts[6];
    msg.cam_exts.push_back(exts_msg);
  }

  // Fiducial Pose
  const vecx_t fiducial_pose = tf_vec(calib->get_fiducial_pose());
  msg.fiducial_pose[0] = fiducial_pose[0];
  msg.fiducial_pose[1] = fiducial_pose[1];
  msg.fiducial_pose[2] = fiducial_pose[2];
  msg.fiducial_pose[3] = fiducial_pose[3];
  msg.fiducial_pose[4] = fiducial_pose[4];
  msg.fiducial_pose[5] = fiducial_pose[5];
  msg.fiducial_pose[6] = fiducial_pose[6];

  // Calibration info
  const matx_t H = calib->calib_info;
  size_t H_idx = 0;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      msg.calib_info[H_idx++] = H(i, j);
    }
  }

  pub.publish(msg);
}

struct calib_nbt_t {
  // Calibration state
  enum CALIB_STATE {
    SETUP = 0,
    INITIALIZE = 1,
    NBT = 2,
    FINISH = 3,
  };

  // Flags
  int state = SETUP;
  std::mutex mtx;
  bool keep_running = true;
  bool finding_nbt = false;

  // Settings
  std::string config_file;
  std::string calib_file;
  std::string data_path;
  std::string imu_dir;
  std::map<int, std::string> cam_dirs;
  std::map<int, std::string> grid_dirs;
  bool use_apriltags3 = true;
  int min_init_views = 10;
  double nbv_reproj_threshold = 50.0;
  double hold_threshold = 1.0;
  const std::string finish_topic = "/yac_ros/nbt_finish";

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  // -- NBT Finish subscriber
  ros::Subscriber finish_sub;
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
  // -- NBT Data publisher
  ros::Publisher nbt_data_pub;
  // -- Reset publisher
  ros::Publisher reset_pub;

  // Calibration
  std::unique_ptr<calib_vi_t> calib;
  std::unique_ptr<aprilgrid_detector_t> detector;

  // Initialization
  aprilgrid_t calib_origin;
  mat4_t T_BS_init;
  mat4_t T_FC0;
  double T_BS_deviation_threshold = 0.3;
  double nbv_reproj_err = -1.0;
  struct timespec hold_tic = (struct timespec){0, 0};

  // NBT
  struct timespec nbt_tic = (struct timespec){0, 0};
  double nbt_pause_threshold = 3.0;
  bool nbt_in_action = false;

  // Data
  profiler_t prof;
  std::map<int, FILE *> cam_files;
  FILE *imu_file;

  // Threads
  std::thread nbt_thread;

  /* Constructor */
  calib_nbt_t() = delete;

  /* Constructor */
  calib_nbt_t(const std::string &node_name_) : node_name{node_name_} {
    ROS_PARAM(ros_nh, node_name_ + "/config_file", config_file);

    // Get camera calibration file
    config_t config{config_file};
    parse(config, "ros.data_path", data_path);
    calib_file = data_path + "/calib_camera/calib-results.yaml";

    // Setup calibrator
    calib = std::make_unique<calib_vi_t>(calib_file);
    calib->max_iter = 30;
    detector =
        std::make_unique<aprilgrid_detector_t>(calib->calib_target.tag_rows,
                                               calib->calib_target.tag_cols,
                                               calib->calib_target.tag_size,
                                               calib->calib_target.tag_spacing);

    // Setup output directories
    if (prep_output() != 0) {
      LOG_WARN("Failed to setup output directories!");
      LOG_WARN("Stopping NBT node...");
      return;
    }

    // Setup ROS
    setup_ros(config_file);

    // Setup initial starting pose
    // -- Calibration origin
    const int cam_idx = 0;
    mat4_t T_FO;
    calib_target_origin(T_FO,
                        calib->calib_target,
                        calib->cam_geoms[cam_idx].get(),
                        calib->cam_params[cam_idx].get());
    // -- Camera pose
    const vec3_t rpy = deg2rad(vec3_t{-180.0, 0.0, 0.0});
    const mat3_t C_FC0 = euler321(rpy);
    T_FC0 = tf(C_FC0, tf_trans(T_FO));
    calib_origin = nbv_target_grid(calib->calib_target,
                                   calib->cam_geoms[cam_idx].get(),
                                   calib->cam_params[cam_idx].get(),
                                   0,
                                   T_FC0);

    // Loop
    LOG_INFO("Move to calibration origin!");
    prof.start("nbt-data_collection");
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
    fprintf(imu_file, "ts,ax,ay,az,gx,gy,gz\n");
    // -- Create Camera directories and image index file
    for (const auto &cam_idx : calib->get_camera_indices()) {
      const auto cam_dir = data_path + "/cam" + std::to_string(cam_idx);
      LOG_INFO("Creating dir [%s]", cam_dir.c_str());

      // Create camera directory
      cam_dirs[cam_idx] = cam_dir;
      if (system(("mkdir -p " + cam_dir + "/data").c_str()) != 0) {
        FATAL("Failed to create dir [%s]", cam_dir.c_str());
      }

      // Create grid directory
      grid_dirs[cam_idx] = data_path + "/grid0/cam" + std::to_string(cam_idx);
      if (system(("mkdir -p " + grid_dirs[cam_idx]).c_str()) != 0) {
        FATAL("Failed to create dir [%s]", grid_dirs[cam_idx].c_str());
      }
      LOG_INFO("Creating dir [%s]", grid_dirs[cam_idx].c_str());

      // Create camera index file
      const std::string csv_path = cam_dir + "/data.csv";
      cam_files[cam_idx] = fopen(csv_path.c_str(), "w");
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
    nbt_data_pub = ros_nh.advertise<yac_ros::CalibVI>("/yac_ros/nbt_data", 0);
    reset_pub = ros_nh.advertise<std_msgs::Bool>("/yac_ros/reset", 0);
    // clang-format on

    // Subscribers
    // clang-format off
    // -- NBT Finish
    LOG_INFO("Subscribing to [%s]", finish_topic.c_str());
    finish_sub = ros_nh.subscribe(finish_topic, 1, &calib_nbt_t::finish_callback, this);
    // -- IMU
    LOG_INFO("Subscribing to imu0 @ [%s]", imu0_topic.c_str());
    imu0_sub = ros_nh.subscribe(imu0_topic, 10000, &calib_nbt_t::imu0_callback, this);
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
          LOG_INFO("Finishing ...");
          finish();
          break;
        case 'n':
          if (state != NBT || calib->calib_info_ok == false) {
            LOG_WARN("Not in NBT mode yet ...");
            return;
          }
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
          reset();
          break;
      }
    }
  }

  /** Draw NBV */
  void draw_nbv(const int cam_idx, cv::Mat &img) {
    // // Draw NBV
    const auto cam_geom = calib->cam_geoms[cam_idx].get();
    const auto cam_params = calib->cam_params[cam_idx].get();
    nbv_draw(calib->calib_target, cam_geom, cam_params, T_FC0, img);

    // Show NBV Reproj Error
    draw_nbv_reproj_error(nbv_reproj_err, img);

    // Show NBV status
    if (nbv_reproj_err > 0.0 && nbv_reproj_err < (nbv_reproj_threshold * 1.5)) {
      std::string text = "Nearly There!";
      if (nbv_reproj_err <= nbv_reproj_threshold) {
        text = "HOLD IT!";
      }
      draw_status_text(text, img);
    }
  }

  /** Check if reached NBV */
  bool check_nbv_reached(const aprilgrid_t &grid) {
    // Check if NBV reached
    std::vector<double> errs;
    bool reached = nbv_reached(calib_origin, grid, nbv_reproj_threshold, errs);
    nbv_reproj_err = mean(errs);
    if (reached == false) {
      hold_tic = (struct timespec){0, 0}; // Reset hold timer
      return false;
    }

    // Start NBV hold timer
    if (hold_tic.tv_sec == 0) {
      hold_tic = tic();
    }

    // If hold threshold not met
    if (toc(&hold_tic) < hold_threshold) {
      return false;
    }

    return true;
  }

  /** Check cam-imu extrinsics */
  bool check_cam_imu_extrinsics() {
    const auto r_BS_init = tf_trans(T_BS_init);
    const auto r_BS_est = tf_trans(calib->get_imu_extrinsics());
    const auto diff = (r_BS_init - r_BS_est).norm();
    if (diff > T_BS_deviation_threshold) {
      LOG_ERROR("Sudden large change in cam0-imu extrinsics (T_cam0_imu0)!");
      LOG_ERROR("This can't be good! Restarting calibration!");
      reset();
      return false;
    }

    return true;
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

  /** Reset */
  void reset() {
    // Reset state
    state = SETUP;

    // Reset calibrator
    calib.reset();
    calib = std::make_unique<calib_vi_t>(calib_file);
    calib->max_iter = 30;

    // Rest calib_infos
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    reset_pub.publish(reset_msg);

    // Remove calibration directory
    const std::string dir_path = data_path;
    const std::string cmd = "rm -rf " + dir_path;
    if (system(cmd.c_str()) != 0) {
      FATAL("Failed to remove dir [%s]", dir_path.c_str());
    }

    // Create calibration directory
    prep_output();
  }

  /** Find NBT */
  void find_nbt() { publish_nbt_data(calib.get(), nbt_data_pub); }

  /** Move camera to calibration origin before initializing calibrator */
  void mode_setup(const timestamp_t ts,
                  const int cam_idx,
                  const cv::Mat &cam_img) {
    // Pre-check
    if (state != SETUP || cam_idx != 0) {
      return;
    }

    // Detect and draw calibration origin
    cv::Mat viz = gray2rgb(cam_img);
    const auto &grid = calib->detector->detect(ts, cam_img);
    if (grid.detected) {
      draw_nbv(0, viz);
      draw_detected(grid, viz);

      if (check_nbv_reached(grid)) {
        state = INITIALIZE;
      }
    }

    // Visualize
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** Initialize Intrinsics + Extrinsics Mode */
  void mode_init() {
    // Visualize
    visualize();

    // Pre-check
    if (calib->grid_buf.size() == 0) {
      return;
    }

    // Check if we have enough grids
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
    calib->window_size = 4;
    calib->reset();

    // Keep track of initial cam-imu extrinsics
    T_BS_init = calib->get_imu_extrinsics();

    // Transition to NBT mode
    LOG_INFO("Transition to NBT mode");
    state = NBT;
  }

  /** NBT Mode **/
  void mode_nbt() {
    // Pre-check
    const int cam_idx = 0;
    if (calib->img_buf.count(cam_idx) == 0) {
      return;
    }

    // // Check cam0-imu extrinsics estimate
    // if (check_cam_imu_extrinsics() == false) {
    //   return;
    // }

    // Convert image gray to rgb
    const auto &img_gray = calib->img_buf[cam_idx].second;
    auto viz = gray2rgb(img_gray);

    // Draw "detected" if AprilGrid was observed
    if (calib->grid_buf.count(cam_idx)) {
      draw_detected(calib->grid_buf[cam_idx], viz);
    }

    // NBT IN ACTION
    if (nbt_in_action) {
      // Check If NBT time threshold met?
      // if (toc(&nbt_tic) < 2.0) {
      //   draw_hcentered_text("Finding NBT! Hold Position!", 1.5, 1, 150, viz);
      // }
      if (toc(&nbt_tic) < nbt_pause_threshold) {
        goto show_viz;
      }

      // // Check if back to calibration origin?
      // if (check_nbv_reached(calib->grid_buf[cam_idx]) == false) {
      //   draw_hcentered_text("Go back to start!", 1.5, 1, 150, viz);
      //   draw_nbv(cam_idx, viz);
      //   goto show_viz;
      // }
    }

    // Find NBT
    // -- Check if detected an AprilGrid
    if (calib->grid_buf.count(cam_idx) == 0) {
      goto show_viz;
    }
    // -- Calculate calib info
    if (calib->recover_calib_info(calib->calib_info) == 0) {
      calib->calib_info_ok = true;
      nbt_tic = tic(); // Start NBT timer
      nbt_in_action = true;
      find_nbt();
    } else {
      calib->calib_info_ok = false;
      nbt_in_action = false;
    }

  show_viz:
    // Show
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** Finish **/
  void finish() {
    // Change state
    state = FINISH;
    cv::destroyAllWindows();
    prof.stop("nbt-data_collection");

    // Unsubscribe imu0
    LOG_INFO("Unsubscribing from [%s]", imu0_topic.c_str());
    imu0_sub.shutdown();

    // Unsubscribe cameras
    for (auto &[cam_idx, cam_sub] : mcam_subs) {
      LOG_INFO("Unsubscribing from [%s]", mcam_topics[cam_idx].c_str());
      cam_sub.shutdown();
    }

    // Solve with full data and save results
    prof.start("nbt-final_solve");
    const auto results_path = data_path + "/calib-results.yaml";
    calib_vi_t final_calib(calib_file);
    final_calib.load_data(data_path);
    final_calib.solve();
    final_calib.show_results();
    final_calib.save_results(results_path);
    prof.stop("nbt-final_solve");

    // Append timings to results file
    // clang-format off
    const auto time_data_collection = prof.record["nbt-data_collection"].back();
    const auto time_final_solve = prof.record["nbt-final_solve"].back();
    const auto time_total = time_data_collection + time_final_solve;
    FILE *results_yaml = fopen(results_path.c_str(), "a");
    fprintf(results_yaml, "profiling:\n");
    fprintf(results_yaml, "  data_collection: %.2f  # [s]\n", time_data_collection);
    fprintf(results_yaml, "  final_solve:     %.2f  # [s]\n", time_final_solve);
    fprintf(results_yaml, "  total_time:      %.2f  # [s]\n", time_total);
    fprintf(results_yaml, "\n");
    fclose(results_yaml);
    // clang-format on

    // Kill loop
    keep_running = false;
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
    // Pre-check
    if (state == SETUP) {
      return;
    }

    // Add measurement
    std::lock_guard<std::mutex> guard(mtx);
    const timestamp_t ts = msg->header.stamp.toNSec();
    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    calib->add_measurement(ts, a_m, w_m);

    // Write imu measurement to file
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

    // Record
    if (state != SETUP) {
      // Add camera image to calibrator
      const bool ready = calib->add_measurement(ts, cam_idx, cam_img);

      // Write image and aprilgrid detection to file in the background
      auto th_lambda = [](const aprilgrid_detector_t *detector,
                          const std::string cam_path,
                          FILE *cam_file,
                          const std::string grid_path,
                          const timestamp_t ts,
                          const cv::Mat cam_img) {
        // Write image measurements to disk
        const std::string img_fname = std::to_string(ts) + ".png";
        const std::string img_path = cam_path + "/data/" + img_fname;
        cv::imwrite(img_path.c_str(), cam_img);
        fprintf(cam_file, "%ld,%ld.png\n", ts, ts);
        fflush(cam_file);

        // // Write full image aprilgrid detection
        // const auto &grid = detector->detect(ts, cam_img);
        // const auto ts_str = std::to_string(ts);
        // grid.save(grid_path + "/" + ts_str + ".csv");
      };
      std::thread th(th_lambda,
                     detector.get(),
                     cam_dirs[cam_idx],
                     cam_files[cam_idx],
                     grid_dirs[cam_idx],
                     ts,
                     cam_img.clone());
      th.detach(); // Do not wait

      if (ready == false) {
        return;
      }
    }

    // States
    switch (state) {
      case SETUP:
        mode_setup(ts, cam_idx, cam_img);
        break;
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

  /** Finish callback */
  void finish_callback(const std_msgs::Bool &msg) {
    // Pre-check
    if (msg.data == false) {
      return;
    }
    finish();
  }

  void loop() {
    while (keep_running) {
      ros::spinOnce();
    }
  }
};

} // namespace yac