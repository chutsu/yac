#ifndef YAC_ROS_CALIB_HPP
#define YAC_ROS_CALIB_HPP

#include <signal.h>
#include <thread>
#include <termios.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

#include "yac.hpp"
#include "ros_utils.hpp"

namespace yac {

void draw_hcentered_text(const std::string &text,
                         const float text_scale,
                         const int text_thickness,
                         const int text_ypos,
                         cv::Mat &image);
void draw_status_text(const std::string &text, cv::Mat &image);
void draw_detected(const aprilgrid_t &grid, cv::Mat &image);
bool tf_ok(const mat4_t &pose);
void update_aprilgrid_model(const ros::Time &ts,
                            const calib_target_t &target,
                            ros::Publisher &rviz_pub);

void publish_nbt(const ctraj_t &traj, ros::Publisher &pub);
void publish_fiducial_tf(const ros::Time &ts,
                         const calib_target_t &target,
                         const mat4_t &T_WF,
                         tf2_ros::TransformBroadcaster &tf_br,
                         ros::Publisher rviz_pub);
void publish_tf(const ros::Time &ts,
                const std::string &pose_name,
                const mat4_t &pose,
                tf2_ros::TransformBroadcaster &tf_br);

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

/* Stereo NBV calibration */
template <typename CAMERA>
struct calib_stereo_nbv_t {
  // ROS
  ros_config_t &config;
  calib_data_t &calib_data;
  typedef sensor_msgs::Image ImageMsg;
  typedef message_filters::Subscriber<ImageMsg> ImageSubscriber;
  typedef message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg> ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSyncer;
  ImageSubscriber cam0_sub;
  ImageSubscriber cam1_sub;
  message_filters::Synchronizer<ImageSyncPolicy> image_sync;

  // State
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBV = 1,
    BATCH = 2
  };
  int state = INITIALIZE;
  bool keep_running = true;
  bool cam_init = false;
  bool capture_event = false;
  bool nbv_event = false;
  bool batch_event = false;
  struct termios term_config_orig;

  // NBV
  mat4s_t poses_init;
  mat4_t nbv_pose = I(4);
  aprilgrid_t *target_grid = nullptr;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // Data
  aprilgrid_detector_t *detector = nullptr;
  std::map<int, std::vector<cv::Mat>> cam_frames;
  std::string results_fpath = "/tmp/calib_results.yaml";

  calib_stereo_nbv_t(ros_config_t &config_, calib_data_t &calib_data_)
      : config{config_},
        calib_data{calib_data_},
        cam0_sub{*config.ros_nh, config.cam0_topic, 30},
        cam1_sub{*config.ros_nh, config.cam1_topic, 30},
        image_sync(ImageSyncPolicy(10), cam0_sub, cam1_sub) {
    // Setup AprilGrid detector
    detector = new aprilgrid_detector_t{calib_data.target.tag_rows,
                                        calib_data.target.tag_cols,
                                        calib_data.target.tag_size,
                                        calib_data.target.tag_spacing};

    // Setup Non-blocking keyboard handler
    struct termios term_config;
    tcgetattr(0, &term_config);
    term_config_orig = term_config;
    term_config.c_lflag &= ~ICANON;
    term_config.c_lflag &= ~ECHO;
    term_config.c_lflag &= ~ISIG;
    term_config.c_cc[VMIN] = 0;
    term_config.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &term_config);

    // ROS setup
    image_transport::ImageTransport it{*config.ros_nh};
    ros_topic_subscribed(cam0_sub.getSubscriber(), config.cam0_topic);
    ros_topic_subscribed(cam1_sub.getSubscriber(), config.cam1_topic);
    image_sync.registerCallback(&calib_stereo_nbv_t::image_cb, this);
    loop();
  }

  ~calib_stereo_nbv_t() {
    if (detector) {
      delete detector;
    }

    // Restore keyboard
    tcsetattr(0, TCSANOW, &term_config_orig);
  }

  void initialize_camera(const aprilgrid_t &grid) {
    for (int cam_idx = 0; cam_idx < calib_data.nb_cams; cam_idx++) {
      if (calib_data.cam_params[cam_idx].initialize({grid}) == false) {
        LOG_WARN("Failed to initialize camera focal lengths, try again!");
        calib_data.cam_grids[cam_idx].clear();
        cam_frames.clear();
        return;
      }
    }

    // Setup initial calibration poses
    poses_init = calib_init_poses<CAMERA>(calib_data.target, calib_data.cam_params[0]);
    LOG_INFO("Camera initialized!");
    cam_init = true;
  }


  void draw_nbv(const mat4_t &T_FC0, cv::Mat &image) {
    // Draw NBV
    nbv_draw<CAMERA>(calib_data.target, calib_data.cam_params[0], T_FC0, image);

    // Show NBV Reproj Error
    {
      // Create NBV Reproj Error str (1 decimal places)
      std::ostringstream out;
      out.precision(1);
      out << std::fixed << nbv_reproj_err;
      out.str();

      const std::string text = "NBV Reproj Error: " + out.str() + " [px]";
      cv::Scalar text_color;
      if (nbv_reproj_err > 20) {
        text_color = cv::Scalar(0, 0, 255);
      } else {
        text_color = cv::Scalar(0, 255, 0);
      }

      const cv::Point text_pos{10, 50};
      const int text_font = cv::FONT_HERSHEY_PLAIN;
      const float text_scale = 1.0;
      const int text_thickness = 1;
      cv::putText(image,
                  text,
                  text_pos,
                  text_font,
                  text_scale,
                  text_color,
                  text_thickness,
                  CV_AA);
    }

    // Show NBV status
    if (nbv_reproj_err < (nbv_reproj_error_threshold * 1.5)) {
      std::string text = "Nearly There!";
      if (nbv_reproj_err <= nbv_reproj_error_threshold) {
        text = "HOLD IT!";
      }
      draw_status_text(text, image);
    }
  }

  void event_handler(int key) {
    if (key != EOF) {
      switch (key) {
      case 113: // 'q' key
        LOG_INFO("User requested program termination!");
        LOG_INFO("Exiting ...");
        keep_running = false;
        break;
      case 126: // Presentation clicker up / down key
      case 99:  // 'c' key
        capture_event = true;
        break;
      case 66:  // 'b' key
        batch_event = true;
        state = BATCH;
        break;
      }
    }
  }

  void visualize(const aprilgrid_t &grid, const cv::Mat &image) {
    // Draw detected
    auto image_rgb = gray2rgb(image);
    draw_detected(grid, image_rgb);

    // Draw NBV
    switch (state) {
    case INITIALIZE:
      if (poses_init.size()) {
        const mat4_t T_FC0 = poses_init[cam_frames[0].size()];
        // draw_nbv(T_FC0, image_rgb);
      }
      break;

    case NBV:
      if (nbv_event == false) {
        draw_nbv(nbv_pose, image_rgb);
      } else {
        draw_status_text("Finding NBV!", image_rgb);
      }
      break;
    case BATCH:
      draw_status_text("Finished Capturing!", image_rgb);
      break;
    }

    // Show
    cv::imshow("Visualize", image_rgb);
    int event = cv::waitKey(1);
    event_handler(event);
  }

  bool nbv_reached(const aprilgrid_t &grid) {
    // Check if target grid is created
    if (target_grid == nullptr) {
      return false;
    }

    // Get grid measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    // See if measured grid matches any NBV grid keypoints
    std::vector<double> reproj_errors;
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_indicies[i];
      if (target_grid->has(tag_id, corner_idx) == false) {
        continue;
      }

      const vec2_t z_measured = keypoints[i];
      const vec2_t z_desired = target_grid->keypoint(tag_id, corner_idx);
      reproj_errors.push_back((z_desired - z_measured).norm());
    }

    // Check if NBV is reached using reprojection errors
    nbv_reproj_err = mean(reproj_errors);
    if (nbv_reproj_err > nbv_reproj_error_threshold) {
      nbv_hold_tic = (struct timespec){0, 0}; // Reset hold timer
      return false;
    }

    // Start NBV hold timer
    if (nbv_hold_tic.tv_sec == 0) {
      nbv_hold_tic = tic();
    }

    // If hold threshold not met
    if (toc(&nbv_hold_tic) < nbv_hold_threshold) {
      return false;
    }

    return true;
  }

  void image_cb(const sensor_msgs::ImageConstPtr &cam0_msg,
                const sensor_msgs::ImageConstPtr &cam1_msg) {
    // Convert message to image
    const cv::Mat cam0_frame_k = msg_convert(cam0_msg);
    const cv::Mat cam1_frame_k = msg_convert(cam1_msg);

    // Detect and visualize
    const timestamp_t ts = cam0_msg->header.stamp.toNSec();
    auto cam0_grid = detector->detect(ts, cam0_frame_k, true);
    auto cam1_grid = detector->detect(ts, cam1_frame_k, true);
    visualize(cam0_grid, cam0_frame_k);
    if (cam0_grid.detected == false || cam1_grid.detected == false) {
      return;
    }

    // Initialize camera
    if (cam_init == false && cam0_grid.fully_observable()) {
      initialize_camera(cam0_grid);
    } else {
      return;
    }

    // Handle capture event
    switch(state) {
    case INITIALIZE:
      if (capture_event) {
        LOG_INFO("Adding image frame [%ld]!", cam_frames[0].size());
        cam0_grid = detector->detect(ts, cam0_frame_k);
        cam1_grid = detector->detect(ts, cam1_frame_k);
        cam_frames[0].push_back(cam0_frame_k);
        cam_frames[1].push_back(cam1_frame_k);
        calib_data.cam_grids[0].push_back(cam0_grid);
        calib_data.cam_grids[1].push_back(cam1_grid);
      }
      break;

    case NBV:
      mat4_t T_C0F;
      if ((capture_event || nbv_reached(cam0_grid)) && nbv_event == false) {
        cam0_grid = detector->detect(ts, cam0_frame_k);
        cam1_grid = detector->detect(ts, cam1_frame_k);
        cam_frames[0].push_back(cam0_frame_k);
        cam_frames[1].push_back(cam1_frame_k);
        calib_data.cam_grids[0].push_back(cam0_grid);
        calib_data.cam_grids[1].push_back(cam1_grid);
        nbv_event = true;
      }
      break;
    }
    capture_event = false;
  }

  void find_nbv() {
    // Solve batch problem
    calib_stereo_solve<CAMERA>(calib_data);

    // Find NBV
    if (nbv_find<CAMERA>(calib_data.target, calib_data, nbv_pose) == -2) {
      state = BATCH;
      return;
    }
    print_matrix("nbv_pose", nbv_pose);

    if (target_grid != nullptr) {
      delete target_grid;
    }
    target_grid = nbv_target_grid<CAMERA>(calib_data.target, calib_data.cam_params[0], nbv_pose);
    printf("target_grid.nb_detections: %d\n", target_grid->nb_detections);
  }

  void mode_init() {
    if (cam_init && cam_frames[0].size() == poses_init.size()) {
      LOG_INFO("Collected enough init camera frames!");
      LOG_INFO("Transitioning to NBV mode!");

      // Transition to NBV mode
      find_nbv();
      state = NBV;
    }
  }

  void mode_nbv() {
    if (nbv_event == false) {
      return;
    }
    LOG_INFO("Find NBV!");
    find_nbv();

    // Reset
    nbv_reproj_err = std::numeric_limits<double>::max();
    nbv_hold_tic = (struct timespec){0, 0};
    nbv_event = false;
  }

  void mode_batch() {
    LOG_INFO("Final Optimization!");
    std::vector<double> errs;
    calib_stereo_inc_solver_t<CAMERA> solver(calib_data);

    printf("\x1B[92m");
    printf("Saving optimization results to [%s]", results_fpath.c_str());
    printf("\033[0m\n");
    if (save_results(results_fpath, calib_data) != 0) {
      LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    }
    keep_running = false;
  }

  void loop() {
    // Terminal capture thread
    std::thread keyboard_thread([&](){
      LOG_INFO("Press 'c' to capture, 'q' to stop!\n");
      LOG_INFO("Starting calibration initialization mode!");
      while (keep_running) {
        int key = getchar();
        event_handler(key);

        switch (state) {
          case INITIALIZE: mode_init(); break;
          case NBV: mode_nbv(); break;
          case BATCH: mode_batch(); break;
        }
      }
    });

    // ROS loop
    while (keep_running) {
      ros::spinOnce();
    }
  }
};

struct calib_vi_init_node_t {
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBV = 1,
    BATCH = 2
  };

  std::mutex mtx_;
  bool initialized_ = false;
  bool optimizing_ = false;
  bool loop_ = true;

  // ROS
  ros_config_t &config;
  calib_data_t &calib_data;
  // -- IMU subscriber
  ros::Subscriber imu0_sub_;
  // -- Image subscribers
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
  ImageSubscriber cam0_sub_;
  ImageSubscriber cam1_sub_;
  image_transport::Publisher uncertainty_pub_;
  message_filters::Synchronizer<ImageSyncPolicy> image_sync_;
  // -- TF broadcaster
  tf2_ros::TransformBroadcaster tf_br_;
  // -- Rviz marker publisher
  ros::Publisher rviz_pub_;

  // Calibrator
  int state = INITIALIZE;
  mat4s_t poses_init;
  std::map<int, std::vector<cv::Mat>> frames_init;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  aprilgrid_detector_t *detector_ = nullptr;
  yac::calib_vi_init_t est_;

  // Keyboard event handling
  struct termios term_config_;
  struct termios term_config_orig_;

  // Threads
  std::thread keyboard_thread_;

  calib_vi_init_node_t(ros_config_t &config_, calib_data_t &calib_data_)
      : config{config_},
        calib_data{calib_data_},
        cam0_sub_{*config.ros_nh, config.cam0_topic, 30},
        cam1_sub_{*config.ros_nh, config.cam1_topic, 30},
        image_sync_(ImageSyncPolicy(10), cam0_sub_, cam1_sub_) {
    // Set terminal to be non-blocking
    tcgetattr(0, &term_config_);
    term_config_orig_ = term_config_;
    term_config_.c_lflag &= ~ICANON;
    term_config_.c_lflag &= ~ECHO;
    term_config_.c_lflag &= ~ISIG;
    term_config_.c_cc[VMIN] = 0;
    term_config_.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &term_config_);

    // ROS setup
    // clang-format off
    image_transport::ImageTransport it(*config.ros_nh);
    imu0_sub_ = config.ros_nh->subscribe(config_.imu0_topic, 1000, &calib_vi_init_node_t::imu0_callback, this);
    image_sync_.registerCallback(&calib_vi_init_node_t::image_callback, this);
    // clang-format on
    // -- Rviz marker publisher
    rviz_pub_ = config.ros_nh->advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);

    // Configure detector
    detector_ = new aprilgrid_detector_t(calib_data_.target.tag_rows,
                                         calib_data_.target.tag_cols,
                                         calib_data_.target.tag_size,
                                         calib_data_.target.tag_spacing);

    // Initialize poses
    poses_init = calib_init_poses<pinhole_radtan4_t>(calib_data.target, calib_data.cam_params[0]);

    // Configure estimator
    for (size_t i = 0; i < calib_data.cam_params.size(); i++) {
      const auto res = calib_data.cam_params[i].resolution;
      const auto proj_model = calib_data.cam_params[i].proj_model;
      const auto dist_model = calib_data.cam_params[i].dist_model;
      const vecx_t proj_params = calib_data.cam_params[i].proj_params();
      const vecx_t dist_params = calib_data.cam_params[i].dist_params();
      est_.add_camera(i, res, proj_model, dist_model, proj_params, dist_params, true);
      est_.add_cam_extrinsics(i, calib_data.cam_exts[i].tf(), true);
    }
    est_.add_imu(calib_data.imu_params);
    est_.add_imu_extrinsics(I(4));
    loop();
  }

  ~calib_vi_init_node_t() {
    // Join threads
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }

    // Detector
    if (detector_) {
      delete detector_;
    }

    // Clean up (restore blocking keyboard handler)
    tcsetattr(0, TCSANOW, &term_config_orig_);
  }

  void event_handler(int key) {
    if (key == EOF) {
      return;
    }

    switch (key) {
    case 'b':
      loop_ = false;
      break;
    case 'q': // q
    case 27:  // ESC
    case 3:   // Control-C
    case 4:   // Control-D
      loop_ = false;
      break;
    }
  }

  void visualize(const aprilgrid_t &grid, const cv::Mat &image) {
    // Draw detected
    auto image_rgb = gray2rgb(image);
    draw_detected(grid, image_rgb);

    // Draw NBV
    switch (state) {
    case INITIALIZE:
      if (poses_init.size()) {
      }
      break;
    }

    // Show
    // cv::imshow("Visualize", image_rgb);
    // int event = cv::waitKey(1);
    // event_handler(event);
  }

  void image_callback(const sensor_msgs::Image &cam0_msg,
                      const sensor_msgs::Image &cam1_msg) {
    // Double check image timestamps
    const auto cam0_ts = cam0_msg.header.stamp;
    const auto cam1_ts = cam1_msg.header.stamp;
    if (cam0_ts != cam1_ts) {
      ROS_FATAL("Images are not synchronized");
    }

    // Detect AprilGrids
    // -- Form images
    const timestamp_t ts = cam0_ts.toNSec();
    std::vector<cv::Mat> cam_images{
      cv_bridge::toCvCopy(cam0_msg)->image,
      cv_bridge::toCvCopy(cam1_msg)->image
    };
    // -- Detect
    std::vector<aprilgrid_t> grids;
    for (int i = 0; i < (int) est_.nb_cams(); i++) {
      auto grid = detector_->detect(ts, cam_images[i], true);
      grid.timestamp = ts;
      grids.push_back(grid);
    }
    if (grids[0].detected == false || grids[1].detected == false) {
      return;
    }
    // -- Add measurement
    est_.add_measurement(0, grids[0]);
    est_.add_measurement(1, grids[1]);
    frames_init[0].push_back(cam_images[0]);
    frames_init[1].push_back(cam_images[1]);

    // Visualize current sensor pose
    if (est_.initialized && config.publish_tfs) {
      const auto ts = cam0_ts;
      const auto target = calib_data.target;
      const auto T_WF = est_.get_fiducial_pose();
      const auto T_WS = est_.get_sensor_pose(-1);
      const auto T_BS = est_.get_imu_extrinsics();
      const auto T_BC0 = est_.get_cam_extrinsics(0);
      const auto T_BC1 = est_.get_cam_extrinsics(1);
      const auto T_WC0 = T_WS * T_BS.inverse() * T_BC0;
      const auto T_WC1 = T_WS * T_BS.inverse() * T_BC1;
      publish_fiducial_tf(ts, target, T_WF, tf_br_, rviz_pub_);
      publish_tf(ts, "T_WS", T_WS, tf_br_);
      publish_tf(ts, "T_WC0", T_WC0, tf_br_);
      publish_tf(ts, "T_WC1", T_WC1, tf_br_);
    }
  }

  void imu0_callback(const sensor_msgs::ImuConstPtr &msg) {
    std::lock_guard<std::mutex> guard(mtx_);
    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const auto ts = msg->header.stamp.toNSec();
    est_.add_measurement(ts, a_m, w_m);
  }

  void loop() {
    // Terminal capture thread
    keyboard_thread_ = std::thread([&](){
      while (loop_) {
        const int key = getchar();
        event_handler(key);
      }
    });

    // ROS loop
    while (loop_) {
      ros::spinOnce();
    }

    est_.solve();
  }
};

} // namespace yac
#endif // YAC_ROS_CALIB_HPP
