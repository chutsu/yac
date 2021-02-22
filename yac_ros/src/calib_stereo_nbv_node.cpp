#include <signal.h>
#include <thread>
#include <termios.h>

#include "ros.hpp"
#include "yac.hpp"

namespace yac {

struct ros_config_t {
  bool ok = false;
  ros::NodeHandle &ros_nh;
  std::string node_name;
  config_t config;

  // ROS TOPICS
  std::string cam0_topic;
  std::string cam1_topic;

  // Camera0
  std::vector<int> cam0_res;
  std::string cam0_proj_model;
  std::string cam0_dist_model;
  camera_params_t cam0_params;

  // Camera1
  std::vector<int> cam1_res;
  std::string cam1_proj_model;
  std::string cam1_dist_model;
  camera_params_t cam1_params;

  // Extrinsics
  std::map<timestamp_t, pose_t> poses;  // T_BF
  extrinsics_t cam0_exts;
  extrinsics_t cam1_exts;

  // Calibration target
  calib_target_t target;

  ros_config_t(ros::NodeHandle &ros_nh_,
               std::string &node_name_)
      : ros_nh{ros_nh_}, node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Parse config
    config = config_t{config_file};
    parse(config, "ros.cam0_topic", cam0_topic);
    parse(config, "ros.cam1_topic", cam1_topic);
    parse(config, "cam0.resolution", cam0_res);
    parse(config, "cam0.proj_model", cam0_proj_model);
    parse(config, "cam0.dist_model", cam0_dist_model);
    parse(config, "cam1.resolution", cam1_res);
    parse(config, "cam1.proj_model", cam1_proj_model);
    parse(config, "cam1.dist_model", cam1_dist_model);

    // Camera0
    cam0_params = camera_params_t{0, 0, cam0_res.data(),
                                  cam0_proj_model, cam0_dist_model,
                                  4, 4};

    // Camera1
    cam1_params = camera_params_t{1, 1, cam1_res.data(),
                                  cam1_proj_model, cam1_dist_model,
                                  4, 4};

    // Extrinsics
    poses.clear();
    cam0_exts = extrinsics_t{2};
    cam1_exts = extrinsics_t{3};

    // Calibration target
    if (calib_target_load(target, config_file, "calib_target") != 0) {
      FATAL("Failed to load calib target [%s]!", config_file.c_str());
    }
  }
};

template <typename CAMERA>
struct calib_stereo_nbv_t {
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBV = 1,
    BATCH = 2
  };

  // ROS
  ros_config_t config;
  typedef sensor_msgs::Image ImageMsg;
  typedef message_filters::Subscriber<ImageMsg> ImageSubscriber;
  typedef message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg> ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSyncer;
  ImageSubscriber cam0_sub;
  ImageSubscriber cam1_sub;
  message_filters::Synchronizer<ImageSyncPolicy> image_sync;

  // State
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
  calib_target_t target;
  aprilgrid_detector_t *detector = nullptr;
  std::vector<cv::Mat> cam0_frames;
  std::vector<cv::Mat> cam1_frames;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;
  std::map<timestamp_t, pose_t> &poses;  // T_BF
  camera_params_t &cam0_params;
  camera_params_t &cam1_params;
  extrinsics_t &cam0_exts;
  extrinsics_t &cam1_exts;

  calib_stereo_nbv_t(ros_config_t &config_)
      : config{config_},
        target{config_.target},
        cam0_sub{config.ros_nh, config.cam0_topic, 30},
        cam1_sub{config.ros_nh, config.cam1_topic, 30},
        image_sync(ImageSyncPolicy(10), cam0_sub, cam1_sub),
        poses{config_.poses},
        cam0_params{config_.cam0_params},
        cam1_params{config_.cam1_params},
        cam0_exts{config_.cam0_exts},
        cam1_exts{config_.cam1_exts} {
    // Setup AprilGrid detector
    detector = new aprilgrid_detector_t{target.tag_rows,
                                        target.tag_cols,
                                        target.tag_size,
                                        target.tag_spacing};

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
    image_transport::ImageTransport it{config.ros_nh};
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
    if (cam0_params.initialize({grid}) == false) {
      LOG_WARN("Failed to initialize camera focal lengths, try again!");
      cam0_frames.clear();
      cam0_grids.clear();
      return;
    }
    if (cam1_params.initialize({grid}) == false) {
      LOG_WARN("Failed to initialize camera focal lengths, try again!");
      cam1_frames.clear();
      cam1_grids.clear();
      return;
    }

		// Setup initial calibration poses
    poses_init = calib_init_poses<CAMERA>(target, cam0_params);
    LOG_INFO("Camera initialized!");
		cam_init = true;
  }

  void draw_status_text(const std::string &text, cv::Mat &image) {
    // Create overlay image
    const int img_w = image.cols;
    const int img_h = image.rows;
    cv::Mat overlay = image.clone();

    // Text properties
    const int text_font = cv::FONT_HERSHEY_PLAIN;
    const float text_scale = 4.0;
    const int text_thickness = 4;
    const cv::Scalar text_color{0, 255, 0};
    int baseline = 0;
    auto text_size = cv::getTextSize(text,
                                      text_font,
                                      text_scale,
                                      text_thickness,
                                      &baseline);
    int text_x = (img_w - text_size.width) / 2.0;
    int text_y = (img_h + text_size.height) / 2.0;
    const cv::Point text_pos{text_x, text_y}; // Bottom left of text string

    // Overlay properties
    const int pad = 20;
    const double alpha = 0.5;
    const int overlay_x1 = ((img_w - text_size.width) / 2.0) - pad / 2.0;
    const int overlay_y1 = ((img_h - text_size.height) / 2.0) - pad / 2.0;
    const int overlay_x2 = overlay_x1 + text_size.width + pad / 2.0;
    const int overlay_y2 = overlay_y1 + text_size.height + pad / 2.0;
    const cv::Point2f p0(overlay_x1, overlay_y1);
    const cv::Point2f p1(overlay_x2, overlay_y2);
    const cv::Scalar overlay_color{0, 0, 0};
    cv::rectangle(overlay, p0, p1, overlay_color, -1);

    // Draw overlay and text
    cv::addWeighted(overlay, alpha, image, 1 - alpha,0, image);
    cv::putText(image,
                text,
                text_pos,
                text_font,
                text_scale,
                text_color,
                text_thickness,
                CV_AA);
  }

  void draw_detected(const aprilgrid_t &grid, cv::Mat &image) {
    // Visualize detected
    std::string text = "AprilGrid: ";
    cv::Scalar text_color;
    if (grid.detected) {
      text += "detected!";
      text_color = cv::Scalar(0, 255, 0);
    } else {
      text += "not detected!";
      text_color = cv::Scalar(0, 0, 255);
    }
    const cv::Point text_pos{10, 30};
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

    // Visualize detected corners
    const auto corner_color = cv::Scalar(0, 255, 0);
    for (const vec2_t &kp : grid.keypoints()) {
      cv::circle(image, cv::Point(kp(0), kp(1)), 1.0, corner_color, 2, 8);
    }
  }

  void draw_nbv(const mat4_t &T_FC0, cv::Mat &image) {
    // Draw NBV
    nbv_draw<CAMERA>(target, cam0_params, T_FC0, image);

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
        const mat4_t T_FC0 = poses_init[cam0_frames.size()];
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
    }

    // Handle capture event
    switch(state) {
    case INITIALIZE:
      if (capture_event) {
        LOG_INFO("Adding image frame [%ld]!", cam0_frames.size());
        cam0_grid = detector->detect(ts, cam0_frame_k);
        cam1_grid = detector->detect(ts, cam1_frame_k);
        cam0_frames.push_back(cam0_frame_k);
        cam1_frames.push_back(cam1_frame_k);
        cam0_grids.push_back(cam0_grid);
        cam1_grids.push_back(cam1_grid);
      }
      break;

    case NBV:
      mat4_t T_C0F;
      if ((capture_event || nbv_reached(cam0_grid)) && nbv_event == false) {
        cam0_grid = detector->detect(ts, cam0_frame_k);
        cam1_grid = detector->detect(ts, cam1_frame_k);
        cam0_frames.push_back(cam0_frame_k);
        cam1_frames.push_back(cam1_frame_k);
        cam0_grids.push_back(cam0_grid);
        cam1_grids.push_back(cam1_grid);
        nbv_event = true;
      }
      break;
    }
    capture_event = false;
  }

  void find_nbv() {
    // Solve batch problem
    calib_stereo_data_t data{target, cam0_grids, cam1_grids,
                             poses, cam0_params, cam1_params,
                             cam0_exts, cam1_exts};
    calib_stereo_solve<CAMERA>(data);

    // Find NBV
    if (nbv_find<CAMERA>(target, data, nbv_pose) == -2) {
      state = BATCH;
      return;
    }
    print_matrix("nbv_pose", nbv_pose);

    if (target_grid != nullptr) {
      delete target_grid;
    }
    target_grid = nbv_target_grid<CAMERA>(target, cam0_params, nbv_pose);
    printf("target_grid.nb_detections: %d\n", target_grid->nb_detections);
  }

  void mode_init() {
    if (cam_init && cam0_frames.size() == poses_init.size()) {
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

    calib_stereo_data_t data{target, cam0_grids, cam1_grids,
                             poses, cam0_params, cam1_params,
                             cam0_exts, cam1_exts};
    calib_stereo_inc_solver_t<CAMERA> solver(data);
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

} // namespace yac

int main(int argc, char *argv[]) {
  // Setup ROS Node
  std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Load ROS Config
  ros::NodeHandle ros_nh;
  yac::ros_config_t config{ros_nh, node_name};

  auto proj_model = config.cam0_proj_model;
  auto dist_model = config.cam0_dist_model;
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    yac::calib_stereo_nbv_t<yac::pinhole_radtan4_t> calib{config};
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    yac::calib_stereo_nbv_t<yac::pinhole_equi4_t> calib{config};
  } else {
    FATAL("Unsupported projection-distorion type [%s-%s]!",
          proj_model.c_str(), dist_model.c_str());
  }

  return 0;
}
