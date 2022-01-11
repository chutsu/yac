#include <signal.h>
#include <thread>
#include <termios.h>

#include "ros.hpp"
#include "yac.hpp"

namespace yac {

template <typename CAMERA>
struct calib_mono_nbv_t {
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBV = 1,
    BATCH = 2
  };

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  ros::Subscriber cam0_sub;
  config_t config;

  // State
  int state = INITIALIZE;
  bool keep_running = true;
  bool cam_init = false;
  bool capture_event = false;
  bool nbv_event = false;
  bool batch_event = false;
  struct termios term_config_orig;

  // Calibration
  std::string proj_model;
  std::string dist_model;
  calib_target_t target;
  mat4s_t poses_init;

  mat4_t nbv_pose = I(4);
  aprilgrid_t *target_grid = nullptr;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // Data
  aprilgrid_detector_t *detector = nullptr;
  std::vector<cv::Mat> frames;
  aprilgrids_t grids;
  camera_params_t cam_params;


  calib_mono_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    // Load config
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
    config = config_t{config_file};

    // Setup calibration target, aprilgrid detector
    if (calib_target_load(target, config_file, "calib_target") != 0) {
      FATAL("Failed to load calib target [%s]!", config_file.c_str());
    }
    detector = new aprilgrid_detector_t{target.tag_rows,
                                        target.tag_cols,
                                        target.tag_size,
                                        target.tag_spacing};
    // Parse camera params
    const int cam_idx = 0;
    std::vector<int> cam_res;
    parse(config, "cam0.resolution", cam_res);
    parse(config, "cam0.proj_model", proj_model);
    parse(config, "cam0.dist_model", dist_model);

    // Setup camera parameters
    cam_params = camera_params_t{0, cam_idx, cam_res.data(),
                                 proj_model, dist_model,
                                 4, 4};

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
    std::string cam0_topic;
    parse(config, "ros.cam0_topic", cam0_topic);
    // -- Subscribe ros topics
    cam0_sub = ros_nh.subscribe(cam0_topic, 1, &calib_mono_nbv_t::image_cb, this);
    ros_topic_subscribed(cam0_sub, cam0_topic);
  }

  ~calib_mono_nbv_t() {
    tcsetattr(0, TCSANOW, &term_config_orig);
    if (detector) {
      delete detector;
    }
  }

  void initialize_camera(const aprilgrid_t &grid) {
    if (cam_params.initialize({grid}) == false) {
      LOG_WARN("Failed to initialize camera focal lengths, try again!");
      frames.clear();
      grids.clear();
      return;
    }

    // Setup initial calibration poses
    poses_init = calib_init_poses<CAMERA>(target, cam_params);

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
    nbv_draw<CAMERA>(target, cam_params, T_FC0, image);

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
        const mat4_t T_FC0 = poses_init[frames.size()];
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

  void image_cb(const sensor_msgs::ImageConstPtr &msg) {
    // Convert message to image
    const cv::Mat frame_k = msg_convert(msg);

    // Detect and visualize
    const timestamp_t ts = msg->header.stamp.toNSec();
    const auto grid = detector->detect(ts, frame_k);
    visualize(grid, frame_k);
    if (grid.detected == false) {
      return;
    }

    // Initialize camera
    if (cam_init == false && grid.fully_observable()) {
      initialize_camera(grid);
    }

    // Handle capture event
    switch(state) {
    case INITIALIZE:
      if (capture_event) {
        LOG_INFO("Adding image frame [%ld]!", frames.size());
        frames.push_back(frame_k);
        grids.push_back(grid);
      }
      break;

    case NBV:
      mat4_t T_C0F;
      if ((capture_event || nbv_reached(grid)) && nbv_event == false) {
        frames.push_back(frame_k);
        grids.push_back(grid);
        nbv_event = true;
      }
      break;
    }
    capture_event = false;
  }

  template <typename T>
  void create_target_grid(const mat4_t &nbv_pose) {
    const int tag_rows = target.tag_rows;
    const int tag_cols = target.tag_cols;
    const double tag_size = target.tag_size;
    const double tag_spacing = target.tag_spacing;

    const auto cam_res = cam_params.resolution;
    const vecx_t proj_params = cam_params.proj_params();
    const vecx_t dist_params = cam_params.dist_params();
    const T camera{cam_res, proj_params, dist_params};

    if (target_grid != nullptr) {
      delete target_grid;
    }
    auto grid = new aprilgrid_t{0, tag_rows, tag_cols, tag_size, tag_spacing};
    const mat4_t T_CF = nbv_pose.inverse();
    for (int tag_id = 0; tag_id < (tag_rows * tag_cols); tag_id++) {
      for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
        const vec3_t r_FFi = grid->object_point(tag_id, corner_idx);
        const vec3_t r_CFi = tf_point(T_CF, r_FFi);

        vec2_t z_hat{0.0, 0.0};
        if (camera.project(r_CFi, z_hat) == 0) {
          grid->add(tag_id, corner_idx, z_hat);
        }
      }
    }
    target_grid = grid;
  }

  void find_nbv() {
    calib_mono_data_t data{grids, cam_params};
    calib_mono_solve<CAMERA>(data);
    if (nbv_find<CAMERA>(target, data, nbv_pose) == -2) {
      state = BATCH;
      return;
    }
    create_target_grid<CAMERA>(nbv_pose);
  }

  void mode_init() {
    if (cam_init && frames.size() == poses_init.size()) {
      LOG_INFO("Collected enough init camera frames!");
      LOG_INFO("Transitioning to NBV mode!");
      calib_mono_data_t data{grids, cam_params};
      calib_mono_solve<CAMERA>(data);

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

    calib_mono_data_t data{grids, cam_params};
    calib_mono_solve<CAMERA>(data);
    reproj_errors<CAMERA>(data, errs);

    const std::string results_fpath = "/tmp/calib-mono.csv";
    if (save_results(results_fpath, cam_params, rmse(errs), mean(errs)) != 0) {
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

} // namespace yac

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start calibrating
  yac::calib_mono_nbv_t calib{node_name};
  calib.loop();

  return 0;
}
