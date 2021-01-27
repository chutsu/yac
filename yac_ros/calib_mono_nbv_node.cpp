#include <signal.h>
#include <thread>
#include <termios.h>

#include "ros.hpp"
#include "yac.hpp"

namespace yac {

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

  // State
  int state = INITIALIZE;
  bool keep_running = true;
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

  // Data
  aprilgrid_detector_t *detector = nullptr;
  std::vector<cv::Mat> frames;
  calib_mono_data_t data;

  calib_mono_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    // Load config
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Setup calibration target, aprilgrid detector
    if (calib_target_load(target, config_file, "calib_target") != 0) {
      FATAL("Failed to load calib target [%s]!", config_file.c_str());
    }
    detector = new aprilgrid_detector_t{target.tag_rows,
                                        target.tag_cols,
                                        target.tag_size,
                                        target.tag_spacing};

    // Setup camera params
    const int cam_idx = 0;
    std::vector<int> cam_res;
    double lens_hfov = 0;
    double lens_vfov = 0;
    config_t config{config_file};
    parse(config, "cam0.resolution", cam_res);
    parse(config, "cam0.lens_hfov", lens_hfov);
    parse(config, "cam0.lens_vfov", lens_vfov);
    parse(config, "cam0.proj_model", proj_model);
    parse(config, "cam0.dist_model", dist_model);

    const double fx = pinhole_focal(cam_res[0], lens_hfov);
    const double fy = pinhole_focal(cam_res[1], lens_vfov);
    const double cx = cam_res[0] / 2.0;
    const double cy = cam_res[1] / 2.0;
    const vec4_t proj_params{fx, fy, cx, cy};
    const vec4_t dist_params = zeros(4, 1);
    data.cam_params = camera_params_t{0, cam_idx, cam_res.data(),
                                      proj_model, dist_model,
                                      proj_params, dist_params};

		// Setup initial calibration poses
		if (proj_model == "pinhole" && dist_model == "radtan4") {
			poses_init = calib_init_poses<pinhole_radtan4_t>(target, data.cam_params);
		} else if (proj_model == "pinhole" && dist_model == "equi4") {
			poses_init = calib_init_poses<pinhole_equi4_t>(target, data.cam_params);
		}

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
    if (proj_model == "pinhole" && dist_model == "radtan4") {
      nbv_draw<pinhole_radtan4_t>(target, data.cam_params, T_FC0, image);
    } else if (proj_model == "pinhole" && dist_model == "equi4") {
      nbv_draw<pinhole_equi4_t>(target, data.cam_params, T_FC0, image);
    } else {
      FATAL("Unsupported projection-distorion type [%s-%s]!",
            proj_model.c_str(), dist_model.c_str());
    }
  }

  void draw_nbv_status(cv::Mat &image) {
    const std::string text = "Finding NBV!";
    const int text_font = cv::FONT_HERSHEY_PLAIN;
    const float text_scale = 3.0;
    const int text_thickness = 3;
    const cv::Scalar text_color{0, 0, 255};

    int baseline = 0;
    auto text_size = cv::getTextSize(text,
                                    text_font,
                                    text_scale,
                                    text_thickness,
                                    &baseline);

    const int img_w = image.cols;
    const int img_h = image.rows;
    int cx = (img_w - text_size.width) / 2.0;
    int cy = (img_h + text_size.height) / 2.0;
    const cv::Point text_pos{cx, cy};

    cv::putText(image,
                text,
                text_pos,
                text_font,
                text_scale,
                text_color,
                text_thickness,
                CV_AA);
  }

  void visualize(const aprilgrid_t &grid, const cv::Mat &image) {
    // Setup
    auto image_rgb = gray2rgb(image);

    // Draw detected
    draw_detected(grid, image_rgb);

    // Draw NBV
    if (state == INITIALIZE) {
      const mat4_t T_FC0 = poses_init[frames.size()];
      draw_nbv(T_FC0, image_rgb);
    } else if (state == NBV) {
      if (nbv_event == false) {
        draw_nbv(nbv_pose, image_rgb);
      } else {
        draw_nbv_status(image_rgb);
      }
    }

    // Show
    cv::imshow("Visualize", image_rgb);
    int event = cv::waitKey(1);
    event_handler(event);
  }

  aprilgrid_t detect(const timestamp_t ts, const cv::Mat &frame) {
    return detector->detect(ts, frame);
  }

  void image_cb(const sensor_msgs::ImageConstPtr &msg) {
    // Convert message to image
    const cv::Mat frame_k = msg_convert(msg);

    // Detect and visualize
    const timestamp_t ts = msg->header.stamp.toNSec();
    const auto grid = detect(ts, frame_k);
    visualize(grid, frame_k);

    // Handle capture event
    if (capture_event) {
      if (grid.detected) {
        if (nbv_event == false) {
          LOG_INFO("Adding image frame [%ld]!", frames.size());
          frames.push_back(frame_k);
          data.grids.push_back(grid);
        } else {
          ROS_WARN("Still optimizing for NBV!");
        }

      } else if (grid.detected == false) {
        ROS_WARN("AprilGrid not detected!");
      }

      if (state == NBV && nbv_event == false) {
        nbv_event = true;
      }

      capture_event = false;
    }
  }

  void mode_init() {
    if (frames.size() == poses_init.size()) {
      LOG_INFO("Collected enough init camera frames!");
      LOG_INFO("Transitioning to NBV mode!");
      calib_mono_solve<pinhole_radtan4_t>(data);
      state = NBV;
      nbv_event = true;
    }
  }

  void mode_nbv() {
    if (nbv_event == false) {
      return;
    }

    LOG_INFO("Find NBV!");
    print_vector("cam_params", data.cam_params.param);
    printf("nb_grids: %ld\n", data.grids.size());
    data.reset();

    if (data.cam_params.proj_model == "pinhole") {
      if (data.cam_params.dist_model == "radtan4") {
        calib_mono_solve<pinhole_radtan4_t>(data);
        nbv_find<pinhole_radtan4_t>(target, data, nbv_pose);
      } else if (data.cam_params.dist_model == "equi4") {
        calib_mono_solve<pinhole_equi4_t>(data);
        nbv_find<pinhole_equi4_t>(target, data, nbv_pose);
      }
    }
    nbv_event = false;
  }

  void mode_batch() {
    LOG_INFO("Final Optimization!");
    std::vector<double> errs;

    if (data.cam_params.proj_model == "pinhole") {
      if (data.cam_params.dist_model == "radtan4") {
        calib_mono_inc_solve<pinhole_radtan4_t>(data);
        reproj_errors<pinhole_radtan4_t>(data);
      } else if (data.cam_params.dist_model == "equi4") {
        calib_mono_inc_solve<pinhole_equi4_t>(data);
        reproj_errors<pinhole_equi4_t>(data);
      }
    }

    const std::string results_fpath = "/tmp/calib-mono.csv";
    if (save_results(results_fpath, data.cam_params, rmse(errs), mean(errs)) != 0) {
      LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
    }
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
