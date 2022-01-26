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

struct calib_nbv_t {
  // Calibration state
  enum CALIB_STATE {
    INITIALIZE = 0,
    NBV = 2,
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
  bool cam_init = false;
  bool capture_event = false;
  bool nbv_event = false;
  bool batch_event = false;
  // struct termios term_config_orig;

  // Calibration
  std::unique_ptr<calib_target_t> calib_target;
  std::unique_ptr<aprilgrid_detector_t> detector;
  std::unique_ptr<calib_camera_t> calib;

  // Settings
  int min_intrinsics_views = 5;
  real_t min_intrinsics_view_diff = 20.0;
  mat4_t nbv_pose = I(4);
  aprilgrid_t *target_grid = nullptr;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // Data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  std::map<int, aprilgrid_t> grid_buffer;
  std::map<int, aprilgrids_t> cam_grids;

  /* Constructor */
  calib_nbv_t() = delete;

  /* Constructor */
  calib_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
    setup_calib_target(config_file);
    setup_calibrator(config_file);
    setup_aprilgrid_detector();
    setup_ros_topics(config_file);
  }

  /* Destructor */
  ~calib_nbv_t() = default;

  /** Setup Calibration Target */
  void setup_calib_target(const std::string &config_file) {
    config_t config{config_file};
    calib_target = std::make_unique<calib_target_t>();
    if (calib_target->load(config_file, "calib_target") != 0) {
      FATAL("Failed to parse calib_target in [%s]!", config_file.c_str());
    }
  }

  /** Setup Camera Calibrator */
  void setup_calibrator(const std::string &config_file) {
    // Load configuration
    config_t config{config_file};
    // -- Parse camera settings
    std::map<int, veci2_t> cam_res;
    std::map<int, std::string> cam_proj_models;
    std::map<int, std::string> cam_dist_models;
    for (int cam_idx = 0; cam_idx < 100; cam_idx++) {
      // Check if key exists
      const std::string cam_str = "cam" + std::to_string(cam_idx);
      if (yaml_has_key(config, cam_str) == 0) {
        continue;
      }

      // Parse
      veci2_t resolution;
      std::string proj_model;
      std::string dist_model;
      parse(config, cam_str + ".resolution", resolution);
      parse(config, cam_str + ".proj_model", proj_model);
      parse(config, cam_str + ".dist_model", dist_model);
      cam_res[cam_idx] = resolution;
      cam_proj_models[cam_idx] = proj_model;
      cam_dist_models[cam_idx] = dist_model;
    }
    if (cam_res.size() == 0) {
      FATAL("Failed to parse any camera parameters...");
    }

    // Setup calibrator
    LOG_INFO("Setting up camera calibrator ...");
    calib = std::make_unique<calib_camera_t>(*calib_target.get());
    for (auto &[cam_idx, _] : cam_res) {
      LOG_INFO("Adding [cam%d] params", cam_idx);
      calib->add_camera(cam_idx,
                        cam_res[cam_idx].data(),
                        cam_proj_models[cam_idx],
                        cam_dist_models[cam_idx]);
      calib->add_camera_extrinsics(cam_idx,
                                   I(4),
                                   (cam_idx == 0) ? true : false);
    }
  }

  /** Setup AprilGrid detector */
  void setup_aprilgrid_detector() {
    detector =
        std::make_unique<aprilgrid_detector_t>(calib_target->tag_rows,
                                               calib_target->tag_cols,
                                               calib_target->tag_size,
                                               calib_target->tag_spacing);
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
      auto cb = std::bind(&calib_nbv_t::image_callback, this, _1, cam_idx);
      mcam_subs[cam_idx] = img_trans.subscribe(topic, 1, cb);
    }
  }

  /** Return Number of Cameras */
  int nb_cams() { return calib->nb_cams(); }

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
        case 126: // Presentation clicker up / down key
        case 99:  // 'c' key
          capture_event = true;
          break;
        case 66: // 'b' key
          batch_event = true;
          break;
      }
    }
  }

  /**
   * Visualize detected AprilGrid
   */
  void visualize() {
    // Form visualization image
    cv::Mat viz;
    for (auto [cam_idx, data] : img_buffer) {
      // Convert image gray to rgb
      auto img_gray = data.second;
      auto img = gray2rgb(img_gray);

      // Draw "detected" if AprilGrid was observed
      if (grid_buffer.count(cam_idx)) {
        draw_detected(grid_buffer[cam_idx], img);
      }

      // Draw NBV
      // if (state == NBV) {
      //   if (nbv_event == false) {
      //     draw_nbv(nbv_pose, image_rgb);
      //   } else {
      //     draw_status_text("Finding NBV!", image_rgb);
      //   }
      // }

      // Stack the images up
      if (viz.empty()) {
        viz = img;
        continue;
      }
      cv::hconcat(viz, img, viz);
    }

    // Show
    cv::imshow("Viz", viz);
    int event = cv::waitKey(1);
    event_handler(event);
  }

  /**
   * Check if reached NBV
   * @param[in] grid AprilGrid
   */
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

  // void create_target_grid(const mat4_t &nbv_pose) {
  //   const int tag_rows = target.tag_rows;
  //   const int tag_cols = target.tag_cols;
  //   const double tag_size = target.tag_size;
  //   const double tag_spacing = target.tag_spacing;
  //
  //   const auto cam_res = cam_params.resolution;
  //   const vecx_t proj_params = cam_params.proj_params();
  //   const vecx_t dist_params = cam_params.dist_params();
  //   const T camera{cam_res, proj_params, dist_params};
  //
  //   if (target_grid != nullptr) {
  //     delete target_grid;
  //   }
  //   auto grid = new aprilgrid_t{0, tag_rows, tag_cols, tag_size,
  //   tag_spacing}; const mat4_t T_CF = nbv_pose.inverse(); for (int tag_id =
  //   0; tag_id < (tag_rows * tag_cols); tag_id++) {
  //     for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
  //       const vec3_t r_FFi = grid->object_point(tag_id, corner_idx);
  //       const vec3_t r_CFi = tf_point(T_CF, r_FFi);
  //
  //       vec2_t z_hat{0.0, 0.0};
  //       if (camera.project(r_CFi, z_hat) == 0) {
  //         grid->add(tag_id, corner_idx, z_hat);
  //       }
  //     }
  //   }
  //   target_grid = grid;
  // }

  /**
   * Initialize Intrinsics + Extrinsics Mode
   */
  int mode_init() {
    // Form calibration data
    for (auto &[cam_idx, data] : img_buffer) {
      // Check if have enough grids already
      if (cam_grids[cam_idx].size() >= min_intrinsics_views) {
        continue;
      }

      // Check if aprilgrid is detected and fully observable
      if (grid_buffer.count(cam_idx) == 0) {
        continue;
      }
      const auto &grid_k = grid_buffer[cam_idx];
      if (grid_k.detected == false || grid_k.fully_observable() == false) {
        continue;
      }

      // Check current grid against previous grid to see if the view has
      // changed enought via reprojection error
      if (cam_grids[cam_idx].size()) {
        const auto &grid_km1 = cam_grids[cam_idx].back();
        const vec2s_t kps_km1 = grid_km1.keypoints();
        const vec2s_t kps_k = grid_k.keypoints();

        std::vector<real_t> errors;
        for (size_t i = 0; i < kps_km1.size(); i++) {
          const vec2_t diff = kps_km1[i] - kps_k[i];
          errors.push_back(diff.norm());
        }

        if (rmse(errors) < min_intrinsics_view_diff) {
          continue;
        }
      }

      // Add to calibration data
      const auto views_left = min_intrinsics_views - cam_grids[cam_idx].size();
      if (views_left) {
        LOG_INFO("Adding cam%d data - still needs %ld views",
                 cam_idx,
                 views_left);
        cam_grids[cam_idx].push_back(grid_k);
      }
    }

    // Check if we have enough grids for all cameras
    bool ready = true;
    for (const auto [cam_idx, grids] : cam_grids) {
      if (grids.size() < min_intrinsics_views) {
        ready = false;
      }
    }
    if (ready == false) {
      return 0;
    }

    // Initialize camera intrinsics + extrinsics
    calib_camera_t cam_calib{*calib_target.get()};
    cam_calib.verbose = false;
    for (const auto [cam_idx, grids] : cam_grids) {
      const auto cam_res = calib->get_camera_resolution(cam_idx);
      const auto proj_model = calib->get_camera_projection_model(cam_idx);
      const auto dist_model = calib->get_camera_distortion_model(cam_idx);
      cam_calib.add_camera(cam_idx, cam_res.data(), proj_model, dist_model);
      cam_calib.add_camera_data(cam_idx, grids);
    }
    cam_calib.solve();

    // Update main calibrator with new camera intrinsics
    for (const auto [cam_idx, grids] : cam_grids) {
      calib->cam_params[cam_idx]->param = cam_calib.get_camera_params(cam_idx);
      calib->cam_exts[cam_idx]->param =
          tf_vec(cam_calib.get_camera_extrinsics(cam_idx));
    }

    // Transition to NBV mode
    state = NBV;
    nbv_event = true;

    return 0;
  }

  /**
   * NBV Mode
   */
  void mode_nbv() {
    if (nbv_event == false) {
      return;
    }

    // // LOG_INFO("Find NBV!");
    // // find_nbv();

    // Reset
    nbv_reproj_err = std::numeric_limits<double>::max();
    nbv_hold_tic = (struct timespec){0, 0};
    nbv_event = false;
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

    // Detect and visualize
    detect();
    visualize();
    if (grid_buffer.size() == 0) {
      return;
    }

    // Initialize
    switch (state) {
      case INITIALIZE:
        mode_init();
        break;
      case NBV:
        mode_nbv();
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
}; // namespace yac

} // namespace yac

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start calibrating
  yac::calib_nbv_t calib{node_name};
  calib.loop();

  return 0;
}
