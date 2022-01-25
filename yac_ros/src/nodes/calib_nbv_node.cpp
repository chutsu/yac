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
  enum CALIB_STATE { INITIALIZE = 0, NBV = 1, BATCH = 2 };

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  image_transport::ImageTransport img_trans{ros_nh};
  std::map<int, std::string> mcam_topics;
  std::map<int, image_transport::Subscriber> mcam_subs;

  // State
  std::mutex calib_mutex;
  int state = INITIALIZE;
  bool keep_running = true;
  bool cam_init = false;
  bool capture_event = false;
  bool nbv_event = false;
  bool batch_event = false;
  struct termios term_config_orig;

  // Calibration
  std::unique_ptr<calib_target_t> calib_target;
  std::unique_ptr<aprilgrid_detector_t> detector;
  std::unique_ptr<calib_camera_t> calib;

  mat4_t nbv_pose = I(4);
  aprilgrid_t *target_grid = nullptr;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // Data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  aprilgrids_t grids;

  calib_nbv_t() = delete;

  calib_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
    setup_calib_target(config_file);
    setup_calibrator(config_file);
    setup_aprilgrid_detector();
    setup_ros_topics(config_file);
  }

  ~calib_nbv_t() = default;

  void setup_calib_target(const std::string &config_file) {
    config_t config{config_file};
    calib_target = std::make_unique<calib_target_t>();
    if (calib_target->load(config_file, "calib_target") != 0) {
      FATAL("Failed to parse calib_target in [%s]!", config_file.c_str());
    }
  }

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
    }
  }

  void setup_aprilgrid_detector() {
    detector =
        std::make_unique<aprilgrid_detector_t>(calib_target->tag_rows,
                                               calib_target->tag_cols,
                                               calib_target->tag_size,
                                               calib_target->tag_spacing);
  }

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
      auto cb = std::bind(&calib_nbv_t::image_cb,
                          this,
                          std::placeholders::_1,
                          cam_idx);
      mcam_subs[cam_idx] = img_trans.subscribe(topic, 1, cb);
    }
  }

  int nb_cams() { return calib->nb_cams(); }

  void initialize_camera(const aprilgrid_t &grid) {
    // if (cam_params.initialize({grid}) == false) {
    //   LOG_WARN("Failed to initialize camera focal lengths, try again!");
    //   frames.clear();
    //   grids.clear();
    //   return;
    // }

    // Setup initial calibration poses
    // poses_init = calib_init_poses<CAMERA>(target, cam_params);
    LOG_INFO("Camera initialized!");
    cam_init = true;
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
        case 66: // 'b' key
          batch_event = true;
          break;
      }
    }
  }

  void visualize(const aprilgrid_t &grid, const cv::Mat &image) {
    // Draw detected
    auto image_rgb = gray2rgb(image);
    draw_detected(grid, image_rgb);

    // // Draw NBV
    // switch (state) {
    //   case INITIALIZE:
    //     if (poses_init.size()) {
    //       const mat4_t T_FC0 = poses_init[frames.size()];
    //       // draw_nbv(T_FC0, image_rgb);
    //     }
    //     break;
    //
    //   case NBV:
    //     if (nbv_event == false) {
    //       draw_nbv(nbv_pose, image_rgb);
    //     } else {
    //       draw_status_text("Finding NBV!", image_rgb);
    //     }
    //     break;
    // }

    // Show
    cv::imshow("Visualize", image_rgb);
    int event = cv::waitKey(1);
    event_handler(event);
  }

  // bool nbv_reached(const aprilgrid_t &grid) {
  //   // Check if target grid is created
  //   if (target_grid == nullptr) {
  //     return false;
  //   }
  //
  //   // Get grid measurements
  //   std::vector<int> tag_ids;
  //   std::vector<int> corner_indicies;
  //   vec2s_t keypoints;
  //   vec3s_t object_points;
  //   grid.get_measurements(tag_ids, corner_indicies, keypoints,
  //   object_points);
  //
  //   // See if measured grid matches any NBV grid keypoints
  //   std::vector<double> reproj_errors;
  //   for (size_t i = 0; i < tag_ids.size(); i++) {
  //     const int tag_id = tag_ids[i];
  //     const int corner_idx = corner_indicies[i];
  //     if (target_grid->has(tag_id, corner_idx) == false) {
  //       continue;
  //     }
  //
  //     const vec2_t z_measured = keypoints[i];
  //     const vec2_t z_desired = target_grid->keypoint(tag_id, corner_idx);
  //     reproj_errors.push_back((z_desired - z_measured).norm());
  //   }
  //
  //   // Check if NBV is reached using reprojection errors
  //   nbv_reproj_err = mean(reproj_errors);
  //   if (nbv_reproj_err > nbv_reproj_error_threshold) {
  //     nbv_hold_tic = (struct timespec){0, 0}; // Reset hold timer
  //     return false;
  //   }
  //
  //   // Start NBV hold timer
  //   if (nbv_hold_tic.tv_sec == 0) {
  //     nbv_hold_tic = tic();
  //   }
  //
  //   // If hold threshold not met
  //   if (toc(&nbv_hold_tic) < nbv_hold_threshold) {
  //     return false;
  //   }
  //
  //   return true;
  // }

  void image_cb(const sensor_msgs::ImageConstPtr &msg, const int cam_idx) {
    std::lock_guard<std::mutex> guard(calib_mutex);

    // Convert message to image and add to image buffer
    const timestamp_t ts_k = msg->header.stamp.toNSec();
    img_buffer[cam_idx] = {ts_k, msg_convert(msg)};

    // Make sure timestamps in in image buffer all the same
    bool synced = true;
    for (auto &[cam_idx, data] : img_buffer) {
      const auto img_ts = data.first;
      const auto &img = data.second;
      if (ts_k > img_ts) {
        synced = false;
      }
    }
    if (synced == false) {
      return;
    }

    // Detect and visualize
    const auto &frame_k = img_buffer[0].second;
    const auto grid = detector->detect(ts_k, frame_k);
    visualize(grid, frame_k);
    if (grid.detected == false) {
      return;
    }
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

  // void find_nbv() {
  //   // calib_mono_data_t data{grids, cam_params};
  //   // calib_mono_solve<CAMERA>(data);
  //   // if (nbv_find<CAMERA>(target, data, nbv_pose) == -2) {
  //   //   state = BATCH;
  //   //   return;
  //   // }
  //   // create_target_grid<CAMERA>(nbv_pose);
  // }

  // void mode_init() {
  //   if (cam_init && frames.size() == poses_init.size()) {
  //     LOG_INFO("Collected enough init camera frames!");
  //     LOG_INFO("Transitioning to NBV mode!");
  //     // calib_mono_data_t data{grids, cam_params};
  //     // calib_mono_solve<CAMERA>(data);
  //
  //     // Transition to NBV mode
  //     find_nbv();
  //     state = NBV;
  //   }
  // }

  // void mode_nbv() {
  //   if (nbv_event == false) {
  //     return;
  //   }
  //
  //   LOG_INFO("Find NBV!");
  //   find_nbv();
  //
  //   // Reset
  //   nbv_reproj_err = std::numeric_limits<double>::max();
  //   nbv_hold_tic = (struct timespec){0, 0};
  //   nbv_event = false;
  // }

  // void mode_batch() {
  //   LOG_INFO("Final Optimization!");
  //   std::vector<double> errs;
  //
  //   // calib_mono_data_t data{grids, cam_params};
  //   // calib_mono_solve<CAMERA>(data);
  //   // reproj_errors<CAMERA>(data, errs);
  //
  //   // const std::string results_fpath = "/tmp/calib-mono.csv";
  //   // if (save_results(results_fpath, cam_params, rmse(errs), mean(errs)) !=
  //   // 0)
  //   // {
  //   //   LOG_ERROR("Failed to save results to [%s]!", results_fpath.c_str());
  //   // }
  //
  //   keep_running = false;
  // }

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
  yac::calib_nbv_t calib{node_name};
  calib.loop();

  return 0;
}
