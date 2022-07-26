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

struct calib_nbv_t {
  // Calibration state
  enum CALIB_STATE {
    INIT_INTRINSICS = 0,
    INIT_EXTRINSICS = 1,
    NBV = 2,
    SAMPLE = 3,
  };
  int state = INIT_INTRINSICS;

  // Settings
  std::string data_path;
  std::string camera_data_path;
  bool enable_auto_init = true;
  int min_intrinsics_views = 5;
  real_t min_intrinsics_view_diff = 10.0;
  bool enable_nbv_mode = false;
  double nbv_reproj_threshold = 15.0;
  double nbv_hold_threshold = 0.5;
  double pause_threshold = 0.5;

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  image_transport::ImageTransport img_trans{ros_nh};
  std::map<int, std::string> mcam_topics;
  std::map<int, image_transport::Subscriber> mcam_subs;
  // -- TF broadcaster
  tf2_ros::TransformBroadcaster tf_br;
  // -- Rviz marker publisher
  ros::Publisher rviz_pub;

  // Flags
  // std::mutex mtx;
  bool keep_running = true;
  bool has_imu = false;
  bool init_extrinsics_ready = false;
  bool find_nbv_event = false;
  bool nbv_reached_event = false;
  int missing_cam_idx = -1;
  bool adding_view = false;
  int reject_counter = 0;
  bool nbv_pose_set = false;

  // Calibration
  int views_added = 0;
  std::string config_file;
  std::map<int, std::string> cam_dirs;
  std::map<int, std::string> grid_dirs;
  calib_camera_t *calib;
  std::unique_ptr<std::thread> calib_thread;
  std::vector<real_t> calib_info_buffer;
  std::map<timestamp_t, real_t> calib_info;
  std::map<timestamp_t, real_t> calib_info_predict;

  // Data
  imu_params_t imu_params;
  std::map<int, vecx_t> cam_params_init;
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  std::map<int, std::pair<aprilgrid_t, cv::Mat>> grid_buffer;
  std::map<int, std::map<timestamp_t, cv::Mat>> cam_images;
  std::map<int, std::map<timestamp_t, aprilgrid_t>> cam_grids;

  // NBV Data
  profiler_t prof;
  std::map<int, mat4s_t> nbv_poses;
  int nbv_cam_idx = 0;
  int nbv_idx = 0;
  real_t nbv_info = 0.0;
  real_t info = 0.0;
  real_t info_gain = 0.0;
  aprilgrid_t nbv_target;
  double nbv_reproj_err = -1.0;
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // Data sampler
  struct timespec pause_tic = (struct timespec){0, 0};

  /* Constructor */
  calib_nbv_t() = delete;

  /* Constructor */
  calib_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Load config file
    config_t config{config_file};

    // Parse calib data path
    parse(config, "ros.data_path", data_path);
    camera_data_path = data_path + "/calib_camera";

    // Parse imu config if exists
    if (yaml_has_key(config, "imu0")) {
      parse_imu_config(config);
    }

    // Setup calibrator
    LOG_INFO("Setting up camera calibrator ...");
    calib = new calib_camera_t{config_file};
    if (prep_dirs() != 0) {
      LOG_WARN("Assuming cameras are calibrated, if not delete the dir!");
      LOG_WARN("Stopping NBV node...");
      return;
    }

    // Setup ROS topics
    LOG_INFO("Setting up ROS subscribers ...");
    parse_camera_topics(config, mcam_topics);
    if (mcam_topics.size() == 0) {
      FATAL("No camera topics found in [%s]!", config_file.c_str());
    }
    // -- Publishers
    // clang-format off
    rviz_pub = ros_nh.advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);
    // clang-format on
    // -- Subscribe
    for (const auto [cam_idx, topic] : mcam_topics) {
      LOG_INFO("Subscribing to cam%d @ [%s]", cam_idx, topic.c_str());
      auto _1 = std::placeholders::_1;
      auto cb = std::bind(&calib_nbv_t::image_callback, this, _1, cam_idx);
      mcam_subs[cam_idx] = img_trans.subscribe(topic, 1, cb);
    }

    // Loop
    LOG_INFO("Point the camera at the calibration target to initialize...");
    prof.start("nbv-data_collection");
    loop();
  }

  /* Destructor */
  ~calib_nbv_t() {
    if (calib) {
      delete calib;
    }

    cv::destroyAllWindows();
  }

  /** Prepare output directories */
  int prep_dirs() {
    // Create calib data directory
    // -- Check to see if directory already exists
    if (opendir(camera_data_path.c_str())) {
      LOG_WARN("Output directory [%s] already exists!",
               camera_data_path.c_str());
      return -1;
    }
    // -- Create Calibration data directory
    LOG_INFO("Creating dir [%s]", camera_data_path.c_str());
    if (system(("mkdir -p " + camera_data_path).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", camera_data_path.c_str());
    }
    // -- Create Camera directories
    for (const auto &cam_idx : calib->get_camera_indices()) {
      auto cam_dir = camera_data_path;
      cam_dir += "/cam" + std::to_string(cam_idx);
      LOG_INFO("Creating dir [%s]", cam_dir.c_str());

      // Create camera directory
      if (system(("mkdir -p " + cam_dir + "/data").c_str()) != 0) {
        FATAL("Failed to create dir [%s]", cam_dir.c_str());
      }

      cam_dirs[cam_idx] = cam_dir;
    }
    // -- Create grid directories
    for (const auto &cam_idx : calib->get_camera_indices()) {
      auto grid_dir = camera_data_path;
      grid_dir += "/grid0/cam" + std::to_string(cam_idx);
      LOG_INFO("Creating dir [%s]", grid_dir.c_str());

      // Create camera directory
      if (system(("mkdir -p " + grid_dir).c_str()) != 0) {
        FATAL("Failed to create dir [%s]", grid_dir.c_str());
      }

      grid_dirs[cam_idx] = grid_dir;
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

  /** Parse IMU config */
  void parse_imu_config(const config_t &config) {
    parse(config, "imu0.rate", imu_params.rate);
    parse(config, "imu0.a_max", imu_params.a_max);
    parse(config, "imu0.g_max", imu_params.g_max);
    parse(config, "imu0.sigma_g_c", imu_params.sigma_g_c);
    parse(config, "imu0.sigma_a_c", imu_params.sigma_a_c);
    parse(config, "imu0.sigma_gw_c", imu_params.sigma_gw_c);
    parse(config, "imu0.sigma_aw_c", imu_params.sigma_aw_c);
    parse(config, "imu0.sigma_bg", imu_params.sigma_bg, true);
    parse(config, "imu0.sigma_ba", imu_params.sigma_ba, true);
    parse(config, "imu0.g", imu_params.g);
    has_imu = true;
  }

  /** Get NBV Target */
  aprilgrid_t get_nbv_target(const int cam_idx) const {
    const auto cam_geom = calib->cam_geoms[cam_idx].get();
    const auto cam_params = calib->cam_params[cam_idx].get();
    const mat4_t T_FC0 = nbv_poses.at(0).at(nbv_idx);
    const mat4_t T_C0Ci = calib->cam_exts[cam_idx]->tf();
    const mat4_t T_FCi = T_FC0 * T_C0Ci;

    // Show NBV Reproj Error
    return nbv_target_grid(calib->calib_target,
                           calib->cam_geoms[cam_idx].get(),
                           calib->cam_params[cam_idx].get(),
                           0,
                           T_FCi);
  }

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
      const auto grid = calib->detector->detect(img_ts, img);
      if (grid.detected) {
        grid_buffer[cam_idx] = {grid, img};
      }
    }
  }

  /** Check view change */
  bool check_view_change() {
    // Check if detected anything
    if (grid_buffer.size() == 0) {
      return false;
    }

    for (const auto &[cam_idx, data] : grid_buffer) {
      // Check if we have previous detection
      auto last_view = calib->get_last_view();
      if (last_view->grids.count(cam_idx) == 0) {
        continue;
      }

      // Get measurements
      const auto grid_k = data.first;
      const auto grid_km1 = last_view->grids[cam_idx];
      std::vector<int> tag_ids;
      std::vector<int> corner_idxs;
      vec2s_t kps_k;
      vec2s_t kps_km1;
      vec3s_t obj_pts;
      aprilgrid_t::common_measurements(grid_km1,
                                       grid_k,
                                       tag_ids,
                                       corner_idxs,
                                       kps_k,
                                       kps_km1,
                                       obj_pts);

      // See if view point change is signficant enough
      std::vector<real_t> reproj_errors;
      for (size_t i = 0; i < tag_ids.size(); i++) {
        reproj_errors.push_back((kps_k[i] - kps_km1[i]).norm());
      }
      if (mean(reproj_errors) > 20.0) {
        return true;
      }
    }

    return false;
  }

  /** Add view to calibrator */
  bool add_view(const std::map<int, std::pair<aprilgrid_t, cv::Mat>> &cam_data,
                const bool estimate_info = false) {
    // Toggle adding_view flag
    adding_view = true;

    // Convert to view_data
    std::map<int, aprilgrid_t> view_data;
    for (const auto &[cam_idx, data] : cam_data) {
      view_data[cam_idx] = data.first;
    }

    // Add view
    bool retval = calib->add_nbv_view(view_data);
    if (retval) {
      if (calib->nb_views() >= calib->min_nbv_views) {
        calib->_remove_outliers(false);
      }

      // Keep track of aprilgrids and images
      timestamp_t view_ts;
      for (const auto &[cam_idx, data] : cam_data) {
        view_ts = data.first.timestamp;
        const auto &img = data.second;
        const auto &grid = calib->calib_views[view_ts]->grids[cam_idx];

        // Keep track of all aprilgrids and images
        cam_images[cam_idx][view_ts] = img;
        cam_grids[cam_idx][view_ts] = grid;
      }

      // Marginalize oldest view
      if (calib->nb_views() > calib->sliding_window_size) {
        calib->marginalize();
      }

      // Estimate info
      if (estimate_info) {
        calib->_calc_info(&calib->info_k, &calib->entropy_k);
        LOG_INFO("Add View! [Number of NBVs: %d, info_gain: %f]",
                 views_added++,
                 calib->info_k);
        calib_info[view_ts] = calib->info_k;
        calib_info_buffer.push_back(calib->info_k);
      } else {
        LOG_INFO("Add View! [Number of NBVs: %d]", views_added++);
      }

      // Reset reject counter
      reject_counter = 0;
    } else {
      LOG_INFO("Reject View!");
      reject_counter++;
    }

    // Solve
    calib->verbose = false;
    calib->enable_nbv = false;
    calib->enable_outlier_filter = false;
    calib->solver->solve(30);

    // Finish adding view
    adding_view = false;

    return retval;
  }

  /** Event Keyboard Handler */
  void event_handler(int key) {
    if (key != EOF) {
      switch (key) {
        case 'q':
          LOG_INFO("User requested program termination!");
          LOG_INFO("Exiting ...");
          quit();
          break;
        case 'f':
          LOG_INFO("Finish!");
          finish();
          break;
        case 'r':
          LOG_INFO("Manual Reset!");
          reset();
          break;
      }
    }
  }

  /** Draw NBV */
  void draw_nbv(cv::Mat &img, const int cam_idx) {
    // Draw NBV
    const auto cam_geom = calib->cam_geoms[cam_idx].get();
    const auto cam_params = calib->cam_params[cam_idx].get();
    const mat4_t T_FC0 = nbv_poses.at(0).at(nbv_idx);
    const mat4_t T_C0Ci = calib->cam_exts[cam_idx]->tf();
    const mat4_t T_FCi = T_FC0 * T_C0Ci;
    nbv_draw(calib->calib_target, cam_geom, cam_params, T_FCi, img);

    // Show NBV Reproj Error
    draw_nbv_reproj_error(nbv_reproj_err, img);

    // Show NBV status
    // if (nbv_reproj_err > 0.0 && nbv_reproj_err < (nbv_reproj_threshold
    // * 1.5)) {
    //   std::string text = "Nearly There!";
    //   if (nbv_reproj_err <= nbv_reproj_threshold) {
    //     text = "HOLD IT!";
    //   }
    //   draw_status_text(text, img);
    // }
  }

  /* Check if extrinsics can be initialized */
  bool check_init_extrinsics(int &missing_cam_idx) {
    // Setup data
    std::map<timestamp_t, std::map<int, aprilgrid_t>> camchain_data;
    for (const auto &[cam_idx, cam_data] : cam_grids) {
      for (const auto &[ts, grid] : cam_data) {
        camchain_data[ts][cam_idx] = grid;
      }
    }

    // Check
    camchain_t camchain(camchain_data, calib->cam_geoms, calib->cam_params);
    for (const auto &[cam_idx, param] : cam_params_init) {
      mat4_t T_C0Ci;
      if (camchain.find(0, cam_idx, T_C0Ci) != 0) {
        LOG_ERROR("Cannot find extrinsics between cam0-cam%d", cam_idx);
        LOG_ERROR(
            "Capture an image where both cameras can detect the AprilGrid!");
        LOG_ERROR("Press 'c' to trigger a manual image capture");
        missing_cam_idx = cam_idx;
        return false;
      }
    }

    return true;
  }

  /** Check if reached NBV */
  bool check_nbv_reached() {
    // Pre-check
    if (state != NBV || find_nbv_event || grid_buffer.size() == 0 ||
        grid_buffer.count(nbv_cam_idx) == 0) {
      return false;
    }

    // Check if NBV reached
    std::vector<double> reproj_errors;
    const bool reached = nbv_reached(nbv_target,
                                     grid_buffer[nbv_cam_idx].first,
                                     nbv_reproj_threshold,
                                     reproj_errors);
    nbv_reproj_err = mean(reproj_errors);
    if (reached == false) {
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

    // NBV reached! Now add measurements to calibrator
    add_view(grid_buffer);

    return true;
  }

  /** Check information gain */
  void check_info_gain() {
    const auto N = calib_info_buffer.size();
    if (N < 5) {
      return;
    }

    const auto info_prev = calib_info_buffer[N - 5];
    const auto info_curr = calib_info_buffer[N - 1];
    const auto info_gain = 0.5 * (info_prev - info_curr);

    if (info_gain < calib->info_gain_threshold) {
      LOG_INFO("Information gain seems to have reached its limit!");
      LOG_INFO("Information gain = 0.5 * (Info[N - 5] - Info[N - 1])");
      LOG_INFO("                 = 0.5 * (%f - %f)", info_curr, info_prev);
      LOG_INFO("                 = %f", info_gain);
      LOG_INFO("Finishing!");
      finish();
    }
  }

  /** Publish Transforms */
  void publish_tfs(const bool publish_camera_pose = true,
                   const bool publish_nbv = false) {
    // Publish fiducial pose
    const ros::Time ros_ts = ros::Time::now();
    const vec3_t rpy_WF{deg2rad(90.0), 0.0, deg2rad(0.0)};
    const vec3_t r_WF{0.0, 0.0, 0.0};
    const mat4_t T_WF = tf(euler321(rpy_WF), r_WF);
    publish_fiducial_tf(ros_ts, calib->calib_target, T_WF, tf_br, rviz_pub);

    // Publish NBV and camera pose
    if (grid_buffer.count(0)) {
      const auto &grid = grid_buffer[0].first;
      const auto cam_geom = calib->cam_geoms[0].get();
      const auto cam_res = calib->cam_params[0]->resolution;
      const vecx_t &cam_param = calib->cam_params[0]->param;
      const mat4_t &cam_exts = calib->cam_exts[0]->tf();

      // Publish camera pose
      if (publish_camera_pose) {
        mat4_t T_CiF;
        if (grid.estimate(cam_geom, cam_res, cam_param, T_CiF) != 0) {
          FATAL("Failed to estimate relative pose!");
        }
        publish_tf(ros_ts, "Camera", T_WF * T_CiF.inverse(), tf_br);
      }

      // Publish NBV
      if (publish_nbv) {
        const mat4_t &nbv_pose = nbv_poses[0][nbv_idx];
        publish_tf(ros_ts, "NBV", T_WF * nbv_pose * cam_exts, tf_br);
      }
    }
  }

  /** Initialize Intrinsics + Extrinsics Mode */
  void mode_init_intrinsics() {
    // Pre-check
    if (grid_buffer.size() == 0) {
      goto viz_init_intrinsics;
    }

    // Auto initializer - capture images without user intervention
    if (enable_auto_init) {
      for (auto &[cam_idx, data] : img_buffer) {
        // Check if have camera has enough grids already
        if (cam_grids[cam_idx].size() >= (size_t)min_intrinsics_views) {
          continue;
        }

        // Check if aprilgrid is detected
        if (grid_buffer.count(cam_idx) == 0) {
          continue;
        }
        const auto &grid_k = grid_buffer[cam_idx].first;
        if (grid_k.detected == false || grid_k.fully_observable() == false) {
          continue;
        }

        // Check current grid against previous grid to see if the view has
        // changed enough via reprojection error
        if (cam_grids[cam_idx].size()) {
          const auto kv = cam_grids[cam_idx].rbegin();
          const auto &grid_km1 = kv->second;
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
        const auto nb_views = cam_grids[cam_idx].size();
        const auto views_left = min_intrinsics_views - nb_views;
        if (views_left) {
          LOG_INFO("Add cam%d view - [%ld views remaining]",
                   cam_idx,
                   views_left);
          const auto ts = data.first;
          const auto &img = data.second;
          cam_images[cam_idx][ts] = img;
          cam_grids[cam_idx][ts] = grid_k;
        }
      }
    }

    // Check if we have detected anything
    if (cam_grids.size() == 0) {
      goto viz_init_intrinsics;
    }

    // Check if we have enough grids for all cameras
    for (const auto [cam_idx, cam_data] : cam_grids) {
      if (cam_data.size() < (size_t)min_intrinsics_views) {
        goto viz_init_intrinsics;
      }
    }

    // Initialize camera intrinsics
    for (const auto [cam_idx, cam_data] : cam_grids) {
      // Camera intrinsics initialized?
      if (cam_params_init.count(cam_idx)) {
        continue;
      }

      // Initialize camera intrinsics
      aprilgrids_t grids;
      for (const auto &[ts, grid] : cam_data) {
        grids.push_back(grid);
      }

      calib_camera_t calib_init{config_file};
      calib_init.verbose = false;
      calib_init.enable_nbv = false;
      calib_init.add_camera_data(cam_idx, grids, false);
      calib_init.solve(true);
      cam_params_init[cam_idx] = calib_init.cam_params[cam_idx]->param;

      const auto cam_str = "cam" + std::to_string(cam_idx);
      const auto params_str = vec2str(calib_init.cam_params[cam_idx]->param);
      LOG_INFO("Initialize %s params: %s", cam_str.c_str(), params_str.c_str());
    }

    // Transition to initialize extrinsics mode
    if (cam_params_init.size() == (size_t)calib->nb_cameras()) {
      state = INIT_EXTRINSICS;
      init_extrinsics_ready = true;
    }

  viz_init_intrinsics:
    // Visualize Initialization mode
    cv::Mat viz;

    for (auto [cam_idx, data] : img_buffer) {
      // Check if camera intrinsics already initialized
      if (cam_params_init.count(cam_idx)) {
        continue;
      }

      // Draw
      viz = gray2rgb(data.second);
      draw_camera_index(cam_idx, viz);
      if (grid_buffer.count(cam_idx)) {
        draw_detected(grid_buffer[cam_idx].first, viz);
      }
      break;
    }

    if (viz.empty() == false) {
      cv::imshow("Viz", viz);
      event_handler(cv::waitKey(1));
    }
  }

  /** Initialize Extrinsics */
  void mode_init_extrinsics() {
    // Pre-check
    if (init_extrinsics_ready == false) {
      goto viz_init_extrinsics;
    }

    // Check extrinsics
    if (check_init_extrinsics(missing_cam_idx) == false) {
      init_extrinsics_ready = false;
      goto viz_init_extrinsics;
    }

    // Initialize calibrator
    LOG_INFO("Refine initial intrinsics + extrinsics");
    calib->add_camera_data(cam_grids);
    for (const auto &[cam_idx, param] : cam_params_init) {
      calib->cam_params[cam_idx]->param = param;
    }
    calib->verbose = false;
    calib->enable_nbv = false;
    calib->_solve_batch();

    // Transition to NBV mode
    if (enable_nbv_mode) {
      LOG_INFO("Transitioning to NBV mode!");
      state = NBV;
      find_nbv_event = true;
      return;
    } else {
      LOG_INFO("Transitioning to SAMPLE mode!");
      state = SAMPLE;
      return;
    }

  viz_init_extrinsics:
    // Visualize Initialization mode
    // -- Draw cam0
    cv::Mat cam0_img = gray2rgb(img_buffer[0].second);
    draw_camera_index(0, cam0_img);
    if (grid_buffer.count(0)) {
      draw_detected(grid_buffer[0].first, cam0_img);
    }
    // -- Draw cam-i
    cv::Mat cami_img = gray2rgb(img_buffer[missing_cam_idx].second);
    draw_camera_index(missing_cam_idx, cami_img);
    if (grid_buffer.count(missing_cam_idx)) {
      draw_detected(grid_buffer[missing_cam_idx].first, cami_img);
    }
    // -- Concatenate images horizontally
    cv::Mat viz;
    cv::hconcat(cam0_img, cami_img, viz);
    // -- Visualize
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  bool find_nbv() {
    // Pre-check
    if (nbv_reached_event == false) {
      nbv_reached_event = check_nbv_reached();
    }
    if (find_nbv_event == false && nbv_reached_event == false) {
      return false;
    }

    // Solve
    printf("Finding NBV\n");
    calib->verbose = true;
    calib->enable_nbv = false;
    calib->enable_outlier_filter = false;
    calib->solver->solve(5);

    // Create NBV poses based on current calibrations
    if (calib_nbv_poses(nbv_poses,
                        calib->calib_target,
                        calib->cam_geoms,
                        calib->cam_params,
                        calib->cam_exts) != 0) {
      LOG_ERROR("Cannot form NBV poses - Might be bad initialization?");
      LOG_ERROR("Resetting ...");
      reset();
      return false;
    }

    // Find NBV
    nbv_cam_idx = 0;
    nbv_idx = 0;
    nbv_info = 0.0;
    info = 0.0;
    info_gain = 0.0;
    prof.start("find_nbv");
    {
      int retval = calib->find_nbv_fast(nbv_poses,
                                        nbv_cam_idx,
                                        nbv_idx,
                                        nbv_info,
                                        info,
                                        info_gain);
      if (retval == 0) {
        // Form target grid
        const mat4_t T_FC0 = nbv_poses.at(nbv_cam_idx).at(nbv_idx);
        const mat4_t T_C0Ci = calib->cam_exts[nbv_cam_idx]->tf();
        const mat4_t T_FCi = T_FC0 * T_C0Ci;
        nbv_target = nbv_target_grid(calib->calib_target,
                                     calib->cam_geoms[nbv_cam_idx].get(),
                                     calib->cam_params[nbv_cam_idx].get(),
                                     0,
                                     T_FCi);

        // Reset NBV data
        nbv_reproj_err = -1.0;
        nbv_hold_tic = (struct timespec){0, 0};
        find_nbv_event = false;
        printf("nbv_cam_idx: %d, nbv_idx: %d\n", nbv_cam_idx, nbv_idx);
        printf("info_k: %f, info_kp1: %f, info_gain: %f\n",
               info,
               nbv_info,
               info_gain);
        printf("find_nbv() took: %f [s]\n", prof.stop("find_nbv"));
        printf("\n");

        // Track info
        const timestamp_t last_ts = calib->calib_views.rbegin()->first;
        calib_info_buffer.push_back(info);
        calib_info[last_ts] = info;
        calib_info_predict[last_ts] = nbv_info;
        check_info_gain();

      } else if (retval == -2) {
        LOG_INFO("NBV Threshold met!");
        LOG_INFO("Finishing!");
        finish();

      } else {
        FATAL("find_nbv() failed!");
      }
    }

    // Reset nbv reached flag
    nbv_reached_event = false;

    return true;
  }

  /** NBV Mode */
  void mode_nbv() {
    // Find NBV
    find_nbv();

    // Visualize NBV mode
    cv::Mat viz = gray2rgb(img_buffer[nbv_cam_idx].second);
    draw_camera_index(nbv_cam_idx, viz);
    if (find_nbv_event == false) {
      draw_nbv(viz, nbv_cam_idx);
    } else {
      draw_status_text("Finding NBV!", viz);
    }

    // Draw detected
    if (grid_buffer.count(nbv_cam_idx)) {
      draw_detected(grid_buffer[nbv_cam_idx].first, viz);
    }

    // Publish transforms
    publish_tfs(true, true);

    // Show viz
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** Add view in the background */
  void add_view_background() {
    calib_thread = std::make_unique<std::thread>(&calib_nbv_t::add_view,
                                                 this,
                                                 grid_buffer,
                                                 true);
    pause_tic = tic(); // Reset timer
  }

  /** Sample Mode */
  void mode_sample() {
    // Pre-check
    if (pause_tic.tv_sec == 0) {
      pause_tic = tic(); // Start timer
    }

    if (nbv_pose_set) {
      if (grid_buffer.size() && grid_buffer.count(nbv_cam_idx)) {
        // Check if NBV reached
        std::vector<double> reproj_errors;
        const bool reached = nbv_reached(nbv_target,
                                         grid_buffer[nbv_cam_idx].first,
                                         nbv_reproj_threshold,
                                         reproj_errors);
        nbv_reproj_err = mean(reproj_errors);

        // NBV reached! Now add measurements to calibrator
        if (reached) {
          if (add_view(grid_buffer, true)) {
            nbv_pose_set = false;
          } else {
            reject_counter++;
          }

          if (reject_counter >= 3) {
            LOG_INFO("NBV Threshold met!");
            LOG_INFO("Finishing!");
            finish();
          }
        }
      }

    } else if (toc(&pause_tic) > pause_threshold && calib_thread == nullptr) {
      if (check_view_change()) {
        add_view_background();
      }

    } else if (calib_thread != nullptr && adding_view == false) {
      // Join thread
      calib_thread->join();
      calib_thread.reset();
      calib_thread = nullptr;

      // Suggest NBV if reject counter is high
      if (reject_counter >= 3) {
        // Reset counter
        reject_counter = 0;

        // Create NBV poses based on current calibrations
        if (calib_nbv_poses(nbv_poses,
                            calib->calib_target,
                            calib->cam_geoms,
                            calib->cam_params,
                            calib->cam_exts) != 0) {
          LOG_ERROR("Cannot form NBV poses - Might be bad initialization?");
          LOG_ERROR("Resetting ...");
          reset();
          return;
        }

        // Find NBV
        nbv_cam_idx = 0;
        nbv_idx = 0;
        nbv_info = 0.0;
        info = 0.0;
        info_gain = 0.0;
        prof.start("find_nbv");
        int retval = calib->find_nbv_fast(nbv_poses,
                                          nbv_cam_idx,
                                          nbv_idx,
                                          nbv_info,
                                          info,
                                          info_gain);
        if (retval == 0) {
          // Form target grid
          const mat4_t T_FC0 = nbv_poses.at(nbv_cam_idx).at(nbv_idx);
          const mat4_t T_C0Ci = calib->cam_exts[nbv_cam_idx]->tf();
          const mat4_t T_FCi = T_FC0 * T_C0Ci;
          nbv_target = nbv_target_grid(calib->calib_target,
                                       calib->cam_geoms[nbv_cam_idx].get(),
                                       calib->cam_params[nbv_cam_idx].get(),
                                       0,
                                       T_FCi);

          // Reset NBV data
          nbv_reproj_err = -1.0;
          nbv_hold_tic = (struct timespec){0, 0};
          find_nbv_event = false;
          printf("nbv_cam_idx: %d, nbv_idx: %d\n", nbv_cam_idx, nbv_idx);
          printf("info_k: %f, info_kp1: %f, info_gain: %f\n",
                 info,
                 nbv_info,
                 info_gain);
          printf("find_nbv() took: %f [s]\n", prof.stop("find_nbv"));
          printf("\n");
          nbv_pose_set = true;

        } else if (retval == -2) {
          LOG_INFO("NBV Threshold met!");
          LOG_INFO("Finishing!");
          finish();
        }
      }
    }

    // Setup visualization image
    cv::Mat viz;
    for (const auto [cam_idx, cam_data] : img_buffer) {
      cv::Mat cam_viz = gray2rgb(cam_data.second);
      draw_camera_index(cam_idx, cam_viz);

      // Draw detected
      if (grid_buffer.count(cam_idx)) {
        draw_detected(grid_buffer[cam_idx].first, cam_viz);
      }

      // Draw NBV pose
      if (nbv_pose_set) {
        draw_nbv(cam_viz, cam_idx);
      }

      // Draw coverage
      const cv::Scalar color(0, 255, 0);
      const int marker_size = 2;
      cv::Size img_dim{cam_viz.cols, cam_viz.rows};
      cv::Mat coverage = cv::Mat{img_dim, CV_8UC3, cv::Scalar{0}};
      for (const auto &[ts, grid] : cam_grids[cam_idx]) {
        for (const auto &kp : grid.keypoints()) {
          cv::Point2d p(kp.x(), kp.y());
          cv::circle(coverage, p, marker_size, color, -1);
        }
      }

      // Stitch images
      cv::vconcat(cam_viz, coverage, cam_viz);
      if (viz.empty()) {
        viz = cam_viz;
      } else {
        cv::hconcat(viz, cam_viz, viz);
      }
    }

    // Show viz
    cv::imshow("Viz", viz);

    // Publish transforms
    publish_tfs(true, false);

    // Event handler
    event_handler(cv::waitKey(1));
  }

  /** Quit */
  void quit() { keep_running = false; }

  /** finish */
  void finish() {
    LOG_INFO("Saving calibration data to [%s]", camera_data_path.c_str());
    cv::destroyAllWindows();
    prof.stop("nbv-data_collection");

    // Save images and image index
    std::map<int, std::string> cam_data_dirs;
    for (const auto &[cam_idx, data] : cam_images) {
      LOG_INFO("Saving cam%d images to %s", cam_idx, cam_dirs[cam_idx].c_str());

      // Data directory
      cam_data_dirs[cam_idx] = cam_dirs[cam_idx] + "/data";

      // Images
      for (const auto &[ts, cam_img] : data) {
        const auto img_fname = std::to_string(ts) + ".png";
        const auto img_path = cam_data_dirs[cam_idx] + "/" + img_fname;
        cv::imwrite(img_path, cam_img);
      }

      // Image index
      const auto img_idx_path = cam_dirs[cam_idx] + "/data.csv";
      FILE *img_idx = fopen(img_idx_path.c_str(), "w");
      for (const auto &[ts, cam_img] : data) {
        UNUSED(cam_img);
        fprintf(img_idx, "%ld,%ld.png\n", ts, ts);
      }
      fclose(img_idx);
    }

    // Save aprilgrid data
    const auto grids_dir = camera_data_path + "/grid0";
    for (const auto &[cam_idx, cam_data] : cam_grids) {
      const auto cam_str = "cam" + std::to_string(cam_idx);

      for (const auto &[ts, cam_grid] : cam_data) {
        const auto ts_str = std::to_string(ts);
        const auto grid_path = grids_dir + "/" + ts_str + ".csv";
        cam_grid.save(grid_path);
      }
    }

    // Final solve and save results
    prof.start("nbv-final_solve");
    LOG_INFO("Optimize over all NBVs");
    const std::string results_path = camera_data_path + "/calib-results.yaml";
    calib_camera_t final_calib{config_file};
    final_calib.enable_nbv = false;
    final_calib.add_camera_data(cam_grids);
    final_calib.solve();
    final_calib.save_results(results_path);
    prof.stop("nbv-final_solve");

    // Append timings to results file
    // clang-format off
    const auto time_data_collection = prof.record["nbv-data_collection"].back();
    const auto time_final_solve = prof.record["nbv-final_solve"].back();
    const auto time_total = time_data_collection + time_final_solve;
    FILE *results_yaml = fopen(results_path.c_str(), "a");
    fprintf(results_yaml, "profiling:\n");
    fprintf(results_yaml, "  data_collection: %.2f  # [s]\n", time_data_collection);
    fprintf(results_yaml, "  final_solve:     %.2f  # [s]\n", time_final_solve);
    fprintf(results_yaml, "  total_time:      %.2f  # [s]\n", time_total);
    fprintf(results_yaml, "\n");
    fclose(results_yaml);
    // clang-format on

    // Save info
    if (enable_nbv_mode) {
      const std::string info_path = camera_data_path + "/calib_info.csv";
      FILE *info_csv = fopen(info_path.c_str(), "w");
      fprintf(info_csv, "#ts,info_current,info_nbt_predict\n"); // Header
      for (const auto &[ts, info] : calib_info) {
        fprintf(info_csv, "%ld,", ts);
        fprintf(info_csv, "%f,", info);
        fprintf(info_csv, "%f\n", calib_info_predict[ts]);
      }
      fclose(info_csv);
    } else {
      const std::string info_path = camera_data_path + "/calib_info.csv";
      FILE *info_csv = fopen(info_path.c_str(), "w");
      fprintf(info_csv, "#ts,view_idx,info\n"); // Header
      int view_idx = 0;
      for (const auto &[ts, info] : calib_info) {
        fprintf(info_csv, "%ld,", ts);
        fprintf(info_csv, "%d,", view_idx++);
        fprintf(info_csv, "%f\n", info);
      }
      fclose(info_csv);
    }

    // Add imu0 settings if it exists in config file
    if (has_imu) {
      // clang-format off
      FILE *res_file = fopen(results_path.c_str(), "a");
      if (res_file == NULL) {
        perror("Error opening file!");
      }
      fprintf(res_file, "imu0:\n");
      fprintf(res_file, "  rate: %f        # [Hz]\n", imu_params.rate);
      fprintf(res_file, "  a_max: %f       # [m/s^2]\n", imu_params.a_max);
      fprintf(res_file, "  g_max: %f       # [rad/s]\n", imu_params.g_max);
      fprintf(res_file, "  sigma_g_c: %e   # [rad/s/sqrt(Hz)]\n", imu_params.sigma_g_c);
      fprintf(res_file, "  sigma_a_c: %e   # [m/s^2/sqrt(Hz)]\n", imu_params.sigma_a_c);
      fprintf(res_file, "  sigma_gw_c: %e  # [rad/s^s/sqrt(Hz)]\n", imu_params.sigma_gw_c);
      fprintf(res_file, "  sigma_aw_c: %e  # [m/s^2/sqrt(Hz)]\n", imu_params.sigma_aw_c);
      fprintf(res_file, "  sigma_bg: %e    # [rad/s]\n", imu_params.sigma_bg);
      fprintf(res_file, "  sigma_ba: %e    # [m/s^2]\n", imu_params.sigma_ba);
      fprintf(res_file, "  g: %f           # [m/s^2]\n", imu_params.g);
      fclose(res_file);
      // clang-format on
    }

    // Stop NBV node
    keep_running = false;
  }

  /**
   * Camera Image Callback
   * @param[in] msg Image message
   * @param[in] cam_idx Camera index
   */
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const int cam_idx) {
    // std::lock_guard<std::mutex> guard(mtx);

    // Update image buffer
    if (update_image_buffer(cam_idx, msg) == false) {
      return;
    }

    // Detect Calibration Target
    detect();

    // States
    switch (state) {
      case INIT_INTRINSICS:
        mode_init_intrinsics();
        break;
      case INIT_EXTRINSICS:
        mode_init_extrinsics();
        break;
      case NBV:
        mode_nbv();
        break;
      case SAMPLE:
        mode_sample();
        break;
      default:
        FATAL("Implementation Error!");
        break;
    }
  }

  void reset() {
    // Flags
    state = INIT_INTRINSICS;
    keep_running = true;
    find_nbv_event = false;
    nbv_reached_event = false;

    // Calibration
    if (calib) {
      delete calib;
    }
    calib = new calib_camera_t{config_file};

    // Data
    cam_params_init.clear();
    img_buffer.clear();
    grid_buffer.clear();
    cam_images.clear();
    cam_grids.clear();

    // NBV data
    nbv_poses.clear();
    nbv_cam_idx = 0;
    nbv_idx = 0;
    nbv_info = 0.0;
    info = 0.0;
    info_gain = 0.0;
    nbv_target = aprilgrid_t();
    nbv_reproj_err = -1.0;
    nbv_hold_tic = (struct timespec){0, 0};
  }

  void loop() {
    while (keep_running) {
      ros::spinOnce();
    }
  }
};

} // namespace yac
