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
    INITIALIZE = 0,
    NBV = 1,
    BATCH = 2,
  };
  int state = INITIALIZE;

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
  std::mutex mtx;
  bool keep_running = true;
  bool find_nbv_event = false;
  bool nbv_reached_event = false;

  // Calibration
  std::string config_file;
  std::unique_ptr<calib_camera_t> calib;

  // Data
  std::map<int, std::pair<timestamp_t, cv::Mat>> img_buffer;
  std::map<int, std::pair<aprilgrid_t, cv::Mat>> grid_buffer;
  std::map<int, std::map<timestamp_t, cv::Mat>> cam_images;
  std::map<int, aprilgrids_t> cam_grids;

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

  // NBV Settings
  int min_intrinsics_views = 5;
  real_t min_intrinsics_view_diff = 10.0;
  double nbv_reproj_threshold = 10.0;
  double nbv_hold_threshold = 1.0;

  /* Constructor */
  calib_nbv_t() = delete;

  /* Constructor */
  calib_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    // Setup calibrator
    LOG_INFO("Setting up camera calibrator ...");
    calib = std::make_unique<calib_camera_t>(config_file);

    // Setup ROS topics
    LOG_INFO("Setting up ROS subscribers ...");
    config_t config{config_file};
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
  }

  /* Destructor */
  ~calib_nbv_t() = default;

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

  /** Add view to calibrator */
  void add_view() {
    std::map<int, aprilgrid_t> view_data;
    for (const auto &[cam_idx, data] : grid_buffer) {
      const auto &grid = data.first;
      const auto ts = grid.timestamp;
      const auto &img = data.second;
      view_data[cam_idx] = data.first;

      // Keep track of all aprilgrids and images
      cam_images[cam_idx][ts] = img;
      cam_grids[cam_idx].push_back(grid);
    }
    calib->add_view(view_data);
    calib->verbose = false;
    calib->enable_nbv = false;
    calib->enable_outlier_filter = false;
    calib->solve(true);
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
          LOG_INFO("Finish!");
          finish();
          keep_running = false;
          break;
        case 'r':
          LOG_INFO("Manual Reset!");
          reset();
          break;
        case 'c':
          LOG_INFO("Manual NBV reached!");
          add_view();
          nbv_reached_event = true;
          break;
      }
    }
  }

  /** Draw NBV */
  void draw_nbv(cv::Mat &img) {
    // Draw NBV
    const auto cam_geom = calib->cam_geoms[nbv_cam_idx];
    const auto cam_params = calib->cam_params[nbv_cam_idx];
    const mat4_t T_FC0 = nbv_poses.at(nbv_cam_idx).at(nbv_idx);
    const mat4_t T_C0Ci = calib->cam_exts[nbv_cam_idx]->tf();
    const mat4_t T_FCi = T_FC0 * T_C0Ci;
    nbv_draw(calib->calib_target, cam_geom, cam_params, T_FCi, img);

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
    add_view();

    return true;
  }

  /** Initialize Intrinsics + Extrinsics Mode */
  int mode_init() {
    // Pre-check
    if (grid_buffer.size() == 0) {
      goto viz_init;
    }

    // Form calibration data
    for (auto &[cam_idx, data] : img_buffer) {
      // Check if have enough grids already
      if (cam_grids[cam_idx].size() >= (size_t)min_intrinsics_views) {
        continue;
      }

      // Check if aprilgrid is detected and fully observable
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

        const auto ts = data.first;
        const auto &img = data.second;
        cam_images[cam_idx][ts] = img;
        cam_grids[cam_idx].push_back(grid_k);
      }
    }

    // Check if we have enough grids for all cameras
    for (const auto [cam_idx, grids] : cam_grids) {
      if (grids.size() < (size_t)min_intrinsics_views) {
        goto viz_init;
      }
    }

    // Initialize camera intrinsics + extrinsics
    calib->add_camera_data(cam_grids);
    calib->enable_nbv = false;
    calib->solve();

    // Transition to NBV mode
    state = NBV;
    find_nbv_event = true;

  viz_init:
    // Visualize Initialization mode
    cv::Mat viz;
    for (auto [cam_idx, data] : img_buffer) {
      // Convert image gray to rgb
      auto img_gray = data.second;
      auto img = gray2rgb(img_gray);

      // Draw "detected" if AprilGrid was observed
      if (grid_buffer.count(cam_idx)) {
        const auto &grid = grid_buffer[cam_idx].first;
        draw_detected(grid_buffer[cam_idx].first, img);
      }

      // Stack the images up
      if (viz.empty()) {
        viz = img;
        continue;
      }
      cv::hconcat(viz, img, viz);
    }
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));

    return 0;
  }

  /** NBV Mode */
  void mode_nbv() {
    // Pre-check
    if (nbv_reached_event == false) {
      nbv_reached_event = check_nbv_reached();
    }
    if (find_nbv_event == false && nbv_reached_event == false) {
      goto viz_nbv;
    }

    // Solve
    printf("Finding NBV\n");
    calib->verbose = true;
    calib->enable_nbv = false;
    calib->enable_outlier_filter = false;
    calib->_solve_batch(false, 5);

    // Create NBV poses based on current calibrations
    nbv_poses = calib_nbv_poses(calib->calib_target,
                                calib->cam_geoms,
                                calib->cam_params,
                                calib->cam_exts);

    // Find NBV
    nbv_cam_idx = 0;
    nbv_idx = 0;
    nbv_info = 0.0;
    info = 0.0;
    info_gain = 0.0;
    prof.start("find_nbt");
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
                                     calib->cam_geoms[nbv_cam_idx],
                                     calib->cam_params[nbv_cam_idx],
                                     0,
                                     T_FCi);

        // Reset NBV data
        nbv_reproj_err = -1.0;
        nbv_hold_tic = (struct timespec){0, 0};
        find_nbv_event = false;
        printf("mean_reproj_errors: %f\n",
               mean(calib->get_all_reproj_errors()));
        printf("nbv_cam_idx: %d, nbv_idx: %d\n", nbv_cam_idx, nbv_idx);
        printf("info_k: %f, info_kp1: %f, info_gain: %f\n",
               info,
               nbv_info,
               info_gain);
        printf("time_taken: %f [s]\n", prof.stop("find_nbt"));
        printf("\n");
      } else if (retval == -2) {
        LOG_INFO("NBV Threshold met!");
        LOG_INFO("Finishing!");

      } else {
        FATAL("Failed to find NBV!");
      }
    }

    // Reset nbv reached flag
    nbv_reached_event = false;

  viz_nbv:
    // Visualize NBV mode
    cv::Mat viz = gray2rgb(img_buffer[nbv_cam_idx].second);
    draw_camera_index(nbv_cam_idx, viz);
    if (find_nbv_event == false) {
      draw_nbv(viz);
    } else {
      draw_status_text("Finding NBV!", viz);
    }

    // Draw detected
    if (grid_buffer.count(nbv_cam_idx)) {
      draw_detected(grid_buffer[nbv_cam_idx].first, viz);
    }

    // Publish fiducial pose
    const ros::Time ros_ts = ros::Time::now();
    const vec3_t rpy_WF{deg2rad(90.0), 0.0, deg2rad(0.0)};
    const vec3_t r_WF{0.0, 0.0, 0.0};
    const mat4_t T_WF = tf(euler321(rpy_WF), r_WF);
    publish_fiducial_tf(ros_ts, calib->calib_target, T_WF, tf_br, rviz_pub);

    // Publish NBV and camera pose
    if (grid_buffer.count(nbv_cam_idx)) {
      const auto &grid = grid_buffer[nbv_cam_idx].first;
      const auto cam_geom = calib->cam_geoms[nbv_cam_idx];
      const auto cam_res = calib->cam_params[nbv_cam_idx]->resolution;
      const vecx_t &cam_param = calib->cam_params[nbv_cam_idx]->param;
      const mat4_t &cam_exts = calib->cam_exts[nbv_cam_idx]->tf();
      const mat4_t &nbv_pose = nbv_poses[nbv_cam_idx][nbv_idx];

      mat4_t T_CiF;
      if (grid.estimate(cam_geom, cam_res, cam_param, T_CiF) != 0) {
        FATAL("Failed to estimate relative pose!");
      }

      publish_tf(ros_ts, "Camera", T_WF * T_CiF.inverse(), tf_br);
      publish_tf(ros_ts, "NBV", T_WF * nbv_pose * cam_exts, tf_br);
    }

    // Show viz
    cv::imshow("Viz", viz);
    event_handler(cv::waitKey(1));
  }

  /** finish */
  void finish() {
    printf("Run final batch solve on all NBVs\n");

    // Create calib data directory
    const std::string calib_data_path = "/tmp/calib_data";
    std::map<int, std::string> cam_dirs;
    // -- Calibration data directory
    if (system(("mkdir -p " + calib_data_path).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", calib_data_path.c_str());
    }
    // -- Camera directories
    for (const auto &[cam_idx, data] : cam_images) {
      const auto cam_dir = calib_data_path + "/cam" + std::to_string(cam_idx);

      // Create camera directory
      if (system(("mkdir -p " + cam_dir).c_str()) != 0) {
        FATAL("Failed to create dir [%s]", cam_dir.c_str());
      }

      // Make sure it is empty
      if (system(("rm " + cam_dir + "/*.png").c_str()) != 0) {
        FATAL("Failed to create dir [%s]", cam_dir.c_str());
      }

      cam_dirs[cam_idx] = cam_dir;
    }

    // Save images
    for (const auto &[cam_idx, data] : cam_images) {
      for (const auto &[ts, cam_img] : data) {
        const auto img_fname = std::to_string(ts) + ".png";
        const auto img_path = cam_dirs[cam_idx] + "/" + img_fname;
        cv::imwrite(img_path, cam_img);
      }
    }

    // Final solve and save results
    calib_camera_t nbv_calib(config_file);
    nbv_calib.add_camera_data(cam_grids);
    nbv_calib.solve();
    nbv_calib.save_results(calib_data_path + "/calib-camera.yaml");
  }

  /**
   * Camera Image Callback
   * @param[in] msg Image message
   * @param[in] cam_idx Camera index
   */
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const int cam_idx) {
    std::lock_guard<std::mutex> guard(mtx);

    // Update image buffer
    if (update_image_buffer(cam_idx, msg) == false) {
      return;
    }

    // Detect Calibration Target
    detect();

    // States
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

  void reset() {
    // Flags
    state = INITIALIZE;
    keep_running = true;
    find_nbv_event = false;
    nbv_reached_event = false;

    // Calibration
    calib.reset();
    calib = std::make_unique<calib_camera_t>(config_file);

    // Data
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

    // NBV Settings
    min_intrinsics_views = 5;
    min_intrinsics_view_diff = 10.0;
    nbv_reproj_threshold = 10.0;
    nbv_hold_threshold = 1.0;
  }

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
