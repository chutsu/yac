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
    DONE = 2
  };

  // ROS
  const std::string node_name;
  ros::NodeHandle ros_nh;
  ros::Subscriber cam0_sub;

  // State
  int state = INITIALIZE;
  bool keep_running = true;
  bool capture_event = false;
  struct termios term_config_orig;

  // Calibration
  mat4s_t init_poses;
  mat4s_t nbv_poses;
  calib_target_t target;
  camera_params_t cam_params;

  // Data
  const aprilgrid_detector_t detector = aprilgrid_detector_t();
  cv::Mat frame_k;
  std::vector<cv::Mat> frames;
  aprilgrids_t grids;

  calib_mono_nbv_t(const std::string &node_name_) : node_name{node_name_} {
    // Setup ROS
    std::string config_file;
    ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

    config_t config{config_file};
    std::string cam0_topic;
    parse(config, "ros.cam0_topic", cam0_topic);
    cam0_sub = ros_nh.subscribe(cam0_topic, 1, &calib_mono_nbv_t::image_cb, this);

    // Setup calibration target
    if (calib_target_load(target, config_file) != 0) {
      FATAL("Failed to load calib target [%s]!", config_file.c_str());
    }

    // Setup camera params
    const int cam_idx = 0;
    std::vector<int> cam_res;
    double lens_hfov = 0;
    double lens_vfov = 0;
    std::string proj_model;
    std::string dist_model;
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
    cam_params = camera_params_t{0, cam_idx, cam_res.data(),
                                 proj_model, dist_model,
                                 proj_params, dist_params};

    // Initialize initial poses and nbv poses
    init_poses = generate_initial_poses(target);
    nbv_poses = generate_nbv_poses(target);

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
  }

  ~calib_mono_nbv_t() {
    tcsetattr(0, TCSANOW, &term_config_orig);
  }

  void image_cb(const sensor_msgs::ImageConstPtr &msg) {
    frame_k = msg_convert(msg);

    if (capture_event) {
      aprilgrid_t grid = detect(frame_k);
      if (grid.detected) {
        frames.push_back(frame_k);
        grids.push_back(grid);
      } else {
        LOG_WARN("No AprilGrid detected!");
      }
      capture_event = false;
    }
  }

  aprilgrid_t detect(const cv::Mat &frame) {
    aprilgrid_t grid;
    aprilgrid_set_properties(grid,
                             target.tag_rows,
                             target.tag_cols,
                             target.tag_size,
                             target.tag_spacing);
    const mat3_t K = pinhole_K(cam_params.proj_params());
    const vec4_t D = cam_params.dist_params();
    aprilgrid_detect(grid, detector, frame, K, D);

    return grid;
  }

  void calibrate() {
    mat4s_t T_CF;
    for (const auto &grid : grids) {
      T_CF.push_back(grid.T_CF);
    }

    int retval = calib_mono_solve(grids, cam_params, T_CF);
    if (retval != 0) {
      LOG_ERROR("Failed to calibrate!");
    }
  }

  void find_nbv() {
    calibrate();

    // Estimate covariance
    int cam_idx = 0;
    calib_mono_covar_est_t covar_est{
      cam_idx,
      cam_params.resolution,
      cam_params.proj_model,
      cam_params.dist_model,
      cam_params.proj_params(),
      cam_params.dist_params()
    };

    for (const auto &grid : grids) {
      covar_est.add(grid);
    }

    matx_t covar;
    int retval = covar_est.estimate(covar);
    if (retval != 0) {
      FATAL("Failed to estimate covariance!");
    }

    // Find next best view
    test_grid_t<pinhole_radtan4_t> test_grid{cam_params};
    for (const auto &T_CF : nbv_poses) {



    }
  }

  void mode_init() {
    if (frames.size() == 4) {
      calibrate();
    }
  }

  void mode_nbv() {
    calibrate();

  }

  void loop() {
    // Capture thread
    std::thread keyboard_thread([&](){
      printf("Press 'c' to capture, 'q' to stop!\n");
      while (keep_running) {
        int n = getchar();
        if (n != EOF) {
          int key = n;
          // std::cout << "key: " << key << std::endl;

          switch (key) {
          case 113: // 'q' key
            keep_running = false;
            break;
          case 126: // Presentation clicker up / down key
          case 99:  // 'c' key
            capture_event = true;
            break;
          }
        }
      }
    });

    // ROS loop
    while (keep_running) {
      switch(state) {
        case INITIALIZE: mode_init(); break;
        case NBV: mode_nbv(); break;
      }

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
