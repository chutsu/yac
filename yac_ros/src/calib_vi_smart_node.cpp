#include <thread>
#include <future>
#include <functional>
#include <signal.h>
#include <termios.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "yac.hpp"
#include "calib_vi/Calibrator.hpp"
#include "calib_vi/Estimator.hpp"
// #include "calib_vi/NBT.hpp"
#include "calib_vi/cv/PinholeCamera.hpp"
#include "calib_vi/cv/RadialTangentialDistortion.hpp"

#include "ros.hpp"

namespace yac {

struct node_config_t {
  std::string mode;
  bool publish_tfs = true;
  std::string image_format = "bgr8";
  std::string output_path;
  std::string calib_file;
  std::string cam0_topic;
  std::string cam1_topic;
  std::string imu0_topic;

  node_config_t(const ros::NodeHandle &ros_nh) {
    ROS_PARAM(ros_nh, "mode", mode);
    ROS_PARAM(ros_nh, "publish_tfs", publish_tfs);
    ROS_PARAM(ros_nh, "image_format", image_format);
    ROS_PARAM(ros_nh, "output_path", output_path);
    ROS_PARAM(ros_nh, "calib_file", calib_file);
    ROS_PARAM(ros_nh, "cam0_topic", cam0_topic);
    ROS_PARAM(ros_nh, "cam1_topic", cam1_topic);
    ROS_PARAM(ros_nh, "imu0_topic", imu0_topic);
  }
};

bool tf_ok(const mat4_t &pose) {
  const auto r = tf_trans(pose);
  if (r.norm() > 100.0) {
    return false;
  }
  return true;
}

class calib_vi_node_t {
public:
  std::mutex mtx_;
  bool initialized_ = false;
  bool optimizing_ = false;
  bool loop_ = true;
  bool batch_opt_ = true;
  profiler_t profiler_;

  // ROS
  ros::NodeHandle ros_nh;
  node_config_t config_{ros_nh};
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
  // -- NBT publisher
  ros::Publisher nbt_pub_;

  // Calibrator
  yac::Estimator est_;
  float cam_rate_ = 30.0;

  // NBT
  bool nbt_precomputing_ = false;
  bool nbt_precomputed_ = false;
  ctrajs_t nbt_trajs_;
  matx_t calib_covar_;
  PinholeRadtan cam0_;
  PinholeRadtan cam1_;
  mat4_t T_WF_;
  mat4_t T_SC0_;
  mat4_t T_SC1_;
  std::vector<matx_t> nbt_calib_infos_;

  // Keyboard event handling
  struct termios term_config_;
  struct termios term_config_orig_;

  // Threads
  std::thread keyboard_thread_;
  std::thread nbt_precompute_thread_;

  bool finding_nbt_ = false;
  std::thread nbt_thread_;

  // Stats
  // -- Intial sensor-camera stddev
  bool sensor_camera_stdev_init_ = false;
  double init_r_SC0_stddev_ = 0.0;
  double init_r_SC1_stddev_ = 0.0;
  double init_alpha_SC0_stddev_ = 0.0;
  double init_alpha_SC1_stddev_ = 0.0;
  double init_cam0_proj_stddev_ = 0.0;
  double init_cam0_dist_stddev_ = 0.0;
  double init_cam1_proj_stddev_ = 0.0;
  double init_cam1_dist_stddev_ = 0.0;
  double r_SC0_baseline_ = 0.0;
  double r_SC1_baseline_ = 0.0;
  double alpha_SC0_baseline_ = 0.0;
  double alpha_SC1_baseline_ = 0.0;
  double cam0_proj_baseline_ = 0.0;
  double cam0_dist_baseline_ = 0.0;
  double cam1_proj_baseline_ = 0.0;
  double cam1_dist_baseline_ = 0.0;
  // -- Reprojection stddev
  bool reproj_stddev_init_ = false;
  double init_reproj_stddev_ = 0.0;
  double reproj_baseline_ = 0.0;

  calib_vi_node_t() :
      cam0_sub_{ros_nh, config_.cam0_topic, 30},
      cam1_sub_{ros_nh, config_.cam1_topic, 30},
      image_sync_(ImageSyncPolicy(10), cam0_sub_, cam1_sub_) {
    // clang-format off
    image_transport::ImageTransport it(ros_nh);
    imu0_sub_ = ros_nh.subscribe(config_.imu0_topic, 1000, &calib_vi_node_t::imu0_callback, this);
    image_sync_.registerCallback(&calib_vi_node_t::image_callback, this);
    uncertainty_pub_ = it.advertise("/yac_ros/uncertainty_map", 1);
    // clang-format on

    // -- Rviz marker publisher
    rviz_pub_ = ros_nh.advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);
    nbt_pub_ = ros_nh.advertise<nav_msgs::Path>("/yac_ros/nbt", 0);

    // Configure estimator
    est_.configure(config_.calib_file);
    // est_.save_estimates_ = false;
    // est_.save_costs_ = false;

    // Set terminal to be non-blocking
    tcgetattr(0, &term_config_);
    term_config_orig_ = term_config_;
    term_config_.c_lflag &= ~ICANON;
    term_config_.c_lflag &= ~ECHO;
    term_config_.c_lflag &= ~ISIG;
    term_config_.c_cc[VMIN] = 0;
    term_config_.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &term_config_);

    // Start keyboard event capture thread
    keyboard_thread_ = std::thread([&](){
      print_usage();

      while (loop_) {
        int n = getchar();
        if (n != EOF) {
          int key = n;

          switch (key) {
          case 'b':
            batch_opt_ = true;
            loop_ = false;
            break;
          case 'q': // q
          case 27:  // ESC
          case 3:   // Control-C
          case 4:   // Control-D
            batch_opt_ = false;
            loop_ = false;
            break;
          case 'p':
            // print_calib_variances();
            break;
          // case 'f':
          //   if (nbt_precomputed_ == false) {
          //     LOG_WARN("Still computing NBT!");
          //   } else if (config_.mode != "batch" && nbt_precomputed_ && finding_nbt_ == false) {
          //     if (nbt_thread_.joinable()) {
          //       nbt_thread_.join();
          //     }
          //     nbt_thread_ = std::thread(&calib_vi_node_t::find_nbt, this);
          //   } else {
          //     LOG_WARN("Already finding NBT!");
          //   }
          //   break;
          }
        }
      }
    });
  }

  ~calib_vi_node_t() {
    // Join threads
		if (keyboard_thread_.joinable()) {
			keyboard_thread_.join();
		}
		if (nbt_precompute_thread_.joinable()) {
			nbt_precompute_thread_.join();
		}
		if (nbt_thread_.joinable()) {
			nbt_thread_.join();
		}

    // Clean up (restore blocking keyboard handler)
    tcsetattr(0, TCSANOW, &term_config_orig_);
	}

	void print_usage() {
    printf("Press:\n");
    if (config_.mode != "batch") {
      printf("  'f' to find nbt\n");
    }
    printf("  'b' to perform batch calbration\n");
    printf("  'q', ESC, Ctrl-c or Ctrl-d to terminate\n");
	}

  void update_aprilgrid_model(const ros::Time &ts) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "T_WF";
    marker.header.stamp = ts;

    marker.ns = "yac_ros";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.mesh_resource = "package://yac_ros/models/aprilgrid/aprilgrid.dae";
    marker.mesh_use_embedded_materials = true;

    const auto target = est_.target_params_;
    const double tag_rows = target.tag_rows;
    const double tag_cols = target.tag_cols;
    const double tag_spacing = target.tag_spacing;
    const double tag_size = target.tag_size;
    const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
    const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
    const double calib_width = tag_cols * tag_size + spacing_x;
    const double calib_height = tag_rows * tag_size + spacing_y;

    marker.pose.position.x = calib_width / 2.0 - (tag_spacing * tag_size);
    marker.pose.position.y = calib_height / 2.0 - (tag_spacing * tag_size);
    marker.pose.position.z = 0;

    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.scale.x = calib_width;
    marker.scale.y = calib_height;
    marker.scale.z = 0.1;

    marker.color.a = 1.0f;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    rviz_pub_.publish(marker);
  }

  void publish_nbt(const ctraj_t &traj) {
    auto ts = ros::Time::now();
    auto frame_id = "map";

    nav_msgs::Path path_msg;
    path_msg.header.seq = 0;
    path_msg.header.stamp = ts;
    path_msg.header.frame_id = frame_id;

    for (size_t i = 0; i < traj.timestamps.size(); i++) {
      auto pose = tf(traj.orientations[i], traj.positions[i]);
      auto pose_stamped = msg_build(0, ts, frame_id, pose);
      path_msg.poses.push_back(pose_stamped);
    }

    nbt_pub_.publish(path_msg);
  }

  void publish_tfs(const ros::Time &ts) {
    // Fiducial pose
    const auto T_WF = est_.getFiducialPoseEstimate();
    if (tf_ok(T_WF) == false) {
      return;
    }
    // Sensor pose
    const auto T_WS_id = est_.pose0_id_;
    const auto T_WS = est_.getSensorPoseEstimate(T_WS_id);
    if (tf_ok(T_WS) == false) {
      return;
    }
    // Sensor cam0 pose
    auto T_SC0 = est_.getSensorCameraPoseEstimate(0);
    const auto T_WC0 = T_WS * T_SC0;
    if (tf_ok(T_WC0) == false) {
      return;
    }
    // Sensor cam1 pose
    auto T_SC1 = est_.getSensorCameraPoseEstimate(1);
    const auto T_WC1 = T_WS * T_SC1;
    if (tf_ok(T_WC1) == false) {
      return;
    }

    // Publish
    const auto T_WF_msg = build_msg(ts, "map", "T_WF", T_WF);
    const auto T_WS_msg = build_msg(ts, "map", "T_WS", T_WS);
    const auto T_WC0_msg = build_msg(ts, "map", "T_WC0", T_WC0);
    const auto T_WC1_msg = build_msg(ts, "map", "T_WC1", T_WC1);
    tf_br_.sendTransform(T_WF_msg);
    tf_br_.sendTransform(T_WS_msg);
    tf_br_.sendTransform(T_WC0_msg);
    tf_br_.sendTransform(T_WC1_msg);
    update_aprilgrid_model(ts);
  }

  void image_callback(const sensor_msgs::Image &cam0_msg,
                      const sensor_msgs::Image &cam1_msg) {
    // Double check image timestamps
    const auto cam0_ts = cam0_msg.header.stamp;
    const auto cam1_ts = cam1_msg.header.stamp;
    if (cam0_ts != cam1_ts) {
      ROS_FATAL("Images are not synchronized");
    }

    // Add image measurement
    const timestamp_t ts = cam0_ts.toNSec();
    std::vector<cv::Mat> cam_images{
      cv_bridge::toCvCopy(cam0_msg)->image,
      cv_bridge::toCvCopy(cam1_msg)->image
    };
		est_.addMeasurement(ts, cam_images);

		// Show calibration variances
    if (config_.mode != "batch") {
      if (est_.frame_index_ > 0 && est_.frame_index_ % 100 == 0){
        // print_calib_variances();
        // find_nbt();
      }
    }

    // // Precompute NBT
    // if (config_.mode != "batch") {
    //   if (nbt_precomputed_ == false && nbt_precomputing_ == false && est_.estimating_) {
    //     nbt_precompute_thread_ = std::thread(&calib_vi_node_t::precompute_nbt, this);
    //   }
    // }

    // Visualize current sensor pose
    if (est_.sensor_init_ && est_.fiducial_init_ && est_.grids_[0].detected) {
      if (config_.publish_tfs) {
        publish_tfs(cam0_ts);
        update_aprilgrid_model(cam0_ts);
      }
    }

    // Terminate if 2 minutes is up
    const size_t limit_nb_frames = cam_rate_ * (2.0 * 60.0);
    if (est_.frame_index_ >= limit_nb_frames) {
      LOG_INFO("Times Up! Finishing Calibration!");
      batch_opt_ = true;
      loop_ = false;
    }
  }

  void imu0_callback(const sensor_msgs::ImuConstPtr &msg) {
    std::lock_guard<std::mutex> guard(mtx_);

    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const auto ts = Time{msg->header.stamp.toSec()};
		est_.addMeasurement(ts, w_m, a_m);
		// est_.profiler_.print();
  }

  void optimize_batch() {
		LOG_INFO("Optimizing batch problem");

    est_.optimizeBatch(30, 4, true);
    est_.saveResults("/tmp/calib_results.yaml");
    loop_ = false;

		LOG_INFO("Finished optimizing batch problem!");
  }

  matx_t est_calib_info() {
    // Estimate current information
    std::vector<ImuError2Ptr> imu_errs;
    std::vector<SpeedAndBiasErrorPtr> sb_errs;
    std::vector<CalibReprojErrorPtr> reproj_errs;
    MarginalizationError *marg_err = est_.marg_error_.get();
    FiducialError *fiducial_prior = est_.T_WF_prior_.get();
    for (const auto &pair : est_.imu_errors_) imu_errs.push_back(pair.second);
    for (const auto &pair : est_.sb_errors_) sb_errs.push_back(pair.second);
    for (const auto &pair : est_.reproj_errors_) reproj_errs.push_back(pair.second);

    matx_t H;
    vecx_t b;
    form_hessian(imu_errs,
                 {},
                 reproj_errs,
                 marg_err,
                 nullptr,
                 fiducial_prior,
                 nullptr,
                 {},
                 {},
                 H,
                 &b);

    const size_t r = 28;
    const size_t m = H.rows() - r;
    schurs_complement(H, b, m, r);
    matx_t calib_info = H;

    return calib_info;
  }

	void precompute_nbt() {
		LOG_INFO("Precomputing NBT");
		nbt_precomputing_ = true;

		auto cam0 = est_.generateCamera(0);
		auto cam1 = est_.generateCamera(1);
		auto T_WF = est_.getFiducialPoseEstimate();
		auto T_SC0 = est_.getSensorCameraPoseEstimate(0);
		auto T_SC1 = est_.getSensorCameraPoseEstimate(1);
		// nbt_compute(est_.target_params_,
		// 						est_.imu_params_,
		// 						cam0, cam1, cam_rate_,
		// 						T_WF, T_SC0, T_SC1,
		// 						nbt_trajs_, nbt_calib_infos_);

		LOG_INFO("Computed %ld NBTs", nbt_trajs_.size());
		LOG_INFO("Computed %ld NBT infos", nbt_calib_infos_.size());

		for (size_t i = 0; i < nbt_calib_infos_.size(); i++) {
      const matx_t covar_nbt = nbt_calib_infos_[i];
      const std::string base_path = "/tmp/yac/nbt_infos";
      const std::string save_path = base_path + "/nbt_calib_info-" + std::to_string(i) + ".csv";
      mat2csv(save_path, covar_nbt);
		}

    nbt_precomputed_ = true;
		LOG_INFO("Finished precomputing NBT");
	}

	// void find_nbt() {
	//   finding_nbt_ = true;
  //
  //   // Find NBT
	// 	if (nbt_trajs_.size() && nbt_calib_infos_.size() && est_.estimating_) {
  //     std::lock_guard<std::mutex> guard(mtx_);
  //
  //     const matx_t calib_info = est_calib_info();
  //     const auto cam0 = est_.generateCamera(0);
  //     const auto cam1 = est_.generateCamera(1);
  //     const auto T_SC0 = est_.getSensorCameraPoseEstimate(0);
  //     const auto T_SC1 = est_.getSensorCameraPoseEstimate(1);
  //     int retval = nbt_find(est_.frame_index_,
  //                           cam0, cam1,
  //                           T_SC0, T_SC1,
  //                           calib_info,
  //                           nbt_trajs_,
  //                           nbt_calib_infos_);
	// 		if (retval != -1) {
	// 			publish_nbt(nbt_trajs_[retval]);
	// 		}
	// 	} else {
  //     LOG_ERROR("No NBT to evaluate! Did you forget to precompute them?");
  //     FATAL("IMPLEMENTATION ERROR!");
	// 	}
  //
	//   finding_nbt_ = false;
	// }

  void print_progress(const real_t percentage, const std::string &prefix) {
    const char *PBSTR =
        "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
    const int PBWIDTH = 60;

    const real_t p = std::min(percentage, 1.0);
    int val = (int) (p * 100);
    int lpad = (int) (p * PBWIDTH);
    int rpad = PBWIDTH - lpad;

    // Terminal color start
    if (p < 0.4) {
      printf("\033[31m"); // Red
    } else if (p < 0.8) {
      printf("\033[33m"); // Orange
    } else if (p <= 1.0) {
      printf("\033[32m"); // Green
    }

    printf("\r%s %3d%% [%.*s%*s]", prefix.c_str(), val, lpad, PBSTR, rpad, "");
    fflush(stdout);

    // Terminal color end
    printf("\033[0m\n");
  }

	// void print_calib_variances() {
	//   // Recover calibration covariance matrix
  //   const matx_t calib_info = est_calib_info();
  //   const matx_t calib_covar = calib_info.ldlt().solve(I(calib_info.rows()));
  //
  //   // Get calibration variances
  //   const auto calib_var = calib_covar.diagonal();
  //   const auto sc0_var = calib_var.segment<6>(0);
  //   const auto sc1_var = calib_var.segment<6>(6);
  //   const auto cam0_var = calib_var.segment<8>(12);
  //   const auto cam1_var = calib_var.segment<8>(20);
  //
  //   // Convert variances to standard deviation
  //   const auto r_SC0_stddev = sqrt(sc0_var.head<3>().norm());
  //   const auto r_SC1_stddev = sqrt(sc1_var.head<3>().norm());
  //   const auto alpha_SC0_stddev = rad2deg(sqrt(sc0_var.tail<3>().norm()));
  //   const auto alpha_SC1_stddev = rad2deg(sqrt(sc1_var.tail<3>().norm()));
  //   const auto cam0_stddev = cam0_var.array().sqrt();
  //   const auto cam1_stddev = cam1_var.array().sqrt();
  //
  //   // Target standard deviation
  //   const auto trans_target = 0.001;  // Translation target stddev [m]
  //   const auto rot_target = 0.05;     // Rotation target stddev [deg]
  //   const auto proj_target = 0.5;     // Projection target stddev [pixels]
  //   const auto dist_target = 0.001;   // Distortion target stddev [unit-less]
  //
  //   // Calculate progress towards target standard deviation
  //   if (sensor_camera_stdev_init_ == false) {
  //     init_r_SC0_stddev_ = r_SC0_stddev;
  //     init_r_SC1_stddev_ = r_SC1_stddev;
  //     init_alpha_SC0_stddev_ = alpha_SC0_stddev;
  //     init_alpha_SC1_stddev_ = alpha_SC1_stddev;
  //     init_cam0_proj_stddev_ = cam0_stddev.head(4).maxCoeff();
  //     init_cam0_dist_stddev_ = cam0_stddev.tail(4).maxCoeff();
  //     init_cam1_proj_stddev_ = cam1_stddev.head(4).maxCoeff();
  //     init_cam1_dist_stddev_ = cam1_stddev.tail(4).maxCoeff();
  //
  //     r_SC0_baseline_ = init_r_SC0_stddev_;
  //     r_SC1_baseline_ = init_r_SC1_stddev_;
  //
  //     alpha_SC0_baseline_ = init_alpha_SC0_stddev_;
  //     alpha_SC1_baseline_ = init_alpha_SC1_stddev_;
  //
  //     cam0_proj_baseline_ = init_cam0_proj_stddev_;
  //     cam0_dist_baseline_ = init_cam0_dist_stddev_;
  //
  //     cam1_proj_baseline_ = init_cam1_proj_stddev_;
  //     cam1_dist_baseline_ = init_cam1_dist_stddev_;
  //   }
  //
  //   // Clear terminal screen
  //   int retval = system("clear");
  //   UNUSED(retval);
  //
  //   // Print stats
  //   printf("Calibration Target Standard Deviations:\n");
  //   printf("---------------------------------------\n");
  //   printf("Translation stddev: %f [m]\n", trans_target);
  //   printf("Rotation stddev:    %f [deg]\n", rot_target);
  //   printf("Projection stddev:  %f [pixel]\n", proj_target);
  //   printf("Distortion stddev:  %f\n", dist_target);
  //   printf("\n");
  //
  //   printf("Current Calibration Parameter Standard Deviations:\n");
  //   printf("--------------------------------------------------\n");
  //   printf("imu-cam0 translation stddev: %.4f [m]\n", r_SC0_stddev);
  //   printf("imu-cam1 translation stddev: %.4f [m]\n", r_SC0_stddev);
  //   printf("imu-cam0 rotation stddev:    %.4f [deg]\n", alpha_SC0_stddev);
  //   printf("imu-cam1 rotation stddev:    %.4f [deg]\n", alpha_SC1_stddev);
  //   print_vector("cam0 proj stddev (fx fy cx cy) [pixels]   ", cam0_stddev.head<4>());
  //   print_vector("cam0 dist stddev (k1 k2 p1 p2) [unit-less]", cam0_stddev.tail<4>());
  //   print_vector("cam1 proj stddev (fx fy cx cy) [pixels]   ", cam1_stddev.head<4>());
  //   print_vector("cam1 dist stddev (k1 k2 p1 p2) [unit-less]", cam1_stddev.tail<4>());
  //   printf("\n");
  //
  //   printf("Progress Towards Meeting Calibration Target Standard Deviations:\n");
  //   printf("-------------------------------------------------------------------------------\n");
  //   print_progress(1.0 - ((r_SC0_stddev - trans_target) / r_SC0_baseline_),                       "imu-cam0 trans:");
  //   print_progress(1.0 - ((r_SC1_stddev - trans_target) / r_SC1_baseline_),                       "imu-cam1 trans:");
  //   print_progress(1.0 - ((alpha_SC0_stddev - rot_target) / alpha_SC0_baseline_),                 "imu-cam0 rot  :");
  //   print_progress(1.0 - ((alpha_SC1_stddev - rot_target) / alpha_SC1_baseline_),                 "imu-cam1 rot  :");
  //   print_progress(1.0 - ((cam0_stddev.head(4).maxCoeff() - proj_target) / cam0_proj_baseline_),  "cam0 proj     :");
  //   print_progress(1.0 - ((cam0_stddev.tail(4).maxCoeff() - dist_target) / cam0_dist_baseline_),  "cam0 dist     :");
  //   print_progress(1.0 - ((cam1_stddev.head(4).maxCoeff() - proj_target) / cam1_proj_baseline_),  "cam1 proj     :");
  //   print_progress(1.0 - ((cam1_stddev.tail(4).maxCoeff() - dist_target) / cam1_dist_baseline_),  "cam1 dist     :");
  //   printf("\n");
  //
  //   // Caculate max reprojection uncertainty
  //   cv::Mat uncertainty_map;
  //   const auto cam0 = est_.generateCamera(0);
  //   const auto cam1 = est_.generateCamera(1);
  //   const mat4_t T_SC0 = est_.getSensorCameraPoseEstimate(0);
  //   const mat4_t T_SC1 = est_.getSensorCameraPoseEstimate(1);
  //   double cam0_reproj_stddev = 0.0;
  //   double cam1_reproj_stddev = 0.0;
  //
  //   if (reproj_stddev_init_ == false) {
  //     cam0_reproj_stddev = eval_reproj_uncertainty(0, calib_covar, cam0, T_SC0);
  //     cam1_reproj_stddev = eval_reproj_uncertainty(1, calib_covar, cam1, T_SC1);
  //     init_reproj_stddev_ = std::max(cam0_reproj_stddev, cam1_reproj_stddev);
  //     reproj_baseline_ = init_reproj_stddev_;
  //     reproj_stddev_init_ = true;
  //
  //   } else {
  //     cam0_reproj_stddev = eval_reproj_uncertainty(0, calib_covar, cam0, T_SC0, &uncertainty_map);
  //     cam1_reproj_stddev = eval_reproj_uncertainty(1, calib_covar, cam1, T_SC1);
  //   }
  //   const double reproj_stddev_init = cam0_reproj_stddev + cam1_reproj_stddev;
  //   printf("Maximum Reprojection Uncertainty [pixels]: %f\n", reproj_stddev_init);
  //   printf("\n");
  //
  //   // Publish uncertainty map
  //   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", uncertainty_map).toImageMsg();
  //   uncertainty_pub_.publish(msg);
  //
  //   print_usage();
  //   printf("\n");
	// }

  void loop() {
    while (loop_) {
      ros::spinOnce();
    }

    if (batch_opt_) {
      optimize_batch();
    }
  }
};

}  // namespace yac

int main(int argc, char **argv) {
  // Setup ROS node
  ros::init(argc, argv, argv[0]);
	google::InitGoogleLogging("yac");
  yac::calib_vi_node_t node;

  // Loop ros node
  node.loop();

  return 0;
}
