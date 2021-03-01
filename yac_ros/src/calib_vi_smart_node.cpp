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
#include "calib_vi/cv/PinholeCamera.hpp"
#include "calib_vi/cv/RadialTangentialDistortion.hpp"

#include "ros.hpp"
#include "ros_calib.hpp"

namespace yac {

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
  // -- NBT publisher
  ros::Publisher nbt_pub_;

  // Calibrator
  yac::Estimator est_;

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

  calib_vi_node_t(ros_config_t &config_, calib_data_t &calib_data_)
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
    imu0_sub_ = config.ros_nh->subscribe(config_.imu0_topic, 1000, &calib_vi_node_t::imu0_callback, this);
    image_sync_.registerCallback(&calib_vi_node_t::image_callback, this);
    uncertainty_pub_ = it.advertise("/yac_ros/uncertainty_map", 1);
    // clang-format on
    // -- Rviz marker publisher
    rviz_pub_ = config.ros_nh->advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);
    nbt_pub_ = config.ros_nh->advertise<nav_msgs::Path>("/yac_ros/nbt", 0);

    // Configure estimator
    est_.configure(calib_data);
    loop();
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
    const auto T_WS_id = est_.sensor_pose_param_ids_.back();
    const auto T_WS = est_.getSensorPoseEstimate(T_WS_id);
    if (tf_ok(T_WS) == false) {
      return;
    }
    // Sensor cam0 pose
    auto T_BC0 = est_.getCameraExtrinsicsEstimate(0);
    auto T_BS = est_.getImuExtrinsicsEstimate();
		auto T_SC0 = T_BS.inverse() * T_BC0;
    const auto T_WC0 = T_WS * T_SC0;
    if (tf_ok(T_WC0) == false) {
      return;
    }
    // Sensor cam1 pose
    auto T_BC1 = est_.getCameraExtrinsicsEstimate(1);
		auto T_SC1 = T_BS.inverse() * T_BC1;
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

    // Visualize current sensor pose
    if (est_.initialized_) {
      if (config.publish_tfs) {
        publish_tfs(cam0_ts);
        update_aprilgrid_model(cam0_ts);
      }
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
  }

  // void optimize_batch() {
	// 	LOG_INFO("Optimizing batch problem");
  //
  //   // est_.optimizeBatch(30, 4, true);
  //   est_.saveResults("/tmp/calib_results.yaml");
  //   loop_ = false;
  //
	// 	LOG_INFO("Finished optimizing batch problem!");
  // }

	void precompute_nbt() {
		LOG_INFO("Precomputing NBT");
		nbt_precomputing_ = true;

		auto cam0 = est_.getCamera(0);
		auto cam1 = est_.getCamera(1);
		auto T_WF = est_.getFiducialPoseEstimate();
		auto T_BS = est_.getImuExtrinsicsEstimate();
		auto T_BC0 = est_.getCameraExtrinsicsEstimate(0);
		auto T_BC1 = est_.getCameraExtrinsicsEstimate(1);
		auto T_SC0 = T_BS.inverse() * T_BC0;
		auto T_SC1 = T_BS.inverse() * T_BC1;
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
  //     const auto cam0 = est_.getCamera(0);
  //     const auto cam1 = est_.getCamera(1);
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

  void event_handler(int key) {
    if (key == EOF) {
      return;
    }

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

  void loop() {
    // Terminal capture thread
    keyboard_thread_ = std::thread([&](){
      print_usage();

      while (loop_) {
        const int key = getchar();
        event_handler(key);
      }
    });

    // ROS loop
    while (loop_) {
      ros::spinOnce();
    }

    // if (batch_opt_) {
    //   optimize_batch();
    // }
  }
};

}  // namespace yac

int main(int argc, char **argv) {
  // Load config and data
  yac::ros_config_t config(argc, argv);

  // Calibrate Stereo
  // {
  //   yac::calib_data_t calib_data{config.calib_file};
  //
  //   const auto proj_model = calib_data.cam_params[0].proj_model;
  //   const auto dist_model = calib_data.cam_params[0].dist_model;
  //   if (proj_model == "pinhole" && dist_model == "radtan4") {
  //     yac::calib_stereo_nbv_t<yac::pinhole_radtan4_t> calib{config, calib_data};
  //   } else if (proj_model == "pinhole" && dist_model == "equi4") {
  //     yac::calib_stereo_nbv_t<yac::pinhole_equi4_t> calib{config, calib_data};
  //   } else {
  //     FATAL("Unsupported projection-distorion type [%s-%s]!",
  //           proj_model.c_str(), dist_model.c_str());
  //   }
  // }

  // Calibrate VI
  {
    yac::calib_data_t calib_data{"/tmp/calib_results.yaml"};
    yac::calib_vi_node_t calib{config, calib_data};
    calib.loop();
  }

  return 0;
}
