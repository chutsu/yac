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
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "yac.hpp"
#include "calib_nbv.hpp"

#include "../ros_utils.hpp"
#include "../ros_calib.hpp"
#include "../calib_vi/Calibrator.hpp"
#include "../calib_vi/Estimator.hpp"
#include "../calib_vi/cv/PinholeCamera.hpp"
#include "../calib_vi/cv/RadialTangentialDistortion.hpp"

namespace yac {

class calib_vi_node_t {
public:
  std::mutex mtx_;
  bool loop_ = true;

  // State
  enum CALIB_STATE {
    IDEL = -1,
    INITIALIZE = 0,
    CALIB_ORIGIN = 1,
    NBT = 2
  };
  int state_ = IDEL;
  size_t frame_idx_ = 0;

  // ROS
  ros_config_t &config_;
  calib_data_t &calib_data_;
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

  // Visualization
  int viz_width = 800;
  int viz_height = 600;

  // Calibrator
  calib_target_t target_;
  camera_params_t cam0_params_;
  aprilgrid_detector_t *detector_ = nullptr;
  Estimator est_;

  // NBV
  mat4_t calib_origin_;  // Calibration origin (X) w.r.t camera frame(C): T_XC
  aprilgrid_t *target_grid = nullptr;
  double nbv_reproj_error_threshold = 10.0;
  double nbv_hold_threshold = 1.0;
  double nbv_reproj_err = std::numeric_limits<double>::max();
  struct timespec nbv_hold_tic = (struct timespec){0, 0};

  // NBT
  bool nbt_precomputing_ = false;
  bool nbt_precomputed_ = false;
  ctrajs_t nbt_trajs_;
  std::vector<matx_t> nbt_calib_infos_;

  // Keyboard event handling
  struct termios term_config_;
  struct termios term_config_orig_;

  // Threads
  std::thread keyboard_thread_;
  bool finding_nbt_ = false;
  std::thread nbt_thread_;

  calib_vi_node_t(ros_config_t &config, calib_data_t &calib_data)
      : config_{config},
        calib_data_{calib_data},
        cam0_sub_{*config_.ros_nh, config.cam0_topic, 30},
        cam1_sub_{*config_.ros_nh, config.cam1_topic, 30},
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
    image_transport::ImageTransport it(*config_.ros_nh);
    imu0_sub_ = config_.ros_nh->subscribe(config_.imu0_topic, 1000, &calib_vi_node_t::imu0_callback, this);
    image_sync_.registerCallback(&calib_vi_node_t::image_callback, this);
    uncertainty_pub_ = it.advertise("/yac_ros/uncertainty_map", 1);
    // clang-format on
    // -- Rviz marker publisher
    rviz_pub_ = config_.ros_nh->advertise<visualization_msgs::Marker>("/yac_ros/rviz", 0);
    nbt_pub_ = config_.ros_nh->advertise<nav_msgs::Path>("/yac_ros/nbt", 0);

		// Setup detector
		target_ = calib_data_.target;
		cam0_params_ = calib_data_.cam_params[0];
    detector_ = new aprilgrid_detector_t(target_.tag_rows,
                                         target_.tag_cols,
                                         target_.tag_size,
                                         target_.tag_spacing);

    // Setup calibration origin
    calib_origin_ = calib_init_poses<pinhole_radtan4_t>(target_, cam0_params_)[0];
    if (target_grid != nullptr) {
      delete target_grid;
    }
    target_grid = nbv_target_grid<pinhole_radtan4_t>(target_, cam0_params_, calib_origin_);

    // Configure estimator
    est_.configure(calib_data_);
    loop();
  }

  ~calib_vi_node_t() {
    // Join threads
		if (keyboard_thread_.joinable()) {
			keyboard_thread_.join();
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

  void event_handler(int key) {
    if (key == EOF) {
      return;
    }

    switch (key) {
    case 'q': // q
    case 27:  // ESC
    case 3:   // Control-C
    case 4:   // Control-D
      loop_ = false;
      break;
    case 'r':
      LOG_INFO("Resetting Estimator!");
      state_ = IDEL;
      est_.resetProblem();
      break;
    case 'f':
      if (finding_nbt_ == false) {
        if (nbt_thread_.joinable()) {
          nbt_thread_.join();
        }
        nbt_thread_ = std::thread(&calib_vi_node_t::find_nbt, this);
      } else {
        LOG_WARN("Already finding NBT!");
      }
      break;
    }
  }

  void draw_nbv(const mat4_t &T_FC0, cv::Mat &image) {
    // Draw NBV
    nbv_draw<pinhole_radtan4_t>(calib_data_.target,
                                calib_data_.cam_params[0],
                                T_FC0,
                                image);

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

  void mode_idel(const aprilgrid_t &grid, cv::Mat &image) {
    auto text = "Go to calib origin!";
    draw_hcentered_text(text, 2.0, 1.0, 100, image);
    draw_nbv(calib_origin_, image);

    if (nbv_reached(grid)) {
      LOG_INFO("Initializing estimator!");
      state_ = INITIALIZE;
    }
  }

  void mode_init(const aprilgrid_t &grid, cv::Mat &image) {
    // nbv_draw<pinhole_radtan4_t>(calib_data_.target,
    //                             calib_data_.cam_params[0],
    //                             calib_origin_,
    //                             image);
    // draw_status_text("Turn your camera!", image);
  }

  void visualize(const aprilgrid_t &grid, const cv::Mat &image) {
    // Draw detected
    auto image_rgb = gray2rgb(image);
    draw_detected(grid, image_rgb);

    // State machine
    switch (state_) {
    case IDEL: mode_idel(grid, image_rgb); break;
    case INITIALIZE: mode_init(grid, image_rgb); break;
    }

    // Show
    cv::resize(image_rgb, image_rgb, cv::Size(viz_width, viz_height));
    cv::imshow("Visualization", image_rgb);
    int event = cv::waitKey(1);
    event_handler(event);

    // Rviz
    if (est_.initialized_ && config_.publish_tfs) {
			const auto ts = ros::Time{ts2sec(grid.timestamp)};
			const auto target = est_.target_params_;
			const auto T_WF = est_.getFiducialPoseEstimate();
			const auto T_WS = est_.getSensorPoseEstimate(-1);
			const auto T_BS = est_.getImuExtrinsicsEstimate();
			const auto T_BC0 = est_.getCameraExtrinsicsEstimate(0);
			const auto T_BC1 = est_.getCameraExtrinsicsEstimate(1);
			const auto T_WC0 = T_WS * T_BS.inverse() * T_BC0;
			const auto T_WC1 = T_WS * T_BS.inverse() * T_BC1;
			publish_fiducial_tf(ts, target, T_WF, tf_br_, rviz_pub_);
			publish_tf(ts, "T_WS", T_WS, tf_br_);
			publish_tf(ts, "T_WC0", T_WC0, tf_br_);
			publish_tf(ts, "T_WC1", T_WC1, tf_br_);
    }
  }

  void image_callback(const sensor_msgs::Image &cam0_msg,
                      const sensor_msgs::Image &cam1_msg) {
    frame_idx_++;

    // Pre-check
    // if (state_ == INITIALIZE && ((frame_idx_ % 5) != 0)) {
    //   return;
    // }

    // Double check image timestamps
    const auto cam0_ts = cam0_msg.header.stamp;
    const auto cam1_ts = cam1_msg.header.stamp;
    if (cam0_ts != cam1_ts) {
      ROS_FATAL("Images are not synchronized");
    }

    // Detect AprilGrids
    // -- Form images
    const timestamp_t ts = cam0_ts.toNSec();
    std::vector<cv::Mat> cam_images{
      cv_bridge::toCvCopy(cam0_msg)->image,
      cv_bridge::toCvCopy(cam1_msg)->image
    };
    // -- Detect
    std::vector<aprilgrid_t> grids;
    for (int i = 0; i < (int) est_.nb_cams(); i++) {
      auto grid = detector_->detect(ts, cam_images[i], true);
      grid.timestamp = ts;
      grids.push_back(grid);
    }
    if (grids[0].detected == false || grids[1].detected == false) {
      return;
    }
    // -- Add measurement
    if (state_ != IDEL) {
      est_.addMeasurement(0, grids[0]);
      est_.addMeasurement(1, grids[1]);
    }
    visualize(grids[0], cam_images[0]);
  }

  void imu0_callback(const sensor_msgs::ImuConstPtr &msg) {
    std::lock_guard<std::mutex> guard(mtx_);

    const auto gyr = msg->angular_velocity;
    const auto acc = msg->linear_acceleration;
    const vec3_t w_m{gyr.x, gyr.y, gyr.z};
    const vec3_t a_m{acc.x, acc.y, acc.z};
    const auto ts = Time{msg->header.stamp.toSec()};
    if (state_ != IDEL) {
      est_.addMeasurement(ts, w_m, a_m);
    }
  }

	void compute_nbt() {
    auto cam_rate = 30.0;
		auto cam0 = est_.getCameraParameters(0);
		auto cam1 = est_.getCameraParameters(1);
		auto T_WF = est_.getFiducialPoseEstimate();
		auto T_BS = est_.getImuExtrinsicsEstimate();
		auto T_BC0 = est_.getCameraExtrinsicsEstimate(0);
		auto T_BC1 = est_.getCameraExtrinsicsEstimate(1);

		nbt_compute<pinhole_radtan4_t>(est_.target_params_,
																	 est_.imu_params_,
																	 cam0, cam1, cam_rate,
																	 T_WF, T_BC0, T_BC1, T_BS,
																	 nbt_trajs_, nbt_calib_infos_);

		nbt_precomputing_ = false;
    nbt_precomputed_ = true;
	}

	void find_nbt() {
	  finding_nbt_ = true;

    // Find NBT
		// if (nbt_trajs_.size() && nbt_calib_infos_.size() && est_.estimating_) {
		if (est_.estimating_) {
			nbt_trajs_.clear();
			nbt_calib_infos_.clear();
			compute_nbt();

      // Obtain calibration covariance
      std::lock_guard<std::mutex> guard(mtx_);
      matx_t calib_covar;
      est_.recoverCalibCovariance(calib_covar);
			print_vector("calib_covar", calib_covar.diagonal());

      // mat2_t fiducial_covar;
      // est_.recoverFiducialCovariance(fiducial_covar);
			// print_vector("fiducial_covar", fiducial_covar.diagonal());

			auto cam0 = est_.getCamera(0);
			auto T_BC0 = est_.getCameraExtrinsicsEstimate(0);
			auto T_BS = est_.getImuExtrinsicsEstimate();
      int retval = nbt_find(cam0, T_BC0, T_BS,
                            calib_covar,
                            nbt_trajs_,
                            nbt_calib_infos_);
			if (retval != -1) {
        publish_nbt(nbt_trajs_[retval], nbt_pub_);
			}
		} else {
      LOG_ERROR("No NBT to evaluate! Did you forget to precompute them?");
      FATAL("IMPLEMENTATION ERROR!");
		}

	  finding_nbt_ = false;
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

  // Initialize body0-imu0
  // {
  //   yac::calib_data_t calib_data{"/tmp/calib-stereo.yaml"};
  //   yac::calib_vi_init_node_t calib{config, calib_data};
  // }

  // Calibrate VI
  {
    // yac::calib_data_t calib_data{"/tmp/calib-stereo.yaml"};
    yac::calib_data_t calib_data{"/tmp/calib_results.yaml"};
    yac::calib_vi_node_t calib{config, calib_data};
    calib.loop();
  }

  return 0;
}
