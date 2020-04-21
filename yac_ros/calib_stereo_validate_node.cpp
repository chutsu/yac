#include <yac/yac.hpp>
#include "yac_ros/ros.hpp"

namespace yac {

struct calib_config_t {
  vec2_t resolution{0.0, 0.0};
  std::string camera_model;
  std::string distortion_model;
  vec4_t intrinsics;
  vec4_t distortion;

  mat3_t cam_K;
  vecx_t cam_D;
  pinhole_radtan4_t camera_geometry;
};

calib_config_t parse_calib_file(const std::string &calib_file) {
  // Parse config
  config_t config{calib_file};
  calib_config_t calib_config;
  parse(config, "cam0.resolution", calib_config.resolution);
  parse(config, "cam0.camera_model", calib_config.camera_model);
  parse(config, "cam0.distortion_model", calib_config.distortion_model);
  parse(config, "cam0.intrinsics", calib_config.intrinsics);
  parse(config, "cam0.distortion", calib_config.distortion);

  // Setup camera intrinsics and distortion
  const pinhole_t camera_model{calib_config.intrinsics};
  const radtan4_t distortion_model{calib_config.distortion};
  calib_config.cam_K = pinhole_K(calib_config.intrinsics.data());
  calib_config.cam_D = calib_config.distortion;
  calib_config.camera_geometry =
      pinhole_radtan4_t{camera_model, distortion_model};

  return calib_config;
}

struct calib_validate_mono_node_t : ros_node_t {
  cv::VideoCapture capture_;
  std::string image_topic_;
  const aprilgrid_detector_t detector_;
  calib_config_t calib_config_;
  calib_target_t calib_target_;

  calib_validate_mono_node_t() : ros_node_t() {}

  int configure() {
    std::string calib_file;
    std::string target_file;

    // Setup ROS
    ros_node_t::configure();
    ROS_PARAM(ros_nh_, node_name_ + "/calib_file", calib_file);
    ROS_PARAM(ros_nh_, node_name_ + "/target_file", target_file);
    ROS_PARAM(ros_nh_, node_name_ + "/image_topic", image_topic_);
    // -- Configure ROS subscribers
    add_image_subscriber(image_topic_,
                         &calib_validate_mono_node_t::image_callback,
                         this);

    // Load calib config
    calib_config_ = parse_calib_file(calib_file);

    // Load calib target
    if (calib_target_load(calib_target_, target_file) != 0) {
      LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
      return -1;
    }

    return 0;
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    // Convert ROS message to cv::Mat
    const cv::Mat image = msg_convert(msg);

    // Detect calib target
    aprilgrid_t grid{0,
                     calib_target_.tag_rows,
                     calib_target_.tag_cols,
                     calib_target_.tag_size,
                     calib_target_.tag_spacing};
    aprilgrid_detect(grid,
                     detector_,
                     image,
                     calib_config_.cam_K,
                     calib_config_.cam_D);

    // Validate
    if (grid.detected) {
      const auto validation =
          validate_intrinsics(image,
                              grid.keypoints,
                              grid.points_CF,
                              calib_config_.camera_geometry);
      cv::imshow("Monocular Camera Validation", validation);
      cv::waitKey(1);
    }
  }
};

} // namespace yac

RUN_ROS_NODE(yac::calib_validate_mono_node_t);
