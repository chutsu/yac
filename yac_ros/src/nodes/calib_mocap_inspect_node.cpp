#include "yac/yac.hpp"
#include "../ros_calib.hpp"
#include "../ros_utils.hpp"

struct inspector_node_t {
  ros::NodeHandle nh;
  image_transport::ImageTransport image_transport{nh};
  image_transport::Subscriber cam_sub;
  ros::Subscriber mocap_sub;

  yac::calib_target_t calib_target;
  std::unique_ptr<yac::aprilgrid_detector_t> detector;

  yac::timestamp_t mocap_ts;
  yac::mat4_t mocap_pose;              // T_WM
  yac::mat4_t mocap_camera_extrinsics; // T_MC
  yac::camera_params_t cam_params;

  inspector_node_t(const std::string calib_file,
                   const std::string camera_topic,
                   const std::string mocap_topic) {
    const auto &cam_cb_ptr = &inspector_node_t::image_callback;
    const auto &mocap_cb_ptr = &inspector_node_t::mocap_callback;
    cam_sub = image_transport.subscribe(camera_topic, 1, cam_cb_ptr, this);
    mocap_sub = nh.subscribe(mocap_topic, 100, mocap_cb_ptr, this);

    // Parse calibration target params
    if (calib_target.load(calib_file, "calib_target") != 0) {
      FATAL("Failed to parse calib file [%s]!", calib_file.c_str());
    }

    // Setup aprilgrid detector
    detector =
        std::make_unique<yac::aprilgrid_detector_t>(calib_target.tag_rows,
                                                    calib_target.tag_cols,
                                                    calib_target.tag_size,
                                                    calib_target.tag_spacing);

    // Load camera and mocap-camera extrinsics
    std::vector<int> cam_res;
    std::string proj_model;
    std::string dist_model;
    yac::vecx_t proj_params;
    yac::vecx_t dist_params;

    yac::config_t config{calib_file};
    parse(config, "cam0.resolution", cam_res);
    parse(config, "cam0.proj_model", proj_model);
    parse(config, "cam0.dist_model", dist_model);
    parse(config, "cam0.proj_params", proj_params);
    parse(config, "cam0.dist_params", dist_params);
    parse(config, "T_mocap_camera", mocap_camera_extrinsics);

    cam_params = yac::camera_params_t{0,
                                      cam_res.data(),
                                      proj_model,
                                      dist_model,
                                      proj_params,
                                      dist_params};

    while (1) {
      ros::spinOnce();
    }
  }

  /** Image callback */
  void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    const yac::timestamp_t ts = msg->header.stamp.toNSec();
    const cv::Mat cam_img = yac::msg_convert(msg);
    const auto grid = detector->detect(ts, cam_img);

    cv::Mat viz;
    if (grid.detected) {
      viz = grid.draw(cam_img);
    } else {
      viz = cam_img;
    }

    cv::imshow("camera", viz);
    cv::waitKey(1);
  }

  /** Mocap callback */
  void mocap_callback(const geometry_msgs::TransformStampedConstPtr &msg) {
    const auto ts = ros::Time(msg->header.stamp).toNSec();
    const auto rx = msg->transform.translation.x;
    const auto ry = msg->transform.translation.y;
    const auto rz = msg->transform.translation.z;
    const auto qx = msg->transform.rotation.x;
    const auto qy = msg->transform.rotation.y;
    const auto qz = msg->transform.rotation.z;
    const auto qw = msg->transform.rotation.w;
    const yac::vec3_t pos{rx, ry, rz};
    const yac::quat_t rot{qw, qx, qy, qz};

    mocap_ts = ts;
    mocap_pose = yac::tf(rot, pos);
  }
};

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = yac::ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Run ROS node
  const ros::NodeHandle nh;
  std::string calib_file;
  std::string camera_topic;
  std::string mocap_topic;
  ROS_PARAM(nh, node_name + "/calib_file", calib_file);
  ROS_PARAM(nh, node_name + "/camera_topic", camera_topic);
  ROS_PARAM(nh, node_name + "/mocap_topic", mocap_topic);
  inspector_node_t node{calib_file, camera_topic, mocap_topic};

  return 0;
}
