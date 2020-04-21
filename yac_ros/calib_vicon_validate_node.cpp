#include "proto/proto.hpp"
#include "proto/calib/calib_vicon_marker.hpp"
#include "proto/ros/ros.hpp"

using namespace proto;

struct node_t : ros_node_t {
  calib_target_t calib_target_;
  aprilgrid_detector_t detector_;
  vec4_t cam0_K_;
  vec4_t cam0_D_;
  mat4_t T_WF_;
  mat4_t T_MC_;
  mat4_t T_WM_;

  node_t() : ros_node_t() {}

  int configure() {
    ros_node_t::configure();

    // Get ROS params
    std::string rosbag_path;
    std::string calib_file;
    std::string cam0_topic;
    std::string body0_topic;
    std::string target0_topic;
    // ROS_PARAM(ros_nh, node_name + "/rosbag", rosbag_path);
    ROS_PARAM(ros_nh_, node_name_ + "/calib_file", calib_file);
    ROS_PARAM(ros_nh_, node_name_ + "/cam0_topic", cam0_topic);
    ROS_PARAM(ros_nh_, node_name_ + "/body0_topic", body0_topic);

    // Parse calib target
    if (calib_target_load(calib_target_, calib_file, "calib_target") != 0) {
      FATAL("Failed to parse calib file [%s]!", calib_file.c_str());
    }

    // Parse camera, T_WF, T_MC parameters
    config_t config{calib_file};
    parse(config, "cam0.intrinsics", cam0_K_);
    parse(config, "cam0.distortion", cam0_D_);
    parse(config, "T_WF", T_WF_);
    parse(config, "T_MC", T_MC_);

    // Print loaded parameters
    std::cout << "Calib Parameters:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Pinhole: " << cam0_K_.transpose() << std::endl;
    std::cout << "Radtan: " << cam0_D_.transpose() << std::endl;
    print_matrix("T_MC", T_MC_);

    // Configure subscribers and loop callbacks
    add_image_subscriber(cam0_topic, &node_t::image_callback, this);
    add_subscriber(body0_topic, &node_t::body_callback, this);

    return 0;
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    // Parse image message
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    auto image = image_ptr->image.clone();
    const auto image_ts = msg->header.stamp;
    const auto img_w = image.cols;
    const auto img_h = image.rows;

    // Predict where aprilgrid points should be
    const mat4_t T_WC = T_WM_ * T_MC_;
    const mat4_t T_CW = T_WC.inverse();

    // Detect AprilGrid
    aprilgrid_t grid;
    grid.tag_rows = calib_target_.tag_rows;
    grid.tag_cols = calib_target_.tag_cols;
    grid.tag_size = calib_target_.tag_size;
    grid.tag_spacing = calib_target_.tag_spacing;
    // const mat3_t cam0_K = pinhole_K(cam0_K_);
    // aprilgrid_detect(grid, detector_, image, cam0_K, cam0_D_);
    // if (grid.detected == false) {
    //   std::cout << "AprilGrid not detected!" << std::endl;
    //   return;
    // }
    // const mat4_t T_WF = T_WC * grid.T_CF;

    mat3_t K = pinhole_K(cam0_K_);
    vec3s_t object_points;
    vec2s_t image_points;
    aprilgrid_object_points(grid, object_points);
    for (const auto &p_F : object_points) {
      const vec3_t p_C = (T_CW * T_WF_ * p_F.homogeneous()).head(3);

      {
        double fx = K(0, 0);
        double fy = K(1, 1);
        double cx = K(0, 2);
        double cy = K(1, 2);
        double x = fx * (p_C(0) / p_C(2)) + cx;
        double y = fy * (p_C(1) / p_C(2)) + cy;
        const bool x_ok = (x > 0 && x < img_w);
        const bool y_ok = (y > 0 && y < img_h);
        if (!x_ok && !y_ok) {
          continue;
        }
      }

      vec_t<8> cam_params;
      cam_params << cam0_K_, cam0_D_;
      vec2_t img_pt;
      if (pinhole_radtan4_project(cam_params, p_C, img_pt) != 0) {
        continue;
      }

      const bool x_ok = (img_pt(0) > 0 && img_pt(0) < img_w);
      const bool y_ok = (img_pt(1) > 0 && img_pt(1) < img_h);
      if (x_ok && y_ok) {
        image_points.push_back(img_pt);
      }
    }

    // Draw on image
    for (const auto &img_pt : image_points) {
      cv::Point2f p(img_pt(0), img_pt(1));
      cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);
    }

    // Display image
    cv::imshow("Image", image);
    cv::waitKey(1);
  }

  // void body_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  //   const quat_t q = msg_convert(msg->pose.orientation);
  //   const vec3_t r = msg_convert(msg->pose.position);
  //   T_WM_ = tf(q, r);
  // }

  void
  body_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    const quat_t q = msg_convert(msg->pose.pose.orientation);
    const vec3_t r = msg_convert(msg->pose.pose.position);
    T_WM_ = tf(q, r);
  }
};

RUN_ROS_NODE(node_t);
