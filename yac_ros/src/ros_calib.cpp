#include "ros_calib.hpp"

namespace yac {

void draw_hcentered_text(const std::string &text,
                         const float text_scale,
                         const int text_thickness,
                         const int text_ypos,
                         cv::Mat &image) {
  // Create overlay image
  const int img_w = image.cols;
  // const int img_h = image.rows;
  cv::Mat overlay = image.clone();

  // Text properties
  const int text_font = cv::FONT_HERSHEY_PLAIN;
  const cv::Scalar text_color{0, 255, 0};
  int baseline = 0;
  auto text_size =
      cv::getTextSize(text, text_font, text_scale, text_thickness, &baseline);
  int text_x = (img_w - text_size.width) / 2.0;
  // int text_y = (img_h + text_size.height) / 2.0;
  int text_y = text_ypos;
  const cv::Point text_pos{text_x, text_y}; // Bottom left of text string

  // Overlay properties
  const int pad = 20;
  const int hpad = pad / 2;
  const double alpha = 0.5;
  const int overlay_x1 = text_x - hpad;
  const int overlay_y1 = text_y - (text_size.height / 2.0) - hpad;
  const int overlay_x2 = text_x + text_size.width + hpad;
  const int overlay_y2 = text_y + (text_size.height / 2.0);
  const cv::Point2f p0(overlay_x1, overlay_y1);
  const cv::Point2f p1(overlay_x2, overlay_y2);
  const cv::Scalar overlay_color{0, 0, 0};
  cv::rectangle(overlay, p0, p1, overlay_color, -1);

  // Draw overlay and text
  cv::addWeighted(overlay, alpha, image, 1 - alpha, 0, image);
  cv::putText(image,
              text,
              text_pos,
              text_font,
              text_scale,
              text_color,
              text_thickness,
              cv::LINE_AA);
}

void draw_camera_index(const int cam_idx, cv::Mat &image) {
  // Text properties
  const std::string text = "cam" + std::to_string(cam_idx);
  const int font = cv::FONT_HERSHEY_PLAIN;
  const float scale = 1.0;
  const int thickness = 1;
  const cv::Scalar color{0, 255, 0};
  int baseline = 0;
  auto size = cv::getTextSize(text, font, scale, thickness, &baseline);
  const cv::Point pos{10, 30}; // Bottom left of text string
  cv::putText(image, text, pos, font, scale, color, thickness, cv::LINE_AA);
}

void draw_nbv_reproj_error(const real_t nbv_reproj_error, cv::Mat &image) {
  // -- Create NBV Reproj Error str (1 decimal places)
  std::ostringstream out;
  out.precision(1);
  out << std::fixed << nbv_reproj_error;
  out.str();
  // -- Change text color based on reprojection error
  const std::string text = "NBV Reproj Error: " + out.str() + " [px]";
  cv::Scalar text_color;
  if (nbv_reproj_error > 20) {
    text_color = cv::Scalar(0, 0, 255);
  } else {
    text_color = cv::Scalar(0, 255, 0);
  }
  // -- Draw text
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
              cv::LINE_AA);
}

void draw_status_text(const std::string &text, cv::Mat &image) {
  // Create overlay image
  const int img_w = image.cols;
  const int img_h = image.rows;
  cv::Mat overlay = image.clone();

  // Text properties
  const int text_font = cv::FONT_HERSHEY_PLAIN;
  const float text_scale = 4.0;
  const int text_thickness = 4;
  const cv::Scalar text_color{0, 255, 0};
  int baseline = 0;
  auto text_size =
      cv::getTextSize(text, text_font, text_scale, text_thickness, &baseline);
  int text_x = (img_w - text_size.width) / 2.0;
  int text_y = (img_h + text_size.height) / 2.0;
  const cv::Point text_pos{text_x, text_y}; // Bottom left of text string

  // Overlay properties
  const int pad = 20;
  const double alpha = 0.5;
  const int overlay_x1 = ((img_w - text_size.width) / 2.0) - pad / 2.0;
  const int overlay_y1 = ((img_h - text_size.height) / 2.0) - pad / 2.0;
  const int overlay_x2 = overlay_x1 + text_size.width + pad / 2.0;
  const int overlay_y2 = overlay_y1 + text_size.height + pad / 2.0;
  const cv::Point2f p0(overlay_x1, overlay_y1);
  const cv::Point2f p1(overlay_x2, overlay_y2);
  const cv::Scalar overlay_color{0, 0, 0};
  cv::rectangle(overlay, p0, p1, overlay_color, -1);

  // Draw overlay and text
  cv::addWeighted(overlay, alpha, image, 1 - alpha, 0, image);
  cv::putText(image,
              text,
              text_pos,
              text_font,
              text_scale,
              text_color,
              text_thickness,
              cv::LINE_AA);
}

void draw_detected(const aprilgrid_t &grid, cv::Mat &image) {
  // Visualize detected
  std::string text = "AprilGrid: ";
  cv::Scalar text_color;
  if (grid.detected) {
    text += "detected!";
    text_color = cv::Scalar(0, 255, 0);
  } else {
    text += "not detected!";
    text_color = cv::Scalar(0, 0, 255);
  }
  const cv::Point text_pos{10, 30};
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
              cv::LINE_AA);

  // Visualize detected corners
  const auto corner_color = cv::Scalar(0, 255, 0);
  for (const vec2_t &kp : grid.keypoints()) {
    cv::circle(image, cv::Point(kp(0), kp(1)), 1.0, corner_color, 2, 8);
  }
}

bool tf_ok(const mat4_t &pose) {
  const auto r = tf_trans(pose);
  if (r.norm() > 100.0) {
    return false;
  }
  return true;
}

void update_aprilgrid_model(const ros::Time &ts,
                            const calib_target_t &target,
                            ros::Publisher &rviz_pub) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "T_WF";
  marker.header.stamp = ts;

  marker.ns = "yac_ros";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.mesh_resource = "package://yac_ros/models/aprilgrid/aprilgrid.dae";
  marker.mesh_use_embedded_materials = true;

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

  rviz_pub.publish(marker);
}

void publish_nbt(const ctraj_t &traj, ros::Publisher &pub) {
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

  pub.publish(path_msg);
}

void publish_fiducial_tf(const ros::Time &ts,
                         const calib_target_t &target,
                         const mat4_t &T_WF,
                         tf2_ros::TransformBroadcaster &tf_br,
                         ros::Publisher rviz_pub) {
  if (tf_ok(T_WF) == false) {
    return;
  }

  const auto msg = build_msg(ts, "map", "T_WF", T_WF);
  tf_br.sendTransform(msg);
  update_aprilgrid_model(ts, target, rviz_pub);
}

void publish_tf(const ros::Time &ts,
                const std::string &pose_name,
                const mat4_t &pose,
                tf2_ros::TransformBroadcaster &tf_br) {
  if (tf_ok(pose) == false) {
    return;
  }

  const auto msg = build_msg(ts, "map", pose_name, pose);
  tf_br.sendTransform(msg);
}

} // namespace yac
