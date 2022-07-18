#include <signal.h>
#include "yac/yac.hpp"
#include "../ros_utils.hpp"

using namespace yac;
std::string test_out_path = "/tmp/mocap_test";

void clear_test_output() {
  std::string cmd = "rm -rf " + test_out_path;
  if (system(cmd.c_str()) != 0) {
    FATAL("Failed to execute system('%s')!", cmd.c_str());
  }
}

void signal_handler(int sig) {
  UNUSED(sig);
  clear_test_output();
}

std::string basename(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

void process_rosbag(const std::string &rosbag_path,
                    const std::string &cam0_topic,
                    const std::string &body0_topic,
                    const std::string &target0_topic) {
  // Check whether ros topics are in bag
  std::vector<std::string> target_topics;
  target_topics.push_back(cam0_topic);
  target_topics.push_back(body0_topic);
  target_topics.push_back(target0_topic);
  if (check_ros_topics(rosbag_path, target_topics) == false) {
    FATAL("Missing necessary ros topic!");
  }

  // Check output dir
  const std::string out_path = replace(rosbag_path, ".bag", "");
  if (dir_exists(out_path) == false) {
    if (dir_create(out_path) != 0) {
      FATAL("Failed to create dir [%s]", out_path.c_str());
    }
  }

  // Prepare data files
  const auto cam0_output_path = out_path + "/cam0";
  const auto body0_output_path = out_path + "/body0";
  const auto target0_output_path = out_path + "/target0";
  auto cam0_csv = camera_init_output_file(cam0_output_path);
  auto body0_csv = pose_init_output_file(body0_output_path);
  auto target0_csv = pose_init_output_file(target0_output_path);

  // Process ROS bag
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  rosbag::View bag_view(bag);
  size_t msg_idx = 0;

  for (const auto &msg : bag_view) {
    // Print progress
    print_progress((double)msg_idx / bag_view.size());

    // Process camera data
    if (msg.getTopic() == cam0_topic) {
      image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);
    }

    // Process body data
    if (msg.getTopic() == body0_topic) {
      tf_message_handler(msg, body0_csv);
    }

    // Process target data
    if (msg.getTopic() == target0_topic) {
      tf_message_handler(msg, target0_csv);
    }

    // Update
    msg_idx++;
  }

  // Clean up rosbag
  print_progress(1.0);
  bag.close();
}

// void detect_aprilgrids(const calib_target_t &calib_target,
//                        const std::string &cam0_path,
//                        const std::string &grid_path,
//                        const std::vector<std::string> &image_paths) {
//   aprilgrid_detector_t detector(calib_target.tag_rows,
//                                 calib_target.tag_cols,
//                                 calib_target.tag_size,
//                                 calib_target.tag_spacing);
//
//   for (size_t i = 0; i < image_paths.size(); i++) {
//     // -- Create output file path
//     auto output_file = basename(image_paths[i]);
//     const timestamp_t ts = std::stoull(output_file);
//     output_file = remove_ext(output_file);
//     output_file += ".csv";
//     const auto save_path = paths_join(grid_path, output_file);
//
//     // -- Setup AprilGrid
//     const int tag_rows = calib_target.tag_rows;
//     const int tag_cols = calib_target.tag_cols;
//     const double tag_size = calib_target.tag_size;
//     const double tag_spacing = calib_target.tag_spacing;
//     aprilgrid_t grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};
//
//     // -- Skip if already preprocessed
//     if (file_exists(save_path) && aprilgrid_load(grid, save_path) == 0) {
//       continue;
//     } else {
//       // Reset AprilGrid
//       grid = aprilgrid_t{ts, tag_rows, tag_cols, tag_size, tag_spacing};
//     }
//
//     // -- Detect
//     const auto image_path = paths_join(cam0_path, image_paths[i]);
//     const cv::Mat image = cv::imread(image_path);
//     aprilgrid_detect(detector, image, grid);
//     grid.timestamp = ts;
//
//     // -- Save AprilGrid
//     if (aprilgrid_save(grid, save_path) != 0) {
//       FATAL("Failed to save aprilgrid to [%s]!", save_path.c_str());
//     }
//   }
// }

// double loop_test_dataset(const std::string test_path,
//                          const calib_target_t &calib_target,
//                          const dataset_t &ds,
//                          bool imshow,
//                          long long ts_offset = 0) {
//   const auto cam0_path = test_path + "/cam0/data";
//   const auto grids_path = test_path + "/grid0/cam0/data";
//   const auto body0_csv_path = test_path + "/body0/data.csv";
//
//   // Detect Aprilgrids in test set
//   std::vector<std::string> image_paths;
//   if (list_dir(cam0_path, image_paths) != 0) {
//     FATAL("Failed to list dir [%s]!", cam0_path.c_str());
//   }
//   sort(image_paths.begin(), image_paths.end());
//   detect_aprilgrids(calib_target, cam0_path, grids_path, image_paths);
//   const aprilgrids_t aprilgrids = load_aprilgrids(grids_path);
//
//   // Vicon marker pose
//   timestamps_t body_timestamps;
//   mat4s_t body_poses;
//   load_body_poses(body0_csv_path, body_timestamps, body_poses);
//
//   // Synchronize grids and body poses
//   aprilgrids_t grids_sync;
//   mat4s_t body_poses_sync;
//   lerp_body_poses(aprilgrids,
//                   body_timestamps,
//                   body_poses,
//                   grids_sync,
//                   body_poses_sync,
//                   ts_offset);
//
//   // // Optimized parameters
//   // vec_t<8> cam_params;
//   // cam_params << ds.cam_model.fx(),
//   //               ds.cam_model.fy(),
//   //               ds.cam_model.cx(),
//   //               ds.cam_model.cy(),
//   //               ds.cam_model.distortion.k1(),
//   //               ds.cam_model.distortion.k2(),
//   //               ds.cam_model.distortion.p1(),
//   //               ds.cam_model.distortion.p2();
//   // const mat4_t T_MC = ds.T_MC;
//   // const mat4_t T_WF = ds.T_WF;
//   //
//   // // Loop over test dataset
//   // size_t pose_idx = 0;
//   // vec2s_t residuals;
//   //
//   // for (const auto &image_file : image_paths) {
//   //   // LOG_INFO("Image [%s]", image_file.c_str());
//   //
//   //   // Load image
//   //   auto image = cv::imread(paths_join(cam0_path, image_file));
//   //   const auto img_w = image.cols;
//   //   const auto img_h = image.rows;
//   //
//   //   // Predict where aprilgrid points should be
//   //   const mat4_t T_WM_ = body_poses_sync[pose_idx];
//   //   const mat4_t T_WC = T_WM_ * T_MC;
//   //   const mat4_t T_CW = T_WC.inverse();
//   //
//   //   // Setup grid
//   //   const auto grid = grids_sync[pose_idx];
//   //
//   //   for (const auto tag_id : grid.ids) {
//   //     // Get keypoints
//   //     vec2s_t keypoints;
//   //     if (aprilgrid_get(grid, tag_id, keypoints) != 0) {
//   //       FATAL("Failed to get AprilGrid keypoints!");
//   //     }
//   //
//   //     // Get object points
//   //     vec3s_t object_points;
//   //     if (aprilgrid_object_points(grid, tag_id, object_points) != 0) {
//   //       FATAL("Failed to calculate AprilGrid object points!");
//   //     }
//   //
//   //     // Calculate reprojection error
//   //     for (size_t i = 0; i < 4; i++) {
//   //       const vec3_t p_F = object_points[i];
//   //       const vec3_t p_C = (T_CW * T_WF * p_F.homogeneous()).head(3);
//   //
//   //       vec2_t z_hat;
//   //       if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
//   //         continue;
//   //       }
//   //       residuals.emplace_back(keypoints[i] - z_hat);
//   //     }
//   //   }
//   //
//   //   // Project object point in fiducial frame to image plane
//   //   vec3s_t object_points;
//   //   vec2s_t image_points;
//   //   aprilgrid_object_points(grid, object_points);
//   //   for (const auto &p_F : object_points) {
//   //     const vec3_t p_C = (T_CW * T_WF * p_F.homogeneous()).head(3);
//   //
//   //     // {
//   //     //   double fx = K(0, 0);
//   //     //   double fy = K(1, 1);
//   //     //   double cx = K(0, 2);
//   //     //   double cy = K(1, 2);
//   //     //   double x = fx * (p_C(0) / p_C(2)) + cx;
//   //     //   double y = fy * (p_C(1) / p_C(2)) + cy;
//   //     //   const bool x_ok = (x > 0 && x < img_w);
//   //     //   const bool y_ok = (y > 0 && y < img_h);
//   //     //   if (!x_ok && !y_ok) {
//   //     //     continue;
//   //     //   }
//   //     // }
//   //
//   //     vec2_t img_pt;
//   //     if (pinhole_radtan4_project(cam_params, p_C, img_pt) != 0) {
//   //       continue;
//   //     }
//   //
//   //     const bool x_ok = (img_pt(0) > 0 && img_pt(0) < img_w);
//   //     const bool y_ok = (img_pt(1) > 0 && img_pt(1) < img_h);
//   //     if (x_ok && y_ok) {
//   //       image_points.push_back(img_pt);
//   //     }
//   //   }
//   //
//   //   // Draw on image
//   //   if (imshow) {
//   //     for (const auto &img_pt : image_points) {
//   //       cv::Point2f p(img_pt(0), img_pt(1));
//   //       cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);
//   //     }
//   //     cv::imshow("Image", image);
//   //     cv::waitKey(0);
//   //   }
//   //
//   //   pose_idx++;
//   // }
//   //
//   // // Calculate RMSE reprojection error
//   // double err_sum = 0.0;
//   // for (auto &residual : residuals) {
//   //   const double err = residual.norm();
//   //   const double err_sq = err * err;
//   //   err_sum += err_sq;
//   // }
//   // const double err_mean = err_sum / (double) residuals.size();
//   // const double rmse = sqrt(err_mean);
//   // std::cout << "TS OFFSET: " << ts_offset * 1e-9 << "s\t";
//   // std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;
//
//   // return rmse;
//   return 0.0;
// }

int main(int argc, char *argv[]) {
  // Setup ROS Node
  signal(SIGINT, signal_handler);
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string config_file;
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);

  // Parse calibration target params
  calib_target_t calib_target;
  if (calib_target.load(config_file, "calib_target") != 0) {
    FATAL("Failed to parse calib file [%s]!", config_file.c_str());
  }

  // Parse config file
  std::string rosbag_path;
  std::string test_bag_path;
  std::string cam0_topic;
  std::string body0_topic;
  std::string target0_topic;

  config_t config{config_file};
  parse(config, "ros.rosbag", rosbag_path);
  parse(config, "ros.cam0_topic", cam0_topic);
  parse(config, "ros.body0_topic", body0_topic);
  parse(config, "ros.target0_topic", target0_topic);

  // Process rosbag
  process_rosbag(rosbag_path, cam0_topic, body0_topic, target0_topic);

  // Calibrate mocap marker to camera extrinsics
  const auto data_path = replace(rosbag_path, ".bag", "");
  const auto results_path = data_path + "/calib_mocap-results.yaml";
  calib_mocap_t calib{config_file, data_path};
  // calib.solve();
  calib.solve_nbv();
  calib.save_results(results_path);

  // // Process test ROS bag
  // process_rosbag(test_bag_path,
  //                test_out_path,
  //                cam0_topic,
  //                body0_topic,
  //                target0_topic);
  // loop_test_dataset(test_out_path, calib_target, ds, true, 0.0);
  // clear_test_output();

  return 0;
}
