#include <signal.h>
#include "yac.hpp"
#include "ros.hpp"
#include "calib_vicon_marker.hpp"

using namespace yac;
std::string test_out_path = "/tmp/vicon_test";

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
                    const std::string &out_path,
                    const std::string &cam0_topic,
                    const std::string &body0_topic,
                    const std::string &target0_topic) {
  // Check whether ros topics are in bag
  std::vector<std::string> target_topics;
  target_topics.push_back(cam0_topic);
  target_topics.push_back(body0_topic);
  target_topics.push_back(target0_topic);
  if (check_ros_topics(rosbag_path, target_topics) == false) {
    FATAL("Failed to create dir [%s]", out_path.c_str());
  }

  // Check output dir
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
    print_progress((double) msg_idx / bag_view.size());
    msg_idx++;

    // Process camera data
    if (msg.getTopic() == cam0_topic) {
      image_message_handler(msg, cam0_output_path + "/data/", cam0_csv);
    }

    // Process body data
    if (msg.getTopic() == body0_topic) {
      pose_message_handler(msg, body0_output_path + "/data/", body0_csv);
    }

    // Process target data
    if (msg.getTopic() == target0_topic) {
      pose_message_handler(msg, target0_output_path + "/data/", target0_csv);
    }
  }

  // Clean up rosbag
  print_progress(1.0);
  bag.close();
}

static aprilgrids_t load_aprilgrids(const std::string &dir_path) {
  std::vector<std::string> csv_files;
  if (list_dir(dir_path, csv_files) != 0) {
    FATAL("Failed to list dir [%s]!", dir_path.c_str());
  }
  sort(csv_files.begin(), csv_files.end());

  aprilgrids_t grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path + "/" + grid_csv;
    aprilgrid_t grid;
    if (aprilgrid_load(grid, csv_path) != 0) {
      FATAL("Failed to load AprilGrid [%s]!", grid_csv.c_str());
    }

    if (grid.detected) {
      grids.push_back(grid);
    }
  }

  return grids;
}

static void load_body_poses(const std::string &fpath,
                            timestamps_t &timestamps,
                            mat4s_t &poses) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";              // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
  str_format += "%lf,%lf,%lf";      // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &qw, &qx, &qy, &qz, &px, &py, &pz);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Record
    timestamps.push_back(ts);
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    poses.push_back(tf(q, r));
  }
}

static mat4_t load_fiducial_pose(const std::string &fpath) {
  mat4_t T_WF;

  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";              // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
  str_format += "%lf,%lf,%lf";      // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &qw, &qx, &qy, &qz, &px, &py, &pz);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Just need 1 pose
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    return tf(q, r);
  }

  return I(4);
}

mat4_t lerp_pose(const timestamp_t &t0,
                 const mat4_t &pose0,
                 const timestamp_t &t1,
                 const mat4_t &pose1,
                 const timestamp_t &t_lerp) {
  // Calculate alpha
  const double numerator = (t_lerp - t0) * 1e-9;
  const double denominator = (t1 - t0) * 1e-9;
  const double alpha = numerator / denominator;

  // Decompose start pose
  const vec3_t trans0 = tf_trans(pose0);
  const quat_t quat0{tf_rot(pose0)};

  // Decompose end pose
  const vec3_t trans1 = tf_trans(pose1);
  const quat_t quat1{tf_rot(pose1)};

  // Interpolate translation and rotation
  const auto trans_interp = lerp(trans0, trans1, alpha);
  const auto quat_interp = quat1.slerp(alpha, quat0);

  return tf(quat_interp, trans_interp);
}

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses,
                     timestamp_t ts_offset = 0) {
  // Make sure AprilGrids are between body poses else we can't lerp poses
  timestamps_t grid_timestamps;
  for (const auto &grid : grids) {
    if (grid.timestamp > body_timestamps.front() &&
        grid.timestamp < body_timestamps.back()) {
      lerped_grids.push_back(grid);
      grid_timestamps.push_back(grid.timestamp);
    }
  }

  // Lerp body poses using AprilGrid timestamps
  assert(body_poses.size() == body_timestamps.size());
  assert(body_timestamps.front() < grid_timestamps.front());
  timestamp_t t0 = 0;
  mat4_t pose0 = I(4);
  timestamp_t t1 = 0;
  mat4_t pose1 = I(4);

  size_t grid_idx = 0;
  for (size_t i = 0; i < body_timestamps.size(); i++) {
    // Make sure we're not going out of bounds
    if (grid_idx > (grid_timestamps.size() - 1)) {
      break;
    }

    // Get time now and desired lerp time
    const auto t_now = body_timestamps[i] + ts_offset;
    const auto t_lerp = grid_timestamps[grid_idx];

    // Update t0
    if (t_now < t_lerp) {
      t0 = t_now;
      pose0 = body_poses[i];
    }

    // Update t1
    if (t_now > t_lerp) {
      // Lerp
      t1 = t_now;
      pose1 = body_poses[i];
      const auto pose = lerp_pose(t0, pose0, t1, pose1, t_lerp);
      lerped_poses.push_back(pose);
      grid_idx++;

      // Reset
      t0 = t_now;
      pose0 = body_poses[i];
      t1 = 0;
      pose1 = I(4);
    }
  }
}

struct dataset_t {
  aprilgrids_t grids;
  pinhole_t<radtan4_t> cam_model;
  mat4s_t T_WM;
  mat4_t T_MC;
  mat4_t T_WF;
};

dataset_t process_dataset(const std::string &data_path,
                          const std::string &calib_file,
                          const calib_target_t &calib_target) {
  LOG_INFO("Processing dataset");
  const auto grid0_path = data_path + "/grid0/cam0/data";
  const auto body0_csv_path = data_path + "/body0/data.csv";
  const auto target0_csv_path = data_path + "/target0/data.csv";

  // Load camera calibration parameters
  LOG_INFO("-- Loading camera calibration parameters");
  vec2_t resolution;
  vec4_t intrinsics;
  vec4_t distortion;
  config_t calib{calib_file};
  parse(calib, "cam0.resolution", resolution);
  parse(calib, "cam0.intrinsics", intrinsics);
  parse(calib, "cam0.distortion", distortion);

  // // Redetect AprilGrid
  // LOG_INFO("-- Re-estimating the AprilGrids data");
  // {
  //   // Remove previously estimated aprilgrid
  //   const auto grid_path = data_path + "/grid0";
  //   const std::string cmd = "rm -rf " + data_path + "/grid0";
  //   if (system(cmd.c_str()) == -1) {
  //     FATAL("Failed to delete [%s]!", grid_path.c_str());
  //   }
  //
  //   // Prepare aprilgrid data directory
  //   const auto grid_data_path = data_path + "/grid0/cam0/data";
  //   if (dir_exists(grid_data_path) == false) {
  //     dir_create(grid_data_path);
  //   }
  //
  //   // Preprocess calibration data
  //   const auto cam_data_path = data_path + "/cam0/data";
  //   int img_w = resolution(0);
  //   int img_h = resolution(1);
  //   pinhole_t<radtan4_t> camera{img_w, img_h, intrinsics, distortion};
  //   int retval = preprocess_camera_data(calib_target,
  //                                       cam_data_path,
  //                                       pinhole_K(intrinsics),
  //                                       distortion,
  //                                       grid_data_path,
  //                                       true);
  //   if (retval != 0) {
  //     FATAL("Failed to preprocess calibration data!");
  //   }
  // }

  // Load dataset
  LOG_INFO("-- Loading dataset");
  dataset_t ds;
  // -- April Grid
  std::cout << "---- Loading AprilGrids" << std::endl;
  aprilgrids_t aprilgrids = load_aprilgrids(grid0_path);
  // -- Camera intrinsics and distortion
  int img_w = resolution(0);
  int img_h = resolution(1);
  ds.cam_model = pinhole_t<radtan4_t>{img_w, img_h, intrinsics, distortion};
  // -- Vicon marker pose
  std::cout << "---- Loading body poses" << std::endl;
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_body_poses(body0_csv_path, body_timestamps, body_poses);
  // -- Synchronize aprilgrids and body poses
  std::cout << "---- Synchronizing ApilGrids" << std::endl;
  lerp_body_poses(aprilgrids, body_timestamps, body_poses, ds.grids, ds.T_WM);
  // ds.grids, ds.T_WM, 0.05e9);
  // -- Vicon Marker to Camera transform
  const vec3_t euler{-90.0, 0.0, -90.0};
  // const vec3_t euler{-180.0, 0.0, -90.0};
  const mat3_t C = euler321(deg2rad(euler));
  ds.T_MC = tf(C, zeros(3, 1));
  // -- Fiducial target pose
  std::cout << "---- Loading fiducial pose" << std::endl;
  ds.T_WF = load_fiducial_pose(target0_csv_path);
  // ds.T_WF = ds.T_WM[0] * ds.T_MC * ds.grids[0].T_CF;
  // print_matrix("T_WM", ds.T_WM[0]);
  // print_matrix("T_MC", ds.T_MC);
  // print_matrix("T_CF", ds.grids[0].T_CF);
  // print_matrix("T_WF", ds.T_WF);

  // Show dataset stats
  std::cout << std::endl;
  std::cout << "Vicon Calibration dataset: " << std::endl;
  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "nb grids: " << ds.grids.size() << std::endl;
  std::cout << "nb poses: " << ds.T_WM.size() << std::endl;
  std::cout << std::endl;
  std::cout << "Pinhole:\n" << ds.cam_model << std::endl;
  std::cout << "Radtan:\n" << ds.cam_model.distortion << std::endl;
  print_matrix("T_MC", ds.T_MC);
  print_matrix("T_WF", ds.T_WF);

  return ds;
}

void detect_aprilgrids(const calib_target_t &calib_target,
                       const std::string &cam0_path,
                       const std::string &grid_path,
                       const std::vector<std::string> &image_paths) {
  aprilgrid_detector_t detector;
  for (size_t i = 0; i < image_paths.size(); i++) {
    // -- Create output file path
    auto output_file = basename(image_paths[i]);
    const timestamp_t ts = std::stoull(output_file);
    output_file = remove_ext(output_file);
    output_file += ".csv";
    const auto save_path = paths_combine(grid_path, output_file);

    // -- Setup AprilGrid
    const int tag_rows = calib_target.tag_rows;
    const int tag_cols = calib_target.tag_cols;
    const double tag_size = calib_target.tag_size;
    const double tag_spacing = calib_target.tag_spacing;
    aprilgrid_t grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

    // -- Skip if already preprocessed
    if (file_exists(save_path) && aprilgrid_load(grid, save_path) == 0) {
      continue;
    } else {
      // Reset AprilGrid
      grid = aprilgrid_t{ts, tag_rows, tag_cols, tag_size, tag_spacing};
    }

    // -- Detect
    const auto image_path = paths_combine(cam0_path, image_paths[i]);
    const cv::Mat image = cv::imread(image_path);
    aprilgrid_detect(grid, detector, image);
    grid.timestamp = ts;

    // -- Save AprilGrid
    if (aprilgrid_save(grid, save_path) != 0) {
      FATAL("Failed to save aprilgrid to [%s]!", save_path.c_str());
    }
  }
}

double loop_test_dataset(const std::string test_path,
                         const calib_target_t &calib_target,
                         const dataset_t &ds,
                         bool imshow,
                         long long ts_offset = 0) {
  const auto cam0_path = test_path + "/cam0/data";
  const auto grids_path = test_path + "/grid0/cam0/data";
  const auto body0_csv_path = test_path + "/body0/data.csv";

  // Detect Aprilgrids in test set
  std::vector<std::string> image_paths;
  if (list_dir(cam0_path, image_paths) != 0) {
    FATAL("Failed to list dir [%s]!", cam0_path.c_str());
  }
  sort(image_paths.begin(), image_paths.end());
  detect_aprilgrids(calib_target, cam0_path, grids_path, image_paths);
  const aprilgrids_t aprilgrids = load_aprilgrids(grids_path);

  // Vicon marker pose
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_body_poses(body0_csv_path, body_timestamps, body_poses);

  // Synchronize grids and body poses
  aprilgrids_t grids_sync;
  mat4s_t body_poses_sync;
  lerp_body_poses(aprilgrids,
                  body_timestamps,
                  body_poses,
                  grids_sync,
                  body_poses_sync,
                  ts_offset);

  // Optimized parameters
  vec_t<8> cam_params;
  cam_params << ds.cam_model.fx(),
                ds.cam_model.fy(),
                ds.cam_model.cx(),
                ds.cam_model.cy(),
                ds.cam_model.distortion.k1(),
                ds.cam_model.distortion.k2(),
                ds.cam_model.distortion.p1(),
                ds.cam_model.distortion.p2();
  const mat4_t T_MC = ds.T_MC;
  const mat4_t T_WF = ds.T_WF;

  // Loop over test dataset
  size_t pose_idx = 0;
  vec2s_t residuals;

  for (const auto &image_file : image_paths) {
    // LOG_INFO("Image [%s]", image_file.c_str());

    // Load image
    auto image = cv::imread(paths_combine(cam0_path, image_file));
    const auto img_w = image.cols;
    const auto img_h = image.rows;

    // Predict where aprilgrid points should be
    const mat4_t T_WM_ = body_poses_sync[pose_idx];
    const mat4_t T_WC = T_WM_ * T_MC;
    const mat4_t T_CW = T_WC.inverse();

    // Setup grid
    const auto grid = grids_sync[pose_idx];

    for (const auto tag_id : grid.ids) {
      // Get keypoints
      vec2s_t keypoints;
      if (aprilgrid_get(grid, tag_id, keypoints) != 0) {
        FATAL("Failed to get AprilGrid keypoints!");
      }

      // Get object points
      vec3s_t object_points;
      if (aprilgrid_object_points(grid, tag_id, object_points) != 0) {
        FATAL("Failed to calculate AprilGrid object points!");
      }

      // Calculate reprojection error
      for (size_t i = 0; i < 4; i++) {
        const vec3_t p_F = object_points[i];
        const vec3_t p_C = (T_CW * T_WF * p_F.homogeneous()).head(3);

        vec2_t z_hat;
        if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
          continue;
        }
        residuals.emplace_back(keypoints[i] - z_hat);
      }
    }

    // Project object point in fiducial frame to image plane
    vec3s_t object_points;
    vec2s_t image_points;
    aprilgrid_object_points(grid, object_points);
    for (const auto &p_F : object_points) {
      const vec3_t p_C = (T_CW * T_WF * p_F.homogeneous()).head(3);

      // {
      //   double fx = K(0, 0);
      //   double fy = K(1, 1);
      //   double cx = K(0, 2);
      //   double cy = K(1, 2);
      //   double x = fx * (p_C(0) / p_C(2)) + cx;
      //   double y = fy * (p_C(1) / p_C(2)) + cy;
      //   const bool x_ok = (x > 0 && x < img_w);
      //   const bool y_ok = (y > 0 && y < img_h);
      //   if (!x_ok && !y_ok) {
      //     continue;
      //   }
      // }

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
    if (imshow) {
      for (const auto &img_pt : image_points) {
        cv::Point2f p(img_pt(0), img_pt(1));
        cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);
      }
      cv::imshow("Image", image);
      cv::waitKey(0);
    }

    pose_idx++;
  }

  // Calculate RMSE reprojection error
  double err_sum = 0.0;
  for (auto &residual : residuals) {
    const double err = residual.norm();
    const double err_sq = err * err;
    err_sum += err_sq;
  }
  const double err_mean = err_sum / (double) residuals.size();
  const double rmse = sqrt(err_mean);
  std::cout << "TS OFFSET: " << ts_offset * 1e-9 << "s\t";
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return rmse;
}

void show_results(const dataset_t &ds) {
  std::cout << std::endl;
  std::cout << "Calibration Results:" << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  // Reprojection Error
  mat4s_t T_CF;
  {
    const mat4_t T_CM = ds.T_MC.inverse();
    for (size_t i = 0; i < ds.grids.size(); i++) {
      const mat4_t T_MW = ds.T_WM[i].inverse();
      T_CF.emplace_back(T_CM * T_MW * ds.T_WF);
    }
  }
  calib_camera_stats<calib_mono_residual_t>(
    ds.grids,
    ds.cam_model.params.data(),
    ds.cam_model.distortion.params.data(),
    T_CF,
    ""
  );
  std::cout << std::endl;

  // Optimized Parameters
  std::cout << "Pinhole:\n" << ds.cam_model << std::endl;
  std::cout << "Radtan:\n" << ds.cam_model.distortion << std::endl;
  print_matrix("T_WF", ds.T_WF);
  print_matrix("T_WM", ds.T_WM[0]);
  print_matrix("T_MC", ds.T_MC);

  const auto r_MC = tf_trans(ds.T_MC);
  const auto q_MC = tf_quat(ds.T_MC);
  printf("r_MC: %f, %f, %f\n", r_MC(0), r_MC(1), r_MC(2));
  printf("q_MC (x, y, z, w): %f, %f, %f, %f\n",
         q_MC.x(),
         q_MC.y(),
         q_MC.z(),
         q_MC.w());
}

void save_results(const std::string &output_path, const dataset_t &ds) {
  LOG_INFO("Saving results to [%s]!", output_path.c_str());
  const aprilgrid_t grid = ds.grids[0];
  const pinhole_t<radtan4_t> cam_model = ds.cam_model;
  const mat4_t T_WF = ds.T_WF;
  const mat4_t T_MC = ds.T_MC;

  // Save calibration results to yaml file
  {
    // Aprilgrid parameters
    FILE *fp = fopen(output_path.c_str(), "w");
    fprintf(fp, "calib_target:\n");
    fprintf(fp, "  target_type: \"aprilgrid\"\n");
    fprintf(fp, "  tag_rows: %d\n", grid.tag_rows);
    fprintf(fp, "  tag_cols: %d\n", grid.tag_cols);
    fprintf(fp, "  tag_size: %f\n", grid.tag_size);
    fprintf(fp, "  tag_spacing: %f\n", grid.tag_spacing);
    fprintf(fp, "\n");

    // Camera parameters
    fprintf(fp, "cam0:\n");
    fprintf(fp, "  camera_model: \"pinhole\"\n");
    fprintf(fp, "  distortion_model: \"radtan\"\n");
    fprintf(fp, "  intrinsics: ");
    fprintf(fp, "[");
    fprintf(fp, "%lf, ", cam_model.fx());
    fprintf(fp, "%lf, ", cam_model.fy());
    fprintf(fp, "%lf, ", cam_model.cx());
    fprintf(fp, "%lf", cam_model.cy());
    fprintf(fp, "]\n");
    fprintf(fp, "  distortion: ");
    fprintf(fp, "[");
    fprintf(fp, "%lf, ", cam_model.distortion.k1());
    fprintf(fp, "%lf, ", cam_model.distortion.k2());
    fprintf(fp, "%lf, ", cam_model.distortion.p1());
    fprintf(fp, "%lf", cam_model.distortion.p2());
    fprintf(fp, "]\n");
    fprintf(fp, "\n");

    // T_WF
    fprintf(fp, "T_WF:\n");
    fprintf(fp, "  rows: 4\n");
    fprintf(fp, "  cols: 4\n");
    fprintf(fp, "  data: [\n");
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(0, 0));
    fprintf(fp, "%lf, ", T_WF(0, 1));
    fprintf(fp, "%lf, ", T_WF(0, 2));
    fprintf(fp, "%lf,\n", T_WF(0, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(1, 0));
    fprintf(fp, "%lf, ", T_WF(1, 1));
    fprintf(fp, "%lf, ", T_WF(1, 2));
    fprintf(fp, "%lf,\n", T_WF(1, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(2, 0));
    fprintf(fp, "%lf, ", T_WF(2, 1));
    fprintf(fp, "%lf, ", T_WF(2, 2));
    fprintf(fp, "%lf,\n", T_WF(2, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_WF(3, 0));
    fprintf(fp, "%lf, ", T_WF(3, 1));
    fprintf(fp, "%lf, ", T_WF(3, 2));
    fprintf(fp, "%lf\n", T_WF(3, 3));
    fprintf(fp, "  ]\n");
    fprintf(fp, "\n");

    // T_MC
    fprintf(fp, "T_MC:\n");
    fprintf(fp, "  rows: 4\n");
    fprintf(fp, "  cols: 4\n");
    fprintf(fp, "  data: [\n");
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC(0, 0));
    fprintf(fp, "%lf, ", T_MC(0, 1));
    fprintf(fp, "%lf, ", T_MC(0, 2));
    fprintf(fp, "%lf,\n", T_MC(0, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC(1, 0));
    fprintf(fp, "%lf, ", T_MC(1, 1));
    fprintf(fp, "%lf, ", T_MC(1, 2));
    fprintf(fp, "%lf,\n", T_MC(1, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC(2, 0));
    fprintf(fp, "%lf, ", T_MC(2, 1));
    fprintf(fp, "%lf, ", T_MC(2, 2));
    fprintf(fp, "%lf,\n", T_MC(2, 3));
    fprintf(fp, "    ");
    fprintf(fp, "%lf, ", T_MC(3, 0));
    fprintf(fp, "%lf, ", T_MC(3, 1));
    fprintf(fp, "%lf, ", T_MC(3, 2));
    fprintf(fp, "%lf\n", T_MC(3, 3));
    fprintf(fp, "  ]\n");
    fprintf(fp, "\n");
    fclose(fp);
  }

  // Record poses
  {
    FILE *fp = fopen("/tmp/poses.csv", "w");
    for (const auto &pose : ds.T_WM) {
      const auto q = tf_quat(pose);
      const auto r = tf_trans(pose);
      fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
      fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_WF.csv", "w");
    const auto q = tf_quat(ds.T_WF);
    const auto r = tf_trans(ds.T_WF);
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_MC.csv", "w");
    const auto q = tf_quat(ds.T_MC);
    const auto r = tf_trans(ds.T_MC);
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_CM.csv", "w");
    const auto T_CM = ds.T_MC.inverse();
    const auto q = tf_quat(T_CM);
    const auto r = tf_trans(T_CM);
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  signal(SIGINT, signal_handler);
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string train_bag_path;
  std::string test_bag_path;
  std::string config_file;
  std::string cam0_topic;
  std::string body0_topic;
  std::string target0_topic;
  ROS_PARAM(ros_nh, node_name + "/train_bag", train_bag_path);
  ROS_PARAM(ros_nh, node_name + "/test_bag", test_bag_path);
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
  ROS_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(ros_nh, node_name + "/body0_topic", body0_topic);
  ROS_PARAM(ros_nh, node_name + "/target0_topic", target0_topic);

  // Parse calibration target params
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    FATAL("Failed to parse calib file [%s]!", config_file.c_str());
  }

  // Parse config file
  std::string data_path;
  std::string calib_results_path;
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", calib_results_path);

  // Calibrate camera intrinsics
  process_rosbag(train_bag_path,
                 data_path,
                 cam0_topic,
                 body0_topic,
                 target0_topic);
  if (calib_camera_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  // Calibrate vicon object to camera transform
  dataset_t ds = process_dataset(data_path, calib_results_path, calib_target);
  calib_vicon_marker_solve(ds.grids,
                           ds.cam_model,
                           ds.T_WM,
                           ds.T_MC,
                           ds.T_WF);
  show_results(ds);
  save_results(calib_results_path, ds);

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
