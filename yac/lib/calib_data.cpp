#include "calib_data.hpp"

namespace yac {

int calib_target_load(calib_target_t &ct,
                      const std::string &target_file,
                      const std::string &prefix) {
  config_t config{target_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  const auto parent = (prefix == "") ? "" : prefix + ".";
  parse(config, parent + "target_type", ct.target_type);
  parse(config, parent + "tag_rows", ct.tag_rows);
  parse(config, parent + "tag_cols", ct.tag_cols);
  parse(config, parent + "tag_size", ct.tag_size);
  parse(config, parent + "tag_spacing", ct.tag_spacing);

  return 0;
}

calib_data_t::calib_data_t() {
  // Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
}

calib_data_t::calib_data_t(const std::string &config_file)
  : config{config_file},
    nb_cams{config_nb_cameras(config_file)},
    nb_imus{config_nb_imus(config_file)} {
  // Problem
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);

  // Calibration target
  if (calib_target_load(target, config_file, "calib_target") != 0) {
    FATAL("Failed to load calib target [%s]!", config_file.c_str());
  }

  // Load Imu
  if (nb_imus > 1) {
    FATAL("YAC currently does not support more than 1 IMU!");
  } else if (nb_imus == 1) {
    imu_params = load_imu_params(config, 0, true);
    imu_exts[0] = extrinsics_t{param_counter++};
  }

  // Load cameras
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    cam_params[cam_idx] = load_cam_params(config, cam_idx, true);
    cam_exts[cam_idx] = load_cam_exts(config, cam_idx, true);
    vision_errors[cam_idx] = std::vector<double>();
  }
}

calib_data_t::~calib_data_t() {
  if (problem) {
    delete problem;
    problem = nullptr;
  }
  if (loss) {
    delete loss;
    loss = nullptr;
  }
}

void calib_data_t::add_calib_target(const calib_target_t &target_) {
  target = target_;
}

void calib_data_t::add_imu(const imu_params_t &params) {
  imu_params = params;
}

void calib_data_t::add_camera(const camera_params_t &params) {
  cam_params[params.cam_index] = params;
  nb_cams += 1;
  vision_errors[params.cam_index] = std::vector<double>();
}

void calib_data_t::add_camera_extrinsics(const int cam_idx, const extrinsics_t &params) {
  cam_exts[cam_idx] = params;
}

void calib_data_t::add_camera_extrinsics(const int cam_idx, const mat4_t &ext) {
  cam_exts[cam_idx] = extrinsics_t{param_counter++, ext};
}

void calib_data_t::add_imu_extrinsics(const int imu_idx, const extrinsics_t &params) {
  imu_exts[imu_idx] = params;
  nb_imus += 1;
}

void calib_data_t::add_imu_extrinsics(const int imu_idx, const mat4_t &ext) {
  imu_exts[imu_idx] = extrinsics_t{param_counter++, ext};
  nb_imus += 1;
}

void calib_data_t::add_grids(const int cam_idx, const aprilgrids_t &grids) {
  cam_grids[cam_idx] = grids;
}

void calib_data_t::add_pose(const timestamp_t &ts, const pose_t &pose) {
  poses[ts] = pose;
}

void calib_data_t::preprocess_data() {
  const int rows = target.tag_rows;
  const int cols = target.tag_cols;
  const double size = target.tag_size;
  const double spacing = target.tag_spacing;

  // Track aprilgrids for each camera and all timestamps
  std::set<timestamp_t> timestamps;
  std::map<int, std::map<timestamp_t, aprilgrid_t>> grid_tracker;
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    for (auto &grid : cam_grids[cam_idx]) {
      grid_tracker[cam_idx][grid.timestamp] = grid;
      timestamps.insert(grid.timestamp);
    }
  }

  // Preprocess vision data
  std::map<int, aprilgrids_t> prep_grids;
  for (const auto &ts : timestamps) {
    for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
      if (grid_tracker[cam_idx].count(ts) == 0) {
        prep_grids[cam_idx].emplace_back(ts, rows, cols, size, spacing);
      } else {
        prep_grids[cam_idx].push_back(grid_tracker[cam_idx][ts]);
      }
    }
  }

  cam_grids.clear();
  cam_grids = prep_grids;
}

void calib_data_t::check_data() {
  bool ok = true;

  auto nb_grids = cam_grids[0].size();
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    if (nb_grids != cam_grids[cam_idx].size()) {
      FATAL("grids[%d].size() != nb_grids!", cam_idx);
      ok = false;
    }
  }

  std::vector<timestamp_t> timestamps;
  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    for (auto &grid : cam_grids[cam_idx]) {
      timestamps.push_back(grid.timestamp);
    }
  }

  for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    for (size_t i = 0; i < cam_grids[cam_idx].size(); i++) {
      auto &grid = cam_grids[cam_idx][i];
      if (grid.timestamp != timestamps[i]) {
        LOG_ERROR("grid.timestamp != timestamps[i]!");
        ok = false;
        break;
      }

      if (grid.tag_rows != target.tag_rows) {
        LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
        LOG_ERROR("grid.tag_rows != target.tag_rows!");
        ok = false;
        break;
      } else if (grid.tag_cols != target.tag_cols) {
        LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
        LOG_ERROR("grid.tag_cols != target.tag_cols!");
        ok = false;
        break;
      } else if (grid.tag_size != target.tag_size) {
        LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
        LOG_ERROR("grid.tag_size != target.tag_size!");
        ok = false;
        break;
      } else if (grid.tag_spacing != target.tag_spacing) {
        LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
        LOG_ERROR("grid.tag_spacing != target.tag_spacing!");
        ok = false;
        break;
      }
    }
  }

  if (ok == false) {
    FATAL("Bad data!");
  }
}

int calib_data_t::config_nb_cameras(const std::string &config_file) {
  config_t config{config_file};

  const int max_cameras = 100;
  for (int i = 0; i < max_cameras; i++) {
    const std::string key = "cam" + std::to_string(i);
    if (yaml_has_key(config, key) == 0) {
      return i;
    }
  }

  return 0;
}

int calib_data_t::config_nb_imus(const std::string &config_file) {
  config_t config{config_file};

  const int max_imus = 100;
  for (int i = 0; i < max_imus; i++) {
    const std::string key = "imu" + std::to_string(i);
    if (yaml_has_key(config, key) == 0) {
      return i;
    }
  }

  return 0;
}

void calib_data_t::load_calib_target_params(const config_t &config,
                                            calib_target_t &params,
                                            bool verbose) {
  parse(config, "calib_target.target_type", params.target_type);
  parse(config, "calib_target.tag_rows", params.tag_rows);
  parse(config, "calib_target.tag_cols", params.tag_cols);
  parse(config, "calib_target.tag_size", params.tag_size);
  parse(config, "calib_target.tag_spacing", params.tag_spacing);

  if (verbose) {
    LOG_INFO("Calibration Target Parameters");
    LOG_INFO("----------------------------------------");
    LOG_INFO("target type: %s", params.target_type.c_str());
    LOG_INFO("tag rows: %d", params.tag_rows);
    LOG_INFO("tag cols: %d", params.tag_cols);
    LOG_INFO("tag size: %f", params.tag_size);
    LOG_INFO("tag spacing: %f", params.tag_spacing);
    LOG_INFO("");
  }
}

camera_params_t calib_data_t::load_cam_params(const config_t &config,
                                              const int cam_idx,
                                              bool verbose) {
  // Load camera calibration
  const std::string cam_str = "cam" + std::to_string(cam_idx);
  std::vector<int> cam_res;
  std::string proj_model;
  std::string dist_model;
  vec4_t proj_params = zeros(4, 1);
  vec4_t dist_params = zeros(4, 1);
  // -- Parse resolution, camera and distortion model
  parse(config, cam_str + ".resolution", cam_res);
  parse(config, cam_str + ".proj_model", proj_model);
  parse(config, cam_str + ".dist_model", dist_model);
  // -- Parse intrinsics
  if (yaml_has_key(config, cam_str + ".proj_params")) {
    parse(config, cam_str + ".proj_params", proj_params);
  }
  // -- Parse distortion
  if (yaml_has_key(config, cam_str + ".dist_params")) {
    parse(config, cam_str + ".dist_params", dist_params);
  }

  if (verbose) {
    LOG_INFO("Camera[%d] Parameters", cam_idx);
    LOG_INFO("----------------------------------------");
    LOG_INFO("proj_params: [%f, %f, %f, %f]",
              proj_params(0),
              proj_params(1),
              proj_params(2),
              proj_params(3));
    LOG_INFO("dist_params: [%f, %f, %f, %f]",
              dist_params(0),
              dist_params(1),
              dist_params(2),
              dist_params(3));
    LOG_INFO("");
  }

  return camera_params_t{param_counter++,
                          cam_idx, cam_res.data(),
                          proj_model, dist_model,
                          proj_params, dist_params};
}

imu_params_t calib_data_t::load_imu_params(const config_t &config,
                                           const int index,
                                           bool verbose) {
  imu_params_t imu_params;
  const std::string imu_key = "imu" + std::to_string(index);
  parse(config, imu_key + ".rate", imu_params.rate);
  parse(config, imu_key + ".a_max", imu_params.a_max);
  parse(config, imu_key + ".g_max", imu_params.g_max);
  parse(config, imu_key + ".sigma_g_c", imu_params.sigma_g_c);
  parse(config, imu_key + ".sigma_a_c", imu_params.sigma_a_c);
  parse(config, imu_key + ".sigma_gw_c", imu_params.sigma_gw_c);
  parse(config, imu_key + ".sigma_aw_c", imu_params.sigma_aw_c);
  parse(config, imu_key + ".g", imu_params.g);

  if (verbose) {
    LOG_INFO("IMU Parameters");
    LOG_INFO("----------------------------------------");
    LOG_INFO("rate: %f", imu_params.rate);
    LOG_INFO("a_max: %f", imu_params.a_max);
    LOG_INFO("g_max: %f", imu_params.g_max);
    LOG_INFO("sigma_g_c: %f", imu_params.sigma_g_c);
    LOG_INFO("sigma_a_c: %f", imu_params.sigma_a_c);
    LOG_INFO("sigma_gw_c: %f", imu_params.sigma_gw_c);
    LOG_INFO("sigma_aw_c: %f", imu_params.sigma_aw_c);
    LOG_INFO("sigma_ba: %f", imu_params.sigma_ba);
    LOG_INFO("sigma_bg: %f", imu_params.sigma_bg);
    LOG_INFO("g: %f", imu_params.g);
    LOG_INFO("");
  }

  return imu_params;
}

extrinsics_t calib_data_t::load_cam_exts(const config_t &config,
                                         const int cam_idx,
                                         bool verbose) {
  const std::string cam_key = "cam" + std::to_string(cam_idx);
  const std::string key = "T_body0_" + cam_key;
  mat4_t exts = I(4);
  if (yaml_has_key(config, key)) {
    parse(config, key, exts);
  }

  if (verbose) {
    LOG_INFO("Camera Extrinsics: T_BC%d", cam_idx);
    LOG_INFO("----------------------------------------");
    LOG_INFO("[%f, %f, %f, %f,", exts(0, 0), exts(0, 1), exts(0, 2), exts(0, 3));
    LOG_INFO(" %f, %f, %f, %f,", exts(1, 0), exts(1, 1), exts(1, 2), exts(1, 3));
    LOG_INFO(" %f, %f, %f, %f,", exts(2, 0), exts(2, 1), exts(2, 2), exts(2, 3));
    LOG_INFO(" %f, %f, %f, %f]", exts(3, 0), exts(3, 1), exts(3, 2), exts(3, 3));
    LOG_INFO("");
  }

  return extrinsics_t{param_counter++, exts};
}

static int get_camera_image_paths(const std::string &image_dir,
                                  std::vector<std::string> &image_paths) {
  // Check image dir
  if (dir_exists(image_dir) == false) {
    LOG_ERROR("Image dir [%s] does not exist!", image_dir.c_str());
    return -1;
  }

  // Get image paths
  if (list_dir(image_dir, image_paths) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", image_dir.c_str());
    return -1;
  }
  std::sort(image_paths.begin(), image_paths.end());

  return 0;
}

int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const std::string &output_dir,
                           const bool show_progress) {
  int retval = 0;

  // Get camera image paths
  std::vector<std::string> image_paths;
  if (get_camera_image_paths(image_dir, image_paths) != 0) {
    return -1;
  }

  // Detect AprilGrid
  aprilgrid_detector_t detector{target.tag_rows,
                                target.tag_cols,
                                target.tag_size,
                                target.tag_spacing};

  // Equalizer
  // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();

  #pragma omp parallel for
  for (size_t i = 0; i < image_paths.size(); i++) {
    // Print progress
    if (show_progress && i % 10 == 0) {
      printf(".");
      fflush(stdout);
    }

    // Create output file path
    auto output_file = parse_fname(image_paths[i]);
    const timestamp_t ts = std::stoull(output_file);
    output_file = remove_ext(output_file);
    output_file += ".csv";
    const auto save_path = paths_join(output_dir, output_file);

    // Skip if already preprocessed
    if (file_exists(save_path)) {
      continue;
    }

    // Read image
    const auto image_path = paths_join(image_dir, image_paths[i]);
    const cv::Mat frame = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);

    // cv::Mat image;
    // clahe->setClipLimit(2);
    // clahe->apply(frame, image);

    // cv::Mat image;
    // cv::GaussianBlur(frame, image, cv::Size(0, 0), 3);
    // cv::addWeighted(frame, 1.5, image, -0.5, 0, image);

    // cv::Mat image_compare;
    // cv::hconcat(frame, image, image_compare);
    // cv::imshow("Image", image_compare);
    // cv::waitKey(1);

    // -- Save AprilGrid
    const auto grid = detector.detect(ts, frame);
    if (grid.save(save_path) != 0) {
      retval = -1;
    }
  }

  // Print newline after print progress has finished
  if (show_progress) {
    printf("\n");
  }

  return retval;
}

int load_camera_calib_data(const std::string &data_dir,
                           aprilgrids_t &aprilgrids,
                           bool detected_only) {
  // Check image dir
  if (dir_exists(data_dir) == false) {
    LOG_ERROR("Image dir [%s] does not exist!", data_dir.c_str());
    return -1;
  }

  // Get detection data
  std::vector<std::string> data_paths;
  if (list_dir(data_dir, data_paths) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", data_dir.c_str());
    return -1;
  }
  std::sort(data_paths.begin(), data_paths.end());

  // Load AprilGrid data
  for (size_t i = 0; i < data_paths.size(); i++) {
    // Load
    const auto data_path = paths_join(data_dir, data_paths[i]);
    aprilgrid_t grid{data_path};

    // Make sure aprilgrid is actually detected
    if (grid.detected || detected_only == false) {
      aprilgrids.emplace_back(grid);
    }
  }

  return 0;
}

void extract_common_calib_data(aprilgrids_t &grids0, aprilgrids_t &grids1) {
  // Loop through both sets of calibration data and only keep apriltags that
  // are seen by both cameras
  size_t nb_detections = std::max(grids0.size(), grids1.size());
  size_t cam0_idx = 0;
  size_t cam1_idx = 0;

  aprilgrids_t final_grids0;
  aprilgrids_t final_grids1;
  for (size_t i = 0; i < nb_detections; i++) {
    // Get grid
    aprilgrid_t &grid0 = grids0[cam0_idx];
    aprilgrid_t &grid1 = grids1[cam1_idx];
    if (grid0.timestamp == grid1.timestamp) {
      cam0_idx++;
      cam1_idx++;
    } else if (grid0.timestamp > grid1.timestamp) {
      cam1_idx++;
      continue;
    } else if (grid0.timestamp < grid1.timestamp) {
      cam0_idx++;
      continue;
    }

    // Keep only common tags between grid0 and grid1
    grid0.intersect(grid1);
    assert(grid0.nb_detections == grid1.nb_detections);

    // Add to results
    final_grids0.emplace_back(grid0);
    final_grids1.emplace_back(grid1);

    // Check if there's more data to go though
    if (cam0_idx >= grids0.size() || cam1_idx >= grids1.size()) {
      break;
    }
  }

  grids0.clear();
  grids1.clear();
  grids0 = final_grids0;
  grids1 = final_grids1;
}

int load_stereo_calib_data(const std::string &cam0_data_dir,
                           const std::string &cam1_data_dir,
                           aprilgrids_t &cam0_grids,
                           aprilgrids_t &cam1_grids) {
  int retval = 0;

  // Load cam0 calibration data
  aprilgrids_t grids0;
  retval = load_camera_calib_data(cam0_data_dir, grids0);
  if (retval != 0) {
    return -1;
  }

  // Load cam1 calibration data
  aprilgrids_t grids1;
  retval = load_camera_calib_data(cam1_data_dir, grids1);
  if (retval != 0) {
    return -1;
  }

  // Loop through both sets of calibration data
  size_t nb_grids = std::max(grids0.size(), grids1.size());
  size_t cam0_idx = 0;
  size_t cam1_idx = 0;

  for (size_t i = 0; i < nb_grids; i++) {
    // Get grid
    aprilgrid_t &grid0 = grids0[cam0_idx];
    aprilgrid_t &grid1 = grids1[cam1_idx];
    if (grid0.timestamp == grid1.timestamp) {
      cam0_idx++;
      cam1_idx++;
    } else if (grid0.timestamp > grid1.timestamp) {
      cam1_idx++;
      continue;
    } else if (grid0.timestamp < grid1.timestamp) {
      cam0_idx++;
      continue;
    }

    // Keep only common tags between grid0 and grid1
    // grid0.intersect(grid1);
    // assert(grid0.nb_detections == grid1.nb_detections);

    // Add to results if detected anything
    if (grid0.detected && grid1.detected) {
      cam0_grids.emplace_back(grid0);
      cam1_grids.emplace_back(grid1);
    }

    // Check if there's more data to go though
    if (cam0_idx >= grids0.size() || cam1_idx >= grids1.size()) {
      break;
    }
  }

  return 0;
}

// int load_multicam_calib_data(const int nb_cams,
//                              const std::vector<std::string> &data_dirs,
//                              std::map<int, aprilgrids_t> &calib_data) {
//   // real_t check nb_cams is equal to data_dirs.size()
//   if (nb_cams != (int) data_dirs.size()) {
//     LOG_ERROR("nb_cams != data_dirs");
//     return -1;
//   }
//
//   // Load calibration data for each camera
//   std::map<int, aprilgrids_t> grids;
//   size_t max_grids = 0;
//   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//     aprilgrids_t data;
//     const int retval = load_camera_calib_data(data_dirs[cam_idx], data);
//     if (retval != 0) {
//       LOG_ERROR("Failed to load calib data [%s]!", data_dirs[cam_idx].c_str());
//       return -1;
//     }
//
//     grids[cam_idx] = data;
//     if (data.size() > max_grids) {
//       max_grids = data.size();
//     }
//   }
//
//   // Aggregate timestamps
//   std::map<timestamp_t, int> ts_count;
//   std::set<timestamp_t> timestamps;
//   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//     for (const auto &grid : grids[cam_idx]) {
//       ts_count[grid.timestamp]++;
//       timestamps.insert(grid.timestamp);
//     }
//   }
//
//   // Initialize grid indicies where key is cam_idx, value is grid_idx
//   std::map<int, size_t> grid_indicies;
//   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//     grid_indicies[cam_idx] = 0;
//   }
//
//   // Loop through timestamps
//   for (const auto &ts : timestamps) {
//     // If only a subset of cameras detected aprilgrids at this timestamp
//     // it means we need to update the grid index of those subset of cameras.
//     // We do this because we don't want missing data, we want **AprilGrid
//     // observed by all cameras at the same timestamp.**
//     if (ts_count[ts] != nb_cams) {
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         auto &grid_idx = grid_indicies[cam_idx];
//         if (grids[cam_idx][grid_idx].timestamp == ts) {
//           grid_idx++;
//         }
//       }
//
//       // Skip this timestamp with continue
//       continue;
//     }
//
//   try_again:
//     // Check if AprilGrids across cameras have same timestamp
//     std::vector<bool> ready(nb_cams, false);
//     for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//       auto &grid_idx = grid_indicies[cam_idx];
//       if (grids[cam_idx][grid_idx].timestamp == ts) {
//         ready[cam_idx] = true;
//       }
//     }
//
//     // Check if aprilgrids are observed by all cameras
//     if (std::all_of(ready.begin(), ready.end(), [](bool x) { return x; })) {
//       // Keep only common tags between all aprilgrids
//       std::vector<aprilgrid_t *> data;
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         auto &grid_idx = grid_indicies[cam_idx];
//         aprilgrid_t *grid = &grids[cam_idx][grid_idx];
//         data.push_back(grid);
//       }
//       aprilgrid_intersection(data);
//
//       // Add to result and update grid indicies
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         calib_data[cam_idx].push_back(*data[cam_idx]);
//         grid_indicies[cam_idx]++;
//       }
//
//     } else {
//       // Update grid indicies on those that do not the same timestamp
//       for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//         // Update grid index
//         if (ready[cam_idx] == false) {
//           grid_indicies[cam_idx]++;
//         }
//
//         // Termination criteria
//         if (grid_indicies[cam_idx] >= grids[cam_idx].size()) {
//           return 0;
//         }
//       }
//
//       // Try again - don't change timestamp yet*
//       goto try_again;
//     }
//   }
//
//   return 0;
// }

cv::Mat draw_calib_validation(const cv::Mat &image,
                              const vec2s_t &measured,
                              const vec2s_t &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color) {
  // Make an RGB version of the input image
  cv::Mat image_rgb = gray2rgb(image);

  // Draw measured points
  for (const auto &p : measured) {
    cv::circle(image_rgb,               // Target image
               cv::Point2f(p(0), p(1)), // Center
               1,                       // Radius
               measured_color,          // Colour
               CV_FILLED,               // Thickness
               8);                      // Line type
  }

  // Draw projected points
  for (const auto &p : projected) {
    cv::circle(image_rgb,               // Target image
               cv::Point2f(p(0), p(1)), // Center
               1,                       // Radius
               projected_color,         // Colour
               CV_FILLED,               // Thickness
               8);                      // Line type
  }

  // Calculate reprojection error and show in image
  const real_t rmse = reprojection_error(measured, projected);
  // -- Convert rmse to string
  std::stringstream stream;
  stream << std::fixed << std::setprecision(2) << rmse;
  const std::string rmse_str = stream.str();
  // -- Draw text
  const auto text = "RMSE Reprojection Error: " + rmse_str;
  const auto origin = cv::Point(0, 18);
  const auto red = cv::Scalar(0, 0, 255);
  const auto font = cv::FONT_HERSHEY_SIMPLEX;
  cv::putText(image_rgb, text, origin, font, 0.6, red, 2);

  return image_rgb;
}

std::ostream &operator<<(std::ostream &os, const calib_target_t &target) {
  os << "target_type: " << target.target_type << std::endl;
  os << "tag_rows: " << target.tag_rows << std::endl;
  os << "tag_cols: " << target.tag_cols << std::endl;
  os << "tag_size: " << target.tag_size << std::endl;
  os << "tag_spacing: " << target.tag_spacing << std::endl;
  return os;
}

} //  namespace yac
