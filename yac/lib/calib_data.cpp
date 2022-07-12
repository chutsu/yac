#include "calib_data.hpp"

namespace yac {

// CALIB UTILS /////////////////////////////////////////////////////////////////

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses,
                     timestamp_t ts_offset) {
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

void load_imu_data(const std::string &csv_path,
                   timestamps_t &timestamps,
                   vec3s_t &a_B,
                   vec3s_t &w_B) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(csv_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", csv_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double ax, ay, az = 0.0;
    double wx, wy, wz = 0.0;
    int retval = fscanf(fp,
                        "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &wx,
                        &wy,
                        &wz,
                        &ax,
                        &ay,
                        &az);
    if (retval != 7) {
      LOG_ERROR("Invalid line: %d", i);
      FATAL("Failed to parse line in [%s]", csv_path.c_str());
    }

    timestamps.push_back(ts);
    a_B.emplace_back(ax, ay, az);
    w_B.emplace_back(wx, wy, wz);
  }
  fclose(fp);
}

// CALIB TARGET ////////////////////////////////////////////////////////////////

calib_target_t::calib_target_t(const std::string target_type_,
                               const int tag_rows_,
                               const int tag_cols_,
                               const real_t tag_size_,
                               const real_t tag_spacing_)
    : target_type{target_type_}, tag_rows{tag_rows_}, tag_cols{tag_cols_},
      tag_size{tag_size_}, tag_spacing{tag_spacing_} {}

int calib_target_t::load(const std::string &target_file,
                         const std::string &prefix) {
  config_t config{target_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  const auto parent = (prefix == "") ? "" : prefix + ".";
  parse(config, parent + "target_type", target_type);
  parse(config, parent + "tag_rows", tag_rows);
  parse(config, parent + "tag_cols", tag_cols);
  parse(config, parent + "tag_size", tag_size);
  parse(config, parent + "tag_spacing", tag_spacing);

  return 0;
}

void calib_target_t::print() const {
  printf("target_type: %s\n", target_type.c_str());
  printf("tag_rows: %d\n", tag_rows);
  printf("tag_cols: %d\n", tag_cols);
  printf("tag_size: %f\n", tag_size);
  printf("tag_spacing: %f\n", tag_spacing);
}

// CALIBRATION DETECTION //////////////////////////////////////////////////////

std::map<int, aprilgrids_t>
calib_data_preprocess(const calib_target_t &calib_target,
                      const std::map<int, std::string> cam_paths,
                      const std::string &grids_path,
                      const bool imshow) {
  printf("Preprocessing calibration data ...\n");

  // AprilGrid detector
  const int tag_rows = calib_target.tag_rows;
  const int tag_cols = calib_target.tag_cols;
  const double tag_size = calib_target.tag_size;
  const double tag_spacing = calib_target.tag_spacing;
  const aprilgrid_detector_t detector{tag_rows,
                                      tag_cols,
                                      tag_size,
                                      tag_spacing};

  // Detect aprilgrids
  std::map<int, aprilgrids_t> cam_grids;

  for (const auto &[cam_idx, cam_path] : cam_paths) {
    // Cam grid path
    const std::string cam_str = "cam" + std::to_string(cam_idx);
    std::string cam_grid_path = grids_path;
    cam_grid_path += (grids_path.back() == '/') ? "" : "/";
    cam_grid_path += cam_str;

    // Create grids path
    if (system(("mkdir -p " + cam_grid_path).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", cam_grid_path.c_str());
    }

    // Get image files
    std::vector<std::string> img_paths;
    list_files(cam_path, img_paths);

    // Loop through image files
    printf("Preprocessing %s data: ", cam_str.c_str());

#pragma omp parallel for shared(cam_grids)
    for (size_t k = 0; k < img_paths.size(); k++) {
      const std::string img_fname = img_paths[k];
      const std::string img_path = cam_path + "/" + img_fname;
      const std::string ts_str = img_fname.substr(0, 19);
      if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
        LOG_WARN("Unexpected image file: [%s]!", img_path.c_str());
        continue;
      }
      const timestamp_t ts = std::stoull(ts_str);
      const std::string grid_fname = ts_str + ".csv";
      const std::string grid_path = cam_grid_path + "/" + grid_fname;
      const auto image = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

      // Load or detect apriltag
      aprilgrid_t grid;
      if (file_exists(grid_path)) {
        grid.load(grid_path);
        if (grid.detected == false) {
          grid.timestamp = ts;
          grid.tag_rows = detector.tag_rows;
          grid.tag_cols = detector.tag_cols;
          grid.tag_size = detector.tag_size;
          grid.tag_spacing = detector.tag_spacing;
        }
      } else {
        grid = detector.detect(ts, image);
        grid.save(grid_path);
      }

      // Imshow
      if (imshow) {
        grid.imshow("viz", image);
        cv::waitKey(1);
      }

#pragma omp critical
      cam_grids[cam_idx].push_back(grid);
      if ((k % 100) == 0) {
        printf(".");
        fflush(stdout);
      }
    }
    printf("\n");
    fflush(stdout);
  }

  return cam_grids;
}

} // namespace yac
