#include "euroc.hpp"
#include "calib_data.hpp"

namespace yac {

struct test_data_t {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const std::string data_path = "/data/euroc/calib/cam_april";
  const std::string grids0_path = data_path + "/grid0/cam0";
  const std::string grids1_path = data_path + "/grid0/cam1";
  euroc_calib_t data{data_path};
  aprilgrids_t grids0;
  aprilgrids_t grids1;
};

test_data_t setup_test_data() {
  test_data_t test_data;
  LOG_INFO("Loading EuRoC data [%s]", test_data.data_path.c_str());

  // preprocess aprilgrids
  LOG_INFO("Preprocessing AprilGrid data ...");
  aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
  const auto grids0_path = test_data.grids0_path;
  const auto grids1_path = test_data.grids1_path;
  if (system(("mkdir -p " + grids0_path).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", grids0_path.c_str());
  }
  if (system(("mkdir -p " + grids1_path).c_str()) != 0) {
    FATAL("Failed to create dir [%s]", grids1_path.c_str());
  }

  auto timeline = test_data.data.timeline();
  size_t i = 0;
  for (const auto &ts : timeline.timestamps) {
    if (i++ % 100 == 0) {
      printf(".");
      fflush(stdout);
    }

    const auto kv = timeline.data.equal_range(ts);
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;
      if (event.type == CAMERA_EVENT) {
        const auto ts = event.ts;
        const int cam_idx = event.camera_index;
        const cv::Mat image = cv::imread(event.image_path);
        const auto grid_fname = std::to_string(ts) + ".csv";

        std::string grid_file;
        if (cam_idx == 0) {
          grid_file = grids0_path + "/" + grid_fname;
        } else if (cam_idx == 1) {
          grid_file = grids1_path + "/" + grid_fname;
        }

        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
          grid = detector.detect(event.ts, image);
          grid.save(grid_file);
        } else {
          grid.load(grid_file);

          if (grid.detected == false) {
            grid.timestamp = ts;
            grid.tag_rows = detector.tag_rows;
            grid.tag_cols = detector.tag_cols;
            grid.tag_size = detector.tag_size;
            grid.tag_spacing = detector.tag_spacing;
          }
        }

        switch (cam_idx) {
        case 0: test_data.grids0.push_back(grid); break;
        case 1: test_data.grids1.push_back(grid); break;
        }
      }
    }
  }
  printf("\n");

  return test_data;
}

}  // namespace yac
