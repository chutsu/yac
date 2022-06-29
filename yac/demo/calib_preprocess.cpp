#include <filesystem>
#include "calib_data.hpp"

namespace fs = std::filesystem;

yac::aprilgrid_t detect(const yac::aprilgrid_detector_t &detector,
                        const yac::timestamp_t ts,
                        const cv::Mat &image,
                        FILE *csv_file) {
  const auto &grid = detector.detect(ts, image);

  std::vector<int> tag_ids;
  std::vector<int> corner_idxs;
  yac::vec2s_t keypoints;
  yac::vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_idxs, keypoints, object_points);

  for (size_t i = 0; i < tag_ids.size(); i++) {
    fprintf(csv_file, "%ld,", ts);
    fprintf(csv_file, "%d,", tag_ids[i]);
    fprintf(csv_file, "%d,", corner_idxs[i]);
    fprintf(csv_file, "%f,", keypoints[i].x());
    fprintf(csv_file, "%f\n", keypoints[i].y());
  }

  return grid;
}

cv::Mat draw_detections(const cv::Mat &image, const yac::aprilgrid_t &grid) {
  const int marker_size = 2;
  const cv::Scalar &color = cv::Scalar{0, 0, 255};
  cv::Mat image_rgb = yac::gray2rgb(image);

  for (const auto &kp : grid.keypoints()) {
    const cv::Point2f p(kp.x(), kp.y());
    cv::circle(image_rgb, p, marker_size, color, -1);
  }

  return image_rgb;
}

int main(int argc, char *argv[]) {
  // Check arguments
  if (argc != 7) {
    printf("usage: %s <tag_rows> <tag_cols> <tag_size> <tag_spacing> "
           "<cam_path> <out_path>\n",
           argv[0]);
    return -1;
  }

  // Parse arguments
  const int tag_rows = atoi(argv[1]);
  const int tag_cols = atoi(argv[2]);
  const double tag_size = stod(argv[3]);
  const double tag_spacing = stod(argv[4]);
  const std::string cam_path = argv[5];
  const std::string save_path = argv[6];

  LOG_INFO("Preprocessing camera data [%s]", cam_path.c_str());
  LOG_INFO("Saving to [%s]", save_path.c_str());
  LOG_INFO("tag_rows: %d", tag_rows);
  LOG_INFO("tag_cols: %d", tag_cols);
  LOG_INFO("tag_size: %f", tag_size);
  LOG_INFO("tag_spacing: %f", tag_spacing);

  // Load image paths
  std::set<std::string> img_paths;
  for (const auto &file : fs::directory_iterator(cam_path)) {
    img_paths.insert(file.path().u8string());
  }

  // Process images
  const std::string cmd = "mkdir -p " + save_path;
  system(cmd.c_str());

  const yac::aprilgrid_detector_t detector{tag_rows,
                                           tag_cols,
                                           tag_size,
                                           tag_spacing};

  FILE *csv_file = fopen("/tmp/yac_detections.csv", "w");
  fprintf(csv_file, "ts,tag_id,corner_idx,kp_x,kp_y\n");

  for (const auto img_path : img_paths) {
    const auto ss = img_path.length() - 23;
    const auto ts_len = 19;
    const std::string ts_str = img_path.substr(ss, ts_len);
    const yac::timestamp_t ts = std::stoull(ts_str);
    const auto &img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

    const auto &grid = detect(detector, ts, img, csv_file);
    const auto &viz = draw_detections(img, grid);

    cv::imshow("viz", viz);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }
  fclose(csv_file);

  return 0;
}
