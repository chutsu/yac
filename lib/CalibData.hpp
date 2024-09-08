#pragma once
#include "Core.hpp"
#include "AprilGrid.hpp"

namespace yac {

class CalibData {
private:
  std::string config_path_;
  std::string dir_path_;

  std::string target_type_;
  int target_rows_;
  int target_cols_;
  double target_size_;
  double target_spacing_;

  std::unordered_map<int, std::map<timestamp_t, std::shared_ptr<CalibTarget>>> camera_data_;

  /** Load Config */
  void loadConfig(const std::string &config_path);

  /** Load Camera Data */
  void loadCameraData(const int camera_index);

public:
  CalibData() = delete;
  CalibData(const std::string &config_path);
  virtual ~CalibData() = default;
};

} // namespace yac
