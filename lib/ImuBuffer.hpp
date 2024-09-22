#pragma once
#include "Core.hpp"

namespace yac {

class ImuBuffer {
private:
  std::vector<timestamp_t> timestamps_;
  std::vector<vec3_t> acc_data_;
  std::vector<vec3_t> gyr_data_;

public:
  ImuBuffer() = default;
  virtual ~ImuBuffer() = default;

  int getNumMeasurements() const { return timestamps_.size(); }

  /** Get timestamp */
  timestamp_t getTimestamp(const int index) const {
    return timestamps_.at(index);
  }

  /** Get accelerometer measurement */
  vec3_t getAcc(const int index) const { return acc_data_.at(index); }

  /** Get gyroscope measurement */
  vec3_t getGyr(const int index) const { return gyr_data_.at(index); }

  /** Add measurement */
  void add(const timestamp_t ts, const vec3_t &acc, const vec3_t gyr) {
    timestamps_.push_back(ts);
    acc_data_.push_back(acc);
    gyr_data_.push_back(gyr);
  }
};

} // namespace yac
