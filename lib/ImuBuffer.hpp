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

  int getNumMeasurements() const;

  /** Get timestamp */
  timestamp_t getTimestamp(const int index) const;

  /** Get accelerometer measurement */
  vec3_t getAcc(const int index) const;

  /** Get gyroscope measurement */
  vec3_t getGyr(const int index) const;

  /** Add measurement */
  void add(const timestamp_t ts, const vec3_t &acc, const vec3_t gyr);
};

} // namespace yac
