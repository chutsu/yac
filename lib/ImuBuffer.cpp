#include "ImuBuffer.hpp"

namespace yac {

int ImuBuffer::getNumMeasurements() const { return timestamps_.size(); }

timestamp_t ImuBuffer::getTimestamp(const int index) const {
  return timestamps_.at(index);
}

vec3_t ImuBuffer::getAcc(const int index) const { return acc_data_.at(index); }

vec3_t ImuBuffer::getGyr(const int index) const { return gyr_data_.at(index); }

void ImuBuffer::add(const timestamp_t ts, const vec3_t &acc, const vec3_t gyr) {
  timestamps_.push_back(ts);
  acc_data_.push_back(acc);
  gyr_data_.push_back(gyr);
}

} // namespace yac
