#pragma once

#include "Core.hpp"

namespace yac {

class CalibTarget {
protected:
  timestamp_t ts_ = 0;
  int num_rows_ = 0;
  int num_cols_ = 0;

public:
  CalibTarget(const timestamp_t ts, const int num_rows, const int num_cols);
  virtual ~CalibTarget() = default;

  /** Get Timestamp **/
  timestamp_t getTimestamp() const;

  /** Get number of rows **/
  int getNumRows() const;

  /** Get number of cols **/
  int getNumCols() const;

  /** Get measurements **/
  virtual void getMeasurements(std::vector<int> &corner_ids,
                               vec2s_t &keypoints,
                               vec3s_t &object_points) const = 0;
};

} // namespace yac
