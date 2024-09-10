#pragma once
#include "Core.hpp"

namespace yac {

/* Parameter Block */
class ParamBlock {
private:
  int param_size_;
  int local_size_;
  vecx_t data_;

public:
  ParamBlock() = delete;

  ParamBlock(const int param_size)
      : param_size_{param_size}, local_size_{param_size}, data_{zeros(
                                                              param_size)} {}

  ParamBlock(const int param_size, const int local_size)
      : param_size_{param_size}, local_size_{local_size}, data_{zeros(
                                                              param_size)} {}

  virtual ~ParamBlock() = default;

  /* Get local size */
  int getLocalSize() const { return local_size_; }

  /* Get param size */
  int getParamSize() const { return param_size_; }

  /* Get Data */
  vecx_t getData() const { return data_; }

  /* Get Data */
  double *getPtr() { return data_.data(); }

  /* Perturb */
  void perturb(const int i, const double step) { data_[i] += step; }
};

/* Pose */
class Pose : ParamBlock {
private:
  timestamp_t ts_;

public:
  Pose() = delete;
  Pose(const timestamp_t ts) : ParamBlock{7, 6}, ts_{ts} {}

  /* Get timestamp */
  timestamp_t getTimestamp() const { return ts_; }
};

/* Extrinsic */
class Extrinsic : ParamBlock {
public:
  Extrinsic() : ParamBlock{7, 6} {}
};

/* Velocity */
class Velocity : ParamBlock {
private:
  timestamp_t ts_;

public:
  Velocity(const timestamp_t ts) : ParamBlock{3}, ts_{ts} {}
};

/* Bias */
class Bias : ParamBlock {
private:
  timestamp_t ts_;

public:
  Bias(const timestamp_t ts) : ParamBlock{3}, ts_{ts} {}
};

} // namespace yac
