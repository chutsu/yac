#pragma once
#include "Core.hpp"

namespace yac {

/** Parameter Block */
class ParamBlock {
private:
  int local_size_;
  int param_size_;
  vecx_t data_;

public:
  ParamBlock() = delete;

  ParamBlock(const int param_size)
      : local_size_{param_size}, param_size_{param_size}, data_{zeros(
                                                              param_size)} {}

  ParamBlock(const int param_size, const int local_size)
      : param_size_{param_size}, local_size_{local_size}, data_{zeros(
                                                              param_size)} {}

  virtual ParamBlock() = default;

  /** Get local size */
  int getLocalSize() const { return local_size_; }

  /** Get param size */
  int getParamSize() const { return param_size_; }

  /** Get Data */
  vecx_t getData() const { return data_; }

  /** Get Data */
  double *getPtr() const { return data_.data(); }
};

/** Pose */
class Pose : ParamBlock {
private:
  timestamp_t ts_;

public:
  Pose() = delete;
  Pose(const timestamp_t ts) : ParamBlock{7, 6} ts_{ts} {}
  virtual Pose() = default;

  /** Get timestamp */
  timestamp_t getTimestamp() const { return ts_; }
};

/** Extrinsic */
class Extrinsic : ParamBlock {
public:
  Extrinsic() : ParamBlock{7, 6} ts_{ts} {}
  virtual Extrinsic() = default;
};

/** Velocity */
class Velocity : ParamBlock {
public:
  Velocity() : ParamBlock{3} ts_{ts} {}
  virtual Velocity() = default;
};

/** Bias */
class Bias : ParamBlock {
public:
  Bias() : ParamBlock{3} ts_{ts} {}
  virtual Bias() = default;
};

} // namespace yac
