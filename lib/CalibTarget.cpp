#include "CalibTarget.hpp"

namespace yac {

CalibTarget::CalibTarget(const timestamp_t ts,
                         const int num_rows,
                         const int num_cols)
    : ts_{ts}, num_rows_{num_rows}, num_cols_{num_cols} {}

timestamp_t CalibTarget::getTimestamp() const { return ts_; }

int CalibTarget::getNumRows() const { return num_rows_; }

int CalibTarget::getNumCols() const { return num_cols_; }

} // namespace yac
