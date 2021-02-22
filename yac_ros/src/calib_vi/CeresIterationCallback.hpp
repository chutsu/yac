#ifndef YAC_CERESITERATIONCALLBACK_HPP
#define YAC_CERESITERATIONCALLBACK_HPP

#include <ceres/iteration_callback.h>

namespace yac {

/**
 * The CeresIterationCallback class tries to enforce a time limit on the
 * optimization. It does not guarantee to stay within the time budget as
 * it assumes the next iteration takes as long as the previous iteration.
 */
class CeresIterationCallback : public ::ceres::IterationCallback {
public:
  CeresIterationCallback(double timeLimit, int iterationMinimum)
      : timeLimit_(timeLimit), iterationMinimum_(iterationMinimum) {}
  ~CeresIterationCallback() {}

  /**
   * This method is called after every iteration in ceres.
   *
   * @param summary Iteration summary
   */
  ::ceres::CallbackReturnType
  operator()(const ::ceres::IterationSummary &summary) {
    // Assume next iteration takes the same time as current iteration
    if (summary.iteration >= iterationMinimum_ &&
        summary.cumulative_time_in_seconds + summary.iteration_time_in_seconds >
            timeLimit_) {
      return ::ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    }
    return ::ceres::SOLVER_CONTINUE;
  }

  /**
   * Change time limit of optimization.
   *
   * If you want to disable the time limit, either set it to a large
   * value, delete the callback in the ceres options or set the minimum
   * iterations the maximum iteration.
   *
   * @param timeLimit desired time limit in seconds
   */
  void setTimeLimit(double timeLimit) { timeLimit_ = timeLimit; }

  /**
   * Chage the minimum iterations the optimization goes through disregarding
   * the time limit
   *
   * @param iterationMinimum
   */
  void setMinimumIterations(int iterationMinimum) {
    iterationMinimum_ = iterationMinimum;
  }

private:
  double timeLimit_;     ///< The set time limit.
  int iterationMinimum_; ///< The set maximum no. iterations.
};

} // namespace yac
#endif /* YAC_CERESITERATIONCALLBACK_HPP */
