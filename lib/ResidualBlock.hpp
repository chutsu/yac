#pragma once
#include <ceres/ceres.h>

#include "ParamBlock.hpp"

namespace yac {

/** Residual Block */
class ResidualBlock : public ceres::CostFunction {
protected:
  // Data
  std::string type_;
  std::vector<double *> param_ptrs_;
  std::vector<int> param_sizes_;
  std::vector<ParamBlock::Type> param_types_;

public:
  /** Constructor */
  ResidualBlock() = delete;
  ResidualBlock(const std::string &type,
                const std::vector<double *> &param_ptrs,
                const std::vector<ParamBlock::Type> &param_types,
                const int num_residuals);

  /** Destructor */
  virtual ~ResidualBlock() = default;

  /** Get type */
  std::string getType() const;

  /** Get parameter block pointers */
  std::vector<double *> getParamPtrs() const;

  /** Evaluate with Minimal jacobians */
  virtual bool EvaluateWithMinimalJacobians(double const *const *params,
                                            double *res,
                                            double **jacs,
                                            double **min_jacs) const = 0;

  /** Evaluate with Minimal jacobians */
  bool Evaluate(double const *const *params, double *res, double **jacs) const;

  /** Check jacobians */
  bool checkJacobians(const int param_idx,
                      const std::string &jac_name,
                      const double step = 1e-8,
                      const double tol = 1e-4) const;
};

} // namespace yac
