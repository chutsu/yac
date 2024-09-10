#pragma once
#include <ceres/ceres.h>

#include "ParamBlock.hpp"

namespace yac {

class Residual : public ceres::CostFunction {
private:
  // Data
  std::string type_;
  std::vector<ParamBlock *> param_blocks;

public:
  /* Constructor */
  Residual() = default;
  Residual(const std::string &type);

  /* Destructor */
  virtual ~Residual() = default;

  /* Get type */
  std::string getType() const;

  /* Get parameter block pointers */
  std::vector<double *> getParamPtrs() const;

  /* Evaluate with Minimal jacobians */
  virtual bool EvaluateWithMinimalJacobians(double const *const *params,
                                            double *res,
                                            double **jacs,
                                            double **min_jacs) const = 0;

  /* Evaluate with Minimal jacobians */
  bool Evaluate(double const *const *params, double *res, double **jacs) const;

  /* Check jacobians */
  bool checkJacobians(const int param_idx,
                      const std::string &jac_name,
                      const double step = 1e-8,
                      const double tol = 1e-4) const;
};

} // namespace yac
