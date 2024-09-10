#include "Residual.hpp"

namespace yac {

static int check_jacobian(const std::string &jac_name,
                          const matx_t &fdiff,
                          const matx_t &jac,
                          const double tol,
                          const bool print) {
  // Pre-check
  if (jac.size() == 0) {
    LOG_ERROR("Provided analytical jacobian is empty!");
    return false;
  } else if (fdiff.size() == 0) {
    LOG_ERROR("Provided numerical jacobian is empty!");
    return false;
  } else if (fdiff.rows() != jac.rows()) {
    LOG_ERROR("rows(fdiff) != rows(jac)");
    return false;
  } else if (fdiff.cols() != jac.cols()) {
    LOG_ERROR("cols(fdiff) != cols(jac)");
    return false;
  }

  // Check if any of the values are beyond the tol
  const matx_t delta = (fdiff - jac);
  bool failed = false;
  for (long i = 0; i < delta.rows(); i++) {
    for (long j = 0; j < delta.cols(); j++) {
      if (fabs(delta(i, j)) >= tol) {
        failed = true;
      }
    }
  }

  // Print result
  int retval = 0;
  if (failed) {
    if (print) {
      LOG_ERROR("Check [%s] failed!\n", jac_name.c_str());
      print_matrix("num diff jac", fdiff);
      print_matrix("analytical jac", jac);
      print_matrix("difference matrix", delta);
      // exit(-1);
    }
    retval = -1;

  } else {
    if (print) {
      printf("Check [%s] passed!\n", jac_name.c_str());
      // print_matrix("num diff jac", fdiff);
      // print_matrix("analytical jac", jac);
      // print_matrix("difference matrix", delta);
    }
    retval = 0;
  }

  return retval;
}

Residual::Residual(const std::string &type) : type_{type} {}

std::string Residual::getType() const { return type_; }

std::vector<double *> Residual::getParamPtrs() const {
  std::vector<double *> ptrs;
  for (auto &param_block : param_blocks) {
    ptrs.push_back(param_block->getPtr());
  }
  return ptrs;
}

bool Residual::Evaluate(double const *const *params,
                        double *res,
                        double **jacs) const {
  return EvaluateWithMinimalJacobians(params, res, jacs, nullptr);
}

bool Residual::checkJacobians(const int param_idx,
                              const std::string &jac_name,
                              const double step,
                              const double tol) const {
  // Setup
  ParamBlock *param_block = param_blocks[param_idx];
  const int r_size = num_residuals();
  const int local_size = param_block->getLocalSize();
  const size_t num_params = param_blocks.size();

  // Params
  std::vector<double *> param_ptrs = getParamPtrs();

  // Jacobians
  double **jac_ptrs = (double **)calloc(num_params, sizeof(double *));
  double **min_jac_ptrs = (double **)calloc(num_params, sizeof(double *));
  for (size_t i = 0; i < num_params; i++) {
    const auto jac_size = r_size * param_blocks[i]->getParamSize();
    const auto min_jac_size = r_size * param_blocks[i]->getLocalSize();
    jac_ptrs[i] = (double *)calloc(jac_size, sizeof(double));
    min_jac_ptrs[i] = (double *)calloc(min_jac_size, sizeof(double));
  }

  // Base-line
  vecx_t r = zeros(r_size);
  EvaluateWithMinimalJacobians(param_ptrs.data(),
                               r.data(),
                               jac_ptrs,
                               min_jac_ptrs);

  // Finite difference
  matx_t fdiff = zeros(r_size, local_size);
  vecx_t r_fd = zeros(r_size);
  for (int i = 0; i < local_size; i++) {
    param_block->perturb(i, step);
    Evaluate(param_ptrs.data(), r_fd.data(), nullptr);
    fdiff.col(i) = (r_fd - r) / step;
    param_block->perturb(i, -step);
  }

  // Check jacobian
  const auto min_jac_ptr = min_jac_ptrs[param_idx];
  Eigen::Map<matx_row_major_t> min_jac(min_jac_ptr, r_size, local_size);
  const int retval = check_jacobian(jac_name, fdiff, min_jac, tol, true);

  // Clean up
  for (size_t i = 0; i < num_params; i++) {
    free(jac_ptrs[i]);
    free(min_jac_ptrs[i]);
  }
  free(jac_ptrs);
  free(min_jac_ptrs);

  return (retval == 0) ? true : false;
}

} // namespace yac
