#ifndef YAC_SOLVER_HPP
#define YAC_SOLVER_HPP
#include "calib_params.hpp"
#include "calib_residuals.hpp"
#include "suitesparse.hpp"

#include <Eigen/SparseQR>
#include <Eigen/SparseCholesky>

namespace yac {

vecx_t linsolve_dense_cholesky(const matx_t &A, const vecx_t &b);
vecx_t linsolve_dense_svd(const matx_t &A, const vecx_t &b);
vecx_t linsolve_dense_qr(const matx_t &A, const vecx_t &b);
vecx_t linsolve_sparse_qr(const matx_t &A, const vecx_t &b);

// SOLVER BASE /////////////////////////////////////////////////////////////////

// clang-format off
using ResidualJacobians = std::map<calib_residual_t *, std::vector<matx_row_major_t>>;
using ResidualValues = std::map<calib_residual_t *, vecx_t>;
using ParameterOrder = std::map<param_t *, int>;
// clang-format on

struct solver_t {
  std::string algorithm_type = "LEVENBERG-MARQUARDT";
  truncated_solver_t tsolver;
  matx_t J;
  vecx_t r;
  int marg_idx = 0;

  // clang-format off
  std::unordered_set<calib_residual_t *> res_fns;
  std::unordered_map<real_t *, param_t *> params;
  std::unordered_map<param_t *, vecx_t> params_cache;
  std::unordered_map<param_t *, std::unordered_set<calib_residual_t *>> param2res;
  std::unordered_map<calib_residual_t *, std::unordered_set<param_t *>> res2param;
  // clang-format on

  real_t initial_cost = 0.0;
  real_t final_cost = 0.0;
  real_t num_iterations = 0.0;

  solver_t() = default;
  virtual ~solver_t() = default;

  virtual size_t num_residuals();
  virtual size_t num_params();
  virtual bool has_param(param_t *param);
  virtual void add_param(param_t *param);
  virtual void add_residual(calib_residual_t *res_fn);
  virtual void remove_param(param_t *param);
  virtual void remove_residual(calib_residual_t *res_fn);

  void _cache_params();
  void _restore_params();
  bool _eval_residual(calib_residual_t *res_fn,
                      ResidualJacobians &res_jacs,
                      ResidualJacobians &res_min_jacs,
                      ResidualValues &res_vals) const;
  bool _eval_residual(calib_residual_t *res_fn, vecx_t &r) const;
  real_t _calculate_cost();
  void _form_jacobian(ParameterOrder &param_order, matx_t &J, vecx_t &r);
  void _form_hessian(ParameterOrder &param_order, matx_t &H, vecx_t &b);
  void _update(const ParameterOrder &param_order, const vecx_t &dx);

  virtual int estimate_covariance(const std::vector<param_t *> params,
                                  matx_t &calib_covar,
                                  const bool verbose = false) const;
  virtual int
  estimate_covariance_determinant(const std::vector<param_t *> params,
                                  real_t &covar_det,
                                  const bool verbose = false);
  virtual void solve(const int max_iter = 30,
                     const bool verbose = false,
                     const int verbose_level = 0) = 0;
};

// CERES-SOLVER ////////////////////////////////////////////////////////////////

// #define ENABLE_CERES_COVARIANCE_ESTIMATOR

struct ceres_solver_t : solver_t {
  int max_num_threads = 4;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;

  PoseLocalParameterization pose_plus;
  std::unordered_map<calib_residual_t *, ceres::ResidualBlockId> res2id;

  ceres_solver_t();
  ~ceres_solver_t();

  size_t num_residuals() override;
  size_t num_params() override;
  bool has_param(param_t *param) override;
  void add_param(param_t *param) override;
  void add_residual(calib_residual_t *res_fn) override;
  void remove_param(param_t *param) override;
  void remove_residual(calib_residual_t *res_fn) override;

#ifdef ENABLE_CERES_COVARIANCE_ESTIMATOR
  int estimate_covariance(const std::vector<param_t *> params,
                          matx_t &covar,
                          const bool verbose = true) const override;
#endif // ENABLE_CERES_COVARIANCE_ESTIMATOR

  void solve(const int max_iter = 30,
             const bool verbose = false,
             const int verbose_level = 0) override;
};

// SOLVER /////////////////////////////////////////////////////////////////////

struct yac_solver_t : solver_t {
  real_t lambda = 1e-4;

  yac_solver_t() = default;
  ~yac_solver_t() = default;

  void _solve_linear_system(const real_t lambda_k,
                            ParameterOrder &param_order,
                            vecx_t &dx);
  void _solve_gn(const int max_iter,
                 const bool verbose,
                 const int verbose_level);
  void _solve_lm(const int max_iter,
                 const bool verbose,
                 const int verbose_level);

  void solve(const int max_iter = 30,
             const bool verbose = false,
             const int verbose_level = 0);
};

} // namespace yac
#endif // YAC_SOLVER_HPP
