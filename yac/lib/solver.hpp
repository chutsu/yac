#include "calib_params.hpp"
#include "calib_errors.hpp"

namespace yac {

// SOLVER BASE /////////////////////////////////////////////////////////////////

struct solver_t {
  std::unordered_set<calib_error_t *> cost_fns;
  std::unordered_map<real_t *, param_t *> params;
  std::unordered_map<param_t *, std::unordered_set<calib_error_t *>> param2res;
  std::unordered_map<calib_error_t *, std::unordered_set<param_t *>> res2param;

  real_t initial_cost = 0.0;
  real_t final_cost = 0.0;
  real_t num_iterations = 0.0;

  virtual size_t num_residuals();
  virtual size_t num_params();
  virtual bool has_param(param_t *param);
  virtual void add_param(param_t *param);
  virtual void add_residual(calib_error_t *cost_fn);
  virtual void remove_param(param_t *param);
  virtual void remove_residual(calib_error_t *cost_fn);
  virtual void solve(const int max_iter = 30,
                     const bool verbose = false,
                     const int verbose_level = 0) = 0;
  virtual int estimate_calib_covar(matx_t &calib_covar) const;
};

// CERES-SOLVER ////////////////////////////////////////////////////////////////

struct ceres_solver_t : solver_t {
  int max_num_threads = 4;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;

  PoseLocalParameterization pose_plus;
  std::unordered_map<calib_error_t *, ceres::ResidualBlockId> res2id;

  ceres_solver_t();
  ~ceres_solver_t();

  size_t num_residuals() override;
  size_t num_params() override;
  bool has_param(param_t *param) override;
  void add_param(param_t *param) override;
  void add_residual(calib_error_t *cost_fn) override;
  void remove_param(param_t *param) override;
  void remove_residual(calib_error_t *cost_fn) override;

  void solve(const int max_iter = 30,
             const bool verbose = false,
             const int verbose_level = 0) override;
};

// SOLVER /////////////////////////////////////////////////////////////////////

struct yac_solver_t : solver_t {
  yac_solver_t() = default;
  ~yac_solver_t() = default;

  void linearize(matx_t &J, vecx_t &b);
  void solve(const int max_iter = 30,
             const bool verbose = false,
             const int verbose_level = 0);
};

} // namespace yac
