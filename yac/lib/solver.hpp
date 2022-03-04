#include "calib_params.hpp"
#include "calib_errors.hpp"

namespace yac {

// SOLVER BASE /////////////////////////////////////////////////////////////////

struct solver_base_t {
  std::unordered_set<calib_error_t *> cost_fns;
  std::unordered_map<real_t *, param_t *> params;
  std::unordered_map<param_t *, std::unordered_set<calib_error_t *>> param2res;
  std::unordered_map<calib_error_t *, std::unordered_set<param_t *>> res2param;

  virtual void add_param(param_t *param);
  virtual void add_residual(calib_error_t *cost_fn);
  virtual void remove_param(param_t *param);
  virtual void remove_residual(calib_error_t *cost_fn);
  virtual void solve(const int max_iter,
                     const bool verbose = false,
                     const int verbose_level = 0) = 0;
};

// CERES-SOLVER ////////////////////////////////////////////////////////////////

struct ceres_solver_t : solver_base_t {
  int max_num_threads = 4;
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;

  PoseLocalParameterization pose_plus;
  std::unordered_map<calib_error_t *, ceres::ResidualBlockId> res2id;

  ceres_solver_t();
  ~ceres_solver_t();

  void add_param(param_t *param);
  void add_residual(calib_error_t *cost_fn);
  void remove_param(param_t *param);
  void remove_residual(calib_error_t *cost_fn);

  void solve(const int max_iter,
             const bool verbose = false,
             const int verbose_level = 0);
};

// SOLVER /////////////////////////////////////////////////////////////////////

struct solver_t : solver_base_t {
  solver_t() = default;
  ~solver_t() = default;

  void linearize(matx_t &J, vecx_t &b);
  void solve(const int max_iter,
             const bool verbose = false,
             const int verbose_level = 0);
};

} // namespace yac
