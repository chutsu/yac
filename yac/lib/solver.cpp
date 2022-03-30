#include "solver.hpp"

namespace yac {

vecx_t linsolve_dense_cholesky(const matx_t &A, const vecx_t &b) {
  return A.ldlt().solve(b);
}

vecx_t linsolve_dense_svd(const matx_t &A, const vecx_t &b) {
  Eigen::JacobiSVD<matx_t> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const matx_t U = svd.matrixU();
  const matx_t V = svd.matrixV();
  const vecx_t sv = svd.singularValues();
  // const auto svd_rank = svd.rank();
  // const vecx_t dx = V.leftCols(svd_rank) *
  //                   sv.head(svd_rank).asDiagonal().inverse() *
  //                   U.leftCols(svd_rank).adjoint() * b;
  // const vecx_t dx = V * sv.asDiagonal().inverse() * U.transpose() * b;
  const vecx_t dx = svd.solve(b);

  return dx;
}

vecx_t linsolve_dense_qr(const matx_t &A, const vecx_t &b) {
  Eigen::HouseholderQR<matx_t> qr(A);
  return qr.solve(b);
}

vecx_t linsolve_sparse_qr(const matx_t &A, const vecx_t &b) {
  profiler_t prof;

  // clang-format off
  prof.start("SparseQR.solve()");
  Eigen::SparseQR<Eigen::SparseMatrix<real_t>, Eigen::COLAMDOrdering<int>> solver;

  // -- Decompose H_sparse
  solver.compute(A.sparseView());
  if (solver.info() != Eigen::Success) {
    FATAL("SparseQR decomp failed!");
  }

  // -- Solve for x
  const vecx_t x = solver.solve(b);
  if (solver.info() != Eigen::Success) {
    FATAL("SparseQR decomp failed!");
  }
  prof.stop("SparseQR.solve()");
  // clang-format on

  // prof.print("H = JtJ");
  // prof.print("H.sparse_view()");
  // prof.print("SparseQR.solve()");
  // printf("\n");
  // clang-format on

  return x;
}

// SOLVER BASE /////////////////////////////////////////////////////////////////

size_t solver_t::num_residuals() { return res_fns.size(); }

bool solver_t::has_param(param_t *param) { return params.count(param->data()); }

size_t solver_t::num_params() { return params.size(); }

void solver_t::add_param(param_t *param) { params[param->data()] = param; }

void solver_t::add_residual(calib_residual_t *res_fn) {
  // Pre-check
  if (res_fns.count(res_fn)) {
    printf("residual: %s, addr: %p\n", res_fn->type.c_str(), res_fn);
    FATAL("Residual already exists in problem!");
  }

  // Insert
  res_fns.insert(res_fn);

  // Update param2res
  for (auto param : res_fn->param_blocks) {
    param2res[param].insert(res_fn);
  }

  // Update res2param
  for (auto param : res_fn->param_blocks) {
    res2param[res_fn].insert(param);
  }
}

void solver_t::remove_param(param_t *param) {
  // Pre-check
  if (params.count(param->data()) == 0) {
    FATAL("Parameter block does not exist in problem!");
  }

  // Erase parameter
  params.erase(param->data());

  // Update param2res book-keeping
  if (param2res.count(param)) {
    for (auto res_fn : param2res[param]) {
      if (res_fns.count(res_fn) == 0) {
        FATAL("Residual block does not exist in problem!");
      }
      res_fns.erase(res_fn);
      res2param.erase(res_fn);
    }
    param2res.erase(param);
  }
}

void solver_t::remove_residual(calib_residual_t *res_fn) {
  // Pre-check
  if (res_fns.count(res_fn) == 0) {
    FATAL("Residual block does not exist in problem!");
  }

  // Erase residual
  res_fns.erase(res_fn);

  // Update param2res
  for (auto param : res_fn->param_blocks) {
    param2res[param].erase(res_fn);
  }

  // Update res2param
  res2param.erase(res_fn);
}

void solver_t::_cache_params() {
  params_cache.clear();
  for (const auto &[param_data, param] : params) {
    params_cache[param] = param->param;
  }
}

void solver_t::_restore_params() {
  for (const auto &[param, param_data] : params_cache) {
    params[param->data()]->param = param_data;
  }
  params_cache.clear();
}

bool solver_t::_eval_residual(calib_residual_t *res_fn, vecx_t &r) const {
  // Setup parameter data
  std::vector<double *> param_ptrs;
  for (auto param_block : res_fn->param_blocks) {
    param_ptrs.push_back(param_block->data());
  }

  // Evaluate residual block
  r = zeros(res_fn->num_residuals(), 1);
  return res_fn->EvaluateWithMinimalJacobians(param_ptrs.data(),
                                              r.data(),
                                              NULL,
                                              NULL);
}

bool solver_t::_eval_residual(calib_residual_t *res_fn,
                              ResidualJacobians &res_jacs,
                              ResidualJacobians &res_min_jacs,
                              ResidualValues &res_vals) const {
  // Setup parameter data
  std::vector<double *> param_ptrs;
  for (auto param_block : res_fn->param_blocks) {
    param_ptrs.push_back(param_block->data());
  }

  // Setup Jacobians data
  const int r_size = res_fn->num_residuals();
  std::vector<double *> jac_ptrs;
  for (auto param_block : res_fn->param_blocks) {
    res_jacs[res_fn].push_back(zeros(r_size, param_block->global_size));
    jac_ptrs.push_back(res_jacs[res_fn].back().data());
  }

  // Setup Min-Jacobians data
  std::vector<double *> min_jac_ptrs;
  for (auto param_block : res_fn->param_blocks) {
    res_min_jacs[res_fn].push_back(zeros(r_size, param_block->local_size));
    min_jac_ptrs.push_back(res_min_jacs[res_fn].back().data());
  }

  // Evaluate residual block
  res_vals[res_fn] = zeros(r_size, 1);
  return res_fn->EvaluateWithMinimalJacobians(param_ptrs.data(),
                                              res_vals[res_fn].data(),
                                              jac_ptrs.data(),
                                              min_jac_ptrs.data());
}

void solver_t::_eval_residuals(ParameterOrder &param_order,
                               std::vector<calib_residual_t *> &res_evaled,
                               ResidualJacobians &res_jacs,
                               ResidualJacobians &res_min_jacs,
                               ResidualValues &res_vals,
                               size_t &residuals_length,
                               size_t &params_length) {
  // Evaluate residuals
  residuals_length = 0;
  for (calib_residual_t *res_fn : res_fns) {
    if (_eval_residual(res_fn, res_jacs, res_min_jacs, res_vals)) {
      res_evaled.push_back(res_fn);
      residuals_length += res_fn->num_residuals();
    }
  }

  // Track unique parameters
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> sb_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  std::vector<param_t *> fiducial_ptrs;
  params_length = 0;

  for (auto res_fn : res_evaled) {
    for (auto param_block : res_fn->param_blocks) {
      // Check if parameter block seen already
      if (params_seen.count(param_block)) {
        continue;
      }
      params_seen[param_block] = true;

      // Check if parameter is fixed
      if (param_block->fixed) {
        continue;
      }

      // Track parameter
      if (param_block->type == "pose_t") {
        pose_ptrs.push_back(param_block);
      } else if (param_block->type == "sb_params_t") {
        sb_ptrs.push_back(param_block);
      } else if (param_block->type == "camera_params_t") {
        cam_ptrs.push_back(param_block);
      } else if (param_block->type == "extrinsics_t") {
        extrinsics_ptrs.push_back(param_block);
      } else if (param_block->type == "fiducial_t") {
        fiducial_ptrs.push_back(param_block);
      }
      params_length += param_block->local_size;
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t idx = 0;
  std::vector<std::vector<param_t *> *> param_groups = {
      &pose_ptrs,
      &sb_ptrs,
      &cam_ptrs,
      &extrinsics_ptrs,
      &fiducial_ptrs,
  };
  param_order.clear();
  for (const auto &param_ptrs : param_groups) {
    for (const auto &param_block : *param_ptrs) {
      param_order.insert({param_block, idx});
      idx += param_block->local_size;
    }
  }

  // Keep a note of the marginalization index
  marg_idx = param_order[cam_ptrs[0]];
}

real_t solver_t::_calculate_cost() {
  real_t cost = 0.0;
  for (calib_residual_t *res_fn : res_fns) {
    vecx_t r;
    if (_eval_residual(res_fn, r)) {
      cost += 0.5 * r.transpose() * r;
    }
  }

  return cost;
}

void solver_t::_form_jacobian(ParameterOrder &param_order,
                              matx_t &J,
                              vecx_t &r) {
  // Evaluate residuals
  std::vector<calib_residual_t *> res_evaled;
  ResidualJacobians res_jacs;
  ResidualJacobians res_min_jacs;
  ResidualValues res_vals;
  size_t residuals_length = 0;
  size_t params_length = 0;
  _eval_residuals(param_order,
                  res_evaled,
                  res_jacs,
                  res_min_jacs,
                  res_vals,
                  residuals_length,
                  params_length);

  // Form Jacobian J and r
  J = zeros(residuals_length, params_length);
  r = zeros(residuals_length, 1);
  size_t res_idx = 0;

  for (calib_residual_t *res_fn : res_evaled) {
    const vecx_t r_val = res_vals[res_fn];
    const int res_size = r_val.size();
    r.segment(res_idx, res_size) = r_val;

    for (size_t i = 0; i < res_fn->param_blocks.size(); i++) {
      const auto &param_i = res_fn->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }

      const int param_idx = param_order[param_i];
      const int param_size = param_i->local_size;
      const matx_t J_param = res_min_jacs[res_fn][i];
      J.block(res_idx, param_idx, res_size, param_size) = J_param;
    }

    res_idx += res_fn->num_residuals();
  }
}

void solver_t::_form_hessian(ParameterOrder &param_order,
                             matx_t &H,
                             vecx_t &b) {
  // Evaluate residuals
  prof.start("form_hessian-eval_residuals");
  std::vector<calib_residual_t *> res_evaled;
  ResidualJacobians res_jacs;
  ResidualJacobians res_min_jacs;
  ResidualValues res_vals;
  size_t residuals_length = 0;
  size_t params_length = 0;
  _eval_residuals(param_order,
                  res_evaled,
                  res_jacs,
                  res_min_jacs,
                  res_vals,
                  residuals_length,
                  params_length);
  prof.stop("form_hessian-eval_residuals");

  // Form Hessian H and R.H.S b
  prof.start("form_hessian-fill_values");
  H = zeros(params_length, params_length);
  b = zeros(params_length, 1);

  for (auto res_fn : res_evaled) {
    const vecx_t r = res_vals[res_fn];

    for (size_t i = 0; i < res_fn->param_blocks.size(); i++) {
      const auto &param_i = res_fn->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }
      const int idx_i = param_order[param_i];
      const int size_i = param_i->local_size;
      const matx_t J_i = res_min_jacs.at(res_fn)[i];

      for (size_t j = i; j < res_fn->param_blocks.size(); j++) {
        const auto &param_j = res_fn->param_blocks[j];
        if (param_j->fixed) {
          continue;
        }
        const int idx_j = param_order[param_j];
        const int size_j = param_j->local_size;
        const matx_t J_j = res_min_jacs.at(res_fn)[j];

        if (i == j) {
          // Form diagonals of H
          H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
        } else {
          // Form off-diagonals of H
          // clang-format off
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() * J_j).transpose();
          // clang-format on
        }
      }

      b.segment(idx_i, size_i) += -J_i.transpose() * r;
    }
  }
  prof.stop("form_hessian-fill_values");

  // printf("profile:\n");
  // prof.print("form_hessian-eval_residuals");
  // prof.print("form_hessian-fill_values");
  // printf("\n");
}

void solver_t::_update(const ParameterOrder &param_order, const vecx_t &dx) {
  for (auto &[param, idx] : param_order) {
    param->plus(dx.segment(idx, param->local_size));
  }
}

void solver_t::clear() {
  tsolver.clear();
  marg_idx = 0;

  res_fns.clear();
  params.clear();
  params_cache.clear();
  param2res.clear();
  res2param.clear();
}

int solver_t::estimate_covariance(const std::vector<param_t *> &params,
                                  matx_t &calib_covar) const {
  // Evaluate residuals
  ResidualJacobians res_jacs;
  ResidualJacobians res_min_jacs;
  ResidualValues res_vals;
  std::vector<calib_residual_t *> good_res_fns;
  size_t residuals_length = 0;

  for (calib_residual_t *res_fn : res_fns) {
    if (_eval_residual(res_fn, res_jacs, res_min_jacs, res_vals)) {
      good_res_fns.push_back(res_fn);
      residuals_length += res_fn->num_residuals();
    }
  }

  // Track unique parameters
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> sb_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  std::vector<param_t *> fiducial_ptrs;

  for (auto res_fn : good_res_fns) {
    for (auto param_block : res_fn->param_blocks) {
      // Check if parameter block seen already
      if (params_seen.count(param_block)) {
        continue;
      }
      params_seen[param_block] = true;

      // Make sure param block is not fixed
      if (param_block->fixed) {
        continue;
      }

      // Check if parameter's uncertainty is to be estimated
      auto begin = params.begin();
      auto end = params.end();
      if (std::find(begin, end, param_block) != end) {
        continue;
      }

      // Track parameter
      if (param_block->type == "pose_t") {
        pose_ptrs.push_back(param_block);
      } else if (param_block->type == "sb_params_t") {
        sb_ptrs.push_back(param_block);
      } else if (param_block->type == "camera_params_t") {
        cam_ptrs.push_back(param_block);
      } else if (param_block->type == "extrinsics_t") {
        extrinsics_ptrs.push_back(param_block);
      } else if (param_block->type == "fiducial_t") {
        fiducial_ptrs.push_back(param_block);
      }
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t m = 0;
  size_t r = 0;
  size_t idx = 0;
  std::vector<std::vector<param_t *> *> param_groups = {
      &pose_ptrs,
      &sb_ptrs,
      &cam_ptrs,
      &extrinsics_ptrs,
      &fiducial_ptrs,
  };
  ParameterOrder param_order;
  // -- Column indices for parameter blocks to be marginalized
  for (const auto &param_ptrs : param_groups) {
    for (const auto &param_block : *param_ptrs) {
      if (param_block->fixed) {
        continue;
      }
      param_order.insert({param_block, idx});
      idx += param_block->local_size;
      m += param_block->local_size;
    }
  }
  // -- Column indices for parameter blocks to remain
  for (const auto &param_block : params) {
    if (param_block->fixed) {
      continue;
    }
    param_order.insert({param_block, idx});
    idx += param_block->local_size;
    r += param_block->local_size;
  }

  // Form Hessian
  const auto H_size = m + r;
  matx_t H = zeros(H_size, H_size);

  for (auto res_fn : good_res_fns) {
    for (size_t i = 0; i < res_fn->param_blocks.size(); i++) {
      const auto &param_i = res_fn->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }
      const int idx_i = param_order[param_i];
      const int size_i = param_i->local_size;
      const matx_t J_i = res_min_jacs.at(res_fn)[i];

      for (size_t j = i; j < res_fn->param_blocks.size(); j++) {
        const auto &param_j = res_fn->param_blocks[j];
        if (param_j->fixed) {
          continue;
        }
        const int idx_j = param_order[param_j];
        const int size_j = param_j->local_size;
        const matx_t J_j = res_min_jacs.at(res_fn)[j];

        if (i == j) {
          // Form diagonals of H
          H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
        } else {
          // Form off-diagonals of H
          // clang-format off
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() * J_j).transpose();
          // clang-format on
        }
      }
    }
  }

  // Marginalize
  // -- Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // entry by its reciprocal, leaving the zeros in place, and transposing
  // the resulting matrix.
  // clang-format off
  matx_t Hmm = H.block(0, 0, m, m);
  Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals_inv = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array().inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();

  // Eigen::JacobiSVD< matx_t> svd(Hmm, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // const auto rank = svd.rank();
  // const matx_t Hmm_inv = V.leftCols(rank) * s.head(rank).asDiagonal().inverse() * U.leftCols(rank).adjoint();
  // printf("svd rank: %ld\n", svd.rank());
  // printf("Hmm: %ldx%ld\n", Hmm.rows(), Hmm.cols());

  // clang-format on
  // -- Calculate Schur's complement
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const matx_t H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  // -- Inverse check
  const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
  if (fabs(inv_check) > 1e-4) {
    // LOG_WARN("Rank: %ld, Hmm_inv: %ldx%ld\n",
    //          rank,
    //          Hmm_inv.rows(),
    //          Hmm_inv.cols());
    // printf("(Hmm * Hmm_inv).norm(): %f\n", (Hmm * Hmm_inv).norm());
    // LOG_WARN("Covariance estimation: Inverse identity check: %f", inv_check);
    // LOG_WARN("This is bad ... Usually means covariance estimated is bad!");
    return -1;
  }

  // Convert
  calib_covar = H_marg.inverse();
  if (std::isnan(calib_covar.determinant())) {
    // LOG_WARN("det(calib_covar) is nan!");
    return -1;
  }

  return 0;
}

int solver_t::estimate_log_covariance_determinant(
    const std::vector<param_t *> &params, real_t &covar_det) {
  UNUSED(params);

  // Form dense jacobian
  ParameterOrder param_order;
  matx_t J;
  vecx_t r;
  _form_jacobian(param_order, J, r);

  // Convert to sparse jacobian
  const auto eps = std::numeric_limits<double>::epsilon();
  cholmod_sparse *J_sparse = cholmod_convert(J, &tsolver.cholmod_, eps);
  tsolver.analyze_marginal(J_sparse, marg_idx);
  cholmod_l_free_sparse(&J_sparse, &tsolver.cholmod_);

  // Estimate covariance determinant: log(det(covar))
  const auto sv_rank = tsolver.svdRank_;
  const vecx_t s = tsolver.getSingularValues();
  covar_det = -1.0 * s.head(sv_rank).array().log().sum();

  return 0;
}

// SOLVER /////////////////////////////////////////////////////////////////////

void yac_solver_t::_solve_gn(const int max_iter,
                             const bool verbose,
                             const int verbose_level) {
  UNUSED(verbose_level);

  // Lambda function to print status
  auto print_stats =
      [&](const int iter, const real_t cost, const real_t dcost) {
        printf("iter: %d, ", iter);
        printf("cost: %e, ", cost);
        printf("dcost: %.2e\n", dcost);
      };

  // Setup
  int iter = 1;
  const real_t cost_init = _calculate_cost();
  real_t cost_km1 = cost_init;
  real_t cost_k = 0.0;
  real_t dcost = 0.0;
  if (verbose) {
    print_stats(0, cost_km1, dcost);
  }

  // Cache params
  _cache_params();

  // Solve
  for (iter = 1; iter < max_iter; iter++) {
    // Form Jacobian J, and residual r
    ParameterOrder param_order;
    matx_t J;
    vecx_t r;
    _form_jacobian(param_order, J, r);

    // Convert dense J and r to sparse
    // clang-format off
    const auto eps = std::numeric_limits<double>::epsilon();
    cholmod_sparse *J_sparse = cholmod_convert(J, &tsolver.cholmod_, eps);
    cholmod_dense *r_dense = cholmod_convert(-1.0 * r, &tsolver.cholmod_);
    // clang-format on

    // Solve: J dx = -r
    vecx_t dx;
    tsolver.solve(J_sparse, r_dense, marg_idx, dx);
    cholmod_l_free_sparse(&J_sparse, &tsolver.cholmod_);
    cholmod_l_free_dense(&r_dense, &tsolver.cholmod_);
    _update(param_order, dx);

    // Calculate convergence
    cost_k = _calculate_cost();
    dcost = cost_k - cost_km1;
    cost_km1 = cost_k;

    // Print stats
    if (verbose) {
      print_stats(iter, cost_k, dcost);
    }
  }

  // Restore if didn't converge
  if (cost_k > cost_init) {
    _restore_params();
  }

  if (verbose) {
    const real_t cost_final = _calculate_cost();
    printf("Initial cost: %.4e\n", cost_init);
    printf("Final cost: %.4e\n", cost_final);
    printf("Num iterations: %d\n\n", iter);
  }
}

void yac_solver_t::_solve_lm(const int max_iter,
                             const bool verbose,
                             const int verbose_level) {
  UNUSED(verbose_level);

  // Lambda function to print status
  auto print_stats = [&](const int iter,
                         const real_t cost,
                         const real_t dcost,
                         const real_t lambda_k) {
    printf("iter: %d, ", iter);
    printf("cost: %e, ", cost);
    printf("dcost: %.2e, ", dcost);
    printf("lambda_k: %.2e\n", lambda_k);
  };

  // Setup
  int iter = 0;
  const real_t cost_init = _calculate_cost();
  real_t lambda_k = lambda;
  real_t cost_km1 = cost_init;
  real_t cost_k = 0.0;
  real_t dcost = 0.0;
  if (verbose) {
    print_stats(0, cost_km1, dcost, lambda_k);
  }

  matx_t H;
  vecx_t b;

  // Cholmod setup
  SuiteSparseQR_factorization<double> *QR = nullptr;
  cholmod_common cc;
  cholmod_l_start(&cc);

  for (iter = 1; iter < max_iter; iter++) {
    // Cache parameters
    _cache_params();

    // Form Hessian H and R.H.S vector b
    prof.start("form_hessian");
    ParameterOrder param_order;
    _form_hessian(param_order, H, b);
    prof.stop("form_hessian");

    // Apply LM-dampening
    H = H + lambda_k * I(H.rows());

    // Solve with SPQR
    // -- Setup H_sparse and b_dense
    prof.start("cholmod-sparsify");
    const auto eps = std::numeric_limits<double>::epsilon();
    cholmod_sparse *H_sparse = cholmod_convert(H, &cc, eps);
    cholmod_dense *b_dense = cholmod_convert(b, &cc);
    CHECK(H_sparse != nullptr);
    CHECK(b_dense != nullptr);
    prof.stop("cholmod-sparsify");

    // -- Factorize matrix symbolically
    prof.start("spqr-factorize-symbolically");
    if (QR && QR->QRsym &&
        (QR->QRsym->m != static_cast<std::ptrdiff_t>(H_sparse->nrow) ||
         QR->QRsym->n != static_cast<std::ptrdiff_t>(H_sparse->ncol) ||
         QR->QRsym->anz != static_cast<std::ptrdiff_t>(H_sparse->nzmax))) {
      SuiteSparseQR_free<double>(&QR, &cc);
      QR = nullptr;
    }
    // -- Factorize matrix symbolically
    if (QR == nullptr) {
      QR = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
                                          SPQR_DEFAULT_TOL,
                                          H_sparse,
                                          &cc);
      CHECK(QR != nullptr);
    }
    prof.stop("spqr-factorize-symbolically");

    // -- Factorize matrix numerically
    prof.start("spqr-factorize-numerically");
    const double qr_tol = std::numeric_limits<double>::epsilon();
    CHECK(SuiteSparseQR_numeric<double>(qr_tol, H_sparse, QR, &cc));
    prof.stop("spqr-factorize-numerically");

    // -- Solve
    prof.start("spqr-solve");
    // cholmod_dense *dx_dense = solve_qr(H_sparse, b_dense, &cholmod);
    auto Y = SuiteSparseQR_qmult<double>(SPQR_QTX, QR, b_dense, &cc);
    auto dx_dense = SuiteSparseQR_solve<double>(SPQR_RETX_EQUALS_B, QR, Y, &cc);

    // Clean up
    cholmod_l_free_dense(&Y, &cc);
    vecx_t dx;
    cholmod_convert(dx_dense, dx);
    cholmod_l_free_sparse(&H_sparse, &cc);
    cholmod_l_free_dense(&b_dense, &cc);
    cholmod_l_free_dense(&dx_dense, &cc);
    prof.stop("spqr-solve");

    // Profiling
    // printf("profile:\n");
    // prof.print("form_hessian");
    // prof.print("cholmod-sparsify");
    // prof.print("spqr-factorize-symbolically");
    // prof.print("spqr-factorize-numerically");
    // prof.print("spqr-solve");
    // printf("\n");

    // Solve: H dx = b
    // dx = linsolve_dense_chol(H, b);
    // dx = linsolve_dense_svd(H, b);
    // dx = linsolve_dense_qr(H, b);
    // const vecx_t dx = linsolve_sparse_qr(H, b);

    // Update
    _update(param_order, dx);

    // Calculate convergence and print stats
    cost_k = _calculate_cost();
    dcost = cost_k - cost_km1;
    if (verbose) {
      print_stats(iter, cost_k, dcost, lambda_k);
    }

    if (dcost < 0.0) {
      // Accept update
      cost_km1 = cost_k;
      lambda_k /= 10.0;
    } else {
      // Reject update
      _restore_params();
      cost_k = _calculate_cost();
      lambda_k *= 10.0;
    }

    // Limit lambda_k value
    // lambda_k = std::max(1e-10, lambda_k);
    // lambda_k = std::min(1e10, lambda_k);
  }

  // Clean up cholmod
  SuiteSparseQR_free(&QR, &cc);
  cholmod_l_finish(&cc);

  if (verbose) {
    const real_t cost_final = _calculate_cost();
    printf("Initial cost: %.4e\n", cost_init);
    printf("Final cost: %.4e\n", cost_final);
    printf("Num iterations: %d\n\n", iter);
  }
}

void yac_solver_t::solve(const int max_iter,
                         const bool verbose,
                         const int verbose_level) {
  printf("num residual blocks: %ld\n", res_fns.size());
  printf("num parameter blocks: %ld\n", params.size());
  printf("\n");

  tsolver.clear();
  if (algorithm_type == "LEVENBERG-MARQUARDT") {
    _solve_lm(max_iter, verbose, verbose_level);
  } else if (algorithm_type == "GAUSS-NEWTON") {
    _solve_gn(max_iter, verbose, verbose_level);
  } else {
    FATAL("algorithm_type: [%s] not implemented!", algorithm_type.c_str());
  }
}

// CERES-SOLVER ////////////////////////////////////////////////////////////////

ceres_solver_t::ceres_solver_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
}

ceres_solver_t::~ceres_solver_t() {
  delete problem;
  if (loss != nullptr) {
    delete loss;
  }
}

size_t ceres_solver_t::num_residuals() { return problem->NumResidualBlocks(); }

size_t ceres_solver_t::num_params() { return problem->NumParameterBlocks(); }

bool ceres_solver_t::has_param(param_t *param) {
  return problem->HasParameterBlock(param->data());
}

void ceres_solver_t::add_param(param_t *param) {
  // Add param
  solver_t::add_param(param);

  // Add param to ceres::Problem
  problem->AddParameterBlock(param->data(), param->global_size);

  // Fixed?
  if (param->fixed) {
    problem->SetParameterBlockConstant(param->data());
  }

  // If pose set appropriate parameterization
  auto is_pose = [](const param_t *param) {
    if (param->type == "pose_t") {
      return true;
    } else if (param->type == "extrinsics_t") {
      return true;
    } else if (param->type == "fiducial_t" && param->global_size == 7) {
      return true;
    }
    return false;
  };
  if (is_pose(param)) {
    problem->SetParameterization(param->data(), &pose_plus);
  }
}

void ceres_solver_t::add_residual(calib_residual_t *res_fn) {
  // Add residual
  solver_t::add_residual(res_fn);

  // Add residual to ceres::Problem
  auto param_blocks = res_fn->param_block_ptrs();
  auto res_id = problem->AddResidualBlock(res_fn, loss, param_blocks);
  res2id[res_fn] = res_id;
}

void ceres_solver_t::remove_param(param_t *param) {
  // Pre-check
  if (params.count(param->data()) == 0) {
    FATAL("Parameter block [%s] does not exist in problem!",
          param->type.c_str());
  }

  // Erase parameter
  params.erase(param->data());

  // Update param2res book-keeping
  if (param2res.count(param)) {
    const auto res_fns = param2res[param];
    for (auto res_fn : res_fns) {
      remove_residual(res_fn);
    }
    param2res.erase(param);
  }

  // Remove param from ceres::Problem
  problem->RemoveParameterBlock(param->data());
}

void ceres_solver_t::remove_residual(calib_residual_t *res_fn) {
  // Pre-check
  if (res_fns.count(res_fn) == 0) {
    FATAL("Residual block does not exist in res_fns!");
  }
  if (res2id.count(res_fn) == 0) {
    FATAL("Residual block does not exist in res2id!");
  }

  // Erase residual
  res_fns.erase(res_fn);

  // Update param2res
  for (auto param : res_fn->param_blocks) {
    param2res[param].erase(res_fn);
  }

  // Update res2param
  res2param.erase(res_fn);

  // Remove residual from ceres::Problem
  auto res_id = res2id[res_fn];
  problem->RemoveResidualBlock(res_id);
  res2id.erase(res_fn);
}

#ifdef ENABLE_CERES_COVARIANCE_ESTIMATOR

int ceres_solver_t::estimate_covariance(const std::vector<param_t *> &params,
                                        matx_t &covar) const {
  // Setup covariance blocks to estimate
  int covar_size = 0;
  std::map<param_t *, int> param_indices;
  std::vector<std::pair<const double *, const double *>> covar_blocks;
  // -- Determine parameter block order and covariance matrix size
  for (auto &param : params) {
    if (param->fixed) {
      continue;
    }
    param_indices[param] = covar_size;
    covar_size += param->local_size;
  }
  // -- Covariance blocks
  for (size_t i = 0; i < params.size(); i++) {
    for (size_t j = i; j < params.size(); j++) {
      auto param_i = params[i];
      auto param_j = params[j];
      covar_blocks.push_back({param_i->data(), param_j->data()});
    }
  }

  // Estimate covariance
  ::ceres::Covariance::Options options;
  ::ceres::Covariance covar_est(options);
  if (covar_est.Compute(covar_blocks, problem) == false) {
    LOG_ERROR("Failed to estimate covariance!");
    LOG_ERROR("Maybe Hessian is not full rank?");
    return -1;
  }
  // -- Form covariance matrix
  covar.resize(covar_size, covar_size);
  covar.setZero();

  for (size_t i = 0; i < params.size(); i++) {
    for (size_t j = i; j < params.size(); j++) {
      auto param_i = params[i];
      auto param_j = params[j];
      auto ptr_i = param_i->data();
      auto ptr_j = param_j->data();
      auto idx_i = param_indices[param_i];
      auto idx_j = param_indices[param_j];
      auto size_i = param_i->local_size;
      auto size_j = param_j->local_size;

      // Diagonal
      matx_row_major_t block;
      block.resize(size_i, size_j);
      block.setZero();
      auto data = block.data();
      bool retval;
      if ((size_i == 6 || size_j == 6)) {
        retval = covar_est.GetCovarianceBlockInTangentSpace(ptr_i, ptr_j, data);
      } else {
        retval = covar_est.GetCovarianceBlock(ptr_i, ptr_j, data);
      }
      if (retval == false) {
        LOG_ERROR("Failed to get covariance block for [%s-%s]",
                  param_i->type.c_str(),
                  param_j->type.c_str());
        return -1;
      }
      covar.block(idx_i, idx_j, size_i, size_j) = block;

      // Off-diagonal
      if (i != j) {
        covar.block(idx_j, idx_i, size_j, size_i) = block.transpose();
      }
    }
  }

  // Check if covar is full-rank?
  if (rank(covar) != covar.rows()) {
    LOG_ERROR("covar is not full rank!");
    return -1;
  }

  return 0;
}

int ceres_solver_t::estimate_log_covariance_determinant(
    const std::vector<param_t *> &params, real_t &covar_det) {
  // Estimate covariance
  matx_t covar;
  if (estimate_covariance(params, covar)) {
    return -1;
  }

  // Return covariance determinant: log(det(covar))
  covar_det = std::log(covar.determinant());

  return 0;
}

#endif // ENABLE_CERES_COVARIANCE_ESTIMATOR

void ceres_solver_t::solve(const int max_iter,
                           const bool verbose,
                           const int verbose_level) {
  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;

  options.initial_trust_region_radius = 10; // Default: 1e4
  options.min_trust_region_radius = 1e-50;  // Default: 1e-32
  options.function_tolerance = 1e-20;       // Default: 1e-6
  options.gradient_tolerance = 1e-20;       // Default: 1e-10
  options.parameter_tolerance = 1e-20;      // Default: 1e-8
  // options.preconditioner_type = ceres::IDENTITY; // Default: ceres::JACOBI

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  // Keep stats
  initial_cost = summary.initial_cost;
  final_cost = summary.final_cost;
  num_iterations = summary.num_successful_steps;

  // Print report
  if (verbose) {
    switch (verbose_level) {
      case 0:
        printf("num residual blocks: %d\n", problem->NumResidualBlocks());
        printf("num parameter blocks: %d\n", problem->NumParameterBlocks());
        printf("\n");
        std::cout << summary.BriefReport() << std::endl << std::endl;
        break;
      case 1:
        std::cout << summary.FullReport() << std::endl << std::endl;
        break;
      default:
        FATAL("Implementation Error!");
        break;
    }
  }
}

} // namespace yac
