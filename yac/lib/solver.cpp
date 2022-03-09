#include "solver.hpp"

namespace yac {

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

int solver_t::estimate_covariance(const std::vector<param_t *> params,
                                  matx_t &calib_covar,
                                  const bool verbose) const {
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
    const vecx_t r = res_vals[res_fn];

    // Fill Hessian
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
  const auto eigvals_inv = (eig.eigenvalues().array() >
  eps).select(eig.eigenvalues().array().inverse(), 0); const matx_t
  Lambda_inv = vecx_t(eigvals_inv).asDiagonal(); const matx_t Hmm_inv = V *
  Lambda_inv * V.transpose();
  // clang-format on
  // -- Calculate Schur's complement
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const matx_t H_marg = Hrr - Hrm * Hmm_inv * Hmr;

  // Convert
  calib_covar = H_marg.inverse();

  return 0;
}

// CERES-SOLVER ////////////////////////////////////////////////////////////////

ceres_solver_t::ceres_solver_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
}

ceres_solver_t::~ceres_solver_t() { delete problem; }

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
  auto res_id = problem->AddResidualBlock(res_fn, nullptr, param_blocks);
  res2id[res_fn] = res_id;
}

void ceres_solver_t::remove_param(param_t *param) {
  // Pre-check
  if (params.count(param->data()) == 0) {
    FATAL("Parameter block does not exist in problem!");
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

int ceres_solver_t::estimate_covariance(const std::vector<param_t *> params,
                                        matx_t &covar,
                                        bool verbose) const {
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
    if (verbose) {
      LOG_ERROR("Failed to estimate covariance!");
      LOG_ERROR("Maybe Hessian is not full rank?");
    }
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
      int retval = 0;
      if ((size_i == 6 || size_j == 6)) {
        retval = covar_est.GetCovarianceBlockInTangentSpace(ptr_i, ptr_j, data);
      } else {
        retval = covar_est.GetCovarianceBlock(ptr_i, ptr_j, data);
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
    if (verbose) {
      LOG_ERROR("covar is not full rank!");
    }
    return -1;
  }

  return 0;
}

void ceres_solver_t::solve(const int max_iter,
                           const bool verbose,
                           const int verbose_level) {
  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;
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
        std::cout << summary.BriefReport() << std::endl;
        break;
      case 1:
        std::cout << summary.FullReport() << std::endl;
        break;
      default:
        FATAL("Implementation Error!");
        break;
    }
  }
}

// SOLVER /////////////////////////////////////////////////////////////////////

void yac_solver_t::_form_jacobian(ParameterOrder &param_order,
                                  matx_t &J,
                                  vecx_t &r) {
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
  size_t params_length = 0;

  for (auto res_fn : good_res_fns) {
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

  // Form Jacobian J and r
  J = zeros(residuals_length, params_length);
  r = zeros(residuals_length, 1);
  size_t res_idx = 0;

  for (calib_residual_t *res_fn : good_res_fns) {
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

void yac_solver_t::_form_hessian(ParameterOrder &param_order,
                                 matx_t &H,
                                 vecx_t &b) {
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

  // Calculate cost
  real_t cost = 0.0;
  for (const auto &[res_fn, r] : res_vals) {
    cost += 0.5 * r.transpose() * r;
  }

  // Track unique parameters
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> sb_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  std::vector<param_t *> fiducial_ptrs;
  size_t params_length = 0;

  for (auto res_fn : good_res_fns) {
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

  // Form Jacobian J and b
  H = zeros(params_length, params_length);
  b = zeros(params_length, 1);
  size_t res_idx = 0;

  for (calib_residual_t *res_fn : good_res_fns) {
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
}

static vecx_t linsolve_dense_cholesky(const matx_t &A, const vecx_t &b) {
  return A.ldlt().solve(b);
}

static vecx_t linsolve_dense_svd(const matx_t &A, const vecx_t &b) {
  Eigen::JacobiSVD<matx_t> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const matx_t U = svd.matrixU();
  const matx_t V = svd.matrixV();
  const vecx_t sv = svd.singularValues();

  // const double eps = std::numeric_limits<double>::epsilon();
  // const double tol = sv(0) * eps * sv.size();
  // size_t rank = sv.size();
  // for (size_t i = sv.size() - 1; i >= 0; --i) {
  //   if (sv(i) > tol) {
  //     break;
  //   } else {
  //     --rank;
  //   }
  // }
  // printf("eps: %e, tol: %e\n", eps, tol);
  // printf("rank: %d, A_shape: %ldx%ld\n", rank, A.rows(), A.cols());

  // const vecx_t dx = V.leftCols(rank) * sv.head(rank).asDiagonal().inverse() *
  //                   U.leftCols(rank).adjoint() * -b;
  // const vecx_t dx = V * sv.asDiagonal().inverse() * U.transpose() * b;
  const vecx_t dx = svd.solve(b);

  return dx;
}

static vecx_t linsolve_dense_qr(const matx_t &A, const vecx_t &b) {
  Eigen::HouseholderQR<matx_t> qr(A);
  return qr.solve(b);
}

static vecx_t linsolve_sparse_qr(const matx_t &A, const vecx_t &b) {
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

static vecx_t truncated_sparse_qr(const matx_t &A, const vecx_t &b) {
  // clang-format off
  profiler_t prof;
  prof.start("SparseQR.solve()");
  Eigen::SparseQR<Eigen::SparseMatrix<real_t>, Eigen::COLAMDOrdering<int>> solver;

  // -- Decompose
  solver.setPivotThreshold(1.0e-5);
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

  // prof.print("H.sparse_view()");
  // prof.print("SparseQR.solve()");
  // printf("\n");

  return x;
}

void yac_solver_t::_solve_linear_system(const real_t lambda_k,
                                        ParameterOrder &param_order,
                                        vecx_t &dx) {
  // matx_t J;
  // vecx_t r;
  // _form_jacobian(param_order, J, r);
  // dx = linsolve_dense_svd(J, r);

  matx_t H;
  vecx_t b;
  _form_hessian(param_order, H, b);
  H = H + lambda_k * I(H.rows());
  // dx = linsolve_dense_chol(H, b);
  // dx = linsolve_dense_svd(H, b);
  // dx = linsolve_dense_qr(H, b);
  // dx = linsolve_sparse_qr(H, b);
  dx = truncated_sparse_qr(H, b);
}

real_t yac_solver_t::_calculate_cost() {
  real_t cost = 0.0;
  for (calib_residual_t *res_fn : res_fns) {
    vecx_t r;
    _eval_residual(res_fn, r);
    cost += 0.5 * r.transpose() * r;
  }

  return cost;
}

void yac_solver_t::_update(const ParameterOrder &param_order,
                           const vecx_t &dx) {
  for (auto &[param, idx] : param_order) {
    param->plus(dx.segment(idx, param->local_size));
  }
}

void yac_solver_t::_solve_gn(const int max_iter,
                             const bool verbose,
                             const int verbose_level) {
  // Setup
  real_t lambda_k = lambda;
  ParameterOrder param_order;
  vecx_t dx;

  // Lambda function to print status
  auto print_stats =
      [&](const int iter, const real_t cost, const real_t dcost) {
        printf("iter: %d, ", iter);
        printf("cost: %e, ", cost);
        printf("dcost: %.2e\n", dcost);
      };

  // Linearize system
  int iter = 1;
  const real_t cost_init = _calculate_cost();
  real_t cost_km1 = cost_init;
  real_t cost_k = 0.0;
  real_t dcost = 0.0;
  if (verbose && verbose_level == 1) {
    print_stats(0, cost_km1, dcost);
  }

  for (iter = 1; iter < max_iter; iter++) {
    _cache_params();
    _solve_linear_system(1e-4, param_order, dx);
    _update(param_order, dx);

    cost_k = _calculate_cost();
    dcost = cost_k - cost_km1;
    cost_km1 = cost_k;

    if (verbose && verbose_level == 1) {
      print_stats(iter, cost_k, dcost);
    }
  }

  if (verbose && verbose_level == 1) {
    const real_t cost_final = _calculate_cost();
    printf("Initial cost: %.4e\n", cost_init);
    printf("Final cost: %.4e\n", cost_final);
    printf("Num iterations: %d\n\n", iter);
  }
}

void yac_solver_t::_solve_lm(const int max_iter,
                             const bool verbose,
                             const int verbose_level) {
  // Setup
  real_t lambda_k = lambda;
  ParameterOrder param_order;
  vecx_t dx;

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

  // Linearize system
  int iter = 0;
  const real_t cost_init = _calculate_cost();
  real_t cost_km1 = cost_init;
  real_t cost_k = 0.0;
  real_t dcost = 0.0;
  if (verbose && verbose_level == 1) {
    print_stats(0, cost_km1, dcost, lambda_k);
  }

  for (iter = 1; iter < max_iter; iter++) {
    _cache_params();
    _solve_linear_system(lambda_k, param_order, dx);
    _update(param_order, dx);

    cost_k = _calculate_cost();
    dcost = cost_k - cost_km1;
    if (verbose && verbose_level == 1) {
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
  }

  if (verbose && verbose_level == 1) {
    const real_t cost_final = _calculate_cost();
    printf("Initial cost: %.4e\n", cost_init);
    printf("Final cost: %.4e\n", cost_final);
    printf("Num iterations: %d\n\n", iter);
  }
}

void yac_solver_t::solve(const int max_iter,
                         const bool verbose,
                         const int verbose_level) {
  _solve_gn(max_iter, verbose, verbose_level);
  // _solve_lm(max_iter, verbose, verbose_level);
}

} // namespace yac
