#include "solver.hpp"

namespace yac {

// SOLVER BASE /////////////////////////////////////////////////////////////////

size_t solver_t::num_residuals() { return cost_fns.size(); }

bool solver_t::has_param(param_t *param) { return params.count(param->data()); }

size_t solver_t::num_params() { return params.size(); }

void solver_t::add_param(param_t *param) { params[param->data()] = param; }

void solver_t::add_residual(calib_error_t *cost_fn) {
  // Pre-check
  if (cost_fns.count(cost_fn)) {
    printf("residual: %s, addr: %p\n", cost_fn->type.c_str(), cost_fn);
    FATAL("Residual already exists in problem!");
  }

  // Insert
  cost_fns.insert(cost_fn);

  // Update param2res
  for (auto param : cost_fn->param_blocks) {
    param2res[param].insert(cost_fn);
  }

  // Update res2param
  for (auto param : cost_fn->param_blocks) {
    res2param[cost_fn].insert(param);
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
    for (auto res : param2res[param]) {
      remove_residual(res);
    }
    param2res.erase(param);
  }
}

void solver_t::remove_residual(calib_error_t *cost_fn) {
  // Pre-check
  if (cost_fns.count(cost_fn) == 0) {
    FATAL("Residual block does not exist in problem!");
  }

  // Erase residual
  cost_fns.erase(cost_fn);

  // Update param2res
  for (auto param : cost_fn->param_blocks) {
    param2res[param].erase(cost_fn);
  }

  // Update res2param
  res2param.erase(cost_fn);
}

int solver_t::estimate_covariance(const std::vector<param_t *> params,
                                  matx_t &calib_covar,
                                  const bool verbose) const {
  // // Track parameters
  // std::vector<calib_error_t *> res_blocks;
  // std::map<param_t *, bool> params_seen;
  // std::vector<param_t *> marg_param_ptrs;
  // std::vector<param_t *> remain_camera_param_ptrs;
  // std::vector<param_t *> remain_extrinsics_ptrs;
  // std::vector<param_t *> remain_fiducial_ptrs;
  // for (auto &res_block : cost_fns) {
  //   for (auto param_block : res_block->param_blocks) {
  //     // Seen parameter block already
  //     if (params_seen.count(param_block)) {
  //       continue;
  //     }
  //
  //     // Keep track of parameter block
  //     if (param_block->type == "camera_params_t") {
  //       remain_camera_param_ptrs.push_back(param_block);
  //     } else if (param_block->type == "extrinsics_t") {
  //       remain_extrinsics_ptrs.push_back(param_block);
  //     } else if (param_block->type == "fiducial_t") {
  //       remain_fiducial_ptrs.push_back(param_block);
  //     } else {
  //       marg_param_ptrs.push_back(param_block);
  //     }
  //     params_seen[param_block] = true;
  //   }
  //   res_blocks.push_back(res_block);
  // }
  //
  // // Determine parameter block column indicies for Hessian matrix H
  // size_t m = 0;
  // size_t r = 0;
  // size_t H_idx = 0;
  // std::map<param_t *, int> param_index;
  // // -- Column indices for parameter blocks to be marginalized
  // for (const auto &param_block : marg_param_ptrs) {
  //   param_index.insert({param_block, H_idx});
  //   H_idx += param_block->local_size;
  //   m += param_block->local_size;
  // }
  // // -- Column indices for parameter blocks to remain
  // std::vector<std::vector<param_t *> *> remain_params = {
  //     &remain_fiducial_ptrs,
  //     &remain_extrinsics_ptrs,
  //     &remain_camera_param_ptrs,
  // };
  // std::vector<param_t *> remain_param_ptrs;
  // for (const auto &param_ptrs : remain_params) {
  //   for (const auto &param_block : *param_ptrs) {
  //     if (param_block->fixed) {
  //       continue;
  //     }
  //     param_index.insert({param_block, H_idx});
  //     H_idx += param_block->local_size;
  //     r += param_block->local_size;
  //     remain_param_ptrs.push_back(param_block);
  //   }
  // }
  //
  // // Form the H and b. Left and RHS of Gauss-Newton.
  // // H = J.T * J
  // // b = -J.T * e
  // const auto local_size = m + r;
  // matx_t H = zeros(local_size, local_size);
  // vecx_t b = zeros(local_size, 1);
  //
  // for (calib_error_t *res_block : res_blocks) {
  //   // Setup parameter data
  //   std::vector<double *> param_ptrs;
  //   for (auto param_block : res_block->param_blocks) {
  //     param_ptrs.push_back(param_block->data());
  //   }
  //
  //   // Setup Jacobians data
  //   const int r_size = res_block->num_residuals();
  //   std::vector<matx_row_major_t> jacs;
  //   std::vector<double *> jac_ptrs;
  //   for (auto param_block : res_block->param_blocks) {
  //     jacs.push_back(zeros(r_size, param_block->global_size));
  //     jac_ptrs.push_back(jacs.back().data());
  //   }
  //
  //   // Setup Min-Jacobians data
  //   std::vector<matx_row_major_t> min_jacs;
  //   std::vector<double *> min_jac_ptrs;
  //   for (auto param_block : res_block->param_blocks) {
  //     min_jacs.push_back(zeros(r_size, param_block->local_size));
  //     min_jac_ptrs.push_back(min_jacs.back().data());
  //   }
  //
  //   // Evaluate residual block
  //   vecx_t r = zeros(r_size, 1);
  //   res_block->EvaluateWithMinimalJacobians(param_ptrs.data(),
  //                                           r.data(),
  //                                           jac_ptrs.data(),
  //                                           min_jac_ptrs.data());
  //
  //   // Fill Hessian
  //   for (size_t i = 0; i < res_block->param_blocks.size(); i++) {
  //     const auto &param_i = res_block->param_blocks[i];
  //     if (param_i->fixed) {
  //       continue;
  //     }
  //     const int idx_i = param_index[param_i];
  //     const int size_i = param_i->local_size;
  //     const matx_t J_i = min_jacs[i];
  //
  //     for (size_t j = i; j < res_block->param_blocks.size(); j++) {
  //       const auto &param_j = res_block->param_blocks[j];
  //       if (param_j->fixed) {
  //         continue;
  //       }
  //       const int idx_j = param_index[param_j];
  //       const int size_j = param_j->local_size;
  //       const matx_t J_j = min_jacs[j];
  //
  //       if (i == j) {
  //         // Form diagonals of H
  //         H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
  //       } else {
  //         // Form off-diagonals of H
  //         // clang-format off
  //         H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
  //         H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() *
  //         J_j).transpose();
  //         // clang-format on
  //       }
  //     }
  //
  //     // RHS of Gauss Newton (i.e. vector b)
  //     b.segment(idx_i, size_i) += -J_i.transpose() * r;
  //   }
  // }
  //
  // // Marginalize
  // // -- Pseudo inverse of Hmm via Eigen-decomposition:
  // //
  // //   A_pinv = V * Lambda_pinv * V_transpose
  // //
  // // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // // entry by its reciprocal, leaving the zeros in place, and transposing
  // // the resulting matrix.
  // // clang-format off
  // matx_t Hmm = H.block(0, 0, m, m);
  // Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  // const double eps = 1.0e-8;
  // const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  // const matx_t V = eig.eigenvectors();
  // const auto eigvals_inv = (eig.eigenvalues().array() >
  // eps).select(eig.eigenvalues().array().inverse(), 0); const matx_t
  // Lambda_inv = vecx_t(eigvals_inv).asDiagonal(); const matx_t Hmm_inv = V *
  // Lambda_inv * V.transpose();
  // // clang-format on
  // // -- Calculate Schur's complement
  // const matx_t Hmr = H.block(0, m, m, r);
  // const matx_t Hrm = H.block(m, 0, r, m);
  // const matx_t Hrr = H.block(m, m, r, r);
  // const vecx_t bmm = b.segment(0, m);
  // const vecx_t brr = b.segment(m, r);
  // const matx_t H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  // const matx_t b_marg = brr - Hrm * Hmm_inv * bmm;
  //
  // // Convert
  // calib_covar = H_marg.inverse();

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

void ceres_solver_t::add_residual(calib_error_t *cost_fn) {
  // Add residual
  solver_t::add_residual(cost_fn);

  // Add residual to ceres::Problem
  auto param_blocks = cost_fn->param_block_ptrs();
  auto res_id = problem->AddResidualBlock(cost_fn, nullptr, param_blocks);
  res2id[cost_fn] = res_id;
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

void ceres_solver_t::remove_residual(calib_error_t *cost_fn) {
  // Pre-check
  if (cost_fns.count(cost_fn) == 0) {
    FATAL("Residual block does not exist in cost_fns!");
  }
  if (res2id.count(cost_fn) == 0) {
    FATAL("Residual block does not exist in res2id!");
  }

  // Erase residual
  cost_fns.erase(cost_fn);

  // Update param2res
  for (auto param : cost_fn->param_blocks) {
    param2res[param].erase(cost_fn);
  }

  // Update res2param
  res2param.erase(cost_fn);

  // Remove residual from ceres::Problem
  auto res_id = res2id[cost_fn];
  problem->RemoveResidualBlock(res_id);
  res2id.erase(cost_fn);
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

bool yac_solver_t::_eval_residual(calib_error_t *res_fn,
                                  ResidualJacobians &res_jacs,
                                  ResidualJacobians &res_min_jacs,
                                  ResidualValues &res_vals) {
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

void yac_solver_t::_solve_linear_system(const matx_t &J,
                                        const vecx_t &b,
                                        vecx_t &dx) {
  const matx_t H = J.transpose() * J;
  dx = H.ldlt().solve(b);
}

void yac_solver_t::_linearize(ParameterOrder &param_index,
                              matx_t &J,
                              vecx_t &b) {
  // Evaluate residuals
  ResidualJacobians res_jacs;
  ResidualJacobians res_min_jacs;
  ResidualValues res_vals;
  std::vector<calib_error_t *> good_res_fns;
  size_t residuals_length = 0;

  for (calib_error_t *res_fn : cost_fns) {
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
      params_seen[param_block] = true;
      params_length += res_fn->num_residuals();
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
  param_index.clear();
  for (const auto &param_ptrs : param_groups) {
    for (const auto &param_block : *param_ptrs) {
      if (param_block->fixed) {
        continue;
      }
      param_index.insert({param_block, idx});
      idx += param_block->local_size;
    }
  }

  // Form Jacobian J and b
  J = zeros(residuals_length, params_length);
  b = zeros(params_length, 1);
  size_t res_idx = 0;

  for (calib_error_t *res_fn : good_res_fns) {
    const vecx_t r = res_vals[res_fn];

    for (size_t i = 0; i < res_fn->param_blocks.size(); i++) {
      const auto &param_i = res_fn->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }

      const int idx_i = param_index[param_i];
      const int size_i = param_i->local_size;
      const matx_t J_i = res_min_jacs[res_fn][i];
      J.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
      b.segment(idx_i, size_i) += -J_i.transpose() * r;
    }

    res_idx += res_fn->num_residuals();
  }
}

void yac_solver_t::_update(const ParameterOrder &param_index,
                           const vecx_t &dx) {
  for (auto &[param, idx] : param_index) {
    param->plus(dx.segment(idx, param->local_size));
  }
}

void yac_solver_t::solve(const int max_iter,
                         const bool verbose,
                         const int verbose_level) {
  for (int iter = 0; iter < max_iter; iter++) {
    // Linearize system
    ParameterOrder param_index;
    matx_t J;
    vecx_t b;
    _linearize(param_index, J, b);

    // Solve linear system
    vecx_t dx;
    _solve_linear_system(J, b, dx);

    // Update
    _update(param_index, dx);
  }
}

} // namespace yac
