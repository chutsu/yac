#include "solver.hpp"

namespace yac {

// SOLVER BASE /////////////////////////////////////////////////////////////////

void solver_base_t::add_param(param_t *param) { params[param->data()] = param; }

void solver_base_t::add_residual(calib_error_t *cost_fn) {
  // Pre-check
  if (cost_fns.count(cost_fn) == 0) {
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

void solver_base_t::remove_param(param_t *param) {
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

void solver_base_t::remove_residual(calib_error_t *cost_fn) {
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

// CERES-SOLVER ////////////////////////////////////////////////////////////////

ceres_solver_t::ceres_solver_t() {
  prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = new ceres::Problem(prob_options);
}

ceres_solver_t::~ceres_solver_t() { delete problem; }

void ceres_solver_t::add_param(param_t *param) {
  // // Add param
  // solver_base_t::add_param(param);

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
  // // Add residual
  // solver_base_t::add_residual(cost_fn);

  // Add residual to ceres::Problem
  auto param_blocks = cost_fn->param_block_ptrs();
  auto res_id = problem->AddResidualBlock(cost_fn, nullptr, param_blocks);
  res2id[cost_fn] = res_id;
}

void ceres_solver_t::remove_param(param_t *param) {
  // // Remove param
  // solver_base_t::remove_param(param);

  // Remove param from ceres::Problem
  problem->RemoveParameterBlock(param->data());
}

void ceres_solver_t::remove_residual(calib_error_t *cost_fn) {
  // // Remove residual
  // solver_base_t::remove_residual(cost_fn);

  // Remove residual from ceres::Problem
  auto res_id = res2id[cost_fn];
  problem->RemoveResidualBlock(res_id);
}

void ceres_solver_t::solve(const int max_iter,
                           const bool verbose,
                           const int verbose_level) {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

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

void solver_t::linearize(matx_t &J, vecx_t &b) {
  // Track parameters
  std::map<param_t *, bool> params_seen;
  std::vector<param_t *> pose_ptrs;
  std::vector<param_t *> sb_ptrs;
  std::vector<param_t *> cam_ptrs;
  std::vector<param_t *> extrinsics_ptrs;
  std::vector<param_t *> fiducial_ptrs;
  for (auto &cost_fn : cost_fns) {
    for (auto param_block : cost_fn->param_blocks) {
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
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t idx = 0;
  std::map<param_t *, int> param_index;
  std::vector<std::vector<param_t *> *> param_groups = {
      &pose_ptrs,
      &sb_ptrs,
      &cam_ptrs,
      &extrinsics_ptrs,
      &fiducial_ptrs,
  };
  for (const auto &param_ptrs : param_groups) {
    for (const auto &param_block : *param_ptrs) {
      if (param_block->fixed) {
        continue;
      }
      param_index.insert({param_block, idx});
      idx += param_block->local_size;
    }
  }

  // Form Jacobian J and b. Left and RHS of Gauss-Newton.
  //   const auto local_size = m + r;
  //   matx_t H = zeros(local_size, local_size);
  //   vecx_t b = zeros(local_size, 1);
  //
  //   for (calib_error_t *res_block : res_blocks) {
  //     // Setup parameter data
  //     std::vector<double *> param_ptrs;
  //     for (auto param_block : res_block->param_blocks) {
  //       param_ptrs.push_back(param_block->data());
  //     }
  //
  //     // Setup Jacobians data
  //     const int r_size = res_block->num_residuals();
  //     std::vector<matx_row_major_t> jacs;
  //     std::vector<double *> jac_ptrs;
  //     for (auto param_block : res_block->param_blocks) {
  //       jacs.push_back(zeros(r_size, param_block->global_size));
  //       jac_ptrs.push_back(jacs.back().data());
  //     }
  //
  //     // Setup Min-Jacobians data
  //     std::vector<matx_row_major_t> min_jacs;
  //     std::vector<double *> min_jac_ptrs;
  //     for (auto param_block : res_block->param_blocks) {
  //       min_jacs.push_back(zeros(r_size, param_block->local_size));
  //       min_jac_ptrs.push_back(min_jacs.back().data());
  //     }
  //
  //     // Evaluate residual block
  //     vecx_t r = zeros(r_size, 1);
  //     res_block->EvaluateWithMinimalJacobians(param_ptrs.data(),
  //                                             r.data(),
  //                                             jac_ptrs.data(),
  //                                             min_jac_ptrs.data());
  //
  //     // Fill Hessian
  //     for (size_t i = 0; i < res_block->param_blocks.size(); i++) {
  //       const auto &param_i = res_block->param_blocks[i];
  //       if (param_i->fixed) {
  //         continue;
  //       }
  //       const int idx_i = param_index[param_i];
  //       const int size_i = param_i->local_size;
  //       const matx_t J_i = min_jacs[i];
  //
  //       for (size_t j = i; j < res_block->param_blocks.size(); j++) {
  //         const auto &param_j = res_block->param_blocks[j];
  //         if (param_j->fixed) {
  //           continue;
  //         }
  //         const int idx_j = param_index[param_j];
  //         const int size_j = param_j->local_size;
  //         const matx_t J_j = min_jacs[j];
  //
  //         if (i == j) {
  //           // Form diagonals of H
  //           H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
  //         } else {
  //           // Form off-diagonals of H
  //           // clang-format off
  //         H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
  //         H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() *
  //         J_j).transpose();
  //           // clang-format on
  //         }
  //       }
  //
  //       // RHS of Gauss Newton (i.e. vector b)
  //       b.segment(idx_i, size_i) += -J_i.transpose() * r;
  //     }
  //   }
}

void solver_t::solve(const int max_iter,
                     const bool verbose,
                     const int verbose_level) {}

} // namespace yac
