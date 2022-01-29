#ifndef YAC_MARG_ERROR_HPP
#define YAC_MARG_ERROR_HPP

#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_params.hpp"

namespace yac {

/* Residual Information */
struct residual_info_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const ceres::CostFunction *cost_fn = nullptr;
  const ceres::LossFunction *loss_fn = nullptr;
  std::vector<param_t *> param_blocks;
  std::vector<double *> param_blocks_raw;

  vecx_t residuals;
  std::vector<matx_row_major_t> jacobians;

  residual_info_t(const ceres::CostFunction *cost_fn_,
                  const std::vector<param_t *> &param_blocks_,
                  const ::ceres::LossFunction *loss_fn_ = nullptr)
      : cost_fn{cost_fn_}, loss_fn{loss_fn_}, param_blocks{param_blocks_} {
    // Pre-allocate memory for residuals
    residuals.resize(cost_fn_->num_residuals());

    // Pre-allocate memory for jacobians
    jacobians.resize(param_blocks.size());
    double **jacobian_ptrs = new double *[param_blocks.size()];
    for (size_t i = 0; i < param_blocks_.size(); i++) {
      const auto &param = param_blocks_[i];
      param_blocks_raw.push_back(param->param.data());
      jacobians[i].resize(residuals.size(), param->global_size);
      jacobian_ptrs[i] = jacobians[i].data();
    }

    // Evaluate cost function to obtain the jacobians and residuals
    cost_fn->Evaluate(param_blocks_raw.data(), residuals.data(), jacobian_ptrs);
    free(jacobian_ptrs);

    // Scale jacobians and residuals if using loss function
    // following ceres-sovler in `internal/ceres/corrector.cc`
    if (loss_fn_) {
      double residual_scaling = 0.0;
      double alpha_sq_norm = 0.0;
      double rho[3] = {0.0};

      double sq_norm = residuals.squaredNorm();
      loss_fn_->Evaluate(sq_norm, rho);
      double sqrt_rho1 = sqrt(rho[1]);

      if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
        residual_scaling = sqrt_rho1;
        alpha_sq_norm = 0.0;
      } else {
        const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
        const double alpha = 1.0 - sqrt(D);
        residual_scaling = sqrt_rho1 / (1 - alpha);
        alpha_sq_norm = alpha / sq_norm;
      }

      for (size_t i = 0; i < param_blocks.size(); i++) {
        // clang-format off
        jacobians[i] = sqrt_rho1 * (
          jacobians[i] - alpha_sq_norm * residuals *
          (residuals.transpose() * jacobians[i])
        );
        // clang-format on
      }

      residuals *= residual_scaling;
    }
  }
};

// class marg_error_t : public ::ceres::CostFunction {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   bool marginalized_ = false;
//
//   // Residual blocks and parameters involved for marginalization
//   std::vector<residual_info_t> blocks_;
//   std::unordered_map<param_t *, int> param_blocks_;
//   std::unordered_map<param_t *, int> param_index_;
//
//   size_t m_ = 0;                             // Size of params to be
//   marginalized size_t r_ = 0;                             // Size of params
//   to remain std::vector<param_t *> marg_param_ptrs_;   // Marg param block
//   pointers std::vector<param_t *> remain_param_ptrs_; // Remain param block
//   pointers
//
//   std::unordered_map<double *, vecx_t> x0_;  // Linearization point x0
//   vecx_t e0_;                                // Linearized residuals at x0
//   matx_t J0_;                                // Linearized jacobians at x0
//
//   void add(const ceres::CostFunction *cost_fn,
//            const std::vector<param_t *> param_blocks,
//            const ::ceres::LossFunction *loss_fn=nullptr) {
//     // Check pointer to cost function
//     if (cost_fn == nullptr) {
//       FATAL("cost_fn == nullptr!");
//     }
//
//     // Check number of param blocks matches what the cost function is
//     expecting const auto param_sizes = cost_fn->parameter_block_sizes(); if
//     (param_blocks.size() != param_sizes.size()) {
//       FATAL("param_blocks.size() != cost_fn->num_params!");
//     }
//
//     // Make sure param blocks are valid
//     for (size_t i = 0; i < param_blocks.size(); i++) {
//       const auto &param = param_blocks[i];
//       const auto &param_size = param_sizes[i];
//       if (param == nullptr) {
//         FATAL("Param block [%ld] is NULL! Implementation Error!", i);
//       }
//
//       if (param->global_size != param_size) {
//         FATAL("Param block [%ld] is not what the cost function expected!",
//         i);
//       }
//     }
//
//     // Keep track
//     blocks_.emplace_back(cost_fn, param_blocks, loss_fn);
//   }
//
//   void form_hessian(matx_t &H, vecx_t &b, bool debug=false) {
//     // Reset marginalization error parameter block sizes
//     mutable_parameter_block_sizes()->clear();
//
//     // Determine parameter block column indicies for matrix H
//     size_t index = 0; // Column index of matrix H
//     // -- Column indices for parameter blocks to be marginalized
//     for (const auto &param_block : marg_param_ptrs_) {
//       param_index_.insert({param_block, index});
//       index += param_block->local_size;
//       m_ += param_block->local_size;
//     }
//     // -- Column indices for parameter blocks to remain
//     for (const auto &param_block : remain_param_ptrs_) {
//       if (param_block->fixed) {
//         continue;
//       }
//
//       param_index_.insert({param_block, index});
//       index += param_block->local_size;
//       r_ += param_block->local_size;
//
//       // !!!!!!!!!!!!!!!!!!!!!!!! VERY IMPORTANT
//       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//       // Update param blocks and sizes for ceres::CostFunction
//       mutable_parameter_block_sizes()->push_back(param_block->global_size);
//     }
//
//     // !!!!!!!!!!!!!!!!!!!!!!!!!! VERY
//     IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     // Now we know the Hessian size, we can update the number of residuals
//     for
//     // ceres::CostFunction
//     set_num_residuals(r_);
//
//     // Form the H and b. Left and RHS of Gauss-Newton.
//     const auto local_size = m_ + r_;
//     H = zeros(local_size, local_size);
//     b = zeros(local_size);
//     if (debug) {
//       printf("m: %zu\n", m_);
//       printf("r: %zu\n", r_);
//       printf("H shape: %zu x %zu\n", H.rows(), H.cols());
//       printf("b shape: %zu x %zu\n", b.rows(), b.cols());
//     }
//
//     for (const auto &block : blocks_) {
//       for (size_t i = 0; i < block.param_blocks.size(); i++) {
//         const auto &param_i = block.param_blocks[i];
//         const int idx_i = param_index_[param_i];
//         const int size_i = param_i->local_size;
//         const matx_t J_i = block.jacobians[i].leftCols(size_i);
//
//         for (size_t j = i; j < block.param_blocks.size(); j++) {
//           const auto &param_j = block.param_blocks[j];
//           const int idx_j = param_index_[param_j];
//           const int size_j = param_j->local_size;
//           const matx_t J_j = block.jacobians[j].leftCols(size_j);
//
//           if (i == j) {
//             // Form diagonals of H
//             H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
//           } else {
//             // Form off-diagonals of H
//             H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
//             H.block(idx_j, idx_i, size_j, size_i) =
//               H.block(idx_i, idx_j, size_i, size_j).transpose();
//           }
//         }
//
//         // RHS of Gauss Newton (i.e. vector b)
//         b.segment(idx_i, size_i) += -J_i.transpose() * block.residuals;
//         // TODO: Double check the signs here^ with the -1?
//       }
//     }
//   }
//
//   void schurs_complement(const matx_t &H,
//                          const vecx_t &b,
//                          const size_t m,
//                          const size_t r,
//                          matx_t &H_marg,
//                          vecx_t &b_marg,
//                          const bool precond,
//                          const bool debug) {
//     assert(m > 0 && r > 0);
//
//     // Setup
//     const long local_size = m + r;
//     H_marg = zeros(local_size, local_size);
//     b_marg = zeros(local_size);
//
//     // Precondition Hmm
//     matx_t Hmm = H.block(0, 0, m, m);
//     if (precond) {
//       Hmm = 0.5 * (Hmm + Hmm.transpose());
//     }
//
//     // Pseudo inverse of Hmm via Eigen-decomposition:
//     //
//     //   A_pinv = V * Lambda_pinv * V_transpose
//     //
//     // Where Lambda_pinv is formed by **replacing every non-zero diagonal
//     entry
//     // by its reciprocal, leaving the zeros in place, and transposing the
//     // resulting matrix.
//     //
//     // clang-format off
//     const double eps = 1.0e-8;
//     const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
//     const matx_t V = eig.eigenvectors();
//     const auto eigvals = eig.eigenvalues().array();
//     const auto eigvals_inv = (eigvals > eps).select(eigvals.inverse(), 0);
//     const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
//     const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
//     // clang-format on
//
//     // Calculate Schur's complement
//     const matx_t Hmr = H.block(0, m, m, r);
//     const matx_t Hrm = H.block(m, 0, r, m);
//     const matx_t Hrr = H.block(m, m, r, r);
//     const vecx_t bmm = b.segment(0, m);
//     const vecx_t brr = b.segment(m, r);
//     H_marg = Hrr - Hrm * Hmm_inv * Hmr;
//     b_marg = brr - Hrm * Hmm_inv * bmm;
//     const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
//     if (fabs(inv_check) > 1e-4) {
//       LOG_ERROR("FAILED!: Inverse identity check: %f", inv_check);
//     }
//
//     if (debug) {
//       mat2csv("/tmp/H.csv", H);
//       mat2csv("/tmp/Hmm.csv", Hmm);
//       mat2csv("/tmp/Hmr.csv", Hmr);
//       mat2csv("/tmp/Hrm.csv", Hrm);
//       mat2csv("/tmp/Hrr.csv", Hrr);
//       mat2csv("/tmp/bmm.csv", bmm);
//       mat2csv("/tmp/brr.csv", brr);
//       mat2csv("/tmp/H_marg.csv", H_marg);
//       mat2csv("/tmp/b_marg.csv", b_marg);
//     }
//   }
//
//   void marginalize(ceres::Problem &problem, bool debug=false) {
//     // Keep track of:
//     // - Pointers to parameter blocks for marginalization.
//     // - Pointers to parameter blocks to remain.
//     for (const auto &block : blocks_) {
//       for (size_t i = 0; i < block.param_blocks.size(); i++) {
//         const auto &param = block.param_blocks[i];
//
//         if (param_blocks_.find(param) == param_blocks_.end()) {
//           if (param->marginalize) {
//             marg_param_ptrs_.push_back(param);
//           } else if (param->fixed == false) {
//             remain_param_ptrs_.push_back(param);
//           }
//           param_blocks_[param] = 1;
//         }
//       }
//     }
//
//     // printf("nb_marg_params: %ld\n", marg_param_ptrs_.size());
//     // for (const auto &param : marg_param_ptrs_) {
//     //   printf("- marg_param: %s\n", param->type.c_str());
//     // }
//     //
//     // printf("nb_remain_params: %ld\n", remain_param_ptrs_.size());
//     // for (const auto &param : remain_param_ptrs_) {
//     //   printf("- remain_param: %s\n", param->type.c_str());
//     // }
//
//     // Form Hessian and RHS of Gauss newton
//     matx_t H;
//     vecx_t b;
//     form_hessian(H, b, debug);
//
//     // Compute Schurs Complement
//     matx_t H_marg;
//     vecx_t b_marg;
//     schurs_complement(H, b, m_, r_, H_marg, b_marg, false, debug);
//     // Note: after the Schur's Complement, H_marg and b_marg will be smaller
//     due
//     // to marginalization.
//
//     // Calculate preconditioner for H
//     const vecx_t p = (H_marg.diagonal().array()
//     > 1.0e-9).select(H_marg.diagonal().cwiseSqrt(), 1.0e-3); const vecx_t
//     p_inv = p.cwiseInverse();
//
//     // Decompose matrix H into J^T * J to obtain J via eigen-decomposition
//     // i.e. H = U S U^T and therefore J = S^0.5 * U^T
//     // const Eigen::SelfAdjointEigenSolver<matx_t> eig(H_marg);
//     const Eigen::SelfAdjointEigenSolver<matx_t> eig(0.5  * p_inv.asDiagonal()
//     * (H_marg + H_marg.transpose()) * p_inv.asDiagonal());
//     // -- Calculate tolerance
//     // const double eps = 1.0e-8;
//     const auto eps = std::numeric_limits<double>::epsilon();
//     const auto tol = eps * H_marg.cols() *
//     eig.eigenvalues().array().maxCoeff();
//     // -- Form J0
//     // auto eigvals = (eig.eigenvalues().array() >
//     eps).select(eig.eigenvalues().array(), 0.0);
//     // const auto eigvals_inv = (eigvals > eps).select(eigvals.inverse(), 0);
//     auto eigvals = (eig.eigenvalues().array() >
//     tol).select(eig.eigenvalues().array(), 0.0); const auto eigvals_inv =
//     (eigvals > tol).select(eigvals.inverse(), 0); const vecx_t S =
//     vecx_t(eigvals); const vecx_t S_sqrt = S.cwiseSqrt();
//     // J0_ = S_sqrt.asDiagonal() * eig.eigenvectors().transpose();
//     J0_ = (p.asDiagonal() * eig.eigenvectors() *
//     (S_sqrt.asDiagonal())).transpose(); printf("tolerance: %f\t", tol);
//     printf("rank: %ld\t", rank(H_marg));
//     printf("size(H_marg): %ldx%ld\n", H_marg.rows(), H_marg.cols());
//     // print_vector("eigvals", eig.eigenvalues());
//
//     // Calculate residual: e0 = -pinv(J^T) * b
//     const vecx_t S_pinv = vecx_t(eigvals_inv);
//     const vecx_t S_pinv_sqrt = S_pinv.cwiseSqrt();
//     // const matx_t J_pinv_T = S_pinv_sqrt.asDiagonal() *
//     eig.eigenvectors().transpose(); const matx_t J_pinv_T =
//     (S_pinv_sqrt.asDiagonal()) * eig.eigenvectors().transpose() *
//     p_inv.asDiagonal(); e0_ = -J_pinv_T * b_marg;
//
//     // Keep track of linearization point x0
//     for (const auto &param_block : remain_param_ptrs_) {
//       x0_.insert({param_block->param.data(), param_block->param});
//     }
//
//     // Remove parameter blocks from problem, which in turn ceres will remove
//     // residual blocks linked to the parameter.
//     for (const auto &marg_param : marg_param_ptrs_) {
//       problem.RemoveParameterBlock(marg_param->param.data());
//     }
//
//     // Change flag to denote terms have been marginalized
//     marginalized_ = true;
//
//     // Debug
//     if (debug) {
//       mat2csv("/tmp/H.csv", H);
//       mat2csv("/tmp/b.csv", b);
//       mat2csv("/tmp/H_marg.csv", H_marg);
//       mat2csv("/tmp/b_marg.csv", b_marg);
//       mat2csv("/tmp/J0.csv", J0_);
//     }
//   }
//
//   size_t get_residual_size() {
//     return r_;
//   }
//
//   std::vector<double *> get_param_ptrs() {
//     std::vector<double *> params;
//     for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
//       params.push_back(remain_param_ptrs_[i]->param.data());
//     }
//     return params;
//   }
//
//   std::vector<param_t *> get_params() {
//     return remain_param_ptrs_;
//   }
//
//   vecx_t compute_delta_chi(double const *const *params) const {
//     assert(marginalized_);
//
//     // Stack DeltaChi vector
//     vecx_t DeltaChi(e0_.size());
//     for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
//       const auto param_block = remain_param_ptrs_[i];
//       if (param_block->fixed) {
//         continue;
//       }
//
//       const auto idx = param_index_.at(param_block) - m_;
//       const vecx_t x0_i = x0_.at(param_block->param.data());
//       const size_t size = param_block->global_size;
//
//       // Calculate i-th DeltaChi
//       const Eigen::Map<const vecx_t> x(params[i], size);
//       if (param_block->type == "pose_t" || param_block->type ==
//       "extrinsics_t") {
//         // Pose minus
//         const vec3_t dr = x.head<3>() - x0_i.head<3>();
//         const quat_t q_i(x(6), x(3), x(4), x(5));
//         const quat_t q_j(x0_i(6), x0_i(3), x0_i(4), x0_i(5));
//         const quat_t dq = q_j.inverse() * q_i;
//         DeltaChi.segment<3>(idx + 0) = dr;
//         DeltaChi.segment<3>(idx + 3) = 2.0 * dq.vec();
//         if (!(dq.w() >= 0)) {
//           DeltaChi.segment<3>(idx + 3) = 2.0 * -dq.vec();
//         }
//
//       } else {
//         // Trivial minus
//         DeltaChi.segment(idx, size) = x - x0_i;
//       }
//     }
//
//     return DeltaChi;
//   }
//
//   bool Evaluate(double const *const *params,
//                 double *residuals,
//                 double **jacobians=nullptr) const {
//     // Residual e
//     const vecx_t Delta_Chi = compute_delta_chi(params);
//     Eigen::Map<vecx_t> e(residuals, e0_.rows());
//     e = e0_ + J0_ * Delta_Chi;
//
//     // Return First Estimate Jacobians (FEJ)
//     if (jacobians == nullptr) {
//       return true;
//     }
//
//     const size_t J_rows = e0_.rows();
//     for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
//       if (jacobians[i] != nullptr) {
//         const auto &param = remain_param_ptrs_[i];
//         const size_t index = param_index_.at(param) - m_;
//         const size_t J_cols = param->global_size;
//         const size_t local_size = param->local_size;
//
//         Eigen::Map<matx_row_major_t> J(jacobians[i], J_rows, J_cols);
//         J.setZero();
//         J.leftCols(local_size) = J0_.middleCols(index, local_size);
//       }
//     }
//
//     return true;
//   }
// };

} // namespace yac
#endif // YAC_MARG_ERROR_HPP
