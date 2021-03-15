#include "autocal/common/Common.hpp"

namespace autocal {

// bool check_jacobians(
//     ErrorInterface *error,
//     std::vector<ParameterBlock *> &param_blocks,
//     double rel_tol, double delta) {
//   // Typedefs
//   // clang-format off
//   typedef Eigen::VectorXd vecx_t;
//   typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat_row_major_t;
//   typedef Eigen::aligned_allocator<mat_row_major_t> alloc_mat_row_major_t;
//   typedef std::vector<mat_row_major_t, alloc_mat_row_major_t> mats_row_major_t;
//   // clang-format on
//
//   // Set up data structures for storage
//   mats_row_major_t J(param_blocks.size());
//   mats_row_major_t J_min(param_blocks.size());
//   mats_row_major_t J_num_diff(param_blocks.size());
//   mats_row_major_t J_min_num_diff(param_blocks.size());
//
//   double **param_ptrs = new double *[param_blocks.size()];
//   double **J_ptrs = new double *[param_blocks.size()];
//   double **J_min_ptrs = new double *[param_blocks.size()];
//   const auto r_dim = error->residualDim();
//
//   for (size_t i = 0; i < param_blocks.size(); ++i) {
//     const auto param_dim = param_blocks[i]->dimension();
//     const auto param_min_dim = param_blocks[i]->minimalDimension();
//
//     // Set up the analytic Jacobians
//     mat_row_major_t Ji(r_dim, param_dim);
//     mat_row_major_t Ji_min(r_dim, param_min_dim);
//
//     // Set up the numeric ones
//     mat_row_major_t Ji_num_diff(r_dim, param_dim);
//     mat_row_major_t Ji_min_num_diff(r_dim, param_min_dim);
//
//     // Fill in
//     J[i].resize(r_dim, param_dim);
//     J_min[i].resize(r_dim, param_min_dim);
//     J_num_diff[i].resize(r_dim, param_dim);
//     J_min_num_diff[i].resize(r_dim, param_min_dim);
//
//     param_ptrs[i] = param_blocks[i]->parameters();
//     J_ptrs[i] = J[i].data();
//     J_min_ptrs[i] = J_min[i].data();
//   }
//
//   // Calculate numerical diff Jacobians
//   // -- Perturb every parameter
//   for (size_t i = 0; i < param_blocks.size(); ++i) {
//     const auto param_dim = param_blocks[i]->dimension();
//     const auto param_min_dim = param_blocks[i]->minimalDimension();
//
//     // -- Perturb every number in parameter, one by one
//     for (size_t j = 0; j < param_min_dim; ++j) {
//       // Setup
//       vecx_t r_p(r_dim);          // Residuals after positive step
//       vecx_t r_m(r_dim);          // Residuals after negative step
//       vecx_t params_p(param_dim); // Parameters after positive step
//       vecx_t params_m(param_dim); // Parameters after negative step
//       vecx_t step(param_min_dim); // Step
//
//       // Apply positive delta
//       step.setZero();
//       step[j] = delta;
//       param_blocks[i]->plus(param_ptrs[i], step.data(), params_p.data());
//       param_ptrs[i] = params_p.data();
//       error->EvaluateWithMinimalJacobians(param_ptrs, r_p.data(), NULL, NULL);
//       param_ptrs[i] = param_blocks[i]->parameters(); // reset
//
//       // Apply negative delta
//       step.setZero();
//       step[j] = -delta;
//       param_blocks[i]->plus(param_ptrs[i], step.data(), params_m.data());
//       param_ptrs[i] = params_m.data();
//       error->EvaluateWithMinimalJacobians(param_ptrs, r_m.data(), NULL, NULL);
//       param_ptrs[i] = param_blocks[i]->parameters(); // reset
//
//       // Calculate central finite numeric difference
//       J_min_num_diff[i].col(j) = (r_p - r_m) * 1.0 / (2.0 * delta);
//     }
//   }
//
//   // Calculate analytic Jacobians and compare
//   bool is_correct = true;
//   vecx_t r(r_dim);
//   for (size_t i = 0; i < param_blocks.size(); ++i) {
//     error->EvaluateWithMinimalJacobians(param_ptrs, r.data(), J_ptrs, J_min_ptrs);
//     // Check
//     double norm = J_min_num_diff[i].norm();
//     Eigen::MatrixXd J_diff = J_min_num_diff[i] - J_min[i];
//     double max_diff = std::max(-J_diff.minCoeff(), J_diff.maxCoeff());
//     if ((max_diff / norm) > rel_tol) {
//       std::cout << "Jacobian inconsistent: " << error->typeInfo() << std::endl;
//       std::cout << "num diff Jacobian[" << i << "]:" << std::endl;
//       std::cout << J_min_num_diff[i] << std::endl;
//       std::cout << "provided Jacobian[" << i << "]:" << std::endl;
//       std::cout << J_min[i] << std::endl;
//       std::cout << "difference matrix:" << std::endl;
//       std::cout << J_diff << std::endl;
//       std::cout << "rel difference matrix:" << std::endl;
//       std::cout << J_diff * (1 / norm) << std::endl;
//       std::cout << "relative error: " << max_diff / norm << ", ";
//       std::cout << "relative tolerance: " << rel_tol << std::endl << std::flush;
//       is_correct = false;
//     }
//
//     // printf("J min num diff[%zu]\n", i);
//     // std::cout << J_min_num_diff[i] << std::endl;
//     // std::cout << std::endl;
//     // printf("J min[%zu]\n", i);
//     // std::cout << J_min[i] << std::endl;
//     // std::cout << std::endl;
//     // printf("J diff[%zu]\n", i);
//     // std::cout << J_diff << std::endl;
//     // std::cout << std::endl;
//     // printf("max_diff: %f\n", max_diff);
//     // std::cout << std::endl;
//     // std::cout << std::endl;
//   }
//   // exit(0);
//
//   delete[] param_ptrs;
//   delete[] J_ptrs;
//   delete[] J_min_ptrs;
//   return is_correct;
// }

}
