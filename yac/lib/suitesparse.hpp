#ifndef YAC_SUITESPARSE_HPP
#define YAC_SUITESPARSE_HPP

#include "util/core.hpp"
#include <suitesparse/SuiteSparseQR.hpp>

namespace yac {

// CHOLMOD UTILS /////////////////////////////////////////////////////////////

cholmod_dense *solve_qr(cholmod_sparse *A,
                        cholmod_dense *b,
                        cholmod_common *cholmod);

inline void deleteCholdmodPtr(cholmod_sparse *&ptr, cholmod_common &cholmod) {
  if (ptr) {
    cholmod_l_free_sparse(&ptr, &cholmod);
  }
}

inline void deleteCholdmodPtr(cholmod_dense *&ptr, cholmod_common &cholmod) {
  if (ptr) {
    cholmod_l_free_dense(&ptr, &cholmod);
  }
}

template <typename T>
struct SelfFreeingCholmodPtr {
  explicit SelfFreeingCholmodPtr(T *ptr, cholmod_common &cholmod)
      : cholmod_(cholmod), ptr_(ptr) {}

  ~SelfFreeingCholmodPtr() { reset(NULL); }

  void reset(T *ptr = nullptr) {
    deleteCholdmodPtr(ptr_, cholmod_);
    ptr_ = ptr;
  }

  SelfFreeingCholmodPtr &operator=(T *ptr) {
    reset(ptr);
    return *this;
  }

  operator T *() { return ptr_; }

  T *operator->() { return ptr_; }

  T **operator&() { return &ptr_; }

private:
  cholmod_common &cholmod_;
  T *ptr_;
};

// CHOLMOD CONVERTER /////////////////////////////////////////////////////////

// CHOLMOD Sparse to Eigen Dense
void cholmod_convert(const cholmod_sparse *in, matx_t &out);

// CHOLMOD Dense to Eigen Dense Copy
void cholmod_convert(const cholmod_dense *in, vecx_t &out);

// Eigen Dense To CHOLMOD Dense
void cholmod_convert(const vecx_t &in, cholmod_dense *out);

// Eigen Dense To CHOLMOD Dense Copy
cholmod_dense *cholmod_convert(const vecx_t &in, cholmod_common *cholmod);

// Eigen Dense to CHOLMOD Sparse Copy
cholmod_sparse *cholmod_convert(const matx_t &A,
                                cholmod_common *cholmod,
                                double eps);

// TRUNCATED SOLVER ///////////////////////////////////////////////////////////

struct trucated_solver_options_t {
  bool columnScaling = true;
  double epsNorm = std::numeric_limits<double>::epsilon();
  double epsSVD = std::numeric_limits<double>::epsilon();
  double epsQR = std::numeric_limits<double>::epsilon();
  double svdTol = -1.0;
  double qrTol = -1.0;
  bool verbose = false;
};

struct truncated_solver_t {
  trucated_solver_options_t tsvd_options_;
  cholmod_common cholmod_;
  SuiteSparseQR_factorization<double> *factor_ = nullptr;

  ssize_t svdRank_;
  double svGap_;
  ssize_t svdRankDeficiency_;
  double svdTolerance_;
  vecx_t singularValues_;
  matx_t matrixU_;
  matx_t matrixV_;
  real_t calib_covar_det = 0.0;

  double linearSolverTime_;
  double marginalAnalysisTime_;

  truncated_solver_t();
  virtual ~truncated_solver_t();

  void solve(cholmod_sparse *A, cholmod_dense *b, size_t j, vecx_t &x);
  void analyze_marginal(cholmod_sparse *A, size_t j);

  void clear();
  const vecx_t &getSingularValues() const;
  double getSingularValuesLog2Sum() const;

  void
  _analyze_svd(const cholmod_sparse *Omega, vecx_t &sv, matx_t &U, matx_t &V);
  void _analyze_svd(cholmod_sparse *Omega);
};

} // namespace yac
#endif // YAC_SUITESPARSE_HPP
