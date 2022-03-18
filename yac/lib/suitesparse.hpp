#ifndef YAC_SUITESPARSE_HPP
#define YAC_SUITESPARSE_HPP

#include "util/core.hpp"
#include <suitesparse/SuiteSparseQR.hpp>

namespace yac {

// CHOLMOD UTILS /////////////////////////////////////////////////////////////

cholmod_dense *solve_qr(SuiteSparseQR_factorization<double> *factor,
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

struct cholmod_converter_t {
  // CHOLMOD Sparse to Eigen Dense
  static void convert(const cholmod_sparse *in, matx_t &out) {
    out.setZero(in->nrow, in->ncol);

    // clang-format off
    const std::ptrdiff_t *row_ind = reinterpret_cast<const std::ptrdiff_t
    *>(in->i); const std::ptrdiff_t *col_ptr = reinterpret_cast<const
    std::ptrdiff_t *>(in->p); const std::ptrdiff_t col_end =
    static_cast<std::ptrdiff_t>(in->ncol); const double *values =
    reinterpret_cast<const double *>(in->x);
    // clang-format on
    for (std::ptrdiff_t col_idx = 0; col_idx < col_end; ++col_idx) {
      for (std::ptrdiff_t val_idx = col_ptr[col_idx];
           val_idx < col_ptr[col_idx + 1];
           ++val_idx) {
        out(row_ind[val_idx], col_idx) = values[val_idx];
        if (in->stype && col_idx != row_ind[val_idx]) {
          out(col_idx, row_ind[val_idx]) = values[val_idx];
        }
      }
    }
  }

  // CHOLMOD Dense to Eigen Dense Copy
  static void convert(const cholmod_dense *in, vecx_t &out) {
    // CHECK_NOTNULL(in);
    out.resize(in->nrow);
    const double *in_val = reinterpret_cast<const double *>(in->x);
    std::copy(in_val, in_val + in->nrow, out.data());
  }

  // Eigen Dense To CHOLMOD Dense
  static void convert(const vecx_t &in, cholmod_dense *out) {
    // CHECK_NOTNULL(out);

    out->nrow = in.size();
    out->ncol = 1;
    out->nzmax = in.size();
    out->d = in.size();
    out->x = reinterpret_cast<void *>(const_cast<double *>(in.data()));
    out->z = nullptr;
    out->xtype = CHOLMOD_REAL;
    out->dtype = CHOLMOD_DOUBLE;
  }

  // Eigen Dense To CHOLMOD Dense Copy
  static cholmod_dense *convert(const vecx_t &in, cholmod_common *cholmod) {
    cholmod_dense *out = cholmod_l_allocate_dense(in.size(),
                                                  1,
                                                  in.size(),
                                                  CHOLMOD_REAL,
                                                  cholmod);
    // CHECK(out != nullptr) << "cholmod_l_allocate_dense failed.";

    double *out_val = reinterpret_cast<double *>(out->x);
    const double *in_val = in.data();
    std::copy(in_val, in_val + in.size(), out_val);
    return out;
  }

  // Eigen Dense to CHOLMOD Sparse Copy
  static cholmod_sparse *convert(const matx_t &A,
                                 cholmod_common *cholmod,
                                 double eps) {
    // CHECK_NOTNULL(cholmod);
    // CHECK_GT(eps, 0.0);

    size_t nzmax = 0;
    for (std::ptrdiff_t i = 0; i < A.rows(); ++i) {
      for (std::ptrdiff_t j = 0; j < A.cols(); ++j) {
        if (std::fabs(A(i, j)) > eps) {
          nzmax++;
        }
      }
    }

    cholmod_sparse *A_cholmod = cholmod_l_allocate_sparse(A.rows(),
                                                          A.cols(),
                                                          nzmax,
                                                          1,
                                                          1,
                                                          0,
                                                          CHOLMOD_REAL,
                                                          cholmod);
    // CHECK(A_cholmod != nullptr) << "cholmod_l_allocate_sparse failed.";

    std::ptrdiff_t *row_ind = reinterpret_cast<std::ptrdiff_t *>(A_cholmod->i);
    std::ptrdiff_t *col_ptr = reinterpret_cast<std::ptrdiff_t *>(A_cholmod->p);
    double *values = reinterpret_cast<double *>(A_cholmod->x);
    std::ptrdiff_t row_it = 0;
    std::ptrdiff_t col_it = 1;
    for (std::ptrdiff_t c = 0; c < A.cols(); ++c) {
      for (std::ptrdiff_t r = 0; r < A.rows(); ++r)
        if (std::fabs(A(r, c)) > eps) {
          values[row_it] = A(r, c);
          row_ind[row_it] = r;
          row_it++;
        }
      col_ptr[col_it] = row_it;
      col_it++;
    }
    return A_cholmod;
  }
};

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
  SuiteSparseQR_factorization<double> *factor_;
  std::ptrdiff_t svdRank_;
  double svGap_;
  std::ptrdiff_t svdRankDeficiency_;
  std::ptrdiff_t margStartIndex_;
  double svdTolerance_;
  vecx_t singularValues_;
  matx_t matrixU_;
  matx_t matrixV_;
  double linearSolverTime_;
  double marginalAnalysisTime_;

  truncated_solver_t();
  virtual ~truncated_solver_t();

  void solve(cholmod_sparse *A, cholmod_dense *b, size_t j, vecx_t &x);
  void analyzeMarginal(cholmod_sparse *A, size_t j);

  void clear();
  std::ptrdiff_t getSVDRank() const;
  std::ptrdiff_t getSVDRankDeficiency() const;
  double getSvGap() const;
  std::ptrdiff_t getQRRank() const;
  std::ptrdiff_t getQRRankDeficiency() const;
  std::ptrdiff_t getMargStartIndex() const;
  void setMargStartIndex(std::ptrdiff_t index);
  double getQRTolerance() const;
  double getSVDTolerance() const;
  const vecx_t &getSingularValues() const;
  const matx_t &getMatrixU() const;
  const matx_t &getMatrixV() const;
  matx_t getNullSpace() const;
  matx_t getRowSpace() const;
  matx_t getCovariance() const;
  matx_t getRowSpaceCovariance() const;
  double getSingularValuesLog2Sum() const;
  double getSymbolicFactorizationTime() const;
  double getNumericFactorizationTime() const;
  void setNThreads(int n);
  void clearSvdAnalysisResultMembers();
  void analyzeSVD(const cholmod_sparse *Omega,
                  Eigen::VectorXd &sv,
                  Eigen::MatrixXd &U,
                  Eigen::MatrixXd &V);
  void analyzeSVD(cholmod_sparse *Omega);
};

} // namespace yac
#endif // YAC_SUITESPARSE_HPP
