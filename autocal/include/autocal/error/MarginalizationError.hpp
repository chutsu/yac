#ifndef AUTOCAL_CERES_MARGINALIZATIONERROR_HPP
#define AUTOCAL_CERES_MARGINALIZATIONERROR_HPP

#include <vector>
#include <deque>
#include <memory>
#include <Eigen/Core>
#include <ceres/ceres.h>

#include "core.hpp"
#include "../common/Common.hpp"
#include "../common/Measurements.hpp"
#include "../common/Variables.hpp"
#include "../common/Transformation.hpp"
#include "../param/ParameterBlock.hpp"
#include "../param/PoseParameterBlock.hpp"
#include "../param/SpeedAndBiasParameterBlock.hpp"
#include "ErrorInterface.hpp"

namespace autocal {

class Map;

// not sized, in order to be flexible.
class MarginalizationError : public ::ceres::CostFunction, public ErrorInterface {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::CostFunction base_t;

  /// \brief Trivial destructor.
  virtual ~MarginalizationError()
  {
  }

  /// \brief Default constructor. Initialises a new okvis::ceres::Map.
  MarginalizationError();

  /// \brief Constructor from okvis::ceres::Map.
  /// @param[in] map The underlying okvis::ceres::Map.
  MarginalizationError(Map& map);

  /// \brief Constructor from okvis::ceres::Map and directly add some residuals
  /// @param[in] map The underlying okvis::ceres::Map.
  /// @param[in] residualBlockIds Residual block IDs to be added directly (\see okvis::ceres::addResidualBlocks)
  MarginalizationError(
      Map& map, std::vector< ::ceres::ResidualBlockId > & residualBlockIds);

  // initialization
  /// \brief Set the underlying okvis::ceres::Map.
  /// @param[in] map The underlying okvis::ceres::Map.
  void setMap(Map& map);

  /// \brief Add some residuals to this marginalisation error. This means, they will get linearised.
  /// \warning Note that once added here, they will be removed from the okvis::ceres::Map and stay linerised
  ///          at exactly the points passed here.
  /// @param[in] residualBlockIds Vector of residual block ids, the corresponding terms of which will be added.
  /// @param[in] keepResidualBlocks Currently not in use.
  bool addResidualBlocks(
      const std::vector< ::ceres::ResidualBlockId > & residualBlockIds,
      const std::vector<bool> & keepResidualBlocks = std::vector<bool>());

  /// \brief Add one residual to this marginalisation error. This means, it will get linearised.
  /// \warning Note that once added here, it will be removed from the okvis::ceres::Map and stay linerised
  ///          at exactly the point passed here.
  /// @param[in] residualBlockId Residual block id, the corresponding term of which will be added.
  /// @param[in] keepResidualBlock Currently not in use.
  bool addResidualBlock(::ceres::ResidualBlockId residualBlockId,
                        bool keepResidualBlock = false);

  /// \brief Info: is this parameter block connected to this marginalization error?
  /// @param[in] parameterBlockId Parameter block id of interest.
  bool isParameterBlockConnected(uint64_t parameterBlockId);

  // marginalization

  /// \brief Marginalise out a set of parameter blocks.
  /// \warning The parameter blocks to be marginalised must have been added before.
  ///          Also: make sure to add all the needed residual blocks beforehand for this to make sense.
  /// \return False if not all necessary residual blocks were added before.
  bool marginalizeOut(const std::vector<uint64_t> & parameterBlockIds,
                      const std::vector<bool> & keepParameterBlocks =
                          std::vector<bool>());

  /// \brief This must be called before optimization after adding residual blocks and/or marginalizing,
  ///        since it performs all the lhs and rhs computations on from a given _H and _b.
  void updateErrorComputation();

  /// \brief Call this in order to (re-)add this error term after whenever it had been modified.
  /// @param[in] parameterBlockPtrs Parameter block pointers in question.
  void getParameterBlockPtrs(
      std::vector<std::shared_ptr<ParameterBlock> >& parameterBlockPtrs);
  void getParameterBlockPtrs(
      std::vector<std::shared_ptr<ParameterBlock> >& parameterBlockPtrs) const;

  // error term and Jacobian implementation (inherited pure virtuals from ::ceres::CostFunction)
  /**
    * @brief This evaluates the error term and additionally computes the Jacobians.
    * @param parameters Pointer to the parameters (see ceres)
    * @param residuals Pointer to the residual vector (see ceres)
    * @param jacobians Pointer to the Jacobians (see ceres)
    * @return success of th evaluation.
    */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobiansMinimal) const;

  // sizes (inherited pure virtuals from ::okvis::ceres::ErrorInterface)
  /// \brief Residual dimension.
  size_t residualDim() const
  {
    return base_t::num_residuals();
  }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameterBlockId) const
  {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const
  {
    return "MarginalizationError";
  }


  /**
   * @brief Pseudo inversion of a symmetric matrix.
   * @warning   This uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
   *            (negative Eigenvalues are set to zero).
   * @tparam Derived Matrix type (auto-deducible).
   * @param[in] a Input Matrix
   * @param[out] result Output, i.e. pseudo-inverse.
   * @param[in] epsilon The tolerance.
   * @param[out] rank Optional rank.
   * @return
   */
  template<typename Derived>
  static bool pseudoInverseSymm(
      const Eigen::MatrixBase<Derived>&a,
      const Eigen::MatrixBase<Derived>&result, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon(), int * rank = 0);

  /**
   * @brief Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
   * @warning   This uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
   *            (negative Eigenvalues are set to zero).
   * @tparam Derived Matrix type (auto-deducible).
   * @param[in] a Input Matrix
   * @param[out] result Output, i.e. the Cholesky decomposition of a pseudo-inverse.
   * @param[in] epsilon The tolerance.
   * @param[out] rank The rank, if of interest.
   * @return
   */
  template<typename Derived>
  static bool pseudoInverseSymmSqrt(
      const Eigen::MatrixBase<Derived>&a,
      const Eigen::MatrixBase<Derived>&result, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon(),
      int* rank = NULL);

  /**
   * @brief Block-wise pseudo inversion of a symmetric matrix with non-zero diagonal blocks.
   * @warning   This uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
   *            (negative Eigenvalues are set to zero).
   * @tparam Derived Matrix type (auto-deducible).
   * @tparam blockDim The block size of the diagonal blocks.
   * @param[in] M_in Input Matrix
   * @param[out] M_out Output, i.e. thepseudo-inverse.
   * @param[in] epsilon The tolerance.
   * @return
   */
  template<typename Derived, int blockDim>
  static void blockPinverse(
      const Eigen::MatrixBase<Derived>& M_in,
      const Eigen::MatrixBase<Derived>& M_out, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon());


  /**
   * @brief Block-wise pseudo inversion and square root (Cholesky decomposition)
   *        of a symmetric matrix with non-zero diagonal blocks.
   * @warning   This uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
   *            (negative Eigenvalues are set to zero).
   * @tparam Derived Matrix type (auto-deducible).
   * @tparam blockDim The block size of the diagonal blocks.
   * @param[in] M_in Input Matrix
   * @param[out] M_out Output, i.e. the Cholesky decomposition of a pseudo-inverse.
   * @param[in] epsilon The tolerance.
   * @return
   */
  template<typename Derived, int blockDim>
  static void blockPinverseSqrt(
      const Eigen::MatrixBase<Derived>& M_in,
      const Eigen::MatrixBase<Derived>& M_out, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon());

 protected:
  Map* mapPtr_; ///< The underlying map.
  ::ceres::ResidualBlockId residualBlockId_; ///< The residual block id of this.

  /// \brief Checks the internal datastructure (debug)
  void check();

  /// \brief Computes the linearized deviation from the references (linearization points)
  bool computeDeltaChi(Eigen::VectorXd& DeltaChi) const;  // use the stored estimates

  /// \brief Computes the linearized deviation from the references (linearization points)
  bool computeDeltaChi(double const* const * parameters,
                       Eigen::VectorXd& DeltaChi) const;  // use the provided estimates

  /// \brief Split for Schur complement op.
  template<typename Derived_A, typename Derived_U, typename Derived_W,
      typename Derived_V>
  static void splitSymmetricMatrix(
      const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
      const Eigen::MatrixBase<Derived_A>& A,  // input
      const Eigen::MatrixBase<Derived_U>& U,  // output
      const Eigen::MatrixBase<Derived_W>& W,  // output
      const Eigen::MatrixBase<Derived_V>& V);  // output

  /// \brief Split for Schur complement op.
  template<typename Derived_b, typename Derived_b_a, typename Derived_b_b>
  static void splitVector(
      const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
      const Eigen::MatrixBase<Derived_b>& b,  // input
      const Eigen::MatrixBase<Derived_b_a>& b_a,  // output
      const Eigen::MatrixBase<Derived_b_b>& b_b);  // output

  /// @name The internal storage of the linearised system.
  /// lhs and rhs:
  /// H_*delta_Chi = _b - H_*Delta_Chi .
  /// the lhs Hessian matrix is decomposed as _H = J^T*J = _U*S*_U^T ,
  /// the rhs is decomposed as _b - _H*Delta_Chi = -J^T * (-pinv(J^T) * _b + J*Delta_Chi) ,
  /// i.e. we have the ceres standard form with weighted Jacobians _J,
  /// an identity information matrix, and an error
  /// _e = -pinv(J^T) * _b + J*Delta_Chi .
  /// _e = _e0 + J*Delta_Chi .
  /// @{
  Eigen::MatrixXd H_;  ///< lhs - Hessian
  Eigen::VectorXd b0_;  ///<  rhs constant part
  Eigen::VectorXd e0_;  ///<  _e0 := pinv(J^T) * _b0
  Eigen::MatrixXd J_;  ///<  Jacobian such that _J^T * J == _H
  Eigen::MatrixXd U_;  ///<  H_ = _U*_S*_U^T lhs Eigen decomposition
  Eigen::VectorXd S_;  ///<  singular values
  Eigen::VectorXd S_sqrt_;  ///<  cwise sqrt of _S, i.e. _S_sqrt*_S_sqrt=_S; _J=_U^T*_S_sqrt
  Eigen::VectorXd S_pinv_;  ///<  pseudo inverse of _S
  Eigen::VectorXd S_pinv_sqrt_;  ///<  cwise sqrt of _S_pinv, i.e. pinv(J^T)=_U^T*_S_pinv_sqrt
  Eigen::VectorXd p_;
  Eigen::VectorXd p_inv_;
  volatile bool errorComputationValid_;  ///<  adding residual blocks will invalidate this. before optimizing, call updateErrorComputation()

  /// \brief Book-keeping of the ordering.
  struct ParameterBlockInfo
  {
    AUTOCAL_DEFINE_EXCEPTION(Exception,std::runtime_error)
    uint64_t parameterBlockId;
    std::shared_ptr<ParameterBlock> parameterBlockPtr;
    size_t orderingIdx;
    size_t dimension;
    size_t minimalDimension;
    size_t localDimension;
    std::shared_ptr<double> linearizationPoint;
    bool isLandmark;
    ParameterBlockInfo()
        : parameterBlockId(0),
          parameterBlockPtr(std::shared_ptr<ParameterBlock>()),
          orderingIdx(0),
          dimension(0),
          minimalDimension(0),
          localDimension(0),
          isLandmark(false)
    {
    }
    ParameterBlockInfo(uint64_t parameterBlockId,
                       std::shared_ptr<ParameterBlock> parameterBlockPtr,
                       size_t orderingIdx, bool isLandmark)
        : parameterBlockId(parameterBlockId),
          parameterBlockPtr(parameterBlockPtr),
          orderingIdx(orderingIdx),
          isLandmark(isLandmark)
    {
      dimension = parameterBlockPtr->dimension();
      minimalDimension = parameterBlockPtr->minimalDimension();
      if (parameterBlockPtr->localParameterizationPtr()) {
        localDimension = parameterBlockPtr->localParameterizationPtr()
            ->LocalSize();
      } else {
        localDimension = minimalDimension;
      }
      if (parameterBlockPtr->fixed()) {
        minimalDimension = 0;
        localDimension = 0;
      }
      linearizationPoint.reset(new double[dimension],
                               std::default_delete<double[]>());
      memcpy(linearizationPoint.get(), parameterBlockPtr->parameters(),
             dimension * sizeof(double));
    }

    /// \brief Reset the linearisation point. Use with caution.
    void resetLinearizationPoint(
        std::shared_ptr<ParameterBlock> parameterBlockPtr)
    {
      AUTOCAL_ASSERT_TRUE_DBG(Exception,dimension==parameterBlockPtr->dimension(),"not initialised.")
      memcpy(linearizationPoint.get(), parameterBlockPtr->parameters(),
             dimension * sizeof(double));
    }
  };

  std::vector<ParameterBlockInfo> parameterBlockInfos_;  ///< Book keeper.
  std::map<uint64_t, size_t> parameterBlockId2parameterBlockInfoIdx_;  ///< Maps parameter block Ids to index in _parameterBlockInfos
  size_t denseIndices_;  ///< Keep track of the size of the dense part of the equation system

  /// @}

};

// Split for Schur complement op.
template<typename Derived_A, typename Derived_U, typename Derived_W,
    typename Derived_V>
void MarginalizationError::splitSymmetricMatrix(
    const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
    const Eigen::MatrixBase<Derived_A>& A,  // input
    const Eigen::MatrixBase<Derived_U>& U,  // output
    const Eigen::MatrixBase<Derived_W>& W,  // output
    const Eigen::MatrixBase<Derived_V>& V) {  // output

  // sanity check
  const int size = A.cols();
  AUTOCAL_ASSERT_TRUE_DBG(Exception, size == A.rows(), "matrix not symmetric");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, V.cols() == V.rows(),
                        "matrix not symmetric");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, V.cols() == W.cols(),
                        "matrix not symmetric");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, U.rows() == W.rows(),
                        "matrix not symmetric");
  AUTOCAL_ASSERT_TRUE_DBG(
      Exception,
      V.rows() + U.rows() == size,
      "matrices supplied to be split into do not form exact upper triangular blocks of the original one");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, U.cols() == U.rows(),
                        "matrix not symmetric");

  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairs2 =
      marginalizationStartIdxAndLengthPairs;
  marginalizationStartIdxAndLengthPairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalizationStartIdxAndLengthPairs2.size();

  int lastIdx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i) {
    int lastIdx_col = 0;
    int start_a_j = 0;
    int start_b_j = 0;
    int thisIdx_row = marginalizationStartIdxAndLengthPairs2[i].first;
    const int size_b_i = marginalizationStartIdxAndLengthPairs2[i].second;  // kept
    const int size_a_i = thisIdx_row - lastIdx_row;  // kept
    for (size_t j = 0; j < length; ++j) {
      int thisIdx_col = marginalizationStartIdxAndLengthPairs2[j].first;
      const int size_a_j = thisIdx_col - lastIdx_col;  // kept
      const int size_b_j = marginalizationStartIdxAndLengthPairs2[j].second;  // kept

      // the kept part - only access and copy if non-zero
      if (size_a_j > 0 && size_a_i > 0) {
        const_cast<Eigen::MatrixBase<Derived_U>&>(U).block(start_a_i, start_a_j,
                                                           size_a_i, size_a_j) =
            A.block(lastIdx_row, lastIdx_col, size_a_i, size_a_j);
      }

      // now the mixed part
      if (size_b_j > 0 && size_a_i > 0) {
        const_cast<Eigen::MatrixBase<Derived_W>&>(W).block(start_a_i, start_b_j,
                                                           size_a_i, size_b_j) =
            A.block(lastIdx_row, thisIdx_col, size_a_i, size_b_j);
      }

      // and finally the marginalized part
      if (size_b_j > 0 && size_b_i > 0) {
        const_cast<Eigen::MatrixBase<Derived_V>&>(V).block(start_b_i, start_b_j,
                                                           size_b_i, size_b_j) =
            A.block(thisIdx_row, thisIdx_col, size_b_i, size_b_j);
      }

      lastIdx_col = thisIdx_col + size_b_j;  // remember
      start_a_j += size_a_j;
      start_b_j += size_b_j;
    }
    lastIdx_row = thisIdx_row + size_b_i;  // remember
    start_a_i += size_a_i;
    start_b_i += size_b_i;
  }
}

// Split for Schur complement op.
template<typename Derived_b, typename Derived_b_a, typename Derived_b_b>
void MarginalizationError::splitVector(
    const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
    const Eigen::MatrixBase<Derived_b>& b,  // input
    const Eigen::MatrixBase<Derived_b_a>& b_a,  // output
    const Eigen::MatrixBase<Derived_b_b>& b_b) {  // output

  const int size = b.rows();
  // sanity check
  AUTOCAL_ASSERT_TRUE_DBG(Exception, b.cols() == 1, "supplied vector not x-by-1");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, b_a.cols() == 1,
                        "supplied vector not x-by-1");
  AUTOCAL_ASSERT_TRUE_DBG(Exception, b_b.cols() == 1,
                        "supplied vector not x-by-1");
  AUTOCAL_ASSERT_TRUE_DBG(
      Exception,
      b_a.rows() + b_b.rows() == size,
      "vector supplied to be split into cannot be concatenated to the original one");

  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairs2 =
      marginalizationStartIdxAndLengthPairs;
  marginalizationStartIdxAndLengthPairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalizationStartIdxAndLengthPairs2.size();

  int lastIdx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i) {
    int thisIdx_row = marginalizationStartIdxAndLengthPairs2[i].first;
    const int size_b_i = marginalizationStartIdxAndLengthPairs2[i].second;  // kept
    const int size_a_i = thisIdx_row - lastIdx_row;  // kept

    // the kept part - only access and copy if non-zero
    if (size_a_i > 0) {
      const_cast<Eigen::MatrixBase<Derived_b_a>&>(b_a).segment(start_a_i,
                                                               size_a_i) = b
          .segment(lastIdx_row, size_a_i);
    }

    // and finally the marginalized part
    if (size_b_i > 0) {
      const_cast<Eigen::MatrixBase<Derived_b_b>&>(b_b).segment(start_b_i,
                                                               size_b_i) = b
          .segment(thisIdx_row, size_b_i);
    }

    lastIdx_row = thisIdx_row + size_b_i;  // remember
    start_a_i += size_a_i;
    start_b_i += size_b_i;
  }
}

// Pseudo inversion of a symmetric matrix.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived>
bool MarginalizationError::pseudoInverseSymm(
    const Eigen::MatrixBase<Derived>&a, const Eigen::MatrixBase<Derived>&result,
    double epsilon, int * rank) {

  AUTOCAL_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                        "matrix supplied is not quadratic");

  Eigen::SelfAdjointEigenSolver<Derived> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();

  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          (saes.eigenvalues().array() > tolerance).select(
              saes.eigenvalues().array().inverse(), 0)).asDiagonal()
      * (saes.eigenvectors().transpose());

  if (rank) {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i) {
      if (saes.eigenvalues()[i] > tolerance)
        (*rank)++;
    }
  }

  return true;
}

// Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived>
bool MarginalizationError::pseudoInverseSymmSqrt(
    const Eigen::MatrixBase<Derived>&a, const Eigen::MatrixBase<Derived>&result,
    double epsilon, int * rank) {

  AUTOCAL_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                        "matrix supplied is not quadratic");

  Eigen::SelfAdjointEigenSolver<Derived> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();

  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          Eigen::VectorXd(
              (saes.eigenvalues().array() > tolerance).select(
                  saes.eigenvalues().array().inverse(), 0)).array().sqrt())
          .asDiagonal();

  if (rank) {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i) {
      if (saes.eigenvalues()[i] > tolerance)
        (*rank)++;
    }
  }

  return true;
}

// Block-wise pseudo inversion of a symmetric matrix with non-zero diagonal blocks.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived, int blockDim>
void MarginalizationError::blockPinverse(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon) {

  AUTOCAL_ASSERT_TRUE_DBG(Exception, M_in.rows() == M_in.cols(),
                        "matrix supplied is not quadratic");

  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim) {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    //const Eigen::Matrix<double,blockDim,blockDim> in1=0.5*(in+in.transpose());
    pseudoInverseSymm(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

// Block-wise pseudo inversion and square root (Cholesky decomposition)
// of a symmetric matrix with non-zero diagonal blocks.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived, int blockDim>
void MarginalizationError::blockPinverseSqrt(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon) {

  AUTOCAL_ASSERT_TRUE_DBG(Exception, M_in.rows() == M_in.cols(),
                        "matrix supplied is not quadratic");

  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim) {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    //const Eigen::Matrix<double,blockDim,blockDim> in1=0.5*(in+in.transpose());
    pseudoInverseSymmSqrt(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

}  // namespace okvis
#endif /* AUTOCAL_CERES_MARGINALIZATIONERROR_HPP_ */
