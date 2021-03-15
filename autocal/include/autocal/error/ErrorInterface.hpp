#ifndef AUTOCAL_CERES_ERRORINTERFACE_HPP
#define AUTOCAL_CERES_ERRORINTERFACE_HPP

#include <vector>
#include <Eigen/Core>

#include "core.hpp"
#include "../common/Common.hpp"
#include "../param/ParameterBlock.hpp"

namespace autocal {

/// @brief Simple interface class the errors implemented here should inherit
/// from.
class ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // mutable double *residuals_debug = nullptr;
  // mutable double *error_debug = nullptr;
  mutable std::vector<size_t> param_ids;
  mutable std::vector<std::shared_ptr<ParameterBlock>> params;

  // mutable bool eval = false;
  // mutable vecx_t r;
  // mutable matxs_t J;

  /// @brief Constructor
  ErrorInterface() {}
  /// @brief Destructor (does nothing).
  virtual ~ErrorInterface() {}

  /// @name Sizes
  /// @{

  /// @brief Get dimension of residuals.
  /// @return The residual dimension.
  virtual size_t residualDim() const = 0;

  /// @brief Get the number of parameter blocks this is connected to.
  /// @return The number of parameter blocks.
  virtual size_t parameterBlocks() const = 0;

  /**
   * @brief get the dimension of a parameter block this is connected to.
   * @param parameterBlockId The ID of the parameter block of interest.
   * @return Its dimension.
   */
  virtual size_t parameterBlockDim(size_t parameterBlockId) const = 0;

  /// @}
  // Error and Jacobian computation
  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to
   * jacobians).
   * @return Success of the evaluation.
   */
  virtual bool
  EvaluateWithMinimalJacobians(double const *const *parameters,
                               double *residuals,
                               double **jacobians,
                               double **jacobiansMinimal) const = 0;

  /// @brief Residual block type as string
  virtual std::string typeInfo() const = 0;
};

} // namespace autocal
#endif // AUTOCAL_CERES_ERRORINTERFACE_HPP
