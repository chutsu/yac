#ifndef YAC_ERRORINTERFACE_HPP
#define YAC_ERRORINTERFACE_HPP

#include <vector>
#include <Eigen/Core>

#include "../param/ParameterBlock.hpp"

namespace yac {

class ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  mutable std::vector<size_t> param_ids;
  mutable std::vector<std::shared_ptr<ParameterBlock>> params;

  ErrorInterface() {}
  virtual ~ErrorInterface() {}

  virtual size_t residualDim() const = 0;
  virtual size_t parameterBlocks() const = 0;

  virtual size_t parameterBlockDim(size_t parameterBlockId) const = 0;

  virtual bool
  EvaluateWithMinimalJacobians(double const *const *parameters,
                               double *residuals,
                               double **jacobians,
                               double **jacobiansMinimal) const = 0;

  virtual std::string typeInfo() const = 0;
};

} // namespace autocal
#endif // YAC_ERRORINTERFACE_HPP
