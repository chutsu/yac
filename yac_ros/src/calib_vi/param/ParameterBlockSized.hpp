#ifndef YAC_PARAMETER_BLOCK_SIZED_HPP
#define YAC_PARAMETER_BLOCK_SIZED_HPP

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <Eigen/Core>

#include "ParameterBlock.hpp"

namespace yac {

/// @brief Base class providing the interface for parameter blocks.
/// @tparam Dim     Dimension of parameter block
/// @tparam MinDim  Minimal dimension of parameter block
/// @tparam T       The type of the estimate
template <int Dim, int MinDim, class T>
class ParameterBlockSized : public ParameterBlock {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Dimension of the parameter block.
  static const int Dimension = Dim;

  /// @brief Internal (minimal) dimension.
  static const int MinimalDimension = MinDim;

  /// \brief Make the parameter type accessible.
  typedef T parameter_t;

  /// \brief Default constructor -- initialises elements in parametes_ to zero.
  ParameterBlockSized() {
    for (int i = 0; i < Dimension; ++i)
      parameters_[i] = 0;
  }

  /// \brief Trivial destructor.
  virtual ~ParameterBlockSized() {}

  /// @name Setters
  /// @{

  /// @brief Set estimate of this parameter block.
  /// @param[in] estimate The estimate to set this to.
  virtual void setEstimate(const parameter_t &estimate) = 0;

  /// @brief Set exact parameters of this parameter block.
  /// @param[in] parameters The parameters to set this to.
  virtual void setParameters(const double *parameters) {
    // AUTOCAL_ASSERT_TRUE_DBG(Exception, parameters != 0, "Null pointer");
    memcpy(parameters_, parameters, Dimension * sizeof(double));
  }

  /// @}

  /// @name Getters
  /// @{

  /// @brief Get estimate.
  /// \return The estimate.
  virtual parameter_t estimate() const = 0;

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual double *parameters() { return parameters_; }

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual const double *parameters() const { return parameters_; }

  /// @brief Get the parameter dimension.
  /// \return The parameter dimension.
  virtual size_t dimension() const { return Dimension; }

  /// @brief Get the internal minimal parameter dimension.
  /// \return The internal minimal parameter dimension.
  virtual size_t minimalDimension() const { return MinimalDimension; }

  /// @}

  /// @name File read/write - implement in derived class, if needed
  /// @{
  /// \brief Reading from file -- not implemented
  virtual bool read(std::istream & /*not implemented: is*/) { return false; }

  /// \brief Writing to file -- not implemented
  virtual bool write(std::ostream & /*not implemented: os*/) const {
    return false;
  }
  /// @}

protected:
  /// @brief Parameters
  double parameters_[Dimension];
};

} // namespace yac
#endif /* YAC_PARAMETER_BLOCK_SIZED_HPP */
