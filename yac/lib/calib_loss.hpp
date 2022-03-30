#ifndef YAC_CALIB_LOSS_HPP
#define YAC_CALIB_LOSS_HPP

#include <ceres/ceres.h>

namespace yac {

// CALIB LOSS /////////////////////////////////////////////////////////////////

struct calib_loss_t : public ceres::LossFunction {
  // Data
  std::string type;

  /* Constructor */
  calib_loss_t(const std::string &type_) : type{type_} {}

  /* Destructor */
  virtual ~calib_loss_t() = default;

  /* Evaluate */
  virtual void Evaluate(double sq_norm, double rho[3]) const = 0;
};

// HUBER LOSS /////////////////////////////////////////////////////////////////

class HuberLoss : public calib_loss_t {
public:
  ceres::HuberLoss *loss;

  /* Constructor */
  HuberLoss(const double s);

  /* Destructor */
  virtual ~HuberLoss();

  /* Evaluate */
  virtual void Evaluate(double sq_norm, double rho[3]) const override;
};

// CAUCHY LOSS /////////////////////////////////////////////////////////////////

class CauchyLoss : public calib_loss_t {
public:
  ceres::CauchyLoss *loss;

  /* Constructor */
  CauchyLoss(const double s);

  /* Destructor */
  virtual ~CauchyLoss();

  /* Evaluate */
  virtual void Evaluate(double sq_norm, double rho[3]) const override;
};

// BLAKE-ZISSERMAN LOSS ///////////////////////////////////////////////////////

/**
 * Modified Blake-Zisserman Loss function
 *
 * Reference:
 *
 *   "Online Self-Calibration for Robotic Systems", by Jerome Maye.
 *   https://doi.org/10.3929/ethz-a-010213941
 *
 * and page 618 in:
 *
 *   "MULTIPLE VIEW GEOMETRY IN COMPUTER VISION, by Richard Hartley and Andrew
 *   Zisserman, Cambridge University Press, Cambridge, 2000.
 *
 */
class BlakeZissermanLoss : public calib_loss_t {
public:
  double eps;

  /**
   * Constructor
   *
   * @param df    Dimension of the error term
   * @param p     Probability of the error you want to down-weight. Setting
   *              p to 0.99 implies 99% of squared errors will not be
   *              down-weighted.
   * @param w_q   Weight (between 0 and 1 not inclusive)
   */
  BlakeZissermanLoss(size_t df, double p = 0.999, double w_q = 0.1);

  /* Evaluate */
  virtual void Evaluate(double sq_norm, double rho[3]) const override;
};

} // namespace yac
#endif // YAC_CALIB_LOSS_HPP
