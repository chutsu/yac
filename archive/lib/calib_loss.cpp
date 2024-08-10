#include "calib_loss.hpp"
#include <boost/math/distributions/chi_squared.hpp>

namespace yac {

// HUBER LOSS /////////////////////////////////////////////////////////////////

HuberLoss::HuberLoss(const double s) : calib_loss_t{"HUBER"} {
  loss = new ceres::HuberLoss(s);
}

HuberLoss::~HuberLoss() {
  if (loss) {
    delete loss;
  }
}

void HuberLoss::Evaluate(double sq_norm, double rho[3]) const {
  loss->Evaluate(sq_norm, rho);
}

// CAUCHY LOSS ////////////////////////////////////////////////////////////////

CauchyLoss::CauchyLoss(const double s) : calib_loss_t{"CAUCHY"} {
  loss = new ceres::CauchyLoss(s);
}

CauchyLoss::~CauchyLoss() {
  if (loss) {
    delete loss;
  }
}

void CauchyLoss::Evaluate(double sq_norm, double rho[3]) const {
  loss->Evaluate(sq_norm, rho);
}

// BLAKE-ZISSERMAN LOSS ///////////////////////////////////////////////////////

BlakeZissermanLoss::BlakeZissermanLoss(size_t df, double p, double w_q)
    : calib_loss_t{"BLAKE-ZISSERMAN"} {
  const auto chisq_dist = boost::math::chi_squared_distribution<>(df);
  const auto q = boost::math::quantile(chisq_dist, p);
  eps = ((1.0 - w_q) / w_q) * exp(-q);
}

void BlakeZissermanLoss::Evaluate(double sq_norm, double rho[3]) const {
  const double x = eps * exp(sq_norm) + 1.0;
  const double x2 = x * x;
  const double x3 = x2 * x;

  rho[0] = -log(eps + exp(-sq_norm));
  rho[1] = 1.0 / x;
  rho[2] = (x - x2) / x3;

  rho[1] = std::max(std::numeric_limits<double>::min(), rho[1]);
  rho[2] = std::max(std::numeric_limits<double>::min(), rho[2]);
}

} // namespace yac
