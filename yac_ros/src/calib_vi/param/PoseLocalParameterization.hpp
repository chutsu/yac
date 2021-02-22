#ifndef YAC_POSELOCALPARAMETERIZATION_HPP
#define YAC_POSELOCALPARAMETERIZATION_HPP

#include <ceres/ceres.h>
// #include "yac/util/assert_macros.hpp"
#include "../common/operators.hpp"
#include "../common/Transformation.hpp"
#include "LocalParamizationAdditionalInterfaces.hpp"

namespace yac {

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
class OKVISPoseLocalParameterization : public ::ceres::LocalParameterization,
                                  public LocalParamizationAdditionalInterfaces {
public:
  /// \brief Trivial destructor.
  virtual ~OKVISPoseLocalParameterization() {}

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const {
		return plus(x, delta, x_plus_delta);
	}

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta.
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double *x,
                     const double *x_plus_delta,
                     double *delta) const {
		return minus(x, x_plus_delta, delta);
	}

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
		return plusJacobian(x, jacobian);
	}

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double *x, double *jacobian) const {
		return liftJacobian(x, jacobian);
	}

  // provide these as static for easy use elsewhere:

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  static bool plus(const double *x, const double *delta, double *x_plus_delta) {
		Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_(delta);

		// transform to yac::kinematics framework
		yac::Transformation T(Eigen::Vector3d(x[0], x[1], x[2]),
																					Eigen::Quaterniond(x[6],
																														x[3],
																														x[4],
																														x[5]));

		// call oplus operator in yac::kinematis
		T.oplus(delta_);

		// copy back
		const Eigen::Vector3d r = T.r();
		x_plus_delta[0] = r[0];
		x_plus_delta[1] = r[1];
		x_plus_delta[2] = r[2];
		const Eigen::Vector4d q = T.q().coeffs();
		x_plus_delta[3] = q[0];
		x_plus_delta[4] = q[1];
		x_plus_delta[5] = q[2];
		x_plus_delta[6] = q[3];

		// AUTOCAL_ASSERT_TRUE_DBG(std::runtime_error,
		//                         T.q().norm() - 1.0 < 1e-15,
		//                         "damn.");

		return true;
	}

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double *x, double *jacobian) {
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
		yac::Transformation T(Eigen::Vector3d(x[0], x[1], x[2]),
																					Eigen::Quaterniond(x[6],
																														x[3],
																														x[4],
																														x[5]));
		T.oplusJacobian(Jp);

		return true;
	}

  /// \brief Computes the minimal difference between a variable x and a
  /// perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  static bool minus(const double *x, const double *x_plus_delta, double *delta) {
		delta[0] = x_plus_delta[0] - x[0];
		delta[1] = x_plus_delta[1] - x[1];
		delta[2] = x_plus_delta[2] - x[2];
		const Eigen::Quaterniond q_plus_delta_(x_plus_delta[6],
																					x_plus_delta[3],
																					x_plus_delta[4],
																					x_plus_delta[5]);
		const Eigen::Quaterniond q_(x[6], x[3], x[4], x[5]);
		Eigen::Map<Eigen::Vector3d> delta_q_(&delta[3]);
		delta_q_ = 2 * (q_plus_delta_ * q_.inverse()).coeffs().template head<3>();
		return true;
	}

  /// \brief Computes the Jacobian from minimal space to naively
  /// overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
	static bool liftJacobian(const double *x, double *jacobian) {
		Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J_lift(jacobian);
		const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);

		J_lift.setZero();
		J_lift.topLeftCorner<3, 3>().setIdentity();

		Eigen::Matrix4d Qplus = kinematics::oplus(q_inv);
		Eigen::Matrix<double, 3, 4> Jq_pinv;
		Jq_pinv.bottomRightCorner<3, 1>().setZero();
		Jq_pinv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 2.0;

		J_lift.bottomRightCorner<3, 4>() = Jq_pinv * Qplus;

		return true;
	}

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const { return 7; }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const { return 6; }

  // added convenient check
  bool VerifyJacobianNumDiff(const double *x,
                             double *jacobian,
                             double *jacobianNumDiff) {
		plusJacobian(x, jacobian);
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jp(jacobian);
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> Jpn(jacobianNumDiff);
		double dx = 1e-9;
		Eigen::Matrix<double, 7, 1> xp;
		Eigen::Matrix<double, 7, 1> xm;
		for (size_t i = 0; i < 6; ++i) {
			Eigen::Matrix<double, 6, 1> delta;
			delta.setZero();
			delta[i] = dx;
			Plus(x, delta.data(), xp.data());
			delta[i] = -dx;
			Plus(x, delta.data(), xm.data());
			Jpn.col(i) = (xp - xm) / (2 * dx);
		}
		if ((Jp - Jpn).norm() < 1e-6)
			return true;
		else
			return false;
	}
};

} // namespace yac
#endif /* YAC_POSELOCALPARAMETERIZATION_HPP */
