#include <thread>
#include <vector>

#include <glog/logging.h>

#include "autocal/error/ImuError.hpp"

namespace autocal {

ImuError::ImuError() {
  // residuals_debug = (double * ) calloc(sizeof(double), 15);
  // error_debug = (double *) malloc(sizeof(double) * 15);
  // J.push_back(zeros(15, 6));
  // J.push_back(zeros(15, 9));
  // J.push_back(zeros(15, 6));
  // J.push_back(zeros(15, 9));
  // J.push_back(zeros(15, 1));
}

ImuError::ImuError(const autocal::ImuMeasurementDeque &imuMeasurements,
                   const autocal::ImuParameters &imuParameters,
                   const autocal::Time &t_0,
                   const autocal::Time &t_1) {
  // residuals_debug = (double * ) calloc(sizeof(double), 15);
  // error_debug = (double *) malloc(sizeof(double) * 15);
  // J.push_back(zeros(15, 6));
  // J.push_back(zeros(15, 9));
  // J.push_back(zeros(15, 6));
  // J.push_back(zeros(15, 9));
  // J.push_back(zeros(15, 1));

  setImuMeasurements(imuMeasurements);
  setImuParameters(imuParameters);
  setT0(t_0);
  setT1(t_1);

  AUTOCAL_ASSERT_TRUE_DBG(Exception,
                          t_0 >= imuMeasurements.front().timeStamp,
                          "First IMU measurement included in ImuError is not "
                          "old "
                          "enough!");
  AUTOCAL_ASSERT_TRUE_DBG(Exception,
                          t_1 <= imuMeasurements.back().timeStamp,
                          "Last IMU measurement included in ImuError is not "
                          "new "
                          "enough!");
}

ImuError::~ImuError() {
  // free(residuals_debug);
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::redoPreintegration(
    const Transformation & /*T_WS*/,
    const autocal::SpeedAndBias &speedAndBiases,
    autocal::Time time,
    autocal::Time end) const {
  // Ensure unique access
  std::lock_guard<std::mutex> lock(preintegrationMutex_);

  // Sanity check:
  assert(imuMeasurements_.front().timeStamp <= time);
  if (!(imuMeasurements_.back().timeStamp >= end))
    return -1; // nothing to do...

  // Increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  C_doubleintegral_ = Eigen::Matrix3d::Zero();
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // Cross matrix accumulatrion
  cross_ = Eigen::Matrix3d::Zero();

  // Sub-Jacobians
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  dv_db_g_ = Eigen::Matrix3d::Zero();
  dp_db_g_ = Eigen::Matrix3d::Zero();

  // Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  for (autocal::ImuMeasurementDeque::const_iterator it =
           imuMeasurements_.begin();
       it != imuMeasurements_.end();
       ++it) {

    // Gyro and Accel measurements at start and end
    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;

    // Time delta
    autocal::Time nexttime;
    if ((it + 1) == imuMeasurements_.end()) {
      nexttime = end;
    } else
      nexttime = (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    // Interpolate IMU measurements that are beyond the image timestamp
    if (end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    // Check dt is larger than 0
    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    // Interpolate the first IMU measurements
    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }
    // std::cout << "-- " << i << " [" << it->timeStamp << "]" << std::endl;

    // Ensure measurement integrity
    double sigma_g_c = imuParameters_.sigma_g_c;
    double sigma_a_c = imuParameters_.sigma_a_c;
    if (fabs(omega_S_0[0]) > imuParameters_.g_max ||
        fabs(omega_S_0[1]) > imuParameters_.g_max ||
        fabs(omega_S_0[2]) > imuParameters_.g_max ||
        fabs(omega_S_1[0]) > imuParameters_.g_max ||
        fabs(omega_S_1[1]) > imuParameters_.g_max ||
        fabs(omega_S_1[2]) > imuParameters_.g_max) {
      sigma_g_c *= 100;
      LOG(WARNING) << "gyr saturation";
    }
    if (fabs(acc_S_0[0]) > imuParameters_.a_max ||
        fabs(acc_S_0[1]) > imuParameters_.a_max ||
        fabs(acc_S_0[2]) > imuParameters_.a_max ||
        fabs(acc_S_1[0]) > imuParameters_.a_max ||
        fabs(acc_S_1[1]) > imuParameters_.a_max ||
        fabs(acc_S_1[2]) > imuParameters_.a_max) {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation";
    }

    // Actual propagation
    // clang-format off
    // -- Orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - speedAndBiases.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
    // -- Rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - speedAndBiases.segment<3>(6));
    const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
    const Eigen::Vector3d acc_integral_1 = acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt;
    // -- Rotation matrix double integral:
    C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral_ += acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;
    // clang-format on

    // Jacobian parts
    // clang-format off
    dalpha_db_g_ += C_1 * rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix() * cross_ + rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d acc_S_x = kinematics::crossMx(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g_ + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    dp_db_g_ += dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    // clang-format on

    // Covariance propagation
    // clang-format off
    Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
    // Transform
    F_delta.block<3, 3>(0, 3) = -kinematics::crossMx(acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
    F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F_delta.block<3, 3>(0, 9) = dt * dv_db_g_ + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(0, 12) = -C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    F_delta.block<3, 3>(3, 9) = -dt * C_1;
    F_delta.block<3, 3>(6, 3) = -kinematics::crossMx(0.5 * (C + C_1) * acc_S_true * dt);
    F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // Add noise. Note that transformations with rotation matrices can be
    // ignored, since the noise is isotropic.
    // F_tot = F_delta*F_tot;
    const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
    P_delta_(3, 3) += sigma2_dalpha;
    P_delta_(4, 4) += sigma2_dalpha;
    P_delta_(5, 5) += sigma2_dalpha;
    const double sigma2_v = dt * sigma_a_c * sigma_a_c;
    P_delta_(6, 6) += sigma2_v;
    P_delta_(7, 7) += sigma2_v;
    P_delta_(8, 8) += sigma2_v;
    const double sigma2_p = 0.5 * dt * dt * sigma2_v;
    P_delta_(0, 0) += sigma2_p;
    P_delta_(1, 1) += sigma2_p;
    P_delta_(2, 2) += sigma2_p;
    const double sigma2_b_g = dt * imuParameters_.sigma_gw_c * imuParameters_.sigma_gw_c;
    P_delta_(9, 9) += sigma2_b_g;
    P_delta_(10, 10) += sigma2_b_g;
    P_delta_(11, 11) += sigma2_b_g;
    const double sigma2_b_a = dt * imuParameters_.sigma_aw_c * imuParameters_.sigma_aw_c;
    P_delta_(12, 12) += sigma2_b_a;
    P_delta_(13, 13) += sigma2_b_a;
    P_delta_(14, 14) += sigma2_b_a;
    // clang-format on

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;
    cross_ = cross_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == end)
      break;
  }

  // store the reference (linearisation) point
  speedAndBiases_ref_ = speedAndBiases;

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();
  // proto::print_matrix("P_delta", P_delta_);

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();

  // std::cout << squareRootInformation_ << std::endl;
  // exit(0);

  return i;
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::propagation(const autocal::ImuMeasurementDeque &imuMeasurements,
                          const autocal::ImuParameters &imuParams,
                          autocal::Transformation &T_WS,
                          autocal::SpeedAndBias &speedAndBiases,
                          const autocal::Time &t_start,
                          const autocal::Time &t_end,
                          covariance_t *covariance,
                          jacobian_t *jacobian) {
  // now the propagation
  autocal::Time time = t_start;
  autocal::Time end = t_end;
  // printf("time_start: [%ld]\n", time.toNSec());
  // printf("time_end:   [%ld]\n", end.toNSec());
  // printf("imu front:  [%ld]\n", imuMeasurements.front().timeStamp.toNSec());
  // printf("imu back:   [%ld]\n", imuMeasurements.back().timeStamp.toNSec());

  // sanity check:
  assert(imuMeasurements.front().timeStamp <= time);
  assert(imuMeasurements.back().timeStamp >= end);
  // if (!(imuMeasurements.back().timeStamp >= end))
  //   return -1; // nothing to do...

  // initial condition
  Eigen::Vector3d r_0 = T_WS.r();
  Eigen::Quaterniond q_WS_0 = T_WS.q();
  Eigen::Matrix3d C_WS_0 = T_WS.C();

  // increments (initialise with identity)
  Eigen::Quaterniond Delta_q(1, 0, 0, 0);
  Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double, 15, 15> P_delta = Eigen::Matrix<double, 15, 15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  for (autocal::ImuMeasurementDeque::const_iterator it = imuMeasurements.begin(); it != imuMeasurements.end(); ++it) {
    // printf("-- ts[%d: %ld]\n", i, it->timeStamp.toNSec());
    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;

    // time delta
    autocal::Time nexttime;
    if ((it + 1) == imuMeasurements.end()) {
      nexttime = t_end;
    } else
      nexttime = (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t_end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imuParams.sigma_g_c;
    double sigma_a_c = imuParams.sigma_a_c;

    if (fabs(omega_S_0[0]) > imuParams.g_max ||
        fabs(omega_S_0[1]) > imuParams.g_max ||
        fabs(omega_S_0[2]) > imuParams.g_max ||
        fabs(omega_S_1[0]) > imuParams.g_max ||
        fabs(omega_S_1[1]) > imuParams.g_max ||
        fabs(omega_S_1[2]) > imuParams.g_max) {
      sigma_g_c *= 100;
      LOG(WARNING) << "gyr saturation";
    }

    if (fabs(acc_S_0[0]) > imuParams.a_max ||
        fabs(acc_S_0[1]) > imuParams.a_max ||
        fabs(acc_S_0[2]) > imuParams.a_max ||
        fabs(acc_S_1[0]) > imuParams.a_max ||
        fabs(acc_S_1[1]) > imuParams.a_max ||
        fabs(acc_S_1[2]) > imuParams.a_max) {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation";
    }

    // actual propagation
    // clang-format off
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - speedAndBiases.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - speedAndBiases.segment<3>(6));
    const Eigen::Matrix3d C_integral_1 = C_integral + 0.5 * (C + C_1) * dt;
    const Eigen::Vector3d acc_integral_1 = acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
    // rotation matrix double integral:
    C_doubleintegral += C_integral * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral += acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

    // Jacobian parts
    dalpha_db_g += dt * C_1;
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix() * cross + autocal::rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d acc_S_x = kinematics::crossMx(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g + 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
    dp_db_g += dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
    // clang-format on

    // covariance propagation
    // clang-format off
    if (covariance) {
      Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
      // transform
      F_delta.block<3, 3>(0, 3) = -kinematics::crossMx( acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
      F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
      F_delta.block<3, 3>(0, 9) = dt * dv_db_g + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(0, 12) = -C_integral * dt + 0.25 * (C + C_1) * dt * dt;

      F_delta.block<3, 3>(3, 9) = -dt * C_1;

      F_delta.block<3, 3>(6, 3) = -kinematics::crossMx(0.5 * (C + C_1) * acc_S_true * dt);
      F_delta.block<3, 3>(6, 9) = 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
      P_delta = F_delta * P_delta * F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      // F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(3, 3) += sigma2_dalpha;
      P_delta(4, 4) += sigma2_dalpha;
      P_delta(5, 5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * imuParams.sigma_a_c;
      P_delta(6, 6) += sigma2_v;
      P_delta(7, 7) += sigma2_v;
      P_delta(8, 8) += sigma2_v;
      const double sigma2_p = 0.5 * dt * dt * sigma2_v;
      P_delta(0, 0) += sigma2_p;
      P_delta(1, 1) += sigma2_p;
      P_delta(2, 2) += sigma2_p;
      const double sigma2_b_g = dt * imuParams.sigma_gw_c * imuParams.sigma_gw_c;
      P_delta(9, 9) += sigma2_b_g;
      P_delta(10, 10) += sigma2_b_g;
      P_delta(11, 11) += sigma2_b_g;
      const double sigma2_b_a = dt * imuParams.sigma_aw_c * imuParams.sigma_aw_c;
      P_delta(12, 12) += sigma2_b_a;
      P_delta(13, 13) += sigma2_b_a;
      P_delta(14, 14) += sigma2_b_a;
    }
    // clang-format on

    // memory shift
    Delta_q = Delta_q_1;
    C_integral = C_integral_1;
    acc_integral = acc_integral_1;
    cross = cross_1;
    dv_db_g = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == t_end)
      break;
  }

  // actual propagation output:
  const Eigen::Vector3d g_W = imuParams.g * Eigen::Vector3d(0, 0, 6371009).normalized();
  T_WS.set(r_0 + speedAndBiases.head<3>() * Delta_t + C_WS_0 * (acc_doubleintegral /*-C_doubleintegral*speedAndBiases.segment<3>(6)*/) - 0.5 * g_W * Delta_t * Delta_t, q_WS_0 * Delta_q);
  speedAndBiases.head<3>() += C_WS_0 * (acc_integral /*-C_integral*speedAndBiases.segment<3>(6)*/) - g_W * Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double, 15, 15> &F = *jacobian;
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g, d/db_a
    F.block<3, 3>(0, 3) =
        -kinematics::crossMx(C_WS_0 * acc_doubleintegral);
    F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * Delta_t;
    F.block<3, 3>(0, 9) = C_WS_0 * dp_db_g;
    F.block<3, 3>(0, 12) = -C_WS_0 * C_doubleintegral;
    F.block<3, 3>(3, 9) = -C_WS_0 * dalpha_db_g;
    F.block<3, 3>(6, 3) = -kinematics::crossMx(C_WS_0 * acc_integral);
    F.block<3, 3>(6, 9) = C_WS_0 * dv_db_g;
    F.block<3, 3>(6, 12) = -C_WS_0 * C_integral;
  }

  // overall covariance, if requested
  if (covariance) {
    Eigen::Matrix<double, 15, 15> &P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double, 15, 15> T = Eigen::Matrix<double, 15, 15>::Identity();
    T.topLeftCorner<3, 3>() = C_WS_0;
    T.block<3, 3>(3, 3) = C_WS_0;
    T.block<3, 3>(6, 6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }

  // std::cout << "IMU T_WS:" << std::endl;
  // std::cout << T_WS.T() << std::endl;
  // exit(0);

  return i;
}

// This evaluates the error term and additionally computes the Jacobians.
bool ImuError::Evaluate(double const *const *parameters,
                         double *residuals,
                         double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool ImuError::EvaluateWithMinimalJacobians(double const *const *parameters,
                                            double *residuals,
                                            double **jacobians,
                                            double **jacobiansMinimal) const {
  // Get poses
  const autocal::Transformation
      T_WS_0(Eigen::Vector3d(parameters[0][0],
                             parameters[0][1],
                             parameters[0][2]),
             Eigen::Quaterniond(parameters[0][6],
                                parameters[0][3],
                                parameters[0][4],
                                parameters[0][5]));

  const autocal::Transformation
      T_WS_1(Eigen::Vector3d(parameters[2][0],
                             parameters[2][1],
                             parameters[2][2]),
             Eigen::Quaterniond(parameters[2][6],
                                parameters[2][3],
                                parameters[2][4],
                                parameters[2][5]));

  // Get speed and bias
  autocal::SpeedAndBias speedAndBiases_0;
  autocal::SpeedAndBias speedAndBiases_1;
  for (size_t i = 0; i < 9; ++i) {
    speedAndBiases_0[i] = parameters[1][i];
    speedAndBiases_1[i] = parameters[3][i];
  }

  // Time delay
#if EST_TIMEDELAY == 1
  const double time_delay = parameters[4][0];
  time_delay_last_ = time_delay;
#endif

  // Keep track of evaulation values
  T_WS_0_last_ = T_WS_0;
  T_WS_1_last_ = T_WS_1;
  speedAndBiases_0_last_ = speedAndBiases_0;
  speedAndBiases_1_last_ = speedAndBiases_1;

  // Modify time_start and time_end with timedelay if performing numdiff
#if EST_TIMEDELAY == 1
  autocal::Time time_start = t0_;
  autocal::Time time_end = t1_;
  // -- Modify t0 and t1
  const autocal::Time delay(time_delay);
  const auto delay_nsec = delay.toNSec();
  const auto t_start_nsec = time_start.toNSec();
  const auto t_end_nsec = time_end.toNSec();
  time_start.fromNSec(t_start_nsec + delay_nsec);
  time_end.fromNSec(t_end_nsec + delay_nsec);
  // -- Bound times to prevent being before or after imu measurements
  if (time_start < imuMeasurements_.front().timeStamp) {
    time_start = imuMeasurements_.front().timeStamp;
  }
  if (time_end > imuMeasurements_.back().timeStamp) {
    time_end = imuMeasurements_.back().timeStamp;
  }
#else
  autocal::Time time_start = t0_;
  autocal::Time time_end = t1_;
#endif

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.C();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double Delta_t = (t1_ - t0_).toSec();
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_);
    Delta_b = speedAndBiases_0.tail<6>() - speedAndBiases_ref_.tail<6>();
  }
  int nb_meas = redoPreintegration(T_WS_0, speedAndBiases_0, time_start, time_end);
  Delta_b.setZero();
  // printf("nb_meas: %d\n", nb_meas);
  // printf("nb imu measurements: %zu\n", imuMeasurements_.size());
  // std::cout << "imu front ts: " << imuMeasurements_.front().timeStamp.toNSec() << std::endl;
  // std::cout << "imu back ts:  " << imuMeasurements_.back().timeStamp.toNSec() << std::endl;
  // std::cout << "time_start:   " << time_start.toNSec() << std::endl;
  // std::cout << "time_end:     " << time_end.toNSec() << std::endl;
  // std::cout << "sb0: " << speedAndBiases_0.transpose() << std::endl;
  // std::cout << "sb1: " << speedAndBiases_1.transpose() << std::endl;
  // redo_ = redo_ || (Delta_b.head<3>().norm() * Delta_t > 0.0001);
  // if (redo_) {
  //   redoPreintegration(T_WS_0, speedAndBiases_0, time_start, time_end);
  //   redoCounter_++;
  //   Delta_b.setZero();
  //   redo_ = false;
  // }

  // Actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_); // this is a bit
                                                            // stupid, but
                                                            // shared read-locks
                                                            // only come in
                                                            // C++14
                                                            // clang-format off
    const Eigen::Vector3d g_W = imuParameters_.g * Eigen::Vector3d(0, 0, 6371009).normalized();
                                                            // clang-format on

    // Assign Jacobian w.r.t. x0
    // clang-format off
    Eigen::Matrix<double, 15, 15> F0 = Eigen::Matrix<double, 15, 15>::Identity(); // holds for d/db_g, d/db_a
    const Eigen::Vector3d delta_p_est_W = T_WS_0.r() - T_WS_1.r() + speedAndBiases_0.head<3>() * Delta_t - 0.5 * g_W * Delta_t * Delta_t;
    const Eigen::Vector3d delta_v_est_W = speedAndBiases_0.head<3>() - speedAndBiases_1.head<3>() - g_W * Delta_t;
    const Eigen::Quaterniond Dq = deltaQ(-dalpha_db_g_ * Delta_b.head<3>()) * Delta_q_;
    F0.block<3, 3>(0, 0) = C_S0_W;
    F0.block<3, 3>(0, 3) = C_S0_W * kinematics::crossMx(delta_p_est_W);
    F0.block<3, 3>(0, 6) = C_S0_W * Eigen::Matrix3d::Identity() * Delta_t;
    F0.block<3, 3>(0, 9) = dp_db_g_;
    F0.block<3, 3>(0, 12) = -C_doubleintegral_;
    F0.block<3, 3>(3, 3) = (kinematics::plus(Dq * T_WS_1.q().inverse()) * kinematics::oplus(T_WS_0.q())) .topLeftCorner<3, 3>();
    F0.block<3, 3>(3, 9) = (kinematics::oplus(T_WS_1.q().inverse() * T_WS_0.q()) * kinematics::oplus(Dq)) .topLeftCorner<3, 3>() * (-dalpha_db_g_);
    F0.block<3, 3>(6, 3) = C_S0_W * kinematics::crossMx(delta_v_est_W);
    F0.block<3, 3>(6, 6) = C_S0_W;
    F0.block<3, 3>(6, 9) = dv_db_g_;
    F0.block<3, 3>(6, 12) = -C_integral_;
    // clang-format on

    // assign Jacobian w.r.t. x1
    // clang-format off
    Eigen::Matrix<double, 15, 15> F1 = -Eigen::Matrix<double, 15, 15>::Identity(); // holds for the biases
    F1.block<3, 3>(0, 0) = -C_S0_W;
    F1.block<3, 3>(3, 3) = -(kinematics::plus(Dq) * kinematics::oplus(T_WS_0.q()) * kinematics::plus(T_WS_1.q().inverse())) .topLeftCorner<3, 3>();
    F1.block<3, 3>(6, 6) = -C_S0_W;
    // clang-format on

    // Overall error vector
    // clang-format off
    Eigen::Matrix<double, 15, 1> error;
    // -- Position
    error.segment<3>(0) = C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3, 6>(0, 9) * Delta_b;
    // -- Orientation
    error.segment<3>(3) = 2 * (Dq * (T_WS_1.q().inverse() * T_WS_0.q())) .vec(); // 2*T_WS_0.q()*Dq*T_WS_1.q().inverse();//
    // -- Velocity
    error.segment<3>(6) = C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3, 6>(6, 9) * Delta_b;
    // -- Biases
    error.tail<6>() = speedAndBiases_0.tail<6>() - speedAndBiases_1.tail<6>();
    // clang-format on

    // FATAL("SOMETHING IS WRONG HERE!");
    // squareRootInformation_  = I(15);

    // Error weighting
    // clang-format off
    if (residuals) {
      Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error(residuals);
      weighted_error = squareRootInformation_ * error;
      // r = weighted_error;
      // print_matrix("sqrt_info", squareRootInformation_);
      // exit(0);

      // if (std::isnan(weighted_error(0))) {
      //   printf("pose0_id: %zu\n", pose0_id);
      //   printf("pose1_id: %zu\n", pose1_id);
      //   printf("state0_id: %zu\n", state0_id);
      //   printf("state1_id: %zu\n", state1_id);
      //   printf("timedelay: %zu\n", timedelay_id);
      //   for (const auto &data : imuMeasurements_ ){
      //     std::cout << "gyr: " << data.measurement.gyroscopes.transpose() << std::endl;
      //     std::cout << "acc: " << data.measurement.accelerometers.transpose() << std::endl;
      //   }
      //   std::cout << "residual: " << weighted_error.transpose() << std::endl;
      // }

      // Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error_debug(residuals_debug);
      // weighted_error_debug = squareRootInformation_ * error;
    }
    // clang-format on
    // printf("residual norm: %f\n", (squareRootInformation_ * error).norm());

    // Get the Jacobians
    if (jacobians != NULL) {
      // Sensor pose 0
      // clang-format off
      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_ * F0.block<15, 6>(0, 0);
        // J[0] = J0_minimal;

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = J0_minimal * J_lift;

        if (debug_jacobians_) {
          std::cout << "J0:" << std::endl;
          std::cout << J0 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[0] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J0_minimal_mapped(jacobiansMinimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      // clang-format on

      // Speed and Bias 0
      if (jacobians[1] != NULL) {
        // clang-format off
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = squareRootInformation_ * F0.block<15, 9>(0, 6);
        // J[1] = J1;

        if (debug_jacobians_) {
          std::cout << "J1:" << std::endl;
          std::cout << J1 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1_minimal_mapped(jacobiansMinimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }
      // clang-format on

      // Sensor pose 1
      // clang-format off
      if (jacobians[2] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J2_minimal = squareRootInformation_ * F1.block<15, 6>(0, 0);
        // J[2] = J2_minimal;

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = J2_minimal * J_lift;

        if (debug_jacobians_) {
          std::cout << "J2:" << std::endl;
          std::cout << J2 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[2] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J2_minimal_mapped(jacobiansMinimal[2]);
            J2_minimal_mapped = J2_minimal;
          }
        }
      }
      // clang-format on

      // Speed and Bias 1
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3(
            jacobians[3]);
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);
        // J[3] = J3;

        if (debug_jacobians_) {
          std::cout << "J3:" << std::endl;
          std::cout << J3 << std::endl;
        }

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                J3_minimal_mapped(jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }

      // Time delay between imu and camera
#if EST_TIMEDELAY == 1
      if (jacobians[4] != NULL) {
        // Calculate numerical differentiation: Basically we are going to
        // resuse the ImuError class and find the jacobians for time_delay.
        // We do this by performing a central difference numerical diff.
        //
        //   (f(time_delay + h) - f(time_delay - h)) / (2 * h)
        //
        // Where f(.) is the residual function, in this case the IMU error.
        // time_delay is the input parameter and h is the num-diff step.
        //
        // In future obviously it would be nice to have a analytical diff, but
        // for now this will have to do.

        // -- Variable used to perturb time_delay (a.k.a num-diff step 'h')
        // Same as the delta in Map::isJacobianCorrect()
        const double delta = 1.0e-8;

        // -- Form parameters
        // clang-format off
        // ---- Sensor poses
        const Eigen::Vector3d r_WS_0 = T_WS_0.r();
        const Eigen::Quaterniond q_WS_0 = T_WS_0.q();
        double T_WS_0_data[7] = {r_WS_0(0), r_WS_0(1), r_WS_0(2),
                                 q_WS_0.x(), q_WS_0.y(), q_WS_0.z(), q_WS_0.w()};
        const Eigen::Vector3d r_WS_1 = T_WS_1.r();
        const Eigen::Quaterniond q_WS_1 = T_WS_1.q();
        double T_WS_1_data[7] = {r_WS_1(0), r_WS_1(1), r_WS_1(2),
                                 q_WS_1.x(), q_WS_1.y(), q_WS_1.z(), q_WS_1.w()};

        // ---- Speed and biases
        double *speed_bias_0_data = speedAndBiases_0.data();
        double *speed_bias_1_data = speedAndBiases_1.data();

        // ---- Time delays
        const double time_delay_0 = time_delay + delta;
        const double time_delay_1 = time_delay - delta;

        // ---- Params
        // clang-format on
        const double *imu0_params[5] = {T_WS_0_data, speed_bias_0_data, T_WS_1_data, speed_bias_1_data, &time_delay_0};
        const double *imu1_params[5] = {T_WS_0_data, speed_bias_0_data, T_WS_1_data, speed_bias_1_data, &time_delay_1};

        // -- Form IMU errors and evaluate jacobians
        double imu0_residuals[15] = {0};
        double imu1_residuals[15] = {0};
        const ImuError imu0{imuMeasurements_, imuParameters_, t0_, t1_};
        const ImuError imu1{imuMeasurements_, imuParameters_, t0_, t1_};
        if (imu0.Evaluate(imu0_params, imu0_residuals, nullptr) == false) {
          // eval = false;
          return false;
        }
        if (imu1.Evaluate(imu1_params, imu1_residuals, nullptr) == false) {
          // eval = false;
          return false;
        }

        // -- Perform actual numerical differentiation
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residuals_0(imu0_residuals);
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residuals_1(imu1_residuals);
        const auto J_time_delay = (residuals_1 - residuals_0) * (1 / (2.0 * delta));

        // -- Set time delay jacobian
        Eigen::Map<Eigen::Matrix<double, 15, 1>> J4(jacobians[4]);
        J4 = -1 * J_time_delay;
        // J[4] = J4;

        if (debug_jacobians_) {
          std::cout << "J4:" << std::endl;
          std::cout << J4 << std::endl;
        }

        // If requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[4] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 1>> J4_minimal_mapped(
                jacobiansMinimal[4]);
            J4_minimal_mapped = J4;
          }
        }

      } // jacobian[4]
#endif // if EST_TIMEDELAY == 1

    }   // jacobian
  }     // mutex

  // eval = true;
  return true;
}

} // namespace autocal
