#include "calib_nbt.hpp"

namespace yac {

/** TRAJECTORY GENERATION ****************************************************/

lissajous_traj_t::lissajous_traj_t(const std::string &traj_type_,
                                   const timestamp_t ts_start_,
                                   const mat4_t &T_WF_,
                                   const mat4_t &T_FO_,
                                   const real_t R_,
                                   const real_t calib_width_,
                                   const real_t calib_height_,
                                   const real_t T_)
    : traj_type{traj_type_}, ts_start{ts_start_}, T_WF{T_WF_}, T_FO{T_FO_},
      calib_width{calib_width_},
      calib_height{calib_height_}, R{R_}, T{T_}, f{1.0 / T}, w{2.0 * M_PI * f} {
  // Trajectory type
  // -- Figure 8
  if (traj_type == "figure8") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 2.0;
    delta = M_PI;
    psi = 2.0;
    A = calib_width * 0.15;
    B = calib_height * 0.15;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "vert-pan") {
    a = 2.0 * M_PI * 0.0;
    b = 2.0 * M_PI * 1.0;
    delta = 0.0;
    psi = 1.0;
    A = calib_width * 0.5;
    B = calib_height * 0.5;
    yaw_bound = 0.0;
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "horiz-pan") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 0.0;
    delta = 0.0;
    psi = 1.0;
    A = calib_width * 0.5;
    B = calib_height * 0.5;
    yaw_bound = atan2(A, R);
    pitch_bound = 0.0;

  } else if (traj_type == "diag0") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 1.0;
    delta = 0.0;
    psi = 1.0;
    A = calib_width * 0.3534;
    B = calib_height * 0.3534;
    yaw_bound = atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "diag1") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 1.0;
    delta = M_PI;
    psi = 1.0;
    A = calib_width * 0.3534;
    B = calib_height * 0.3534;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else {
    throw std::runtime_error("Implementation Error!");
  }
}

quat_t lissajous_traj_t::get_q_OS(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const real_t w = 2.0 * M_PI * f;
  const real_t k = pow(sin(0.25 * w * t), 2.0);

  const real_t att_x = M_PI + pitch_bound * sin(2 * M_PI * psi * k);
  const real_t att_y = yaw_bound * sin(2 * M_PI * k);
  const real_t att_z = 0.0;

  const real_t phi = att_x;
  const real_t theta = att_y;
  const real_t psi = att_z;

  const real_t c_phi = cos(phi / 2.0);
  const real_t c_theta = cos(theta / 2.0);
  const real_t c_psi = cos(psi / 2.0);
  const real_t s_phi = sin(phi / 2.0);
  const real_t s_theta = sin(theta / 2.0);
  const real_t s_psi = sin(psi / 2.0);

  const real_t qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
  const real_t qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi;
  const real_t qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
  const real_t qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;

  const real_t mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  const quat_t q_OS{qw / mag, qx / mag, qy / mag, qz / mag};

  return q_OS;
}

quat_t lissajous_traj_t::get_q_OS_dot(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const real_t w = 2.0 * M_PI * f;
  const real_t psi_k = psi;

  // clang-format off
  const real_t q_OS_w_dot = -1.5707963267948966*pitch_bound*psi_k*w*sin(0.25*t*w)*sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*cos(0.25*t*w)*cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const real_t q_OS_x_dot = 1.5707963267948966*pitch_bound*psi_k*w*sin(0.25*t*w)*cos(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(0.25*t*w)*cos(6.2831853071795862*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const real_t q_OS_y_dot = -1.5707963267948966*pitch_bound*psi_k*w*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) + 1.5707963267948966*w*yaw_bound*sin(0.25*t*w)*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const real_t q_OS_z_dot = -1.5707963267948966*pitch_bound*psi_k*w*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*cos(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*sin(0.25*t*w)*sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2)))*cos(6.2831853071795862*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*psi_k*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  // clang-format on

  return quat_t{q_OS_w_dot, q_OS_x_dot, q_OS_y_dot, q_OS_z_dot};
}

mat4_t lissajous_traj_t::get_pose(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  // Setup
  const real_t w = 2.0 * M_PI * f;
  const real_t theta = pow(sin(0.25 * w * t), 2);

  // Rotation
  const real_t pitch = pitch_bound * sin(psi * w * T * theta);
  const real_t yaw = yaw_bound * sin(w * T * theta);
  const vec3_t rpy{M_PI + pitch, yaw, 0.0};
  const mat3_t C_OS = euler321(rpy);

  // Position
  const real_t x = A * sin(a * theta + delta);
  const real_t y = B * sin(b * theta);
  const real_t z = sqrt(R * R - x * x - y * y);
  const vec3_t r_OS{x, y, z};

  // Form pose
  const mat4_t T_OS = tf(C_OS, r_OS);
  const mat4_t T_WS = T_WF * T_FO * T_OS;

  return T_WS;
}

vec3_t lissajous_traj_t::get_velocity(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const real_t vx = 0.5 * A * a * w * sin(0.25 * t * w) * cos(0.25 * t * w) *
                    cos(a * pow(sin(0.25 * t * w), 2) + delta);
  const real_t vy = 0.5 * B * b * w * sin(0.25 * t * w) *
                    cos(b * pow(sin(0.25 * t * w), 2)) * cos(0.25 * t * w);
  const real_t vz =
      (-0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
           sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
           cos(0.25 * t * w)) /
      sqrt(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
           pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) + pow(R, 2));
  const vec3_t v_OS{vx, vy, vz};

  // Transform velocity from calib origin frame (O) to world frame (W)
  const mat4_t T_WO = T_WF * T_FO;
  const mat3_t C_WO = tf_rot(T_WO);
  const vec3_t v_WS = C_WO * v_OS;

  return v_WS;
}

vec3_t lissajous_traj_t::get_acceleration(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const real_t ax =
      A * a * pow(w, 2) *
      (-0.03125 * a * (1 - cos(1.0 * t * w)) *
           sin(-1.0 / 2.0 * a * cos(0.5 * t * w) + (1.0 / 2.0) * a + delta) -
       0.125 * pow(sin(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) +
       0.125 * pow(cos(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta));
  const real_t ay =
      -0.25 * B * pow(b, 2) * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
          pow(sin(0.25 * t * w), 2) * pow(cos(0.25 * t * w), 2) -
      0.125 * B * b * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
          cos(b * pow(sin(0.25 * t * w), 2)) +
      0.125 * B * b * pow(w, 2) * cos(b * pow(sin(0.25 * t * w), 2)) *
          pow(cos(0.25 * t * w), 2);
  const real_t az =
      (-0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
           sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
           cos(0.25 * t * w)) *
          (0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
               sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
               cos(a * pow(sin(0.25 * t * w), 2) + delta) +
           0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
               sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
               cos(0.25 * t * w)) /
          pow(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
                  pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) +
                  pow(R, 2),
              3.0 / 2.0) +
      (0.25 * pow(A, 2) * pow(a, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) *
           pow(cos(0.25 * t * w), 2) -
       0.25 * pow(A, 2) * pow(a, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(cos(0.25 * t * w), 2) *
           pow(cos(a * pow(sin(0.25 * t * w), 2) + delta), 2) +
       0.125 * pow(A, 2) * a * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.125 * pow(A, 2) * a * pow(w, 2) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) *
           pow(cos(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) +
       0.25 * pow(B, 2) * pow(b, 2) * pow(w, 2) *
           pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) *
           pow(sin(0.25 * t * w), 2) * pow(cos(0.25 * t * w), 2) -
       0.25 * pow(B, 2) * pow(b, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(cos(b * pow(sin(0.25 * t * w), 2)), 2) *
           pow(cos(0.25 * t * w), 2) +
       0.125 * pow(B, 2) * b * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
           pow(sin(0.25 * t * w), 2) * cos(b * pow(sin(0.25 * t * w), 2)) -
       0.125 * pow(B, 2) * b * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
           cos(b * pow(sin(0.25 * t * w), 2)) * pow(cos(0.25 * t * w), 2)) /
          sqrt(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
               pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) +
               pow(R, 2));
  const vec3_t a_OS{ax, ay, az};

  // Transform velocity from calib origin frame (O) to world frame (W)
  const mat4_t T_WO = T_WF * T_FO;
  const mat3_t C_WO = tf_rot(T_WO);
  const vec3_t a_WS = C_WO * a_OS;

  return a_WS;
}

vec3_t lissajous_traj_t::get_angular_velocity(const timestamp_t ts_k) const {
  const real_t t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  // Angular velocity
  const quat_t q_OS = get_q_OS(ts_k);
  const quat_t q_OS_dot = get_q_OS_dot(ts_k);
  const vec3_t w_OS = 2.0 * (q_OS_dot * q_OS.inverse()).vec();

  // Transform velocity from calib origin frame (O) to world frame (W)
  const mat4_t T_WO = T_WF * T_FO;
  const mat3_t C_WO = tf_rot(T_WO);
  const vec3_t w_WS = C_WO * w_OS;

  return w_WS;
}

int lissajous_traj_t::save(const std::string &save_path) const {
  // Setup output file
  std::ofstream file{save_path};
  if (file.good() != true) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // File Header
  file << "#ts,";                                    // Timestamp
  file << "r_WS_W_x,r_WS_W_y,r_WS_W_z,";             // Position
  file << "q_WS_W_x,q_WS_W_y,q_WS_W_z,q_WS_W_w,";    // Rotation
  file << "v_WS_W_x,v_WS_W_y,v_WS_W_z,";             // Velocity
  file << "a_WS_S_x,a_WS_S_y,a_WS_S_z,";             // Acceleartion
  file << "w_WS_S_x,w_WS_S_y,w_WS_S_z" << std::endl; // Angular velocity

  // Output trajectory timestamps, positions and orientations as csv
  const timestamp_t ts_end = ts_start + sec2ts(T);
  const size_t num_positions = 1000;
  const auto timestamps = linspace(ts_start, ts_end, num_positions);

  for (const auto ts : timestamps) {
    // Sensor pose, velocity, acceleration and angular velocity
    const mat4_t T_WS_W = get_pose(ts);
    const mat3_t C_WS_W = tf_rot(T_WS_W);
    const vec3_t r_WS_W = tf_trans(T_WS_W);
    const quat_t q_WS_W = tf_quat(T_WS_W);
    const vec3_t v_WS_W = get_velocity(ts);
    const vec3_t a_WS_W = get_acceleration(ts);
    const vec3_t w_WS_W = get_angular_velocity(ts);

    // Compute accelerometer and gyroscope measurement
    const vec3_t g_W{0.0, 0.0, 10.0}; // Gravity vector
    const mat3_t C_SW_W = C_WS_W.transpose();
    const vec3_t w_WS_S = C_SW_W * w_WS_W;
    const vec3_t a_WS_S = C_SW_W * (a_WS_W + g_W);

    // Timestamp
    file << ts << ",";
    // Position in world frame
    file << r_WS_W.x() << ",";
    file << r_WS_W.y() << ",";
    file << r_WS_W.z() << ",";
    // Rotation in world frame
    file << q_WS_W.x() << ",";
    file << q_WS_W.y() << ",";
    file << q_WS_W.z() << ",";
    file << q_WS_W.w() << ",";
    // Velocity in world frame
    file << v_WS_W.x() << ",";
    file << v_WS_W.y() << ",";
    file << v_WS_W.z() << ",";
    // Acceleartion in body frame
    file << a_WS_S.x() << ",";
    file << a_WS_S.y() << ",";
    file << a_WS_S.z() << ",";
    // Angular velocity in body frame
    file << w_WS_S.x() << ",";
    file << w_WS_S.y() << ",";
    file << w_WS_S.z() << std::endl;
  }

  // Close file
  file.close();
  return 0;
}

void nbt_lissajous_trajs(const timestamp_t &ts_start,
                         const timestamp_t &ts_end,
                         const calib_target_t &target,
                         const mat4_t &T_WF,
                         lissajous_trajs_t &trajs) {
  // Calculate calib width and height
  const real_t tag_rows = target.tag_rows;
  const real_t tag_cols = target.tag_cols;
  const real_t tag_spacing = target.tag_spacing;
  const real_t tag_size = target.tag_size;
  const real_t spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const real_t spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const real_t calib_width = tag_cols * tag_size + spacing_x;
  const real_t calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const mat4_t T_FO = tf(I(3), calib_center);

  // Lissajous parameters
  const real_t R = std::max(calib_width, calib_height) * 1.0;
  const real_t w = calib_width;
  const real_t h = calib_height;
  const real_t T = ts2sec(ts_end) - ts2sec(ts_start);

  // Add trajectories
  trajs.emplace_back("figure8", ts_start, T_WF, T_FO, R, w, h, T);
  trajs.emplace_back("vert-pan", ts_start, T_WF, T_FO, R, w, h, T);
  trajs.emplace_back("horiz-pan", ts_start, T_WF, T_FO, R, w, h, T);
  trajs.emplace_back("diag0", ts_start, T_WF, T_FO, R, w, h, T);
  trajs.emplace_back("diag1", ts_start, T_WF, T_FO, R, w, h, T);
}

/** SIMULATION ***************************************************************/

aprilgrid_t simulate_aprilgrid(const timestamp_t ts,
                               const calib_target_t &target,
                               const mat4_t &T_FC0,
                               const camera_geometry_t *cam_geom,
                               const camera_params_t *cam_param,
                               const extrinsics_t *cam_exts) {
  // Calibration target
  const int tag_rows = target.tag_rows;
  const int tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  // Camera params and extrinsics
  const auto cam_res = cam_param->resolution;
  const vecx_t params = cam_param->param;
  const mat4_t T_CiC0 = cam_exts->tf().inverse();

  // Simulate AprilGrid observation
  const mat4_t T_C0F = T_FC0.inverse();
  aprilgrid_t grid(ts, tag_rows, tag_cols, tag_size, tag_spacing);

  for (int tag_id = 0; tag_id < (tag_rows * tag_cols); tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
      const vec3_t r_C0Fi = tf_point(T_C0F, r_FFi);
      const vec3_t r_CiFi = tf_point(T_CiC0, r_C0Fi);

      vec2_t z_hat;
      if (cam_geom->project(cam_res, params, r_CiFi, z_hat) == 0) {
        grid.add(tag_id, corner_idx, z_hat);
      }
    }
  }

  return grid;
}

void simulate_cameras(const timestamp_t &ts_start,
                      const timestamp_t &ts_end,
                      const lissajous_traj_t &traj,
                      const calib_target_t &target,
                      const CamIdx2Geometry &cam_geoms,
                      const CamIdx2Parameters &cam_params,
                      const CamIdx2Extrinsics &cam_exts,
                      const real_t cam_rate,
                      const mat4_t &T_WF,
                      camera_data_t &cam_grids,
                      std::map<timestamp_t, mat4_t> &T_WC0_sim) {
  // Simulate camera measurements with AprilGrids that will be observed
  const timestamp_t dt = sec2ts(1.0 / cam_rate);
  timestamp_t ts_k = ts_start;

  while (ts_k <= ts_end) {
    // Calculate transform of fiducial (F) w.r.t. camera (C)
    const mat4_t T_WC0 = traj.get_pose(ts_k);
    const mat4_t T_C0F = T_WC0.inverse() * T_WF;
    const mat4_t T_FC0 = T_C0F.inverse(); // NBV pose

    // Create an AprilGrid that represents what the camera would see if it
    // was positioned at T_C0F
    for (auto &[cam_idx, _] : cam_geoms) {
      UNUSED(_);
      const auto geom = cam_geoms.at(cam_idx).get();
      const auto &cam = cam_params.at(cam_idx).get();
      const auto &ext = cam_exts.at(cam_idx).get();
      const auto &grid =
          simulate_aprilgrid(ts_k, target, T_FC0, geom, cam, ext);
      cam_grids[ts_k][cam_idx] = grid;
    }

    T_WC0_sim[ts_k] = T_WC0;
    ts_k += dt;
  }
}

void simulate_imu(const timestamp_t &ts_start,
                  const timestamp_t &ts_end,
                  const lissajous_traj_t &traj,
                  const imu_params_t &imu_params,
                  timestamps_t &imu_time,
                  vec3s_t &imu_accel,
                  vec3s_t &imu_gyro,
                  mat4s_t &imu_poses,
                  vec3s_t &imu_vels) {
  // Setup
  std::default_random_engine rndeng;
  sim_imu_t sim_imu;
  sim_imu.rate = imu_params.rate;
  sim_imu.sigma_g_c = imu_params.sigma_g_c;
  sim_imu.sigma_a_c = imu_params.sigma_a_c;
  sim_imu.sigma_gw_c = imu_params.sigma_gw_c;
  sim_imu.sigma_aw_c = imu_params.sigma_aw_c;
  sim_imu.g = imu_params.g;

  // Simulate IMU measurements
  const timestamp_t dt = sec2ts(1.0 / imu_params.rate);
  timestamp_t ts_k = ts_start;
  while (ts_k <= ts_end) {
    // Get sensor pose, acceleration and angular velocity
    const mat4_t T_WS_W = traj.get_pose(ts_k);
    const vec3_t v_WS_W = traj.get_velocity(ts_k);
    const vec3_t a_WS_W = traj.get_acceleration(ts_k);
    const vec3_t w_WS_W = traj.get_angular_velocity(ts_k);

    // Simulate IMU measurement
    vec3_t a_WS_S{0.0, 0.0, 0.0};
    vec3_t w_WS_S{0.0, 0.0, 0.0};
    sim_imu_measurement(sim_imu,
                        rndeng,
                        ts_k,
                        T_WS_W,
                        w_WS_W,
                        a_WS_W,
                        a_WS_S,
                        w_WS_S);

    // Update
    imu_time.push_back(ts_k);
    imu_accel.push_back(a_WS_S);
    imu_gyro.push_back(w_WS_S);
    imu_poses.push_back(T_WS_W);
    imu_vels.push_back(v_WS_W);
    ts_k += dt;
  }
}

/** NBT EVALUATION ***********************************************************/

nbt_data_t::nbt_data_t(const calib_vi_t &calib) {
  // Calibration target
  calib_target = calib.calib_target;

  // Imu parameters
  imu_rate = calib.get_imu_rate();
  imu_params = calib.imu_params;
  imu_exts = std::make_unique<extrinsics_t>(calib.imu_exts->tf());
  const auto td_val = calib.time_delay->param(0);
  const auto td_fixed = calib.time_delay->fixed;
  time_delay = std::make_unique<time_delay_t>(td_val, td_fixed);

  // Camera parameters
  cam_rate = calib.get_camera_rate();

  for (const auto &[cam_idx, cam_geom] : calib.cam_geoms) {
    if (cam_geom->type == "PINHOLE-RADTAN4") {
      cam_geoms[cam_idx] = std::make_shared<pinhole_radtan4_t>();
    } else if (cam_geom->type == "PINHOLE-EQUI4") {
      cam_geoms[cam_idx] = std::make_shared<pinhole_equi4_t>();
    } else {
      FATAL("cam_geom->type: [%s] not implemented!", cam_geom->type.c_str());
    }
  }

  for (const auto &[cam_idx, cam_param] : calib.cam_params) {
    cam_params[cam_idx] =
        std::make_shared<camera_params_t>(cam_param->cam_index,
                                          cam_param->resolution,
                                          cam_param->proj_model,
                                          cam_param->dist_model,
                                          cam_param->proj_params(),
                                          cam_param->dist_params(),
                                          cam_param->fixed);
  }

  for (const auto &[cam_idx, exts] : calib.cam_exts) {
    cam_exts[cam_idx] = std::make_shared<extrinsics_t>(exts->tf(), exts->fixed);
  }

  // Fiducial pose
  T_WF = calib.get_fiducial_pose();
}

void nbt_create_timeline(const camera_data_t &cam_grids,
                         const timestamps_t &imu_ts,
                         const vec3s_t &imu_acc,
                         const vec3s_t &imu_gyr,
                         timeline_t &timeline) {
  // Add imu events
  for (size_t i = 0; i < imu_ts.size(); i++) {
    const timestamp_t ts = imu_ts[i];
    const vec3_t a_B = imu_acc[i];
    const vec3_t w_B = imu_gyr[i];
    timeline.add(ts, a_B, w_B);
  }

  // Add aprilgrids observed from all cameras
  for (const auto &[ts, data] : cam_grids) {
    for (const auto &[cam_idx, grid] : data) {
      if (grid.detected) {
        timeline.add(ts, cam_idx, grid);
      }
    }
  }
}

static void schurs_complement(const matx_t &H,
                              const size_t m, // Marginalize
                              const size_t r, // Remain
                              matx_t &H_marg) {
  // Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // entry by its reciprocal, leaving the zeros in place, and transposing
  // the resulting matrix.
  // clang-format off
  matx_t Hmm = H.block(r, r, m, m);
  Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals_inv = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array().inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  // const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
  // if (fabs(inv_check) > 1e-4) {
  //   LOG_WARN("Inverse identity check: %f", inv_check);
  //   LOG_WARN("This is bad ... Usually means marg_residual_t is bad!");
  // }
  // clang-format on

  // Calculate Schur's complement
  // H = [Hrr, Hrm,
  //      Hmr, Hmm]
  const matx_t Hrr = H.block(0, 0, r, r);
  const matx_t Hrm = H.block(0, r, r, m);
  const matx_t Hmr = H.block(r, 0, m, r);

  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
}

int nbt_eval(const lissajous_traj_t &traj,
             const nbt_data_t &nbt_data,
             const matx_t &H,
             matx_t &H_nbt) {
  // Setup
  const timestamp_t ts_start = traj.ts_start;
  const timestamp_t ts_end = ts_start + sec2ts(traj.T);
  // const real_t cam_rate = calib.get_camera_rate();
  // const real_t imu_rate = calib.get_imu_rate();
  // if ((imu_rate - calib.imu_params.rate) > 50) {
  //   LOG_ERROR("(imu_rate - imu_params.rate) > 50");
  //   LOG_ERROR("imu_rate: %f", imu_rate);
  //   LOG_ERROR("imu_params.rate: %f", calib.imu_params.rate);
  //   FATAL("Measured IMU rate is different to configured imu
  //   rate!");
  // }

  // Simulate imu measurements
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  mat4s_t imu_poses;
  vec3s_t imu_vels;
  simulate_imu(ts_start,
               ts_end,
               traj,
               nbt_data.imu_params,
               imu_ts,
               imu_acc,
               imu_gyr,
               imu_poses,
               imu_vels);

  // Simulate camera frames
  camera_data_t cam_grids;
  std::map<timestamp_t, mat4_t> cam_poses;
  simulate_cameras(ts_start,
                   ts_end,
                   traj,
                   nbt_data.calib_target,
                   nbt_data.cam_geoms,
                   nbt_data.cam_params,
                   nbt_data.cam_exts,
                   nbt_data.cam_rate,
                   nbt_data.T_WF,
                   cam_grids,
                   cam_poses);

  // Add NBT data into calibrator
  timeline_t timeline;
  nbt_create_timeline(cam_grids, imu_ts, imu_acc, imu_gyr, timeline);

  // Form calibrator
  calib_vi_t calib_nbt{nbt_data.calib_target};
  calib_nbt.verbose = false;
  calib_nbt.enable_marginalization = false;
  // -- Add IMU
  calib_nbt.add_imu(nbt_data.imu_params,
                    nbt_data.imu_exts->tf(),
                    nbt_data.time_delay->param(0),
                    nbt_data.imu_exts->fixed,
                    nbt_data.time_delay->fixed);
  // -- Add cameras
  for (const auto &[cam_idx, cam] : nbt_data.cam_params) {
    const auto cam_ext = nbt_data.cam_exts.at(cam_idx).get();
    calib_nbt.add_camera(cam_idx,
                         cam->resolution,
                         cam->proj_model,
                         cam->dist_model,
                         cam->proj_params(),
                         cam->dist_params(),
                         cam_ext->tf(),
                         cam->fixed,
                         cam_ext->fixed);
  }
  // -- Add fiducial pose
  calib_nbt.fiducial = std::make_unique<fiducial_t>(nbt_data.T_WF);
  calib_nbt.problem->AddParameterBlock(calib_nbt.fiducial->param.data(),
                                       FIDUCIAL_PARAMS_SIZE);
  if (calib_nbt.fiducial->param.size() == 7) {
    calib_nbt.problem->SetParameterization(calib_nbt.fiducial->param.data(),
                                           &calib_nbt.pose_plus);
  }

  for (const auto &ts : timeline.timestamps) {
    const auto kv = timeline.data.equal_range(ts);

    // Handle multiple events in the same timestamp
    for (auto it = kv.first; it != kv.second; it++) {
      const auto event = it->second;

      // Aprilgrid event
      if (auto grid_event = dynamic_cast<aprilgrid_event_t *>(event)) {
        auto cam_idx = grid_event->cam_idx;
        auto &grid = grid_event->grid;
        calib_nbt.add_measurement(cam_idx, grid);
      }

      // Imu event
      if (auto imu_event = dynamic_cast<imu_event_t *>(event)) {
        const auto ts = imu_event->ts;
        const vec3_t &acc = imu_event->acc;
        const vec3_t &gyr = imu_event->gyr;
        calib_nbt.add_measurement(ts, acc, gyr);
      }
    }
  }
  // calib_nbt.verbose = true;
  calib_nbt.max_iter = 5;
  calib_nbt.solve();

  // Calculate info
  if (calib_nbt.recover_calib_info(H_nbt) != 0) {
    return -1;
  }
  H_nbt = H + H_nbt;

  return 0;
}

int nbt_find(const lissajous_trajs_t &trajs,
             const nbt_data_t &nbt_data,
             const matx_t &H,
             const bool verbose,
             real_t *calib_info_k,
             real_t *calib_info_kp1) {
  // Calculate initial info
  const matx_t calib_covar = H.inverse();
  const real_t info_k = std::log(calib_covar.determinant()) / std::log(2.0);

  // Evaluate trajectories
  std::map<int, real_t> info_kp1;
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    // Evaluate
    matx_t H_nbt;
    const int retval = nbt_eval(trajs[traj_idx], nbt_data, H, H_nbt);

    // Check evaluation
    if (retval != 0) {
      if (verbose) {
        LOG_WARN("Failed to evaulate NBT traj [%ld]", traj_idx);
      }
      info_kp1[traj_idx] = 0.0;
      continue;
    }

    // Calculate information
    const matx_t covar = H_nbt.inverse();
    info_kp1[traj_idx] = std::log(covar.determinant()) / std::log(2.0);
  }

  // Find max information gain
  int best_idx = -1;
  real_t best_gain = 0.0;
  for (size_t traj_idx = 0; traj_idx < trajs.size(); traj_idx++) {
    const auto info_gain = 0.5 * (info_k - info_kp1[traj_idx]);
    if (info_gain > best_gain) {
      best_idx = traj_idx;
      best_gain = info_gain;
    }

    if (verbose) {
      printf("traj_idx: %ld ", traj_idx);
      printf("info_k: %f ", info_k);
      printf("info_kp1: %f ", info_kp1[traj_idx]);
      printf("info_gain: %f ", info_gain);
      printf("\n");
    }
  }

  // Print results
  if (verbose) {
    printf("\nFind NBT:\n");
    printf("best_traj: %d\n", best_idx);
    printf("best_gain: %f\n", best_gain);
  }

  // Update
  if (calib_info_k) {
    *calib_info_k = info_k;
  }
  if (calib_info_kp1) {
    *calib_info_kp1 = info_kp1[best_idx];
  }

  return best_idx;
}

} // namespace yac
