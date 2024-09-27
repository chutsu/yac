#include "Sim.hpp"

namespace yac {

Sim::Sim() {
  const vec3_t g{0.0, 0.0, 9.81};
  const double imu_rate = 200.0;
  const double circle_r = 10.0;
  const double circle_v = 1.0;
  const double circle_dist = 2.0 * M_PI * circle_r;
  const double time_taken = circle_dist / circle_v;

  const double w = -2.0 * M_PI * (1.0 / time_taken);
  const double theta_init = M_PI;
  const double yaw_init = M_PI / 2.0;

  timestamp_t ts = 0;
  timestamp_t dt_ns = sec2ts(1.0 / imu_rate);
  double theta = theta_init;
  double yaw = yaw_init;

  while (ts <= sec2ts(time_taken)) {
    // IMU pose
    const double rx = circle_r * cos(theta);
    const double ry = circle_r * sin(theta);
    const double rz = 0.0;
    const vec3_t r_WS{rx, ry, rz};
    const mat3_t C_WS = euler321(vec3_t{0.0, 0.0, yaw});
    const mat4_t T_WS = tf(C_WS, r_WS);

    // IMU velocity
    const double vx = -circle_r * w * sin(theta);
    const double vy = circle_r * w * cos(theta);
    const double vz = 0.0;
    const vec3_t v_WS{vx, vy, vz};

    // IMU acceleration
    const double ax = -circle_r * w * w * cos(theta);
    const double ay = -circle_r * w * w * sin(theta);
    const double az = 0.0;
    const vec3_t a_WS{ax, ay, az};

    // IMU angular velocity
    const double wx = 0.0;
    const double wy = 0.0;
    const double wz = w;
    const vec3_t w_WS{wx, wy, wz};

    // Update
    timestamps.push_back(ts);
    poses[ts] = T_WS;
    vel[ts] = v_WS;
    imu_acc[ts] = C_WS.transpose() * (a_WS + g);
    imu_gyr[ts] = C_WS.transpose() * w_WS;

    theta += w * ts2sec(dt_ns);
    yaw += w * ts2sec(dt_ns);
    ts += dt_ns;
  }
}

int Sim::getNumMeasurements() const { return timestamps.size(); }

ImuBuffer Sim::formImuBuffer(const int start_index, const int end_index) const {
  ImuBuffer buffer;
  for (int i = 0; i < (end_index - start_index); i++) {
    const timestamp_t ts = timestamps.at(start_index + i);
    const vec3_t acc = imu_acc.at(ts);
    const vec3_t gyr = imu_gyr.at(ts);
    buffer.add(ts, acc, gyr);
  }
  return buffer;
}

void Sim::save(const std::string &save_path) const {
  // Open csv file
  FILE *csv_file = fopen(save_path.c_str(), "w");

  // Write header
  fprintf(csv_file, "ts,");
  fprintf(csv_file, "x,y,z,");
  fprintf(csv_file, "qx,qy,qz,qw,");
  fprintf(csv_file, "vx,vy,vz,");
  fprintf(csv_file, "ax,ay,az,");
  fprintf(csv_file, "wx,wy,wz\n");

  // Write data
  for (int k = 0; k < getNumMeasurements(); k++) {
    const timestamp_t ts = timestamps.at(k);
    const mat4_t pose = poses.at(ts);
    const vec3_t r = tf_trans(pose);
    const quat_t q = tf_quat(pose);
    const vec3_t v = vel.at(ts);
    const vec3_t acc = imu_acc.at(ts);
    const vec3_t gyr = imu_gyr.at(ts);

    fprintf(csv_file, "%ld,", ts);
    fprintf(csv_file, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(csv_file, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(csv_file, "%f,%f,%f,", v.x(), v.y(), v.z());
    fprintf(csv_file, "%f,%f,%f,", acc.x(), acc.y(), acc.z());
    fprintf(csv_file, "%f,%f,%f\n", gyr.x(), gyr.y(), gyr.z());
  }

  fclose(csv_file);
}

} // namespace yac
