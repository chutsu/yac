#!/usr/bin/env python3
import pandas

import proto
from proto import tf
from proto import tf_trans
from proto import tf_rot
from proto import plot_tf
from proto import plot_set_axes_equal

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d


def plot_imu_measurements(imu_data):
  """ Plot IMU Measurements """
  ts = imu_data['ts'].to_numpy()
  time = (ts - ts[0]) * 1e-9
  acc = imu_data[['ax', 'ay', 'az']].to_numpy()
  gyr = imu_data[['wx', 'wy', 'wz']].to_numpy()

  plt.figure()
  plt.subplot(211)
  plt.plot(time, acc[:, 0], 'r-', label="ax")
  plt.plot(time, acc[:, 1], 'g-', label="ay")
  plt.plot(time, acc[:, 2], 'b-', label="az")
  plt.title("Accelerometer Measurements")
  plt.xlabel("Time [s]")
  plt.ylabel("Acceleration [ms^-2]")
  plt.legend(loc=0)

  plt.subplot(212)
  plt.plot(time, gyr[:, 0], 'r-', label="wx")
  plt.plot(time, gyr[:, 1], 'g-', label="wy")
  plt.plot(time, gyr[:, 2], 'b-', label="wz")
  plt.title("Gyroscope Measurements")
  plt.xlabel("Time [s]")
  plt.ylabel("Angular Velocity [rad s^-1]")
  plt.legend(loc=0)
  plt.subplots_adjust(hspace=0.5)


def plot_imu_velocities(imu_data):
  """ Plot IMU Velocities """
  ts = imu_data['ts'].to_numpy()
  time = (ts - ts[0]) * 1e-9
  vel = imu_data[['vx', 'vy', 'vz']].to_numpy()

  plt.figure()
  plt.plot(time, vel[:, 0], 'r-', label="vx")
  plt.plot(time, vel[:, 1], 'g-', label="vy")
  plt.plot(time, vel[:, 2], 'b-', label="vz")
  plt.title("IMU Velocities")
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [ms^-1]")
  plt.legend(loc=0)


def plot_scene(imu_data):
  """ Plot 3D Scene """
  plt.figure()
  ax = plt.axes(projection='3d')

  for idx, data in imu_data.iterrows():
    if (idx % 50) == 0:
      q_WS = data[['qw', 'qx', 'qy', 'qz']].to_numpy()
      r_WS = data[['rx', 'ry', 'rz']].to_numpy()
      T_WS = tf(q_WS, r_WS)
      plot_tf(ax, T_WS, size=0.3)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)


if __name__ == "__main__":
  imu_data = pandas.read_csv("/tmp/imu_data.csv")
  plot_imu_measurements(imu_data)
  plot_imu_velocities(imu_data)
  plot_scene(imu_data)
  plt.show()
