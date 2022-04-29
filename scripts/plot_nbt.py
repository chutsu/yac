#!/usr/bin/env python3
import os
import glob

import pandas
import numpy as np
from numpy import genfromtxt

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

import proto
from proto import load_poses
from proto import plot_tf
from proto import plot_set_axes_equal

def plot_poses():
  # Load fiducial pose
  T_WF = genfromtxt("/tmp/nbt/fiducial_pose.csv", delimiter=",")

  # Plot NBT
  plt.figure()
  ax = plt.axes(projection='3d')

  plot_tf(ax, T_WF, name="T_WF", size=0.1)
  for traj_path in glob.glob("/tmp/nbt/traj/*.csv"):
    idx = 0
    for ts, pose in load_poses(traj_path):
      if idx % 20 == 0:
        plot_tf(ax, pose, size=0.1)
      idx += 1

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()


def plot_state():
  data = pandas.read_csv("/tmp/traj.csv")

  ts = data['#ts'].to_numpy()
  time = (ts - ts[0]) * 1e-9
  pos = data[['rx', 'ry', 'rz']].to_numpy()
  quat = data[['qw', 'qx', 'qy', 'qz']].to_numpy()
  vel = data[['vx', 'vy', 'vz']].to_numpy()
  acc = data[['ax', 'ay', 'az']].to_numpy()
  gyr = data[['wx', 'wy', 'wz']].to_numpy()

  plt.figure()
  plt.subplot(311)
  plt.plot(time, pos[:, 0], 'r-', label="x")
  plt.plot(time, pos[:, 1], 'g-', label="y")
  plt.plot(time, pos[:, 2], 'b-', label="z")
  plt.title("Position")
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")
  plt.legend(loc=0)

  plt.subplot(312)
  plt.plot(time, vel[:, 0], 'r-', label="vx")
  plt.plot(time, vel[:, 1], 'g-', label="vy")
  plt.plot(time, vel[:, 2], 'b-', label="vz")
  plt.title("Velocity")
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [ms^-1]")
  plt.legend(loc=0)

  plt.subplot(313)
  plt.plot(time, acc[:, 0], 'r-', label="ax")
  plt.plot(time, acc[:, 1], 'g-', label="ay")
  plt.plot(time, acc[:, 2], 'b-', label="az")
  plt.title("Acceleration")
  plt.xlabel("Time [s]")
  plt.ylabel("Acceleration [ms^-2]")
  plt.legend(loc=0)

  # plt.subplot(212)
  # rpy = []
  # for q in quat
  # plt.plot(time, pos[:, 0], 'r-', label="ax")
  # plt.plot(time, pos[:, 1], 'g-', label="ay")
  # plt.plot(time, pos[:, 2], 'b-', label="az")
  # plt.title("Position")
  # plt.xlabel("Time [s]")
  # plt.ylabel("Position [m]")
  # plt.legend(loc=0)

  # plt.subplot(215)
  # plt.plot(time, gyr[:, 0], 'r-', label="wx")
  # plt.plot(time, gyr[:, 1], 'g-', label="wy")
  # plt.plot(time, gyr[:, 2], 'b-', label="wz")
  # plt.title("Angular Velocity")
  # plt.xlabel("Time [s]")
  # plt.ylabel("Angular Velocity [rad s^-1]")
  # plt.legend(loc=0)
  # plt.subplots_adjust(hspace=0.95, left=0.1, right=0.95, top=0.95)

# plot_poses()
plot_state()
plt.show()
