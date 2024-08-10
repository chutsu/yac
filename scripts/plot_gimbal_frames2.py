#!/usr/bin/env python3
""" Plot gimbal frames """
import os
import sys
import yaml
import math
import glob

from proto import quat2rot
from proto import euler321
from proto import rot2quat
from proto import rotz
from proto import tf
from proto import tf_trans
from proto import tf_quat
from proto import AprilGrid
from proto import plot_tf
from proto import plot_set_axes_equal

import cv2
import numpy as np
import matplotlib.pylab as plt
from matplotlib import animation


def form_transform(pose):
  """ Form 4x4 Transformation Matrix from pose vector """
  x, y, z = pose[0:3]
  qw, qx, qy, qz = pose[3:7]
  r = np.array([x, y, z])
  C = quat2rot(np.array([qw, qx, qy, qz]))
  return tf(C, r)


def form_pose(T):
  """ Form pose vector from 4x4 Transformation matrix """
  r = tf_trans(T)
  q = tf_quat(T)
  return np.array([*r, *q])


def gimbal_joint_transform(theta):
  """ Form 4x4 Transformation Matrix from joint angle """
  r = np.array([0.0, 0.0, 0.0])
  C = rotz(theta)
  return tf(C, r)


def load_calib_config(data_path):
  calib_config_file = open(f"{data_path}/calib_gimbal-results.yaml", "r")
  calib_config = yaml.safe_load(calib_config_file)
  return calib_config

def load_joint_data(data_path):
  joints_file = open(f"{data_path}/joint_angles.dat", "r")
  joints_lines = joints_file.read().split("\n")

  _, num_views = joints_lines[0].split(":")
  _, num_joints = joints_lines[1].split(":")

  timestamps = []
  joint0_data = {}
  joint1_data = {}
  joint2_data = {}
  for i in range(int(num_views)):
    ts, joint0, joint1, joint2 = joints_lines[4 + i].split(",")
    ts = int(ts)
    timestamps.append(ts)
    joint0_data[ts] = float(joint0)
    joint1_data[ts] = float(joint1)
    joint2_data[ts] = float(joint2)

  return {"timestamps": timestamps, "joint0": joint0_data, "joint1": joint1_data, "joint2": joint2_data}


def load_camera_data(data_path):
  cam0_images = {}
  for cam0_file in glob.glob(f"{data_path}/cam0/*.png"):
    fname = os.path.basename(cam0_file).replace(".png", "")
    ts = int(fname)
    img = cv2.imread(cam0_file)
    cam0_images[ts] = img

  cam1_images = {}
  for cam1_file in glob.glob(f"{data_path}/cam1/*.png"):
    fname = os.path.basename(cam1_file).replace(".png", "")
    ts = int(fname)
    img = cv2.imread(cam1_file)
    cam1_images[ts] = img

  cam2_images = {}
  for cam2_file in glob.glob(f"{data_path}/cam2/*.png"):
    fname = os.path.basename(cam2_file).replace(".png", "")
    ts = int(fname)
    img = cv2.imread(cam2_file)
    cam2_images[ts] = img

  cam3_images = {}
  for cam3_file in glob.glob(f"{data_path}/cam3/*.png"):
    fname = os.path.basename(cam3_file).replace(".png", "")
    ts = int(fname)
    img = cv2.imread(cam3_file)
    cam3_images[ts] = img

  return {
    0: cam0_images,
    1: cam1_images,
    2: cam2_images,
    3: cam3_images
  }


def remove_tf(tf):
  tf[0].remove()
  tf[1].remove()
  tf[2].remove()
  tf[3].remove()


if __name__ == "__main__":
  # Load data
  data_path = "/data/gimbal_experiments/exp-240220/calib-240212/calib_gimbal"
  calib_config = load_calib_config(data_path)
  joint_data = load_joint_data(data_path)
  camera_data = load_camera_data(data_path)

  # Transforms
  T_C0M0 = form_transform(calib_config["gimbal_ext"])
  T_L0M1 = form_transform(calib_config["link0_ext"])
  T_L1M2 = form_transform(calib_config["link1_ext"])
  T_L2C2 = form_transform(calib_config["end_ext"])
  T_C0C0 = form_transform(calib_config["cam0_ext"])
  T_C0C1 = form_transform(calib_config["cam1_ext"])
  T_C2C2 = form_transform(calib_config["cam2_ext"])
  T_C2C3 = form_transform(calib_config["cam3_ext"])

  # yapf:disable
  T_WC0 = np.array([
      [0.000000, 0.000000, 1.000000, 0.000000],
      [-1.000000, 0.000000, 0.000000, 0.000000],
      [0.000000, -1.000000, 0.000000, 0.000000],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])
  T_WC1 = T_WC0 @ T_C0C1
  # yapf:enable

  print(f"T_C0M0:\n{T_C0M0}\n")
  print(f"T_L0M1:\n{T_L0M1}\n")
  print(f"T_L1M2:\n{T_L1M2}\n")
  print(f"T_L2C2:\n{T_L2C2}\n")
  print(f"T_C0C1:\n{T_C0C1}\n")
  print(f"T_C2C3:\n{T_C2C3}\n")

  # Setup plot
  fs = 0.01
  fig = plt.figure()
  ax0 = fig.add_subplot(1, 1, 1, projection="3d")
  ax0.set_xlabel("x [m]")
  ax0.set_ylabel("y [m]")
  ax0.set_zlabel("z [m]")

  plt.draw()
  plt.pause(1)
  plt.ion()
  plt.show()

  # Loop over timestamps
  for ts in joint_data["timestamps"]:
    joint0 = joint_data["joint0"][ts]
    joint1 = joint_data["joint1"][ts]
    joint2 = joint_data["joint2"][ts]

    T_M0L0 = gimbal_joint_transform(joint0)
    T_M1L1 = gimbal_joint_transform(joint1)
    T_M2L2 = gimbal_joint_transform(joint2)

    T_WL0 = T_WC0 @ T_C0M0 @ T_M0L0
    T_WL1 = T_WL0 @ T_L0M1 @ T_M1L1
    T_WL2 = T_WL1 @ T_L1M2 @ T_M2L2
    T_WC2 = T_WL2 @ T_L2C2
    T_WC3 = T_WL2 @ T_L2C2 @ T_C2C3

    # Plot frames
    plot_T_WL0 = plot_tf(ax0, T_WL0, name="joint0", size=fs)
    plot_T_WL1 = plot_tf(ax0, T_WL1, name="joint1", size=fs)
    plot_T_WL2 = plot_tf(ax0, T_WL2, name="joint2", size=fs)
    plot_T_WC0 = plot_tf(ax0, T_WC0, name="cam0", size=fs)
    plot_T_WC1 = plot_tf(ax0, T_WC1, name="cam1", size=fs)
    plot_T_WC2 = plot_tf(ax0, T_WC2, name="cam2", size=fs)
    plot_T_WC3 = plot_tf(ax0, T_WC3, name="cam3", size=fs)
    plot_set_axes_equal(ax0)

    # Draw
    plt.draw()
    plt.pause(1)

    # Plot cam0 image
    cv2.imshow("Viz", camera_data[2][ts])
    cv2.waitKey(1)

    # Remove frames
    remove_tf(plot_T_WL0)
    remove_tf(plot_T_WL1)
    remove_tf(plot_T_WL2)
    remove_tf(plot_T_WC0)
    remove_tf(plot_T_WC1)
    remove_tf(plot_T_WC2)
    remove_tf(plot_T_WC3)
