""" Plot gimbal frames """
import yaml
import math

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


if __name__ == "__main__":
  calib_config_file = open("/home/chutsu/calib_gimbal/calib.config", "r")
  calib_config = yaml.safe_load(calib_config_file)

  T_WB = np.eye(4)
  T_BM0 = form_transform(calib_config["gimbal_ext"])
  T_L0M1 = form_transform(calib_config["link0_ext"])
  T_L1M2 = form_transform(calib_config["link1_ext"])
  T_L2E = form_transform(calib_config["end_ext"])
  T_EC0 = form_transform(calib_config["cam0_ext"])
  T_EC1 = form_transform(calib_config["cam1_ext"])

  # C_L2E = euler321(-math.pi / 2.0, math.pi / 2, 0)
  # r_L2E = np.array([0.0, 0.0, 0.0])
  # T_L2E = tf(C_L2E, r_L2E)
  # print(form_pose(T_L2E))

  joint0 = 0.0
  joint1 = 0.0
  joint2 = 0.0
  T_M0L0 = gimbal_joint_transform(joint0)
  T_M1L1 = gimbal_joint_transform(joint1)
  T_M2L2 = gimbal_joint_transform(joint2)

  T_WL0 = T_WB @ T_BM0 @ T_M0L0
  T_WL1 = T_WB @ T_BM0 @ T_M0L0 @ T_L0M1 @ T_M1L1
  T_WL2 = T_WB @ T_BM0 @ T_M0L0 @ T_L0M1 @ T_M1L1 @ T_L1M2 @ T_M2L2
  T_WC0 = T_WL2 @ T_L2E @ T_EC0
  T_WC1 = T_WL2 @ T_L2E @ T_EC1

  plt.figure()
  ax = plt.axes(projection='3d')
  fs = 0.01

  plot_tf(ax, T_WB, name="T_WB", size=fs)
  plot_tf(ax, T_WL0, name="joint0", size=fs)
  plot_tf(ax, T_WL1, name="joint1", size=fs)
  plot_tf(ax, T_WL2, name="joint2", size=fs)
  plot_tf(ax, T_WC0, name="cam0", size=fs)
  plot_tf(ax, T_WC1, name="cam1", size=fs)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()
