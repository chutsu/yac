#!/usr/bin/python3 env
"""
Lissajous Prototype Script
"""
from math import pi
from math import atan2

import matplotlib.pylab as plt
from matplotlib import animation
from mpl_toolkits import mplot3d

import numpy as np
from numpy import cos
from numpy import sin
from numpy import sqrt

import sympy
from sympy import ccode

from proto import tf
from proto import tf_trans
from proto import tf_rot
from proto import tf_quat
from proto import plot_tf
from proto import plot_set_axes_equal
from proto import Exp
from proto import skew
from proto import skew_inv
from proto import euler321
from proto import rot2euler
from proto import rot2quat
from proto import quat_conj
from proto import quat_omega
from proto import quat2rot
from proto import quat2euler
from proto import quat_mul
from proto import quat_normalize
from proto import AprilGrid


def sympy_diff(gencode=False):
  """ Use Sympy to calcualte jacobians """
  f, t = sympy.symbols("f t")
  A, a, delta = sympy.symbols("A a delta")
  B, b = sympy.symbols("B b")
  R = sympy.symbols("R")
  w = sympy.symbols("w")  # w = 2.0 * pi * f

  T = 1.0 / f
  theta = sympy.sin(w * t * 1.0 / 4.0)**2
  x = A * sympy.sin(a * theta + delta)
  y = B * sympy.sin(b * theta)
  z = sympy.sqrt(R**2 - x**2 - y**2)

  vx = sympy.simplify(sympy.diff(x, t))
  vy = sympy.diff(y, t)
  vz = sympy.diff(z, t)

  ax = sympy.simplify(sympy.diff(vx, t))
  ay = sympy.diff(vy, t)
  az = sympy.diff(vz, t)

  yaw_bound = sympy.symbols("yaw_bound")
  pitch_bound = sympy.symbols("pitch_bound")
  psi_k = sympy.symbols("psi_k")

  att_x = pi + pitch_bound * sympy.sin(psi_k * w * T * theta)
  att_y = yaw_bound * sympy.sin(w * T * theta)
  att_z = 0.0

  psi = att_z
  theta = att_y
  phi = att_x

  cpsi = sympy.cos(psi)
  spsi = sympy.sin(psi)
  ctheta = sympy.cos(theta)
  stheta = sympy.sin(theta)
  cphi = sympy.cos(phi)
  sphi = sympy.sin(phi)

  C11 = cpsi * ctheta
  C21 = spsi * ctheta
  C31 = -stheta

  C12 = cpsi * stheta * sphi - spsi * cphi
  C22 = spsi * stheta * sphi + cpsi * cphi
  C32 = ctheta * sphi

  C13 = cpsi * stheta * cphi + spsi * sphi
  C23 = spsi * stheta * cphi - cpsi * sphi
  C33 = ctheta * cphi

  C_WS = np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])
  C_WS_dot = sympy.diff(C_WS, t)

  # print("# Position")
  # if gencode:
  #   print(f"x = {ccode(x)}")
  #   print(f"y = {ccode(y)}")
  #   print(f"z = {ccode(z)}")
  # else:
  #   print(f"x = {x}")
  #   print(f"y = {y}")
  #   print(f"z = {z}")
  # print("")

  # print("# Velocity")
  # if gencode:
  #   print(f"vx = {ccode(vx)}")
  #   print(f"vy = {ccode(vy)}")
  #   print(f"vz = {ccode(vz)}")
  # else:
  #   print(f"vx = {vx}")
  #   print(f"vy = {vy}")
  #   print(f"vz = {vz}")
  # print("")

  # print("# Acceleration")
  # if gencode:
  #   print(f"ax = {ccode(ax)}")
  #   print(f"ay = {ccode(ay)}")
  #   print(f"az = {ccode(az)}")
  # else:
  #   print(f"ax = {ax}")
  #   print(f"ay = {ay}")
  #   print(f"az = {az}")
  # print("")

  # print("# Rotation")
  # print(f"{sympy.python(C_WS_dot)}")
  # print(f"{C_WS_dot}")
  # print("")

  # print("# Angular Velocity")
  # if gencode:
  #   print(f"wx = {ccode(wx)}")
  #   print(f"wy = {ccode(wy)}")
  #   print(f"wz = {ccode(wz)}")
  # else:
  #   print(f"wx = {wx}")
  #   print(f"wy = {wy}")
  #   print(f"wz = {wz}")
  # print("")


def sympy_diff_quat():
  t = sympy.symbols("t")
  T = sympy.symbols("T")
  yaw_bound = sympy.symbols("yaw_bound")
  pitch_bound = sympy.symbols("pitch_bound")
  offset = sympy.symbols("offset")

  f = 1 / T
  w = 2.0 * sympy.pi * f
  k = sympy.sin(0.25 * w * t)**2

  att_x = pi + pitch_bound * sympy.sin(2 * sympy.pi * offset * k)
  att_y = yaw_bound * sympy.sin(2 * sympy.pi * k)
  att_z = 0.0

  phi = att_x
  theta = att_y
  psi = att_z

  c_phi = sympy.cos(phi / 2.0)
  c_theta = sympy.cos(theta / 2.0)
  c_psi = sympy.cos(psi / 2.0)
  s_phi = sympy.sin(phi / 2.0)
  s_theta = sympy.sin(theta / 2.0)
  s_psi = sympy.sin(psi / 2.0)

  qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi
  qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi
  qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi
  qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi

  mag = sympy.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
  q = np.array([qw / mag, qx / mag, qy / mag, qz / mag])

  print(sympy.diff(q, t))


# sympy_diff_quat()

# pitch_bound = 1.0
# yaw_bound = 1.0
# t = 0.1
# T = 3.0
# offset = 1.0

# q_OS = quat(pitch_bound, yaw_bound, t, T, offset)
# q_OS_dot = quat_dot(pitch_bound, yaw_bound, t, T, offset)
# w_L = (2.0 * quat_mul(quat_conj(q_OS), q_OS_dot))[1:4]
# w_G = (2.0 * quat_mul(q_OS_dot, quat_conj(q_OS)))[1:4]

# # print(w_L)
# print(q_OS_dot)
# print(0.5 * quat_mul(q_OS, np.array([0.0, w_L[0], w_L[1], w_L[2]])))
# print(0.5 * quat_mul(np.array([0.0, w_G[0], w_G[1], w_G[2]]), q_OS))
# exit(0)


class LissajousTraj:
  """ Lissajous Trajectory """
  def __init__(self, traj_type, fiducial_pose, **kwargs):
    """ Constructor """
    self.T_WF = fiducial_pose
    self.calib_target = AprilGrid()
    self.calib_dims = self.calib_target.get_dimensions()
    self.T_FO = tf(np.eye(3), self.calib_target.get_center())

    self.R = kwargs.get("R",
                        np.max(self.calib_dims) *
                        1.0)  # Distance from calib target
    self.A = kwargs.get("A", self.calib_dims[0] * 0.5)  # Amplitude in x-axis
    self.B = kwargs.get("B", self.calib_dims[1] * 0.5)  # Amplitude in y-axis
    self.T = kwargs.get("T", 3.0)  # Period - Time it takes to complete [secs]
    self.f = 1.0 / self.T
    self.w = 2.0 * pi * self.f
    self.t = np.linspace(0, self.T, 250)

    # Trajectory type
    # -- Figure 8
    if traj_type == "figure8":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 2.0
      self.delta = pi
      self.psi = 2.0
      self.yaw_bound = -atan2(self.A, self.R)
      self.pitch_bound = -atan2(self.B, self.R)

    # -- Vertical pan
    elif traj_type == "vert-pan":
      self.a = 2.0 * pi * 0.0
      self.b = 2.0 * pi * 1.0
      self.delta = 0.0
      self.psi = 1.0
      self.yaw_bound = 0.0
      self.pitch_bound = -atan2(self.B, self.R)

    # -- Horizontal pan
    elif traj_type == "horiz-pan":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 0.0
      self.delta = 0.0
      self.psi = 1.0
      self.yaw_bound = atan2(self.A, self.R)
      self.pitch_bound = 0.0

    # -- Diagonal (bottom left to top right) pan
    elif traj_type == "diag0":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 1.0
      self.delta = 0.0
      self.psi = 1.0
      self.yaw_bound = atan2(self.A, self.R)
      self.pitch_bound = -atan2(self.B, self.R)

    # -- Diagonal (top left to bottom right) pan
    elif traj_type == "diag1":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 1.0
      self.delta = pi
      self.psi = 1.0
      self.yaw_bound = -atan2(self.A, self.R)
      self.pitch_bound = -atan2(self.B, self.R)

    else:
      raise RuntimeError(f"Invalid traj_type[{traj_type}]")

  def get_pose(self, t):
    """ Return pose """
    # Setup
    w = 2.0 * pi * self.f
    theta = np.sin(w * t * 1.0 / 4.0)**2

    # Rotation
    pitch = self.pitch_bound * np.sin(2.0 * pi * self.psi * theta)
    yaw = self.yaw_bound * np.sin(2.0 * pi * theta)
    C_OS = euler321(0.0, yaw, pi + pitch)

    # Position
    x = self.A * np.sin(self.a * theta + self.delta)
    y = self.B * np.sin(self.b * theta)
    z = np.sqrt(self.R**2 - x**2 - y**2)
    r_OS = np.array([x, y, z])

    # Form pose
    T_OS = tf(C_OS, r_OS)
    T_WS = self.T_WF @ self.T_FO @ T_OS

    return T_WS

  def get_q_OS(self, t):
    """ Return q_OS """
    pitch_bound = self.pitch_bound
    yaw_bound = self.yaw_bound
    offset = self.psi
    f = 1 / self.T
    w = 2.0 * np.pi * f
    k = np.sin(0.25 * w * t)**2

    att_x = pi + pitch_bound * np.sin(2 * np.pi * offset * k)
    att_y = yaw_bound * np.sin(2 * np.pi * k)
    att_z = 0.0

    phi = att_x
    theta = att_y
    psi = att_z

    c_phi = np.cos(phi / 2.0)
    c_theta = np.cos(theta / 2.0)
    c_psi = np.cos(psi / 2.0)
    s_phi = np.sin(phi / 2.0)
    s_theta = np.sin(theta / 2.0)
    s_psi = np.sin(psi / 2.0)

    qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi
    qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi
    qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi
    qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi

    mag = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    q = np.array([qw / mag, qx / mag, qy / mag, qz / mag])
    return q

  def get_q_OS_dot(self, t):
    """ Derivative of q_OS """
    pitch_bound = self.pitch_bound
    yaw_bound = self.yaw_bound
    offset = self.psi
    T = self.T
    return np.array([
        -1.0 * pi**2 * offset * pitch_bound * sin(pi * t / (2 * T)) *
        sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) *
        cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t /
                                               (2 * T))**2)) * cos(pi * t /
                                                                   (2 * T)) *
        cos(2 * pi * offset * sin(pi * t / (2 * T))**2) / (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)) - 1.0 * pi**2 * yaw_bound *
        sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2)) *
        sin(pi * t / (2 * T)) * cos(2 * pi * sin(pi * t /
                                                 (2 * T))**2) * cos(pi * t /
                                                                    (2 * T)) *
        cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)),
        1.0 * pi**2 * offset * pitch_bound * sin(pi * t / (2 * T)) *
        cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2)) *
        cos(pi * t / (2 * T)) * cos(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) *
        cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)) - 1.0 * pi**2 * yaw_bound *
        sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t /
                                               (2 * T))**2)) * sin(pi * t /
                                                                   (2 * T)) *
        sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) * cos(2 * pi * sin(pi * t /
                                                (2 * T))**2) * cos(pi * t /
                                                                   (2 * T)) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)), -1.0 * pi**2 * offset * pitch_bound *
        sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t /
                                               (2 * T))**2)) * sin(pi * t /
                                                                   (2 * T)) *
        sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) * cos(pi * t / (2 * T)) *
        cos(2 * pi * offset * sin(pi * t / (2 * T))**2) / (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)) +
        1.0 * pi**2 * yaw_bound * sin(pi * t /
                                      (2 * T)) * cos(2 * pi * sin(pi * t /
                                                                  (2 * T))**2) *
        cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t /
                                               (2 * T))**2)) * cos(pi * t /
                                                                   (2 * T)) *
        cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)), -1.0 * pi**2 * offset * pitch_bound *
        sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2)) *
        sin(pi * t /
            (2 * T)) * cos(pi * t /
                           (2 * T)) * cos(2 * pi * offset * sin(pi * t /
                                                                (2 * T))**2) *
        cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2)) - 1.0 * pi**2 * yaw_bound * sin(pi * t /
                                                                      (2 * T)) *
        sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                          (2 * T))**2) +
            1.5707963267949) * cos(2 * pi * sin(pi * t / (2 * T))**2) *
        cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t /
                                               (2 * T))**2)) * cos(pi * t /
                                                                   (2 * T)) /
        (T * sqrt(
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 +
            sin(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2 *
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 +
            cos(0.5 * yaw_bound * sin(2 * pi * sin(pi * t / (2 * T))**2))**2 *
            cos(0.5 * pitch_bound * sin(2 * pi * offset * sin(pi * t /
                                                              (2 * T))**2) +
                1.5707963267949)**2))
    ])

  def get_velocity(self, t):
    """ Return velocity """
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    w = 2.0 * pi * self.f

    vx = 0.5 * A * a * w * np.sin(0.25 * t * w) * np.cos(
        0.25 * t * w) * np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta)
    vy = 0.5 * B * b * w * np.sin(0.25 * t * w) * np.cos(
        b * pow(np.sin(0.25 * t * w), 2)) * np.cos(0.25 * t * w)
    vz = (-0.5 * pow(A, 2) * a * w * np.sin(0.25 * t * w) *
          np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta) * np.cos(
              0.25 * t * w) * np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta) -
          0.5 * pow(B, 2) * b * w * np.sin(b * pow(np.sin(0.25 * t * w), 2)) *
          np.sin(0.25 * t * w) * np.cos(b * pow(np.sin(0.25 * t * w), 2)) *
          np.cos(0.25 * t * w)) / np.sqrt(
              -pow(A, 2) *
              pow(np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta), 2) -
              pow(B, 2) * pow(np.sin(b * pow(np.sin(0.25 * t * w), 2)), 2) +
              pow(R, 2))
    v_OS = np.array([vx, vy, vz])

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    v_WS = C_WO @ v_OS

    return v_WS

  def get_acceleration(self, t):
    """ Return acceleration """
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    w = 2.0 * pi * self.f

    ax = A * a * pow(w, 2) * (-0.03125 * a * (1 - np.cos(1.0 * t * w)) *
                              np.sin(-1.0 / 2.0 * a * np.cos(0.5 * t * w) +
                                     (1.0 / 2.0) * a + delta) -
                              0.125 * pow(np.sin(0.25 * t * w), 2) *
                              np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta) +
                              0.125 * pow(np.cos(0.25 * t * w), 2) *
                              np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta))
    ay = -0.25 * B * pow(b, 2) * pow(w, 2) * np.sin(
        b * pow(np.sin(0.25 * t * w), 2)) * pow(np.sin(0.25 * t * w), 2) * pow(
            np.cos(0.25 * t * w), 2) - 0.125 * B * b * pow(w, 2) * pow(
                np.sin(0.25 * t * w), 2) * np.cos(
                    b * pow(np.sin(0.25 * t * w), 2)) + 0.125 * B * b * pow(
                        w, 2) * np.cos(b * pow(np.sin(0.25 * t * w), 2)) * pow(
                            np.cos(0.25 * t * w), 2)
    az = (
        -0.5 * pow(A, 2) * a * w * np.sin(0.25 * t * w) *
        np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta) * np.cos(
            0.25 * t * w) * np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta) -
        0.5 * pow(B, 2) * b * w * np.sin(b * pow(np.sin(0.25 * t * w), 2)) *
        np.sin(0.25 * t * w) * np.cos(b * pow(np.sin(0.25 * t * w), 2)) *
        np.cos(0.25 * t * w)
    ) * (0.5 * pow(A, 2) * a * w * np.sin(0.25 * t * w) *
         np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta) *
         np.cos(0.25 * t * w) * np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta)
         + 0.5 * pow(B, 2) * b * w * np.sin(b * pow(np.sin(0.25 * t * w), 2)) *
         np.sin(0.25 * t * w) * np.cos(b * pow(np.sin(0.25 * t * w), 2)) *
         np.cos(0.25 * t * w)) / pow(
             -pow(A, 2) *
             pow(np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta), 2) -
             pow(B, 2) * pow(np.sin(b * pow(np.sin(0.25 * t * w), 2)), 2) +
             pow(R, 2), 3.0 / 2.0
         ) + (0.25 * pow(A, 2) * pow(a, 2) * pow(w, 2) *
              pow(np.sin(0.25 * t * w), 2) *
              pow(np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta), 2) *
              pow(np.cos(0.25 * t * w), 2) - 0.25 * pow(A, 2) * pow(a, 2) *
              pow(w, 2) * pow(np.sin(0.25 * t * w), 2) *
              pow(np.cos(0.25 * t * w), 2) *
              pow(np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta), 2) +
              0.125 * pow(A, 2) * a * pow(w, 2) * pow(np.sin(0.25 * t * w), 2) *
              np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta) *
              np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta) -
              0.125 * pow(A, 2) * a * pow(w, 2) *
              np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta) *
              pow(np.cos(0.25 * t * w), 2) *
              np.cos(a * pow(np.sin(0.25 * t * w), 2) + delta) +
              0.25 * pow(B, 2) * pow(b, 2) * pow(w, 2) *
              pow(np.sin(b * pow(np.sin(0.25 * t * w), 2)), 2) *
              pow(np.sin(0.25 * t * w), 2) * pow(np.cos(0.25 * t * w), 2) - 0.25
              * pow(B, 2) * pow(b, 2) * pow(w, 2) * pow(np.sin(0.25 * t * w), 2)
              * pow(np.cos(b * pow(np.sin(0.25 * t * w), 2)), 2) *
              pow(np.cos(0.25 * t * w), 2) + 0.125 * pow(B, 2) * b * pow(w, 2) *
              np.sin(b * pow(np.sin(0.25 * t * w), 2)) *
              pow(np.sin(0.25 * t * w), 2) *
              np.cos(b * pow(np.sin(0.25 * t * w), 2)) - 0.125 * pow(B, 2) * b *
              pow(w, 2) * np.sin(b * pow(np.sin(0.25 * t * w), 2)) *
              np.cos(b * pow(np.sin(0.25 * t * w), 2)) *
              pow(np.cos(0.25 * t * w), 2)) / np.sqrt(
                  -pow(A, 2) *
                  pow(np.sin(a * pow(np.sin(0.25 * t * w), 2) + delta), 2) -
                  pow(B, 2) * pow(np.sin(b * pow(np.sin(0.25 * t * w), 2)), 2) +
                  pow(R, 2))
    a_OS = np.array([ax, ay, az])

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    a_WS = C_WO @ a_OS

    return a_WS

  def get_angular_velocity(self, t):
    """ Return angular velocity """
    w = 2.0 * pi * self.f
    f = self.f
    psi = self.psi

    q_OS = self.get_q_OS(t)
    q_OS_dot = self.get_q_OS_dot(t)
    w_OS = (2.0 * quat_mul(q_OS_dot, quat_conj(q_OS)))[1:4]

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    w_WS = C_WO @ w_OS

    return w_WS

  def plot_xy(self):
    """ Plot XY """
    positions = []
    for t in self.t:
      T_WS = self.get_pose(t)
      positions.append(tf_trans(T_WS))
    positions = np.array(positions)

    plt.figure()
    plt.plot(positions[0, :], positions[1, :])
    plt.title("X-Y Displacement")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

  def plot_xyz(self):
    """ Plot XYZ """
    # positions = self.get_position(self.t)
    positions = []
    quats = []
    velocities = []
    accelerations = []
    angular_velocities = []
    for t in self.t:
      positions.append(tf_trans(self.get_pose(t)))
      quats.append(tf_quat(self.get_pose(t)))
      velocities.append(self.get_velocity(t))
      accelerations.append(self.get_acceleration(t))
      angular_velocities.append(self.get_angular_velocity(t))
    positions = np.array(positions)
    quats = np.array(quats)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    angular_velocities = np.array(angular_velocities)

    # Plot position, velocity and acceleartion
    plt.figure()
    plt.subplot(311)
    plt.plot(self.t, positions[:, 0], "-r")
    plt.plot(self.t, positions[:, 1], "-g")
    plt.plot(self.t, positions[:, 2], "-b")
    plt.title("Displacement")
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")

    plt.subplot(312)
    plt.plot(self.t, velocities[:, 0], "r-", label="vx")
    plt.plot(self.t, velocities[:, 1], "g-", label="vy")
    plt.plot(self.t, velocities[:, 2], "b-", label="vz")
    plt.title("Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")

    plt.subplot(313)
    plt.plot(self.t, accelerations[:, 0], "r-", label="ax")
    plt.plot(self.t, accelerations[:, 1], "g-", label="ay")
    plt.plot(self.t, accelerations[:, 2], "b-", label="ay")
    plt.title("Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s^2]")

    # Plot orientation and angular velocity
    rpy = []
    for idx in range(len(self.t)):
      q_WS = quats[idx, :]
      ypr = quat2euler(q_WS)
      rpy.append(np.flip(ypr, 0))
    rpy = np.array(rpy)

    body_rates = []
    for idx in range(len(self.t)):
      q_WS = quats[idx, :]
      C_WS = quat2rot(q_WS)
      w_WS_W = angular_velocities[idx, :]

      w_WS_S = C_WS.T @ w_WS_W
      body_rates.append(w_WS_S)
    body_rates = np.array(body_rates)

    plt.figure()
    plt.subplot(311)
    plt.plot(self.t, rpy[:, 0], 'r-', label="roll")
    plt.plot(self.t, rpy[:, 1], 'g-', label="pitch")
    plt.plot(self.t, rpy[:, 2], 'b-', label="yaw")
    plt.title("Attitude")
    plt.xlabel("Time [s]")
    plt.ylabel("Attitude [deg]")
    plt.legend(loc=0)

    plt.subplot(312)
    plt.plot(self.t, angular_velocities[:, 0], 'r-', label="wx")
    plt.plot(self.t, angular_velocities[:, 1], 'g-', label="wy")
    plt.plot(self.t, angular_velocities[:, 2], 'b-', label="wz")
    plt.title("Angular Velocity in Inertial Frame")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad s^-1]")
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(self.t, body_rates[:, 0], 'r-', label="wx")
    plt.plot(self.t, body_rates[:, 1], 'g-', label="wy")
    plt.plot(self.t, body_rates[:, 2], 'b-', label="wz")
    plt.title("Angular Velocity in Body Frame ")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad s^-1]")
    plt.legend(loc=0)
    plt.subplots_adjust(hspace=0.95, left=0.1, right=0.95, top=0.95)

  def plot_theta(self, t):
    """ Plot Theta """
    f = self.f
    plt.rcParams['text.usetex'] = True
    plt.figure()
    plt.plot(t, np.sin(2.0 * pi * f * t), "r-", label=r"$\sin(w t)$")
    plt.plot(t, np.sin(2.0 * pi * f * t)**2, "g-", label=r"$\sin^{2}(w t)$")
    plt.plot(t,
             np.sin(2.0 * pi * f * t * 1.0 / 4.0)**2,
             "b-",
             label=r"$\sin^{2}(0.25 w t)$")
    plt.legend(loc=0)
    plt.xlabel("Time [s]")
    plt.ylabel("Theta [rad]")

  def plot_3d(self, **kwargs):
    """ Plot 3D """
    # Setup
    save_anim = kwargs.get("save_anim", False)
    save_path = kwargs.get("save_path", "traj.mp4")
    show_plot = kwargs.get("show_plot", not save_anim)
    elev = kwargs.get("elev", 30.0)
    azim = kwargs.get("azim", -60.0)

    # Get positions
    positions = []
    for t in self.t:
      T_OS = self.get_pose(t)
      positions.append(tf_trans(T_OS))
    positions = np.array(positions)

    # Create figure
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2])
    self.calib_target.plot(ax, self.T_WF)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=elev, azim=azim)
    plot_set_axes_equal(ax)

    # Setup ffmegwriter
    writer = None
    if save_anim:
      writer = animation.FFMpegWriter(fps=30)
      writer.setup(fig, save_path, 100)

    # Draw camera poses
    skip_every = int(len(self.t) * 0.1)
    for idx, t in enumerate(self.t):
      if save_anim is False and idx % skip_every != 0:
        continue

      T_WS = self.get_pose(t)
      tf_data = plot_tf(ax, T_WS, name="T_WS", size=0.05)

      if save_anim:
        writer.grab_frame()
        ax.lines.remove(tf_data[0])
        ax.lines.remove(tf_data[1])
        ax.lines.remove(tf_data[2])
        ax.texts.remove(tf_data[3])

    if show_plot:
      plt.show()


def generate_animations():
  """ Generate animations """
  r_WF = np.array([0.0, 0.0, 0.0])
  C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
  T_WF = tf(C_WF, r_WF)

  traj = LissajousTraj("figure8", T_WF)
  traj.plot_3d(save_path="traj-figure8.mp4", save_anim=False)

  # traj = LissajousTraj("vert-pan", T_WF)
  # traj.plot_3d(save_path="traj-vert.mp4", save_anim=True)

  # traj = LissajousTraj("horiz-pan", T_WF)
  # traj.plot_3d(save_path="traj-horiz.mp4", save_anim=True)

  # traj = LissajousTraj("diag0", T_WF)
  # traj.plot_3d(save_path="traj-diag0.mp4",
  #              save_anim=True,
  #              elev=45.0,
  #              azim=-180.0)

  # traj = LissajousTraj("diag1", T_WF)
  # traj.plot_3d(save_path="traj-diag1.mp4",
  #              save_anim=True,
  #              elev=45.0,
  #              azim=-180.0)


def test_velocity():
  """ Test velocity """
  r_WF = np.array([0.0, 0.0, 0.0])
  C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
  T_WF = tf(C_WF, r_WF)
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate velocity
  T_WS = traj.get_pose(traj.t[0])
  r_WS = tf_trans(T_WS)
  v_WS = traj.get_velocity(0)
  dt = np.diff(traj.t)[0]

  pos_est = [r_WS]
  pos_gnd = [r_WS]
  vel_gnd = [v_WS]
  for t in traj.t[1:]:
    T_WS = traj.get_pose(t)
    v_WS = traj.get_velocity(t)
    r_WS = r_WS + v_WS * dt
    pos_est.append(r_WS)
    pos_gnd.append(tf_trans(T_WS))
    vel_gnd.append(v_WS)
  pos_est = np.array(pos_est)
  pos_gnd = np.array(pos_gnd)
  vel_gnd = np.array(vel_gnd)

  # -- Plot input velocity
  plt.figure()
  plt.subplot(311)
  plt.plot(traj.t, vel_gnd[:, 0], 'r-')
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [m/s]")

  plt.subplot(312)
  plt.plot(traj.t, vel_gnd[:, 1], 'g-')
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [m/s]")

  plt.subplot(313)
  plt.plot(traj.t, vel_gnd[:, 2], 'b-')
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [m/s]")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Velocity")

  # -- Plot integrated velocity
  plt.figure()
  plt.subplot(311)
  plt.plot(traj.t, pos_est[:, 0], 'r-')
  plt.plot(traj.t, pos_gnd[:, 0], 'r--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplot(312)
  plt.plot(traj.t, pos_est[:, 1], 'g-')
  plt.plot(traj.t, pos_gnd[:, 1], 'g--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplot(313)
  plt.plot(traj.t, pos_est[:, 2], 'b-')
  plt.plot(traj.t, pos_gnd[:, 2], 'b--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Test Integrate Inertial Velocity")
  plt.show()


def test_acceleration():
  """ Test acceleration """
  r_WF = np.array([0.0, 0.0, 0.0])
  C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
  T_WF = tf(C_WF, r_WF)
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate acceleration
  r_WS = tf_trans(traj.get_pose(traj.t[0]))
  v_WS = traj.get_velocity(traj.t[0])
  a_WS = traj.get_acceleration(traj.t[0])
  dt = np.diff(traj.t)[0]

  pos_est = [r_WS]
  pos_gnd = [r_WS]
  for t in traj.t[1:]:
    a_WS = traj.get_acceleration(t)
    r_WS = r_WS + (v_WS * dt) + (0.5 * a_WS * dt**2)
    v_WS = v_WS + (a_WS * dt)
    pos_est.append(r_WS)
    pos_gnd.append(tf_trans(traj.get_pose(t)))
  pos_est = np.array(pos_est)
  pos_gnd = np.array(pos_gnd)

  # -- Plot integrated velocity
  plt.subplot(311)
  plt.plot(traj.t, pos_est[:, 0], 'r-')
  plt.plot(traj.t, pos_gnd[:, 0], 'r--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplot(312)
  plt.plot(traj.t, pos_est[:, 1], 'g-')
  plt.plot(traj.t, pos_gnd[:, 1], 'g--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplot(313)
  plt.plot(traj.t, pos_est[:, 2], 'b-')
  plt.plot(traj.t, pos_gnd[:, 2], 'b--')
  plt.xlabel("Time [s]")
  plt.ylabel("Position [m]")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Test Integrate Inertial Acceleration")
  plt.show()


def test_angular_velocity():
  """ Test angular_velocity """
  r_WF = np.array([0.0, 0.0, 0.0])
  C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
  T_WF = tf(C_WF, r_WF)
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate angular_velocity
  C_WS = tf_rot(traj.get_pose(traj.t[0]))
  dt = np.diff(traj.t)[0]

  euler_est = [np.flip(rot2euler(C_WS), 0)]
  euler_gnd = [np.flip(rot2euler(C_WS), 0)]
  angvel_world = [traj.get_angular_velocity(0)]
  angvel_body = [C_WS.T @ traj.get_angular_velocity(0)]

  for t in traj.t[1:]:
    C_WS_gnd = tf_rot(traj.get_pose(t))
    w_WS_W = traj.get_angular_velocity(t)
    w_WS_S = C_WS.T @ w_WS_W
    C_WS = C_WS @ Exp(w_WS_S * dt)
    euler_est.append(np.flip(rot2euler(C_WS), 0))
    euler_gnd.append(np.flip(rot2euler(C_WS_gnd), 0))
    angvel_world.append(w_WS_W)
    angvel_body.append(w_WS_S)

  euler_est = np.array(euler_est)
  euler_gnd = np.array(euler_gnd)
  angvel_world = np.array(angvel_world)
  angvel_body = np.array(angvel_body)

  # -- Plot Attitude
  fig = plt.figure()
  plt.subplot(311)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 0]), 'r-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 0]), 'r--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplot(312)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 1]), 'g-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 1]), 'g--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplot(313)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 2]), 'b-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 2]), 'b--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Test Integrate Angular Velocity")

  # -- Plot Angular Velocities
  fig = plt.figure()
  plt.subplot(211)
  plt.plot(traj.t, angvel_world[:, 0], 'r-')
  plt.plot(traj.t, angvel_world[:, 1], 'g-')
  plt.plot(traj.t, angvel_world[:, 2], 'b-')
  plt.xlabel("Time [s]")
  plt.title("Angular Velocities in World Frame")

  plt.subplot(212)
  plt.plot(traj.t, angvel_body[:, 0], 'r-')
  plt.plot(traj.t, angvel_body[:, 1], 'g-')
  plt.plot(traj.t, angvel_body[:, 2], 'b-')
  plt.xlabel("Time [s]")
  plt.title("Angular Velocities in Body Frame")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Angular Velocities")

  plt.show()


def test_integration():
  """ Test integration """
  calib_target = AprilGrid()
  r_WF = np.array([0.0, 0.0, 0.0])
  C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
  T_WF = tf(C_WF, r_WF)
  traj = LissajousTraj("figure8", T_WF)

  # Integrate angular velocity
  r_WS = tf_trans(traj.get_pose(traj.t[0]))
  v_WS = traj.get_velocity(traj.t[0])
  C_WS = tf_rot(traj.get_pose(traj.t[0]))
  a_WS = traj.get_acceleration(traj.t[0])
  w_WS = traj.get_angular_velocity(traj.t[0])
  a_S_WS = C_WS.T @ a_WS
  w_S_WS = C_WS.T @ w_WS
  dt = np.diff(traj.t)[0]

  pos_est = [r_WS]
  pos_gnd = [r_WS]

  euler_est = [np.flip(rot2euler(C_WS), 0)]
  euler_gnd = [np.flip(rot2euler(C_WS), 0)]
  angle_diff = [0.0]

  acc = [traj.get_acceleration(0)]
  ang_vel = [traj.get_angular_velocity(0)]

  for t in traj.t[1:]:
    r_WS_gnd = tf_trans(traj.get_pose(t))
    C_WS_gnd = tf_rot(traj.get_pose(t))

    # Transform acceleration from world to body frame
    a_WS = traj.get_acceleration(t)
    a_S_WS = C_WS.T @ a_WS
    acc.append(a_WS)

    # Transform angular velocity from world to body frame
    w_WS = traj.get_angular_velocity(t)
    w_S_WS = C_WS.T @ w_WS
    ang_vel.append(w_WS)

    # Integrate
    r_WS = r_WS + (v_WS * dt) + (0.5 * C_WS @ a_S_WS * dt**2)
    v_WS = v_WS + (C_WS @ a_S_WS * dt)
    C_WS = C_WS @ Exp(w_S_WS * dt)
    # q_WS = rot2quat(C_WS)
    # dqx = w_S_WS[0] * dt / 2.0
    # dqy = w_S_WS[1] * dt / 2.0
    # dqz = w_S_WS[2] * dt / 2.0
    # dq = np.array([1.0, dqx, dqy, dqz])
    # q_WS = quat_mul(q_WS, dq)
    # C_WS = quat2rot(q_WS)

    # q_WS = rot2quat(C_WS)
    # q_WS = quat_normalize(q_WS)
    # C_WS = quat2rot(q_WS)

    # Calculate angle difference using axis-angle equation
    dC = C_WS_gnd.T @ C_WS
    ddeg = np.rad2deg(np.arccos((np.trace(dC) - 1.0) / 2.0))
    angle_diff.append(ddeg)

    # Track integrated vs ground truth postion and rotation
    pos_est.append(r_WS)
    pos_gnd.append(r_WS_gnd)
    euler_est.append(np.flip(rot2euler(C_WS), 0))
    euler_gnd.append(np.flip(rot2euler(C_WS_gnd), 0))

  pos_est = np.array(pos_est)
  pos_gnd = np.array(pos_gnd)
  euler_est = np.array(euler_est)
  euler_gnd = np.array(euler_gnd)
  acc = np.array(acc)
  ang_vel = np.array(ang_vel)

  # plt.figure()
  # plt.subplot(311)
  # plt.plot(traj.t, ang_vel[:, 0], 'r-')
  # plt.subplot(312)
  # plt.plot(traj.t, ang_vel[:, 1], 'g-')
  # plt.subplot(313)
  # plt.plot(traj.t, ang_vel[:, 2], 'b-')
  # plt.show()
  # exit()

  # -- Plot 3D
  save_anim = False
  save_path = "test_integration.mp4"
  show_plot = not save_anim

  poses = []
  positions = []
  for idx, (pos, euler) in enumerate(zip(pos_est, euler_est)):
    C_WS = euler321(*np.flip(euler, 0))
    r_WS = pos
    T_WS = tf(C_WS, r_WS)
    poses.append(T_WS)
    positions.append(r_WS)
  poses = np.array(poses)
  positions = np.array(positions)

  fig = plt.figure()
  ax = plt.axes(projection='3d')
  ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2])
  calib_target.plot(ax, T_WF)
  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  # ax.view_init(elev=0.0, azim=0.0)
  # ax.view_init(elev=30.0, azim=-30.0)
  plot_set_axes_equal(ax)

  # Setup ffmegwriter
  writer = None
  if save_anim:
    writer = animation.FFMpegWriter(fps=30)
    writer.setup(fig, save_path, 100)

  # Draw camera poses
  skip_every = int(len(poses) * 0.05)
  for idx, T_WS in enumerate(poses):
    if save_anim is False and idx % skip_every != 0:
      continue

    (xaxis, yaxis, zaxis, text) = plot_tf(ax, T_WS, name="T_WS", size=0.05)
    if save_anim:
      writer.grab_frame()
      ax.lines.remove(xaxis)
      ax.lines.remove(yaxis)
      ax.lines.remove(zaxis)
      ax.texts.remove(text)

  if show_plot:
    plt.show()

  # -- Plot Attitude
  fig = plt.figure()
  plt.subplot(311)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 0]), 'r-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 0]), 'r--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplot(312)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 1]), 'g-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 1]), 'g--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplot(313)
  plt.plot(traj.t, np.rad2deg(euler_est[:, 2]), 'b-')
  plt.plot(traj.t, np.rad2deg(euler_gnd[:, 2]), 'b--')
  plt.xlabel("Time [s]")
  plt.ylabel("Euler Angle [deg]")

  plt.subplots_adjust(top=0.9, right=0.95, hspace=0.7)
  plt.suptitle("Test Integrate Inertial Angular Velocity")

  # -- Plot angle difference
  fig = plt.figure()
  plt.plot(traj.t, angle_diff)
  plt.xlabel("Time [s]")
  plt.ylabel("Angle Difference [deg]")

  if show_plot:
    plt.show()


# Tests
# generate_animations()
# test_velocity()
# test_acceleration()
# test_angular_velocity()
test_integration()
