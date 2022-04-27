#!/usr/bin/python3 env
from math import pi
from math import atan2

import matplotlib
import matplotlib.pylab as plt
from matplotlib import animation
from mpl_toolkits import mplot3d

import numpy as np
from numpy import sin
from numpy import cos
from numpy import sqrt

from proto import tf
from proto import tf_trans
from proto import tf_rot
from proto import plot_tf
from proto import plot_set_axes_equal
from proto import lookat
from proto import Exp
from proto import euler321
from proto import rot2euler
from proto import rot2quat
from proto import quat2rot
from proto import quat_mul
from proto import quat_normalize
from proto import AprilGrid


# Use Sympy to calcualte jacobians
def sympy_diff(gencode=False):
  import sympy
  from sympy import ccode

  f, t = sympy.symbols("f t")
  A, a, delta = sympy.symbols("A a delta")
  B, b = sympy.symbols("B b")
  R = sympy.symbols("R")
  w = sympy.symbols("w")   # w = 2.0 * pi * f

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
  psi = sympy.symbols("psi")

  att_x = pi + pitch_bound * sympy.sin(psi * w * T * theta)
  att_y = yaw_bound * sympy.sin(w * T * theta)
  att_z = 0.0

  wx = sympy.diff(att_x, t)
  wy = sympy.diff(att_y, t)
  wz = sympy.diff(att_z, t)

  print("# Position")
  if gencode:
    print(f"x = {ccode(x)}")
    print(f"y = {ccode(y)}")
    print(f"z = {ccode(z)}")
  else:
    print(f"x = {x}")
    print(f"y = {y}")
    print(f"z = {z}")
  print("")

  print("# Velocity")
  if gencode:
    print(f"vx = {ccode(vx)}")
    print(f"vy = {ccode(vy)}")
    print(f"vz = {ccode(vz)}")
  else:
    print(f"vx = {vx}")
    print(f"vy = {vy}")
    print(f"vz = {vz}")
  print("")

  print("# Acceleration")
  if gencode:
    print(f"ax = {ccode(ax)}")
    print(f"ay = {ccode(ay)}")
    print(f"az = {ccode(az)}")
  else:
    print(f"ax = {ax}")
    print(f"ay = {ay}")
    print(f"az = {az}")
  print("")

  print("# Angular Velocity")
  if gencode:
    print(f"wx = {ccode(wx)}")
    print(f"wy = {ccode(wy)}")
    print(f"wz = {ccode(wz)}")
  else:
    print(f"wx = {wx}")
    print(f"wy = {wy}")
    print(f"wz = {wz}")
  print("")

# sympy_diff()
# exit(0)


class LissajousTraj:
  def __init__(self, traj_type, T_WF, **kwargs):
    self.T_WF = T_WF
    self.calib_target = AprilGrid()
    self.calib_dims = self.calib_target.get_dimensions()
    self.T_FO = tf(np.eye(3), self.calib_target.get_center())

    self.R = kwargs.get("R", np.max(self.calib_dims) * 1.0)  # Distance from calibration target
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
    # Setup
    w = 2.0 * pi * self.f
    theta = sin(w * t * 1.0 / 4.0)**2

    # Rotation
    att_theta_yaw = np.sin(self.w * self.T * theta)
    att_theta_pitch = np.sin(self.psi * self.w * self.T * theta)
    yaw = self.yaw_bound * att_theta_yaw
    pitch = self.pitch_bound * att_theta_pitch
    C_OC = euler321(0.0, yaw, np.deg2rad(180.0) + pitch)

    # Position
    x = self.A * sin(self.a * theta + self.delta)
    y = self.B * sin(self.b * theta)
    z = sqrt(self.R**2 - x**2 - y**2)
    r_OC = np.array([x, y, z])

    # Form pose
    T_OC = tf(C_OC, r_OC)
    T_WC = self.T_WF @ self.T_FO @ T_OC

    return T_WC

  def get_velocity(self, t):
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    w = 2.0 * pi * self.f

    vx = 0.5*A*a*w*np.sin(0.25*t*w)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta)
    vy = 0.5*B*b*w*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w)
    vz = (-0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) - 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))/sqrt(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2))
    v_OC = np.array([vx, vy, vz])

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    v_WC = C_WO @ v_OC

    return v_WC

  def get_acceleration(self, t):
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    w = 2.0 * pi * self.f

    ax = A*a*pow(w, 2)*(-0.03125*a*(1 - np.cos(1.0*t*w))*np.sin(-1.0/2.0*a*np.cos(0.5*t*w) + (1.0/2.0)*a + delta) - 0.125*pow(np.sin(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) + 0.125*pow(np.cos(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta))
    ay = -0.25*B*pow(b, 2)*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2) - 0.125*B*b*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*np.cos(b*pow(np.sin(0.25*t*w), 2)) + 0.125*B*b*pow(w, 2)*np.cos(b*pow(np.sin(0.25*t*w), 2))*pow(np.cos(0.25*t*w), 2)
    az = (-0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) - 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))*(0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) + 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))/pow(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2), 3.0/2.0) + (0.25*pow(A, 2)*pow(a, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta), 2)*pow(np.cos(0.25*t*w), 2) - 0.25*pow(A, 2)*pow(a, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2)*pow(np.cos(a*pow(np.sin(0.25*t*w), 2) + delta), 2) + 0.125*pow(A, 2)*a*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) - 0.125*pow(A, 2)*a*pow(w, 2)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta)*pow(np.cos(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta) + 0.25*pow(B, 2)*pow(b, 2)*pow(w, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2) - 0.25*pow(B, 2)*pow(b, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(b*pow(np.sin(0.25*t*w), 2)), 2)*pow(np.cos(0.25*t*w), 2) + 0.125*pow(B, 2)*b*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*pow(np.sin(0.25*t*w), 2)*np.cos(b*pow(np.sin(0.25*t*w), 2)) - 0.125*pow(B, 2)*b*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.cos(b*pow(np.sin(0.25*t*w), 2))*pow(np.cos(0.25*t*w), 2))/sqrt(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2))
    a_OC = np.array([ax, ay, az])

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    a_WC = C_WO @ a_OC

    return a_WC

  def get_angular_velocity(self, t):
    w = 2.0 * pi * self.f
    f = self.f
    psi = self.psi

    wx = 0.5*self.pitch_bound*psi*w**2*np.sin(0.25*t*w)*np.cos(0.25*t*w)*np.cos(1.0*psi*w*np.sin(0.25*t*w)**2/f)/f
    wy = 0.5*w**2*self.yaw_bound*np.sin(0.25*t*w)*np.cos(0.25*t*w)*np.cos(1.0*w*np.sin(0.25*t*w)**2/f)/f
    wz = 0

    w_OC = np.array([wx, wy, wz])

    # Transform velocity from calib origin frame (O) to world frame (W)
    T_WO = self.T_WF @ self.T_FO
    C_WO = tf_rot(T_WO)
    w_WC = C_WO @ w_OC

    return w_WC

  def plot_xy(self):
    positions = self.get_position(self.t)

    plt.figure()
    plt.plot(positions[0, :], positions[1, :])
    plt.title("X-Y Displacement")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

  def plot_xyz(self):
    positions = self.get_position(self.t)
    velocities = self.get_velocity(self.t)
    accelerations = self.get_acceleration(self.t)

    plt.figure()
    plt.subplot(311)
    plt.plot(self.t, positions[0, :], '-r')
    plt.plot(self.t, positions[1, :], '-g')
    plt.plot(self.t, positions[2, :], '-b')
    plt.title("Displacement")
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")

    plt.subplot(312)
    plt.plot(self.t, velocities[0, :], 'r-', label="vx")
    plt.plot(self.t, velocities[1, :], 'g-', label="vy")
    plt.plot(self.t, velocities[2, :], 'b-', label="vz")
    plt.title("Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")

    plt.subplot(313)
    plt.plot(self.t, accelerations[0, :], 'r-', label="ax")
    plt.plot(self.t, accelerations[1, :], 'g-', label="ay")
    plt.plot(self.t, accelerations[2, :], 'b-', label="ay")
    plt.title("Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s^2]")

  def plot_theta(t, theta):
    plt.rcParams['text.usetex'] = True
    plt.figure()
    plt.plot(t, np.sin(2.0 * pi * f * t), 'r-', label="$\sin(w t)$")
    plt.plot(t, np.sin(2.0 * pi * f * t)**2, 'g-', label="$\sin^{2}(w t)$")
    plt.plot(t, np.sin(2.0 * pi * f * t * 1.0 / 4.0)**2, 'b-', label="$\sin^{2}(0.25 w t)$")
    plt.legend(loc=0)
    plt.xlabel("Time [s]")
    plt.ylabel("Theta [rad]")

  def plot_3d(self, **kwargs):
    # Setup
    save_anim = kwargs.get("save_anim", False)
    save_path = kwargs.get("save_path", "traj.mp4")
    show_plot = kwargs.get("show_plot", not save_anim)

    # Get positions
    positions = []
    for t in self.t:
      T_OC = self.get_pose(t)
      positions.append(tf_trans(T_OC))
    positions = np.array(positions)

    # Create figure
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2])
    self.calib_target.plot(ax, self.T_WF)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    # ax.view_init(elev=0.0, azim=0.0)
    ax.view_init(elev=30.0, azim=-30.0)
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

      T_WC = self.get_pose(t)
      tf_data = plot_tf(ax, T_WC, name="T_WC", size=0.05)

      if save_anim:
        writer.grab_frame()
        ax.lines.remove(tf_data[0])
        ax.lines.remove(tf_data[1])
        ax.lines.remove(tf_data[2])
        ax.texts.remove(tf_data[3])

    if show_plot:
      plt.show()

def test_velocity():
  # Test velocity
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate velocity
  T_WC = traj.get_pose(traj.t[0])
  r_WC = tf_trans(T_WC)
  dt = np.diff(traj.t)[0]

  pos_est = [r_WC]
  pos_gnd = [r_WC]
  for t in traj.t[1:]:
    T_WC = traj.get_pose(t)
    v_WC = traj.get_velocity(t)
    r_WC = r_WC + v_WC * dt
    pos_est.append(r_WC)
    pos_gnd.append(tf_trans(T_WC))
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
  plt.suptitle("Test Integrate Inertial Velocity")
  plt.show()

def test_acceleration():
  # Test acceleration
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate acceleration
  r_WC = tf_trans(traj.get_pose(traj.t[0]))
  v_WC = traj.get_velocity(traj.t[0])
  a_WC = traj.get_acceleration(traj.t[0])
  dt = np.diff(traj.t)[0]

  pos_est = [r_WC]
  pos_gnd = [r_WC]
  for t in traj.t[1:]:
    a_WC = traj.get_acceleration(t)
    r_WC = r_WC + (v_WC * dt) + (0.5 * a_WC * dt**2)
    v_WC = v_WC + (a_WC * dt)
    pos_est.append(r_WC)
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


def test_integration():
  # Test angular_velocity
  traj = LissajousTraj("figure8", T_WF)

  # -- Integrate angular velocity
  r_WC = tf_trans(traj.get_pose(traj.t[0]))
  v_WC = traj.get_velocity(traj.t[0])
  C_WC = tf_rot(traj.get_pose(traj.t[0]))
  a_WC = traj.get_acceleration(traj.t[0])
  w_WC = traj.get_angular_velocity(traj.t[0])
  a_C_WC = C_WC.T @ a_WC
  w_C_WC = C_WC.T @ w_WC
  dt = np.diff(traj.t)[0]

  pos_est = [r_WC]
  pos_gnd = [r_WC]

  euler_est = [np.flip(rot2euler(C_WC), 0)]
  euler_gnd = [np.flip(rot2euler(C_WC), 0)]
  angle_diff = [0.0]
  for t in traj.t[1:]:
    # Transform acceleration from world to body frame
    a_WC = traj.get_acceleration(t)
    a_C_WC = C_WC.T @ a_WC

    # Transform angular velocity from world to body frame
    w_WC = traj.get_angular_velocity(t)
    w_C_WC = C_WC.T @ w_WC

    # Integrate
    r_WC = r_WC + (v_WC * dt) + (0.5 * C_WC @ a_C_WC * dt**2)
    v_WC = v_WC + (C_WC @ a_C_WC * dt)
    C_WC = C_WC @ Exp(w_C_WC * dt)

    r_WC_gnd = tf_trans(traj.get_pose(t))
    C_WC_gnd = tf_rot(traj.get_pose(t))

    dC = C_WC_gnd.T @ C_WC
    ddeg = np.rad2deg(np.arccos((np.trace(dC) - 1.0) / 2.0))
    angle_diff.append(ddeg)

    pos_est.append(r_WC)
    pos_gnd.append(r_WC_gnd)
    euler_est.append(np.flip(rot2euler(C_WC), 0))
    euler_gnd.append(np.flip(rot2euler(C_WC_gnd), 0))

  pos_est = np.array(pos_est)
  pos_gnd = np.array(pos_gnd)
  euler_est = np.array(euler_est)
  euler_gnd = np.array(euler_gnd)

  # -- Plot 3D
  save_anim = False
  save_path = "test_angular_velocity.mp4"
  show_plot = not save_anim

  poses = []
  positions = []
  for idx, (pos, euler) in enumerate(zip(pos_est, euler_est)):
    C_WC = euler321(*np.flip(euler, 0))
    r_WC = pos
    T_WC = tf(C_WC, r_WC)
    poses.append(T_WC)
    positions.append(r_WC)
  poses = np.array(poses)
  positions = np.array(positions)

  fig = plt.figure()
  ax = plt.axes(projection='3d')
  ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2])
  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  ax.view_init(elev=0.0, azim=0.0)
  # ax.view_init(elev=30.0, azim=-30.0)
  plot_set_axes_equal(ax)

  # Setup ffmegwriter
  writer = None
  if save_anim:
    writer = animation.FFMpegWriter(fps=30)
    writer.setup(fig, save_path, 100)

  # Draw camera poses
  skip_every = int(len(poses) * 0.1)
  for idx, T_WC in enumerate(poses):
    if save_anim is False and idx % skip_every != 0:
      continue

    tf_data = plot_tf(ax, T_WC, name="T_WC", size=0.05)
    if save_anim:
      writer.grab_frame()
      ax.lines.remove(tf_data[0])
      ax.lines.remove(tf_data[1])
      ax.lines.remove(tf_data[2])
      ax.texts.remove(tf_data[3])

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

  fig = plt.figure()
  plt.plot(traj.t, angle_diff)
  plt.xlabel("Time [s]")
  plt.ylabel("Angle Difference [deg]")

  plt.show()

# Fiducial target pose in world frame
r_WF = np.array([0.0, 0.0, 0.0])
C_WF = euler321(np.deg2rad(-90.0), 0.0, np.deg2rad(90.0))
T_WF = tf(C_WF, r_WF)

# traj = LissajousTraj("figure8", T_WF)
# traj.plot_3d(save_path="traj-figure8.mp4", save_anim=True)

# traj = LissajousTraj("vert-pan", T_WF)
# traj.plot_3d(save_path="traj-vert.mp4", save_anim=True)

# traj = LissajousTraj("horiz-pan", T_WF)
# traj.plot_3d(save_path="traj-horiz.mp4", save_anim=True)

# traj = LissajousTraj("diag0", T_WF)
# traj.plot_3d(save_path="traj-diag0.mp4", save_anim=True)

# traj = LissajousTraj("diag1", T_WF)
# traj.plot_3d(save_path="traj-diag1.mp4", save_anim=True)

# Tests
# test_velocity()
# test_acceleration()
test_integration()
