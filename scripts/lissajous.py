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
from proto import plot_tf
from proto import plot_set_axes_equal
from proto import lookat
from proto import euler321
from proto import rot2euler


# Use Sympy to calcualte jacobians

def sympy_diff(gencode=False):
  import sympy
  from sympy import ccode

  f, t = sympy.symbols("f t")
  A, a, delta = sympy.symbols("A a delta")
  B, b = sympy.symbols("B b")
  R = sympy.symbols("R")
  w = sympy.symbols("w")   # w = 2.0 * pi * f
  phi = sympy.symbols("phi")

  theta = sympy.sin(w * t * 1.0 / 4.0)**2
  x = A * sympy.sin(a * theta + delta + phi)
  y = B * sympy.sin(b * theta)
  z = sympy.sqrt(R**2 - x**2 - y**2)

  vx = sympy.simplify(sympy.diff(x, t))
  vy = sympy.diff(y, t)
  vz = sympy.diff(z, t)

  ax = sympy.simplify(sympy.diff(vx, t))
  ay = sympy.diff(vy, t)
  az = sympy.diff(vz, t)

  yaw_start, yaw_end = sympy.symbols("yaw_start yaw_end")
  pitch_start, pitch_end = sympy.symbols("pitch_start pitch_end")

  att_theta = sympy.sin(w * t * 1.0 / 2.0 + pi / 4.0)**2
  yaw = yaw_start * (1.0 - att_theta) + att_theta * yaw_end
  pitch = pitch_start * (1.0 - att_theta) + att_theta * pitch_end

  att_x = 0.0
  att_y = yaw
  att_z = np.deg2rad(180.0) + pitch

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
  def __init__(self, traj_type, **kwargs):
    # Lissajous curve parameters
    self.R = kwargs.get("R", 1.5)  # Distance from calibration target
    self.A = kwargs.get("A", 1.0)  # Amplitude in x-axis
    self.B = kwargs.get("B", 0.5)  # Amplitude in y-axis
    self.T = kwargs.get("T", 3.0)  # Period - Time it takes to complete [secs]

    # Time
    self.f = 1.0 / self.T
    self.w = 2.0 * pi * self.f
    self.t = np.linspace(0, self.T, 100)
    self.theta = np.sin(self.w * self.t * 1.0 / 4.0)**2
    self.att_theta = np.sin(self.w * self.t * 1.0 / 2.0 + pi / 4.0)**2

    # Trajectory type
    # -- Figure 8
    if traj_type == "figure8":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 2.0
      self.delta = pi / 2.0
      self.phase_offset = pi / 2.0
      self.yaw_start = atan2(self.A, self.R)
      self.yaw_end = -atan2(self.A, self.R)
      self.pitch_start = atan2(self.B, self.R)
      self.pitch_end = -atan2(self.B, self.R)

    # -- Vertical pan
    elif traj_type == "vert-pan":
      self.a = 2.0 * pi * 0.0
      self.b = 2.0 * pi * 1.0
      self.delta = 0.0
      self.phase_offset = pi / 2.0
      self.yaw_start = 0.0
      self.yaw_end = 0.0
      self.pitch_start = atan2(self.B, self.R)
      self.pitch_end = -atan2(self.B, self.R)

    # -- Horizontal pan
    elif traj_type == "horiz-pan":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 0.0
      self.delta = 0.0
      self.phase_offset = 0.0
      self.yaw_start = -atan2(self.A, self.R)
      self.yaw_end = atan2(self.A, self.R)
      self.pitch_start = atan2(self.B, self.R)
      self.pitch_end = -atan2(self.B, self.R)

    # -- Diagonal (bottom left to top right) pan
    elif traj_type == "diag0":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 1.0
      self.delta = 0.0
      self.phase_offset = 0.0
      self.yaw_start = -atan2(self.A, self.R)
      self.yaw_end = atan2(self.A, self.R)
      self.pitch_start = atan2(self.B, self.R)
      self.pitch_end = -atan2(self.B, self.R)

    # -- Diagonal (top left to bottom right) pan
    elif traj_type == "diag1":
      self.a = 2.0 * pi * 1.0
      self.b = 2.0 * pi * 1.0
      self.delta = pi
      self.phase_offset = 0.0
      self.yaw_start = atan2(self.A, self.R)
      self.yaw_end = -atan2(self.A, self.R)
      self.pitch_start = atan2(self.B, self.R)
      self.pitch_end = -atan2(self.B, self.R)

    else:
      raise RuntimeError(f"Invalid traj_type[{traj_type}]")

  def get_position(self, t):
    w = 2.0 * pi * self.f
    theta = sin(w * t * 1.0 / 4.0)**2
    phi = self.phase_offset
    x = self.A * sin(self.a * theta + self.delta + phi)
    y = self.B * sin(self.b * theta)
    z = sqrt(self.R**2 - x**2 - y**2)
    return np.array([x, y, z])

  def get_attitude(self, t):
    th = np.sin(self.w * t * 1.0 / 2.0 + pi / 4.0)**2
    yaw = self.yaw_start * (1.0 - th) + th * self.yaw_end
    pitch = self.pitch_start * (1.0 - th) + th * self.pitch_end
    C_WC = euler321(0.0, yaw, np.deg2rad(180.0) + pitch)
    return C_WC

  def get_pose(self, t):
    C_WC = self.get_attitude(t)
    r_WC = self.get_position(t)
    T_WC = tf(C_WC, r_WC)
    return T_WC

  def get_velocity(self, t):
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    phi = self.phase_offset
    w = 2.0 * pi * self.f

    vx = 0.5*A*a*w*np.sin(0.25*t*w)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi)
    vy = 0.5*B*b*w*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w)
    vz = (-0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) - 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))/sqrt(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2))

    return np.array([vx, vy, vz])

  def get_acceleration(self, t):
    A = self.A
    B = self.B
    a = self.a
    b = self.b
    R = self.R
    delta = self.delta
    phi = self.phase_offset
    w = 2.0 * pi * self.f

    ax = A*a*pow(w, 2)*(-0.03125*a*(1 - np.cos(1.0*t*w))*np.sin(-1.0/2.0*a*np.cos(0.5*t*w) + (1.0/2.0)*a + delta + phi) - 0.125*pow(np.sin(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) + 0.125*pow(np.cos(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi))
    ay = -0.25*B*pow(b, 2)*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2) - 0.125*B*b*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*np.cos(b*pow(np.sin(0.25*t*w), 2)) + 0.125*B*b*pow(w, 2)*np.cos(b*pow(np.sin(0.25*t*w), 2))*pow(np.cos(0.25*t*w), 2)
    az = (-0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) - 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))*(0.5*pow(A, 2)*a*w*np.sin(0.25*t*w)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi)*np.cos(0.25*t*w)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) + 0.5*pow(B, 2)*b*w*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.sin(0.25*t*w)*np.cos(b*pow(np.sin(0.25*t*w), 2))*np.cos(0.25*t*w))/pow(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2), 3.0/2.0) + (0.25*pow(A, 2)*pow(a, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi), 2)*pow(np.cos(0.25*t*w), 2) - 0.25*pow(A, 2)*pow(a, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2)*pow(np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi), 2) + 0.125*pow(A, 2)*a*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) - 0.125*pow(A, 2)*a*pow(w, 2)*np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi)*pow(np.cos(0.25*t*w), 2)*np.cos(a*pow(np.sin(0.25*t*w), 2) + delta + phi) + 0.25*pow(B, 2)*pow(b, 2)*pow(w, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(0.25*t*w), 2) - 0.25*pow(B, 2)*pow(b, 2)*pow(w, 2)*pow(np.sin(0.25*t*w), 2)*pow(np.cos(b*pow(np.sin(0.25*t*w), 2)), 2)*pow(np.cos(0.25*t*w), 2) + 0.125*pow(B, 2)*b*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*pow(np.sin(0.25*t*w), 2)*np.cos(b*pow(np.sin(0.25*t*w), 2)) - 0.125*pow(B, 2)*b*pow(w, 2)*np.sin(b*pow(np.sin(0.25*t*w), 2))*np.cos(b*pow(np.sin(0.25*t*w), 2))*pow(np.cos(0.25*t*w), 2))/sqrt(-pow(A, 2)*pow(np.sin(a*pow(np.sin(0.25*t*w), 2) + delta + phi), 2) - pow(B, 2)*pow(np.sin(b*pow(np.sin(0.25*t*w), 2)), 2) + pow(R, 2))

    return np.array([ax, ay, az])

  def get_angular_velocity(self, t):
    w = 2.0 * pi * self.f

    wx = 0.0 * np.ones(len(t))
    wy = 1.0*w*self.yaw_end*np.sin(0.5*t*w + 0.785398163397448)*np.cos(0.5*t*w + 0.785398163397448) - 1.0*w*self.yaw_start*np.sin(0.5*t*w + 0.785398163397448)*np.cos(0.5*t*w + 0.785398163397448)
    wz = 1.0*w*(self.pitch_end - self.pitch_start)*np.sin(0.5*t*w + 0.78539816339744828)*np.cos(0.5*t*w + 0.78539816339744828)
    return np.array([wx, wy, wz])

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
    positions = self.get_position(self.t)
    x = positions[0, :]
    y = positions[1, :]
    z = positions[2, :]

    # Create figure
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(*positions)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plot_set_axes_equal(ax)

    # Setup ffmegwriter
    writer = None
    if save_anim:
      writer = animation.FFMpegWriter(fps=30)
      writer.setup(fig, save_path, 100)

    # Draw camera poses
    for idx, t in enumerate(self.t):
      if save_anim is False and idx % (len(self.att_theta) * 0.05) == 0:
        continue

      T_WC = self.get_pose(t)
      tf_data = plot_tf(ax, T_WC, name="T_WC", size=0.3)

      if save_anim:
        writer.grab_frame()
        ax.lines.remove(tf_data[0])
        ax.lines.remove(tf_data[1])
        ax.lines.remove(tf_data[2])
        ax.texts.remove(tf_data[3])


# Main
traj = LissajousTraj("figure8")
# traj.plot_3d(save_path="traj-figure8.mp4", save_anim=True)
# traj.plot_xyz()
# plt.show()

# roll = []
# pitch = []
# yaw = []
# for t in traj.t:
#   C_WC = traj.get_attitude(t)
#   r, p, y= rot2euler(C_WC)
#   roll.append(r)
#   pitch.append(p)
#   yaw.append(y)

# plt.subplot(211)
# plt.plot(traj.t, roll, 'r-', label='roll')
# plt.plot(traj.t, pitch, 'g-', label='pitch')
# plt.plot(traj.t, yaw, 'b-', label='yaw')

ang_vels = traj.get_angular_velocity(traj.t)
plt.subplot(212)
plt.plot(traj.t, ang_vels[0, :], 'r-', label="wx")
plt.plot(traj.t, ang_vels[1, :], 'g-', label="wy")
plt.plot(traj.t, ang_vels[2, :], 'b-', label="wy")
plt.show()

# traj = LissajousTraj("vert-pan")
# traj.plot_3d(save_path="traj-vert.mp4", save_anim=True)

# traj = LissajousTraj("horiz-pan")
# traj.plot_3d(save_path="traj-horiz.mp4", save_anim=True)

# traj = LissajousTraj("diag0")
# traj.plot_3d(save_path="traj-diag0.mp4", save_anim=True)

# traj = LissajousTraj("diag1")
# traj.plot_3d(save_path="traj-diag1.mp4", save_anim=True)
