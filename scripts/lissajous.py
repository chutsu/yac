#!/usr/bin/python3 env
from math import pi
from math import atan2
import matplotlib.pylab as plt
from mpl_toolkits import mplot3d
import numpy as np
from numpy import sin
from numpy import cos
from numpy import sqrt

from proto import plot_tf
from proto import plot_set_axes_equal
from proto import lookat


# Use Sympy to calcualte jacobians

def sympy_diff():
  import sympy
  from sympy import ccode

  f, t = sympy.symbols("f t")
  A, a, delta = sympy.symbols("A a delta")
  B, b = sympy.symbols("B b")
  R = sympy.symbols("R")
  w = 2.0 * sympy.pi * f

  k = sympy.sin(w * t * 1.0 / 4.0)**2
  x = A * sympy.sin(a * k + delta)
  y = B * sympy.sin(b * k)
  z = sympy.sqrt(R**2 - x**2 - y**2)

  vx = sympy.simplify(sympy.diff(x, t))
  vy = sympy.diff(y, t)
  vz = sympy.diff(z, t)

  ax = sympy.simplify(sympy.diff(vx, t))
  ay = sympy.diff(vy, t)
  az = sympy.diff(vz, t)

  print("# Position")
  # print(f"x = {x}")
  # print(f"y = {y}")
  # print(f"z = {z}")
  print(f"x = {ccode(x)}")
  print(f"y = {ccode(y)}")
  print(f"z = {ccode(z)}")
  print("")

  print("# Velocity")
  # print(f"vx = {vx}")
  # print(f"vy = {vy}")
  # print(f"vz = {vz}")
  print(f"vx = {ccode(vx)}")
  print(f"vy = {ccode(vy)}")
  print(f"vz = {ccode(vz)}")
  print("")

  print("# Acceleration")
  # print(f"ax = {ax}")
  # print(f"ay = {ay}")
  # print(f"az = {az}")
  print(f"ax = {ccode(ax)}")
  print(f"ay = {ccode(ay)}")
  print(f"az = {ccode(az)}")
  print("")

# sympy_diff()
# exit(0)


# Lissajous curve parameters
R = 1.5  # Distance from calibration target
A = 1    # Amplitude in x-axis
B = 0.5  # Amplitude in y-axis
T = 3.0  # Period - Time it takes to complete trajectory [secs]

# -- Figure 8
a = 2.0 * pi * 1.0
b = 2.0 * pi * 2.0
delta = pi / 2.0
# # -- Vertical pan
# a = 0.0
# b = 1.0
# delta = 0.0
# # -- Horizontal pan
# a = 1.0
# b = 0.0
# delta = 0.0
# # -- Diagonal (bottom left to top right) pan
# a = 1.0
# b = 1.0
# delta = 0.0
# # -- Diagonal (top left to bottom right) pan
# a = 1.0
# b = 1.0
# delta = pi

# Generate trajectory
f = 1.0 / T
w = 2.0 * pi * f
t = np.linspace(0, T, 100)
theta = 2.0 * pi * np.sin(w * t * 1.0 / 4.0)**2

# -- Position
x = A*sin(a*sin(pi*f*t/2)**2 + delta)
y = B*sin(b*sin(pi*f*t/2)**2)
z = sqrt(-A**2*sin(a*sin(pi*f*t/2)**2 + delta)**2 - B**2*sin(b*sin(pi*f*t/2)**2)**2 + R**2)

# -- Velocity
vx = pi*A*a*f*sin(pi*f*t/2)*cos(pi*f*t/2)*cos(a*sin(pi*f*t/2)**2 + delta)
vy = pi*B*b*f*sin(pi*f*t/2)*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2)
vz = (-pi*A**2*a*f*sin(pi*f*t/2)*sin(a*sin(pi*f*t/2)**2 + delta)*cos(pi*f*t/2)*cos(a*sin(pi*f*t/2)**2 + delta) - pi*B**2*b*f*sin(b*sin(pi*f*t/2)**2)*sin(pi*f*t/2)*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2))/sqrt(-A**2 *sin(a*sin(pi*f*t/2)**2 + delta)**2 - B**2*sin(b*sin(pi*f*t/2)**2)**2 + R**2)

# -- Acceleration
ax = pi**2*A*a*f**2*(-a*sin(pi*f*t)**2*sin(-a*cos(pi*f*t)/2 + a/2 + delta)/2 - 2*sin(pi*f*t/2)**2*cos(a*sin(pi*f*t/2)**2 + delta) + cos(a*sin(pi*f*t/2)**2 + delta))/2
ay = -pi**2*B*b**2*f**2*sin(b*sin(pi*f*t/2)**2)*sin(pi*f*t/2)**2*cos(pi*f*t/2)**2 - pi**2*B*b*f**2*sin(pi*f*t/2)**2*cos(b*sin(pi*f*t/2)**2)/2 + pi**2*B*b*f**2*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2)**2/2
az = (-pi*A**2*a*f*sin(pi*f*t/2)*sin(a*sin(pi*f*t/2)**2 + delta)*cos(pi*f*t/2)*cos(a*sin(pi*f*t/2)**2 + delta) - pi*B**2*b*f*sin(b*sin(pi*f*t/2)**2)*sin(pi*f*t/2)*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2))*(pi*A**2*a*f*sin(pi*f*t/2)*sin(a*sin(pi*f*t/2)**2 + delta)*cos(pi*f*t/2)*cos(a*sin(pi*f*t/2)**2 + delta) + pi*B**2*b*f*sin(b*sin(pi*f*t/2)**2)*sin(pi*f*t/2)*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2))/(-A**2*sin(a*sin(pi*f*t/2)**2 + delta)**2 - B**2*sin(b*sin(pi*f*t/2)**2)**2 + R**2)**(3/2) + (pi**2*A**2*a**2*f**2*sin(pi*f*t/2)**2*sin(a*sin(pi*f*t/2)**2 + delta)**2*cos(pi*f*t/2)**2 - pi**2*A**2*a**2*f**2*sin(pi*f*t/2)**2*cos(pi*f*t/2)**2*cos(a*sin(pi*f*t/2)**2 + delta)**2 + pi**2*A**2*a*f**2*sin(pi*f*t/2)**2*sin(a*sin(pi*f*t/2)**2 + delta)*cos(a*sin(pi*f*t/2)**2 + delta)/2 - pi**2*A**2*a*f**2*sin(a*sin(pi*f*t/2)**2 + delta)*cos(pi*f*t/2)**2*cos(a*sin(pi*f*t/2)**2 + delta)/2 + pi**2*B**2*b**2*f**2*sin(b*sin(pi*f*t/2)**2)**2*sin(pi*f*t/2)**2*cos(pi*f*t/2)**2 - pi**2*B**2*b**2*f**2*sin(pi*f*t/2)**2*cos(b*sin(pi*f*t/2)**2)**2*cos(pi*f*t/2)**2 + pi**2*B**2 *b*f**2*sin(b*sin(pi*f*t/2)**2)*sin(pi*f*t/2)**2*cos(b*sin(pi*f*t/2)**2)/2 - pi**2*B**2*b*f**2*sin(b*sin(pi*f*t/2)**2)*cos(b*sin(pi*f*t/2)**2)*cos(pi*f*t/2)**2/2)/sqrt(-A**2*sin(a*sin(pi*f*t/2)**2 + delta)**2 - B **2*sin(b*sin(pi*f*t/2)**2)**2 + R**2)

# Plot
def plot_xy(x, y):
  plt.figure()
  plt.plot(x, y)
  plt.title("X-Y Displacement")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")

def plot_theta(t, theta):
  plt.rcParams['text.usetex'] = True
  plt.figure()
  plt.plot(t, np.sin(2.0 * pi * f * t), 'r-', label="$\sin(w t)$")
  plt.plot(t, np.sin(2.0 * pi * f * t)**2, 'g-', label="$\sin^{2}(w t)$")
  plt.plot(t, np.sin(2.0 * pi * f * t * 1.0 / 4.0)**2, 'b-', label="$\sin^{2}(0.25 w t)$")
  plt.legend(loc=0)
  plt.xlabel("Time [s]")
  plt.ylabel("Theta [rad]")

def plot_xyz(t, x, y, z, vx, vy, vz, ax, ay, az):
  plt.figure()
  plt.subplot(311)
  plt.plot(t, x, '-r')
  plt.plot(t, y, '-g')
  plt.plot(t, z, '-b')
  plt.title("Displacement")
  plt.xlabel("Time [s]")
  plt.ylabel("Displacement [m]")

  plt.subplot(312)
  plt.plot(t, vx, 'r-', label="vx")
  plt.plot(t, vy, 'g-', label="vy")
  plt.title("Velocity")
  plt.xlabel("Time [s]")
  plt.ylabel("Velocity [m/s]")

  plt.subplot(313)
  plt.plot(t, ax, 'r-', label="ax_sympy")
  plt.plot(t, ay, 'g-', label="ay_sympy")
  plt.title("Acceleration")
  plt.xlabel("Time [s]")
  plt.ylabel("Acceleration [m/s^2]")


def plot_3d(x, y, z):
  p_start = atan2(z[0], x[0])
  p_end = atan2(z[50], x[50])

  f = 1.0 / T
  w = 2.0 * pi * f
  t = np.linspace(0, T, 100)
  theta = np.sin(w * t)
  plt.plot(t, theta)

  fig = plt.figure()
  ax = plt.axes(projection='3d')
  ax.plot3D(x, y, z)

  cam_pos = np.array([x[0], y[0], z[0]])
  target_pos = np.array([0.0, 0.0, 0.0])
  T_WC = lookat(cam_pos, target_pos)
  plot_tf(ax, T_WC, name="T_WC", size=0.3)

  cam_pos = np.array([x[50], y[50], z[50]])
  target_pos = np.array([0.0, 0.0, 0.0])
  T_WC = lookat(cam_pos, target_pos)
  plot_tf(ax, T_WC, name="T_WC", size=0.3)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)


# Main
# plot_xy(x, y)
# plot_theta(t, theta)
# plot_xyz(t, x, y, z, vx, vy, vz, ax, ay, az)
plot_3d(x, y, z)
plt.show()
