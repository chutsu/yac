from math import cos
from math import sin
from math import acos
from math import sqrt

import numpy as np
from numpy import sinc
from numpy.linalg import norm
from numpy.linalg import inv

import matplotlib.pylab as plt


def skew(vec):
  """ Form skew-symmetric matrix from vector `vec` """
  assert vec.shape == (3,) or vec.shape == (3, 1)
  x, y, z = vec
  return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot2quat(C):
  """
  Convert 3x3 rotation matrix to quaternion.
  """
  assert C.shape == (3, 3)

  m00 = C[0, 0]
  m01 = C[0, 1]
  m02 = C[0, 2]

  m10 = C[1, 0]
  m11 = C[1, 1]
  m12 = C[1, 2]

  m20 = C[2, 0]
  m21 = C[2, 1]
  m22 = C[2, 2]

  tr = m00 + m11 + m22

  if tr > 0:
    S = sqrt(tr + 1.0) * 2.0
    # S=4*qw
    qw = 0.25 * S
    qx = (m21 - m12) / S
    qy = (m02 - m20) / S
    qz = (m10 - m01) / S
  elif ((m00 > m11) and (m00 > m22)):
    S = sqrt(1.0 + m00 - m11 - m22) * 2.0
    # S=4*qx
    qw = (m21 - m12) / S
    qx = 0.25 * S
    qy = (m01 + m10) / S
    qz = (m02 + m20) / S
  elif m11 > m22:
    S = sqrt(1.0 + m11 - m00 - m22) * 2.0
    # S=4*qy
    qw = (m02 - m20) / S
    qx = (m01 + m10) / S
    qy = 0.25 * S
    qz = (m12 + m21) / S
  else:
    S = sqrt(1.0 + m22 - m00 - m11) * 2.0
    # S=4*qz
    qw = (m10 - m01) / S
    qx = (m02 + m20) / S
    qy = (m12 + m21) / S
    qz = 0.25 * S

  return quat_normalize(np.array([qw, qx, qy, qz]))


# QUATERNION ##################################################################


def quat_norm(q):
  """ Returns norm of a quaternion """
  qw, qx, qy, qz = q
  return sqrt(qw**2 + qx**2 + qy**2 + qz**2)


def quat_normalize(q):
  """ Normalize quaternion """
  n = quat_norm(q)
  qw, qx, qy, qz = q
  return np.array([qw / n, qx / n, qy / n, qz / n])


def quat_conj(q):
  """ Return conjugate quaternion """
  qw, qx, qy, qz = q
  q_conj = np.array([qw, -qx, -qy, -qz])
  return q_conj


def quat_inv(q):
  """ Invert quaternion """
  return quat_conj(q)


def quat_left(q):
  """ Quaternion left product matrix """
  qw, qx, qy, qz = q
  row0 = [qw, -qx, -qy, -qz]
  row1 = [qx, qw, -qz, qy]
  row2 = [qy, qz, qw, -qx]
  row3 = [qz, -qy, qx, qw]
  return np.array([row0, row1, row2, row3])


def quat_right(q):
  """ Quaternion right product matrix """
  qw, qx, qy, qz = q
  row0 = [qw, -qx, -qy, -qz]
  row1 = [qx, qw, qz, -qy]
  row2 = [qy, -qz, qw, qx]
  row3 = [qz, qy, -qx, qw]
  return np.array([row0, row1, row2, row3])


def quat_lmul(p, q):
  """ Quaternion left multiply """
  assert len(p) == 4
  assert len(q) == 4
  lprod = quat_left(p)
  return lprod @ q


def quat_rmul(p, q):
  """ Quaternion right multiply """
  assert len(p) == 4
  assert len(q) == 4
  rprod = quat_right(q)
  return rprod @ p


def quat_mul(p, q):
  """ Quaternion multiply p * q """
  return quat_lmul(p, q)


def quat_omega(w):
  """ Quaternion omega matrix """
  return np.block([[-1.0 * skew(w), w], [w.T, 0.0]])


def quat_delta(dalpha):
  """ Form quaternion from small angle rotation vector dalpha """
  half_norm = 0.5 * norm(dalpha)
  scalar = cos(half_norm)
  vector = sinc(half_norm) * 0.5 * dalpha

  dqw = scalar
  dqx, dqy, dqz = vector
  dq = np.array([dqw, dqx, dqy, dqz])

  return dq


def quat_integrate(q_k, w, dt):
  """
  Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv
  preprint arXiv:1711.02508 (2017).
  [Section 4.6.1 Zeroth-order integration, p.47]
  """
  w_norm = norm(w)
  q_scalar = 0.0
  q_vec = np.array([0.0, 0.0, 0.0])

  if w_norm > 1e-5:
    q_scalar = cos(w_norm * dt * 0.5)
    q_vec = w / w_norm * sin(w_norm * dt * 0.5)
  else:
    q_scalar = 1.0
    q_vec = [0.0, 0.0, 0.0]

  q_kp1 = quat_mul(q_k, np.array([q_scalar, q_vec]))
  return q_kp1


def quat_slerp(q_i, q_j, t):
  """ Quaternion Slerp `q_i` and `q_j` with parameter `t` """
  assert len(q_i) == 4
  assert len(q_j) == 4
  assert t >= 0.0 and t <= 1.0

  # Compute the cosine of the angle between the two vectors.
  dot_result = q_i @ q_j

  # If the dot product is negative, slerp won't take
  # the shorter path. Note that q_j and -q_j are equivalent when
  # the negation is applied to all four components. Fix by
  # reversing one quaternion.
  if dot_result < 0.0:
    q_j = -q_j
    dot_result = -dot_result

  DOT_THRESHOLD = 0.9995
  if dot_result > DOT_THRESHOLD:
    # If the inputs are too close for comfort, linearly interpolate
    # and normalize the result.
    return q_i + t * (q_j - q_i)

  # Since dot is in range [0, DOT_THRESHOLD], acos is safe
  theta_0 = acos(dot_result)  # theta_0 = angle between input vectors
  theta = theta_0 * t  # theta = angle between q_i and result
  sin_theta = sin(theta)  # compute this value only once
  sin_theta_0 = sin(theta_0)  # compute this value only once

  # == sin(theta_0 - theta) / sin(theta_0)
  s0 = cos(theta) - dot_result * sin_theta / sin_theta_0
  s1 = sin_theta / sin_theta_0

  return (s0 * q_i) + (s1 * q_j)


# TF ##########################################################################


def quat2rot(q):
  """
  Convert quaternion to 3x3 rotation matrix.

  Source:
  Blanco, Jose-Luis. "A tutorial on se (3) transformation parameterizations
  and on-manifold optimization." University of Malaga, Tech. Rep 3 (2010): 6.
  [Page 18, Equation (2.20)]
  """
  assert len(q) == 4
  qw, qx, qy, qz = q

  qx2 = qx**2
  qy2 = qy**2
  qz2 = qz**2
  qw2 = qw**2

  # Homogeneous form
  C11 = qw2 + qx2 - qy2 - qz2
  C12 = 2.0 * (qx * qy - qw * qz)
  C13 = 2.0 * (qx * qz + qw * qy)

  C21 = 2.0 * (qx * qy + qw * qz)
  C22 = qw2 - qx2 + qy2 - qz2
  C23 = 2.0 * (qy * qz - qw * qx)

  C31 = 2.0 * (qx * qz - qw * qy)
  C32 = 2.0 * (qy * qz + qw * qx)
  C33 = qw2 - qx2 - qy2 + qz2

  return np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])


def euler321(yaw, pitch, roll):
  """
  Convert yaw, pitch, roll in radians to a 3x3 rotation matrix.

  Source:
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  [Page 85-86, "The Aerospace Sequence"]
  """
  psi = yaw
  theta = pitch
  phi = roll

  cpsi = cos(psi)
  spsi = sin(psi)
  ctheta = cos(theta)
  stheta = sin(theta)
  cphi = cos(phi)
  sphi = sin(phi)

  C11 = cpsi * ctheta
  C21 = spsi * ctheta
  C31 = -stheta

  C12 = cpsi * stheta * sphi - spsi * cphi
  C22 = spsi * stheta * sphi + cpsi * cphi
  C32 = ctheta * sphi

  C13 = cpsi * stheta * cphi + spsi * sphi
  C23 = spsi * stheta * cphi - cpsi * sphi
  C33 = ctheta * cphi

  return np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])


def tf(rot, trans):
  """
  Form 4x4 homogeneous transformation matrix from rotation `rot` and
  translation `trans`. Where the rotation component `rot` can be a rotation
  matrix or a quaternion.
  """
  C = None
  if rot.shape == (4,) or rot.shape == (4, 1):
    C = quat2rot(rot)
  elif rot.shape == (3, 3):
    C = rot
  else:
    raise RuntimeError("Invalid rotation!")

  T = np.eye(4, 4)
  T[0:3, 0:3] = C
  T[0:3, 3] = trans
  return T


def tf_rot(T):
  """ Return rotation matrix from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return T[0:3, 0:3]


def tf_quat(T):
  """ Return quaternion from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return rot2quat(tf_rot(T))


def tf_trans(T):
  """ Return translation vector from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return T[0:3, 3]


def tf_inv(T):
  """ Invert 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return np.linalg.inv(T)


def tf_point(T, p):
  """ Transform 3d point """
  assert T.shape == (4, 4)
  assert p.shape == (3,) or p.shape == (3, 1)
  hpoint = np.array([p[0], p[1], p[2], 1.0])
  return (T @ hpoint)[0:3]


def plot_set_axes_equal(ax):
  """
  Make axes of 3D plot have equal scale so that spheres appear as spheres,
  cubes as cubes, etc..  This is one possible solution to Matplotlib's
  ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

  Input
    ax: a matplotlib axis, e.g., as output from plt.gca().
  """
  x_limits = ax.get_xlim3d()
  y_limits = ax.get_ylim3d()
  z_limits = ax.get_zlim3d()

  x_range = abs(x_limits[1] - x_limits[0])
  x_middle = np.mean(x_limits)
  y_range = abs(y_limits[1] - y_limits[0])
  y_middle = np.mean(y_limits)
  z_range = abs(z_limits[1] - z_limits[0])
  z_middle = np.mean(z_limits)

  # The plot bounding box is a sphere in the sense of the infinity
  # norm, hence I call half the max range the plot radius.
  plot_radius = 0.5 * max([x_range, y_range, z_range])

  ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
  ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
  ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def tf_perturb(T, i, step_size):
  """ Perturb transformation matrix """
  assert T.shape == (4, 4)
  assert i >= 0 and i <= 5

  # Setup
  C = tf_rot(T)
  r = tf_trans(T)

  if i >= 0 and i <= 2:
    # Perturb translation
    r[i] += step_size

  elif i >= 3 and i <= 5:
    # Perturb rotation
    rvec = np.array([0.0, 0.0, 0.0])
    rvec[i - 3] = step_size

    q = rot2quat(C)
    dq = quat_delta(rvec)

    q_diff = quat_mul(q, dq)
    q_diff = quat_normalize(q_diff)

    C = quat2rot(q_diff)

  return tf(C, r)


def plot_tf(ax, T, **kwargs):
  """
  Plot 4x4 Homogeneous Transform

  Args:

    ax (matplotlib.axes.Axes): Plot axes object
    T (np.array): 4x4 homogeneous transform (i.e. Pose in the world frame)

  Keyword args:

    size (float): Size of the coordinate-axes
    linewidth (float): Thickness of the coordinate-axes
    name (str): Frame name
    name_offset (np.array or list): Position offset for displaying the frame's name
    fontsize (float): Frame font size
    fontweight (float): Frame font weight

  """
  assert T.shape == (4, 4)

  size = kwargs.get('size', 0.3)
  linewidth = kwargs.get('linewidth', 3)
  name = kwargs.get('name', None)
  name_offset = kwargs.get('name_offset', [0, 0, -0.01])
  fontsize = kwargs.get('fontsize', 10)
  fontweight = kwargs.get('fontweight', 'bold')
  colors = kwargs.get('colors', ['r-', 'g-', 'b-'])

  origin = tf_trans(T)
  lx = tf_point(T, np.array([size, 0.0, 0.0]))
  ly = tf_point(T, np.array([0.0, size, 0.0]))
  lz = tf_point(T, np.array([0.0, 0.0, size]))

  # Draw x-axis
  px = [origin[0], lx[0]]
  py = [origin[1], lx[1]]
  pz = [origin[2], lx[2]]
  ax.plot(px, py, pz, colors[0], linewidth=linewidth)

  # Draw y-axis
  px = [origin[0], ly[0]]
  py = [origin[1], ly[1]]
  pz = [origin[2], ly[2]]
  ax.plot(px, py, pz, colors[1], linewidth=linewidth)

  # Draw z-axis
  px = [origin[0], lz[0]]
  py = [origin[1], lz[1]]
  pz = [origin[2], lz[2]]
  ax.plot(px, py, pz, colors[2], linewidth=linewidth)

  # Draw label
  if name is not None:
    x = origin[0] + name_offset[0]
    y = origin[1] + name_offset[1]
    z = origin[2] + name_offset[2]
    ax.text(x, y, z, name, fontsize=fontsize, fontweight=fontweight)


if __name__ == "__main__":
  plt.figure()
  ax = plt.axes(projection='3d')

  # yapf: disable
  T_WF = np.array([
      [-0.121411, -0.234188, 0.964580, -0.858724],
      [0.992576, -0.035726, 0.116261, -0.407895],
      [0.007234, 0.971535, 0.236787, -0.486840],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])
  T_WS = np.array([
      # [-0.324045, 0.036747, -0.945328, 0.100000],
      # [0.036747, 0.998980, 0.026236, 0.00000],
      # [0.945328, -0.026236, -0.325065, 0.00000],
      # [0.000000, 0.000000, 0.000000, 1.000000]
      [-1.0, 0.0, 0.0, 0.1],
      [0.0, -1.0, 0.0, 0.0],
      [0.0, 0.0, 1.0, 0.0],
      [0.0, 0.0, 0.0, 1.0]
  ])
  T_BC0 = np.array([
      [0.0, -1.0, 0.0, 0.001],
      [1.0, 0.0, 0.0, 0.001],
      [0.0, 0.0, 1.0, 0.001],
      [0.0, 0.0, 0.0, 1.0]
  ])
  T_BS = np.array([
      [0.0, 0.0, 1.0, 0.001],
      [0.0, -1.0, 0.0, 0.001],
      [1.0, 0.0, 0.0, 0.001],
      [0.0, 0.0, 0.0, 1.0]
  ])
  T_WB = T_WS @ inv(T_BS)
  # yapf: enable
  plot_tf(ax, T_WF, name="T_WF")
  # plot_tf(ax, T_WB, name="T_WB")
  # plot_tf(ax, T_WS, name="T_WS")
  # plot_tf(ax, T_WB @ T_BC0, name="T_WC0")
  plot_tf(ax, T_WS @ inv(T_BS) @ T_BC0, name="T_WC0")

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()
