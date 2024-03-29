""" Plot Poses """
import numpy as np
from numpy.linalg import inv

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

from proto import plot_tf
from proto import plot_set_axes_equal

if __name__ == "__main__":
  plt.figure()
  ax = plt.axes(projection='3d')

  # yapf:disable
  T_WS = np.array([
      [0.320679, 0.062018, 0.945155, -0.827194],
      [0.000000, -0.997854, 0.065476, -0.526340],
      [0.947188, -0.020997, -0.319991, 0.513301],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])

  T_WF = np.array([
      [0.199203, 0.220606, -0.954804, 0.000000],
      [-0.979300, 0.080513, -0.185712, 0.000000],
      [0.035905, 0.972034, 0.232078, 0.000000],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])

  T_BS = np.array([
      [1.000000, 0.000000, 0.000000, 0.000000],
      [0.000000, 1.000000, 0.000000, 0.000000],
      [0.000000, 0.000000, 1.000000, 0.000000],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])

  T_BC0 = np.array([
      [0.014866, -0.999881, 0.004140, -0.021640],
      [0.999557, 0.014967, 0.025716, -0.064677],
      [-0.025774, 0.003756, 0.999661, 0.009811],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])

  T_BC1 = np.array([
      [0.012555, -0.999755, 0.018224, -0.019844],
      [0.999599, 0.013012, 0.025159, 0.045369],
      [-0.025390, 0.017901, 0.999517, 0.007862],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])
  # yapf:enable

  plot_tf(ax, T_WF, name="T_WF", size=0.3)
  plot_tf(ax, T_WS, name="T_WS", size=0.3)
  plot_tf(ax, T_WS @ inv(T_BS) @ T_BC0, name="T_WC0", size=0.3)
  plot_tf(ax, T_WS @ inv(T_BS) @ T_BC1, name="T_WC1", size=0.3)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()
