""" Plot Poses """
import sys

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

from proto import load_poses
from proto import plot_tf
from proto import plot_set_axes_equal

if __name__ == "__main__":
  calib_poses = load_poses(sys.argv[1])

  plt.figure()
  ax = plt.axes(projection='3d')

  for calib_pose in calib_poses:
    _, T_OC = calib_pose
    plot_tf(ax, T_OC, size=0.1)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()
