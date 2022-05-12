#!/usr/bin/env python3
"""
Plot nbt_find() results
"""
# import numpy as np
import matplotlib.pylab as plt

# import proto
from proto import load_poses
from proto import plot_tf
from proto import plot_set_axes_equal


def plot_poses():
  """ Plot poses """
  before = load_poses("/tmp/nbt_before/poses_est.csv")
  after = load_poses("/tmp/nbt_after/poses_est.csv")

  # Plot NBT
  plt.figure()
  ax = plt.axes(projection='3d')

  for pose_data in before:
    _, pose = pose_data
    plot_tf(ax, pose, size=0.1, colors=['r', 'r', 'r'])

  for pose_data in after:
    _, pose = pose_data
    plot_tf(ax, pose, size=0.1, colors=['b', 'b', 'b'])

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()


plot_poses()
plt.show()
