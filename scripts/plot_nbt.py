#!/usr/bin/env python3
import os
import glob

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

import proto
from proto import load_poses
from proto import plot_tf
from proto import plot_set_axes_equal

plt.figure()
ax = plt.axes(projection='3d')

for traj_path in glob.glob("/tmp/nbt/traj/*.csv"):
  for ts, pose in load_poses(traj_path):
    plot_tf(ax, pose, size=0.1)

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
plot_set_axes_equal(ax)
plt.show()
