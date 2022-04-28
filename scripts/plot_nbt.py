#!/usr/bin/env python3
import os
import glob

import numpy as np
from numpy import genfromtxt

import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

import proto
from proto import load_poses
from proto import plot_tf
from proto import plot_set_axes_equal

# Load fiducial pose
T_WF = genfromtxt("/tmp/nbt/fiducial_pose.csv", delimiter=",")

# Plot NBT
plt.figure()
ax = plt.axes(projection='3d')

plot_tf(ax, T_WF, name="T_WF", size=0.1)
for traj_path in glob.glob("/tmp/nbt/traj/*.csv"):
  idx = 0
  for ts, pose in load_poses(traj_path):
    if idx % 20 == 0:
      plot_tf(ax, pose, size=0.1)
    idx += 1

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
plot_set_axes_equal(ax)
plt.show()
