#!/usr/bin/env python3
import os
from os import listdir
from os.path import join
import numpy as np
import matplotlib.pylab as plt

from proto import plot_tf
from proto import plot_set_axes_equal
from proto import AprilGrid


def parse_timestamp(fname):
  ts, fext = fname.split(".")
  return int(ts)


if __name__ == "__main__":
  grid_path = "/tmp/sim_cam"
  grid_fnames = listdir(grid_path)

  for grid_fname in sorted(grid_fnames, key=parse_timestamp):
    grid = AprilGrid.load(join(grid_path, grid_fname))
    grid_data = grid.get_measurements()

    kps = []
    for tag_id, corner_idx, obj_pt, kp in grid_data:
      kps.append(kp)
    kps = np.array(kps)

    plt.figure()
    plt.plot(kps[:, 0], kps[:, 1], 'r.')
    plt.xlim([0, 640])
    plt.ylim([0, 480])
    plt.xlabel("pixel")
    plt.ylabel("pixel")
    plt.show()
