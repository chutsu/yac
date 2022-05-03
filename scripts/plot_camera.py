""" Plot camera view """
#!/usr/bin/env python3
from os import listdir
from os.path import join
import numpy as np
import matplotlib.pylab as plt
from matplotlib import animation

from proto import AprilGrid


def parse_timestamp(fname):
  """ Parse timestamp """
  ts, _ = fname.split(".")
  return int(ts)


if __name__ == "__main__":
  # Load data
  save_path = "/tmp/sim_cam/animation.mp4"
  grid_path = "/tmp/sim_cam/cam0"
  grid_fnames = listdir(grid_path)

  # Figure setup
  fig = plt.figure()
  ax = plt.axes()
  ax.set_xlim([0, 640])
  ax.set_ylim([0, 480])
  ax.set_xlabel("pixel")
  ax.set_ylabel("pixel")
  ax.invert_yaxis()
  ax.xaxis.tick_top()
  ax.xaxis.set_label_position('top')

  # Setup FFMpeg writer
  writer = animation.FFMpegWriter(fps=30)
  writer.setup(fig, save_path, 100)

  # Animate
  for grid_fname in sorted(grid_fnames, key=parse_timestamp):
    grid = AprilGrid.load(join(grid_path, grid_fname))
    grid_data = grid.get_measurements()

    kps = []
    for tag_id, corner_idx, obj_pt, kp in grid_data:
      kps.append(kp)
    kps = np.array(kps)

    points = ax.plot(kps[:, 0], kps[:, 1], 'r.')[0]
    writer.grab_frame()
    ax.lines.remove(points)
