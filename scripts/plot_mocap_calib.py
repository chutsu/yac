""" Plot mocap calibration results """
#!/usr/bin/env python3
import glob
import yaml
from os.path import join

import proto
import numpy as np
from scipy import signal
from scipy import interpolate
import matplotlib.pylab as plt

# data_dir = "/mnt/koala/calibration_d455_ros_20220713/calibration_d455_ros_20220713_183241"
# data_dir = "/mnt/koala/calibration_s550_jetson_20220714/calibration_s550_jetson_20220714_192445"
data_dir = "/data/calib0"


def lag_finder(y1, y2, sr):
  n = len(y1)

  corr = signal.correlate(y2, y1, mode='same') / np.sqrt(
      signal.correlate(y1, y1, mode='same')[int(n / 2)] *
      signal.correlate(y2, y2, mode='same')[int(n / 2)])

  delay_arr = np.linspace(-0.5 * n / sr, 0.5 * n / sr, n)
  delay = delay_arr[np.argmax(corr)]
  print('y2 is ' + str(delay) + ' behind y1')

  plt.figure()
  plt.plot(delay_arr, corr)
  plt.title('Lag: ' + str(np.round(delay, 3)) + ' s')
  plt.xlabel('Lag')
  plt.ylabel('Correlation coeff')

  plt.figure()
  plt.plot(range(len(y1)), y1, "r-")
  plt.plot(range(len(y2)), y2, "g-")

  plt.show()


if __name__ == "__main__":
  # Load results file
  results_file = join(data_dir, "calib_mocap-results.yaml")
  results_data = yaml.safe_load(open(results_file, "r").read())

  # Camera parameters
  cam_idx = 0
  cam_res = np.array(results_data["cam0"]["resolution"])
  proj_model = results_data["cam0"]["proj_model"]
  dist_model = results_data["cam0"]["dist_model"]
  proj_params = results_data["cam0"]["proj_params"]
  dist_params = results_data["cam0"]["dist_params"]
  param_data = np.array(proj_params + dist_params)
  args = [cam_idx, cam_res, proj_model, dist_model, param_data]
  cam_params = proto.camera_params_setup(*args)

  # Mocap-Camera extrinsics
  T_MC0 = np.array(results_data["T_mocap_camera"]["data"])
  T_MC0 = T_MC0.reshape((4, 4))

  # Fiducial pose
  T_WF = np.array(results_data["T_world_fiducial"]["data"])
  T_WF = T_WF.reshape((4, 4))

  # Mocap poses
  mocap_poses = {}
  nb_rows = results_data["mocap_poses"]["rows"]
  nb_cols = results_data["mocap_poses"]["cols"]
  mocap_data = np.array(results_data["mocap_poses"]["data"])
  mocap_data = mocap_data.reshape((nb_rows, nb_cols))

  mocap_camera_timestamps = []
  mocap_camera_poses = np.zeros((nb_rows, 7))

  for k in range(results_data["mocap_poses"]["rows"]):
    ts, rx, ry, rz, qx, qy, qz, qw = mocap_data[k, :]
    ts = int(ts)
    r = np.array([rx, ry, rz])
    q = np.array([qw, qx, qy, qz])
    T_WM = proto.tf(q, r)
    mocap_poses[ts] = T_WM

    T_WC0 = T_WM @ T_MC0
    mocap_camera_timestamps.append(ts)
    mocap_camera_poses[k, :] = proto.tf2pose(T_WC0)
  mocap_camera_timestamps = np.array(mocap_camera_timestamps)

  # AprilGrids
  grid_files = glob.glob(join(data_dir, "grid0", "cam0", "data", "*.csv"))
  grid_files = sorted(grid_files)

  grids = []
  camera_timestamps = []
  camera_poses = []
  for k, grid_file in enumerate(grid_files):
    grid = proto.AprilGrid.load(grid_file)
    if grid is None:
      continue

    grids.append(grid)
    T_C0F = grid.solvepnp(cam_params)
    T_FC0 = np.linalg.inv(T_C0F)
    T_WC0 = T_WF @ T_FC0

    camera_timestamps.append(grid.ts)
    camera_poses.append(proto.tf2pose(T_WC0))
  camera_timestamps = np.array(camera_timestamps)
  camera_poses = np.array(camera_poses)

  first_ts = None
  if camera_timestamps[0] < mocap_camera_timestamps[0]:
    first_ts = camera_timestamps[0]
  else:
    first_ts = mocap_camera_timestamps[0]

  vision_time = (camera_timestamps - first_ts) * 1e-9
  mocap_time = (mocap_camera_timestamps - first_ts) * 1e-9

  mocap_x_fn = interpolate.interp1d(mocap_camera_timestamps,
                                    mocap_camera_poses[:, 0])
  mocap_y_fn = interpolate.interp1d(mocap_camera_timestamps,
                                    mocap_camera_poses[:, 1])
  mocap_z_fn = interpolate.interp1d(mocap_camera_timestamps,
                                    mocap_camera_poses[:, 2])

  camera_x_fn = interpolate.interp1d(camera_timestamps, camera_poses[:, 0])
  camera_y_fn = interpolate.interp1d(camera_timestamps, camera_poses[:, 1])
  camera_z_fn = interpolate.interp1d(camera_timestamps, camera_poses[:, 2])

  interp_timestamps = []
  timestamps = None
  if camera_timestamps[0] > mocap_camera_timestamps[0]:
    timestamps = camera_timestamps
  else:
    timestamps = mocap_camera_timestamps

  for ts in timestamps:
    if ts > camera_timestamps[-1] or ts > mocap_camera_timestamps[-1]:
      break
    interp_timestamps.append(ts)

  # dt = (vision_time[-1] - vision_time[0])
  # sr = len(vision_time) / dt
  # print(sr)
  # lag_finder(camera_x_fn(interp_timestamps), mocap_x_fn(interp_timestamps), sr)
  # lag_finder(camera_y_fn(interp_timestamps), mocap_y_fn(interp_timestamps), sr)
  # lag_finder(camera_z_fn(interp_timestamps), mocap_z_fn(interp_timestamps), sr)

  # Plot camera poses
  plt.figure()

  plt.subplot(311)
  plt.plot(vision_time, camera_poses[:, 0], 'r-', label="vision - x")
  plt.plot(mocap_time, mocap_camera_poses[:, 0], 'r--', label="mocap - x")
  plt.xlabel("Time [s]")
  plt.ylabel("Displacement [m]")
  plt.legend(loc=0)

  plt.subplot(312)
  plt.plot(vision_time, camera_poses[:, 1], 'g-', label="vision - y")
  plt.plot(mocap_time, mocap_camera_poses[:, 1], 'g--', label="mocap - y")
  plt.xlabel("Time [s]")
  plt.ylabel("Displacement [m]")
  plt.legend(loc=0)

  plt.subplot(313)
  plt.plot(vision_time, camera_poses[:, 2], 'b-', label="vision - z")
  plt.plot(mocap_time, mocap_camera_poses[:, 2], 'b--', label="mocap - z")
  plt.xlabel("Time [s]")
  plt.ylabel("Displacement [m]")
  plt.legend(loc=0)

  plt.show()
