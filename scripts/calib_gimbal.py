#!/usr/bin/env python3
import os
from os.path import join
import yaml

import numpy as np
import proto
from proto import plot_tf
from proto import plot_set_axes_equal
import matplotlib.pylab as plt
from mpl_toolkits import mplot3d


def load_yaml(file_path):
  file = open(file_path, "r")
  return yaml.safe_load(file)


def create_calib_camimu_config(camera_calib_file):
  default_config = open(join(calib_dir, "calib_camimu.config"))
  calib_camimu_conf = yaml.safe_load(default_config)
  camera_calib = yaml.safe_load(open(camera_calib_file, "r"))
  calib_target = camera_calib["calib_target"]
  cam0_proj_params = camera_calib["cam0"]["proj_params"]
  cam0_dist_params = camera_calib["cam0"]["dist_params"]
  cam1_proj_params = camera_calib["cam1"]["proj_params"]
  cam1_dist_params = camera_calib["cam1"]["dist_params"]
  imu_params = camera_calib["imu0"]
  T = camera_calib["T_cam0_cam1"]["data"]

  calib_conf = f"""\
settings:
  format_v2: {calib_camimu_conf["settings"]["format_v2"]}
  enable_outlier_rejection: {calib_camimu_conf["settings"]["enable_outlier_rejection"]}
  img_scale: {calib_camimu_conf["settings"]["img_scale"]}

calib_target:
  target_type: {calib_target["target_type"]}
  tag_rows: {calib_target["tag_rows"]}
  tag_cols: {calib_target["tag_cols"]}
  tag_size: {calib_target["tag_size"]}
  tag_spacing: {calib_target["tag_spacing"]}

cam0:
  fixed: false
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam0_proj_params}
  dist_params: {cam0_dist_params}

cam1:
  fixed: false
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam1_proj_params}
  dist_params: {cam1_dist_params}

T_cam0_cam1:
  fixed: false
  rows: 4
  cols: 4
  data: [
    {T[0]}, {T[1]}, {T[2]}, {T[3]},
    {T[4]}, {T[5]}, {T[6]}, {T[7]},
    {T[8]}, {T[9]}, {T[10]}, {T[11]},
    {T[12]}, {T[13]}, {T[14]}, {T[15]}
  ]

imu0:
  rate: 200.000000
  sigma_a_c: {imu_params["sigma_a_c"]}
  sigma_g_c: {imu_params["sigma_g_c"]}
  sigma_aw_c: {imu_params["sigma_aw_c"]}
  sigma_gw_c: {imu_params["sigma_gw_c"]}
  g: {imu_params["g"]}
  """
  save_path = "/tmp/calib_camimu.config"
  calib_config_file = open(save_path, "w")
  calib_config_file.write(calib_conf)
  calib_config_file.close()

  return save_path


def tf_vector(T):
  """ Return tf vector """
  r = proto.tf_trans(T)
  q = proto.tf_quat(T)
  return np.array([r[0], r[1], r[2], q[0], q[1], q[2], q[3]])


def create_gimbal_calib_config(rs0_results, rs1_results, save_path):
  np.set_printoptions(linewidth=1000)
  rs0 = load_yaml(rs0_results)
  rs1 = load_yaml(rs1_results)

  imu_params = rs0["imu0"]
  cam0_proj_params = rs0["cam0"]["proj_params"]
  cam1_proj_params = rs0["cam1"]["proj_params"]
  cam2_proj_params = rs1["cam0"]["proj_params"]
  cam3_proj_params = rs1["cam1"]["proj_params"]
  cam0_dist_params = rs0["cam0"]["dist_params"]
  cam1_dist_params = rs0["cam1"]["dist_params"]
  cam2_dist_params = rs1["cam0"]["dist_params"]
  cam3_dist_params = rs1["cam1"]["dist_params"]

  T_cam0_cam1 = np.array(rs0["T_cam0_cam1"]["data"]).reshape((4, 4))
  T_cam2_cam3 = np.array(rs1["T_cam0_cam1"]["data"]).reshape((4, 4))
  cam0_cam1_ext = tf_vector(T_cam0_cam1)
  cam2_cam3_ext = tf_vector(T_cam2_cam3)

  # Gimbal extrinsic
  r_C0M0 = np.array([0.025, -0.105, -0.045])
  q_C0M0 = proto.euler2quat(np.pi / 2, -np.pi / 2, 0.0)
  T_C0M0 = proto.tf(q_C0M0, r_C0M0)
  gimbal_ext = tf_vector(T_C0M0)

  # Joint 0
  r_M0L0 = np.array([0.0, 0.0, 0.0])
  q_M0L0 = proto.euler2quat(0.0, 0.0, 0.0)
  T_M0L0 = proto.tf(q_M0L0, r_M0L0)

  # Link 0
  r_L0M1 = np.array([0.0, 0.0, 0.045])
  q_L0M1 = proto.euler2quat(0.0, np.pi / 2, 0.0)
  T_L0M1 = proto.tf(q_L0M1, r_L0M1)
  link0_ext = tf_vector(T_L0M1)

  # Joint 1
  r_M1L1 = np.array([0.0, 0.0, 0.0])
  q_M1L1 = proto.euler2quat(0.0, 0.0, 0.0)
  T_M1L1 = proto.tf(q_M1L1, r_M1L1)

  # Link1
  r_L1M2 = np.array([0.0, 0.0, 0.03])
  q_L1M2 = proto.euler2quat(0.0, 0.0, -np.pi / 2)
  T_L1M2 = proto.tf(q_L1M2, r_L1M2)
  link1_ext = tf_vector(T_L1M2)

  # Joint 2
  r_M2L2 = np.array([0.0, 0.0, 0.0])
  q_M2L2 = proto.euler2quat(0.0, 0.0, 0.0)
  T_M2L2 = proto.tf(q_M2L2, r_M2L2)

  # End effector extrinsic
  r_L2C2 = np.array([0.0, -0.035, 0.02])
  q_L2C2 = proto.euler2quat(-np.pi / 2, np.pi / 2, 0.0)
  T_L2C2 = proto.tf(q_L2C2, r_L2C2)
  end_ext = tf_vector(T_L2C2)

  calib_conf = f"""\
num_cams: 4
num_links: 3

cam0:
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam0_proj_params}
  dist_params: {cam0_dist_params}

cam1:
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam1_proj_params}
  dist_params: {cam1_dist_params}

cam2:
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam2_proj_params}
  dist_params: {cam2_dist_params}

cam3:
  resolution: [640, 480]
  proj_model: "pinhole"
  dist_model: "radtan4"
  proj_params: {cam3_proj_params}
  dist_params: {cam3_dist_params}

gimbal_ext: {gimbal_ext.tolist()}
link0_ext: {link0_ext.tolist()}
link1_ext: {link1_ext.tolist()}
end_ext: {end_ext.tolist()}
cam0_cam1_ext: {cam0_cam1_ext.tolist()}
cam2_cam3_ext: {cam2_cam3_ext.tolist()}
  """
  conf_file = open(save_path, "w")
  conf_file.write(calib_conf)
  conf_file.close()

  plt.figure()
  ax = plt.axes(projection='3d')

  # yapf:disable
  T_WC0 = np.array([
      [0.000000, 0.000000, 1.000000, 0.000000],
      [-1.000000, 0.000000, 0.000000, 0.000000],
      [0.000000, -1.000000, 0.000000, 0.000000],
      [0.000000, 0.000000, 0.000000, 1.000000]
  ])
  # yapf:enable

  T_WC1 = T_WC0 @ T_cam0_cam1
  T_WM0 = T_WC0 @ T_C0M0
  T_WM1 = T_WM0 @ T_M0L0 @ T_L0M1
  T_WM2 = T_WM1 @ T_M1L1 @ T_L1M2
  T_WC2 = T_WM2 @ T_M2L2 @ T_L2C2
  T_WC3 = T_WC2 @ T_cam2_cam3

  plot_tf(ax, T_WC0, name="C0", size=0.02)
  plot_tf(ax, T_WC1, name="C1", size=0.02)
  plot_tf(ax, T_WM0, name="M0", size=0.02)
  plot_tf(ax, T_WM1, name="M1", size=0.02)
  plot_tf(ax, T_WM2, name="M2", size=0.02)
  plot_tf(ax, T_WC2, name="C2", size=0.02)
  plot_tf(ax, T_WC3, name="C3", size=0.02)

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  plot_set_axes_equal(ax)
  plt.show()


if __name__ == "__main__":
  # calib_dir = "/data/gimbal_experiments/calib"
  calib_dir = "/home/chutsu/calib-240212"
  calib_camera_conf = join(calib_dir, "calib_camera.config")
  calib_camera_rs0 = join(calib_dir, "calib_camera-rs0")
  calib_camera_rs1 = join(calib_dir, "calib_camera-rs1")
  calib_camimu_rs0 = join(calib_dir, "calib_camimu-rs0")
  calib_camimu_rs1 = join(calib_dir, "calib_camimu-rs1")
  rs0_camera_results = join(calib_camera_rs0, "calib_camera-results.yaml")
  rs1_camera_results = join(calib_camera_rs1, "calib_camera-results.yaml")
  rs0_camimu_results = join(calib_camimu_rs0, "calib_vi-results.yaml")
  rs1_camimu_results = join(calib_camimu_rs1, "calib_vi-results.yaml")
  calib_gimbal = join(calib_dir, "calib_gimbal", "calib.config")

  # Calibrate camera intrinsics and extrinsics
  # os.system(f"./build/calib_camera {calib_camera_conf} {calib_camera_rs0}")
  # os.system(f"./build/calib_camera {calib_camera_conf} {calib_camera_rs1}")

  # Calibrate camera-imu extrinsics
  # -- Calibrate rs0
  # calib_camimu_path = create_calib_camimu_config(rs0_camera_results)
  # calib_camimu_rs0 = join(calib_dir, "calib_camimu-rs0")
  # os.system(f"./build/calib_vi {calib_camimu_path} {calib_camimu_rs0}")
  # -- Calibrate rs1
  # calib_camimu_path = create_calib_camimu_config(rs1_camera_results)
  # calib_camimu_rs1 = join(calib_dir, "calib_camimu-rs1")
  # os.system(f"./build/calib_vi {calib_camimu_path} {calib_camimu_rs1}")

  # Create gimbal calibration file
  # create_gimbal_calib_config(rs0_camimu_results, rs1_camimu_results,
  #                            calib_gimbal)
