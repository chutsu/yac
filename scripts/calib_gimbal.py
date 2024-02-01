#!/usr/bin/env python3
import os
from os.path import join
import yaml
import numpy as np


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


def create_gimbal_calib_config(rs0_results, rs1_results, save_path):
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
  T_imu0_cam0 = rs0["T_imu0_cam0"]["data"]
  T_cam0_cam1 = rs0["T_cam0_cam1"]["data"]
  T_cam2_cam3 = rs1["T_cam0_cam1"]["data"]

  calib_conf = f"""\
num_cams: 2
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

T_imu0_cam0:
  rows: 4
  cols: 4
  data: [
    {T_imu0_cam0[0]}, {T_imu0_cam0[1]}, {T_imu0_cam0[2]}, {T_imu0_cam0[3]},
    {T_imu0_cam0[4]}, {T_imu0_cam0[5]}, {T_imu0_cam0[6]}, {T_imu0_cam0[7]},
    {T_imu0_cam0[8]}, {T_imu0_cam0[9]}, {T_imu0_cam0[10]}, {T_imu0_cam0[11]},
    {T_imu0_cam0[12]}, {T_imu0_cam0[13]}, {T_imu0_cam0[14]}, {T_imu0_cam0[15]}
  ]

T_cam0_cam1:
  rows: 4
  cols: 4
  data: [
    {T_cam0_cam1[0]}, {T_cam0_cam1[1]}, {T_cam0_cam1[2]}, {T_cam0_cam1[3]},
    {T_cam0_cam1[4]}, {T_cam0_cam1[5]}, {T_cam0_cam1[6]}, {T_cam0_cam1[7]},
    {T_cam0_cam1[8]}, {T_cam0_cam1[9]}, {T_cam0_cam1[10]}, {T_cam0_cam1[11]},
    {T_cam0_cam1[12]}, {T_cam0_cam1[13]}, {T_cam0_cam1[14]}, {T_cam0_cam1[15]}
  ]

T_cam2_cam3:
  rows: 4
  cols: 4
  data: [
    {T_cam2_cam3[0]}, {T_cam2_cam3[1]}, {T_cam2_cam3[2]}, {T_cam2_cam3[3]},
    {T_cam2_cam3[4]}, {T_cam2_cam3[5]}, {T_cam2_cam3[6]}, {T_cam2_cam3[7]},
    {T_cam2_cam3[8]}, {T_cam2_cam3[9]}, {T_cam2_cam3[10]}, {T_cam2_cam3[11]},
    {T_cam2_cam3[12]}, {T_cam2_cam3[13]}, {T_cam2_cam3[14]}, {T_cam2_cam3[15]}
  ]

imu0:
  rate: 200.000000
  sigma_a_c: {imu_params["sigma_a_c"]}
  sigma_g_c: {imu_params["sigma_g_c"]}
  sigma_aw_c: {imu_params["sigma_aw_c"]}
  sigma_gw_c: {imu_params["sigma_gw_c"]}
  g: {imu_params["g"]}
  """
  conf_file = open(save_path, "w")
  conf_file.write(calib_conf)
  conf_file.close()


if __name__ == "__main__":
  calib_dir = "/data/gimbal_experiments/calib"
  calib_camera_conf = join(calib_dir, "calib_camera.config")
  calib_camera_rs0 = join(calib_dir, "calib_camera-rs0")
  calib_camera_rs1 = join(calib_dir, "calib_camera-rs1")
  calib_camimu_rs0 = join(calib_dir, "calib_camimu-rs0")
  calib_camimu_rs1 = join(calib_dir, "calib_camimu-rs1")
  rs0_camera_results = join(calib_camera_rs0, "calib_camera-results.yaml")
  rs1_camera_results = join(calib_camera_rs1, "calib_camera-results.yaml")
  rs0_camimu_results = join(calib_camimu_rs0, "calib_vi-results.yaml")
  rs1_camimu_results = join(calib_camimu_rs1, "calib_vi-results.yaml")
  calib_gimbal = join(calib_dir, "calib_gimbal.yaml")

  # Calibrate camera intrinsics and extrinsics
  # os.system(f"./build/calib_camera {calib_camera_conf} {calib_camera_rs0}")
  # os.system(f"./build/calib_camera {calib_camera_conf} {calib_camera_rs1}")

  # # Calibrate camera-imu extrinsics
  # # -- Calibrate rs0
  # calib_camimu_path = create_calib_camimu_config(rs0_camera_results)
  # calib_camimu_rs0 = join(calib_dir, "calib_camimu-rs0")
  # os.system(f"./build/calib_vi {calib_camimu_path} {calib_camimu_rs0}")
  # # -- Calibrate rs1
  # calib_camimu_path = create_calib_camimu_config(rs1_camera_results)
  # calib_camimu_rs1 = join(calib_dir, "calib_camimu-rs1")
  # os.system(f"./build/calib_vi {calib_camimu_path} {calib_camimu_rs1}")

  # Create gimbal calibration file
  # create_gimbal_calib_config(rs0_camimu_results, rs1_camimu_results, calib_gimbal)

