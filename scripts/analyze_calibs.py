#!/usr/bin/env python3
from math import sqrt
from math import atan2
from math import asin
from math import cos
from math import sin

import yaml
import numpy as np

calib_files = [
    "results-4-stddev/calib_stereo-1.yaml",
    "results-4-stddev/calib_stereo-2.yaml",
    "results-4-stddev/calib_stereo-3.yaml",
    "results-4-stddev/calib_stereo-4.yaml",
    "results-4-stddev/calib_stereo-5.yaml",
    "results-4-stddev/calib_stereo-6.yaml",
    "results-4-stddev/calib_stereo-7.yaml",
    "results-4-stddev/calib_stereo-8.yaml",
    "results-4-stddev/calib_stereo-9.yaml",
    "results-4-stddev/calib_stereo-10.yaml"
]


def quat2euler(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    qw2 = qw * qw
    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz

    x = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
    y = asin(2 * (qy * qw - qx * qz))
    z = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

    return [x, y, z]


def rot2quat(R):
    m00 = R[0, 0]
    m01 = R[0, 1]
    m02 = R[0, 2]

    m10 = R[1, 0]
    m11 = R[1, 1]
    m12 = R[1, 2]

    m20 = R[2, 0]
    m21 = R[2, 1]
    m22 = R[2, 2]

    tr = m00 + m11 + m22;

    if (tr > 0):
        S = sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif ((m00 > m11) and (m00 > m22)):
        S = sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif (m11 > m22):
        S = sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return np.array([qw, qx, qy, qz])


def euler321(rpy):
    phi = rpy[0]
    theta = rpy[1]
    psi = rpy[2]

    R11 = cos(psi) * cos(theta)
    R21 = sin(psi) * cos(theta)
    R31 = -sin(theta)

    R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
    R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
    R32 = cos(theta) * sin(phi)

    R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)
    R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)
    R33 = cos(theta) * cos(phi)

    R = np.array([R11, R12, R13, R21, R22, R23, R31, R32, R33])
    R = R.reshape((3, 3))
    return R


rpy = []
r_C1C0 = []
for calib_file in calib_files:
    calib = yaml.safe_load(open(calib_file))

    T_C1C0 = np.array(calib["T_C1C0"]["data"]).reshape((4, 4))
    r_C1C0.append(T_C1C0[0:3, 3])

    C_C1C0 = T_C1C0[0:3, 0:3]
    rpy_C1C0 = np.rad2deg(quat2euler(rot2quat(C_C1C0)))
    rpy.append(rpy_C1C0)

rpy = np.array(rpy)
r_C1C0 = np.array(r_C1C0)


def mean_extrinsics(calib_files):
    rpy = []
    trans = []
    for calib_file in calib_files:
        calib = yaml.safe_load(open(calib_file))

        T_C1C0 = np.array(calib["T_C1C0"]["data"]).reshape((4, 4))
        trans.append(T_C1C0[0:3, 3])

        C_C1C0 = T_C1C0[0:3, 0:3]
        rpy_C1C0 = np.rad2deg(quat2euler(rot2quat(C_C1C0)))
        rpy.append(rpy_C1C0)

    rpy = np.array(rpy)
    trans = np.array(trans)

    mean_x = np.mean(trans[:, 0])
    mean_y = np.mean(trans[:, 1])
    mean_z = np.mean(trans[:, 2])
    r_C1C0 = np.array([[mean_x], [mean_y], [mean_z]])

    mean_roll = np.deg2rad(np.mean(rpy[:, 0]))
    mean_pitch = np.deg2rad(np.mean(rpy[:, 1]))
    mean_yaw = np.deg2rad(np.mean(rpy[:, 2]))
    C_C1C0 = euler321([mean_roll, mean_pitch, mean_yaw])

    T_C1C0 = np.block([[C_C1C0, r_C1C0], [0, 0, 0, 1]])

    return T_C1C0

def mean_camera_params(cam_idx, calib_files):
    resolution = None
    proj_model = None
    dist_model = None
    proj_params = []
    dist_params = []
    for calib_file in calib_files:
        calib = yaml.safe_load(open(calib_file))

        proj = np.array(calib["cam" + str(cam_idx)]["proj_params"])
        proj = proj.reshape((4, 1))
        dist = np.array(calib["cam" + str(cam_idx)]["dist_params"])
        dist = dist.reshape((4, 1))

        if resolution is None:
            resolution = calib["cam" + str(cam_idx)]["resolution"]

        if proj_model is None:
            proj_model = calib["cam" + str(cam_idx)]["proj_model"]

        if dist_model is None:
            dist_model = calib["cam" + str(cam_idx)]["dist_model"]

        proj_params.append(proj)
        dist_params.append(dist)

    proj_params = np.array(proj_params)
    dist_params = np.array(dist_params)

    mean_fx = np.mean(proj_params[:, 0])
    mean_fy = np.mean(proj_params[:, 1])
    mean_cx = np.mean(proj_params[:, 2])
    mean_cy = np.mean(proj_params[:, 3])

    mean_k1 = np.mean(dist_params[:, 0])
    mean_k2 = np.mean(dist_params[:, 1])
    mean_k3 = np.mean(dist_params[:, 2])
    mean_k4 = np.mean(dist_params[:, 3])

    return {
        "camera_index": cam_idx,
        "resolution": resolution,
        "proj_model": proj_model,
        "proj_params": [mean_fx, mean_fy, mean_cx, mean_cy],
        "dist_model": dist_model,
        "dist_params": [mean_k1, mean_k2, mean_k3, mean_k4]
    }

cam0_params = mean_camera_params(0, calib_files)
cam1_params = mean_camera_params(1, calib_files)
T_C1C0 = mean_extrinsics(calib_files)

img_w = cam0_params["resolution"][0]
img_h = cam0_params["resolution"][1]

yaml_file = """\
cam0:
    resolution: [{img_w}, {img_h}]
    proj_model: \"{cam0_proj_model}\"
    dist_model: \"{cam0_dist_model}\"
    proj_params: [{cam0_proj_params}]
    dist_params: [{cam0_dist_params}]

cam1:
    resolution: [{img_w}, {img_h}]
    proj_model: \"{cam1_proj_model}\"
    dist_model: \"{cam1_dist_model}\"
    proj_params: [{cam1_proj_params}]
    dist_params: [{cam1_dist_params}]

T_C1C0:
    rows: 4
    cols: 4
    data: [
        {r00}, {r01}, {r02}, {r03},
        {r10}, {r11}, {r12}, {r13},
        {r20}, {r21}, {r22}, {r23},
        0.0, 0.0, 0.0, 1.0
    ]
""".format(img_w=img_w, img_h=img_h,
           cam0_proj_model=cam0_params["proj_model"],
           cam0_dist_model=cam0_params["dist_model"],
           cam0_proj_params=",".join(["%.10f" % x for x in cam0_params["proj_params"]]),
           cam0_dist_params=",".join(["%.10f" % x for x in cam0_params["dist_params"]]),
           cam1_proj_model=cam1_params["proj_model"],
           cam1_dist_model=cam1_params["dist_model"],
           cam1_proj_params=",".join(["%.10f" % x for x in cam1_params["proj_params"]]),
           cam1_dist_params=",".join(["%.10f" % x for x in cam1_params["dist_params"]]),
           r00=T_C1C0[0, 0], r01=T_C1C0[0, 1], r02=T_C1C0[0, 2], r03=T_C1C0[0, 3],
           r10=T_C1C0[1, 0], r11=T_C1C0[1, 1], r12=T_C1C0[1, 2], r13=T_C1C0[1, 3],
           r20=T_C1C0[2, 0], r21=T_C1C0[2, 1], r22=T_C1C0[2, 2], r23=T_C1C0[2, 3])

calib_file = open("calib_stereo.yaml", "w")
calib_file.write(yaml_file)
calib_file.close()


# print(rpy[:, 1])

# print("r_C1C0 - x [m]:")
# print("  mean:  ", np.mean(r_C1C0[:, 0]))
# print("  median:", np.median(r_C1C0[:, 0]))
# print("  std:   ", np.std(r_C1C0[:, 0]))
# print("")
#
# print("r_C1C0 - y [m]:")
# print("  mean:  ", np.mean(r_C1C0[:, 1]))
# print("  median:", np.median(r_C1C0[:, 1]))
# print("  std:   ", np.std(r_C1C0[:, 1]))
# print("")
#
# print("r_C1C0 - z [m]:")
# print("  mean:  ", np.mean(r_C1C0[:, 2]))
# print("  median:", np.median(r_C1C0[:, 2]))
# print("  std:   ", np.std(r_C1C0[:, 2]))
# print("")
#
# print("T_C1C0 - roll [deg]:")
# print("  mean:  ", np.mean(rpy[:, 0]))
# print("  median:", np.median(rpy[:, 0]))
# print("  std:   ", np.std(rpy[:, 0]))
# print("")
#
# print("T_C1C0 - pitch [deg]:")
# print("  mean:  ", np.mean(rpy[:, 1]))
# print("  median:", np.median(rpy[:, 1]))
# print("  std:   ", np.std(rpy[:, 1]))
# print("")
#
# print("T_C1C0 - yaw [deg]:")
# print("  mean:  ", np.mean(rpy[:, 2]))
# print("  median:", np.median(rpy[:, 2]))
# print("  std:   ", np.std(rpy[:, 2]))
#
