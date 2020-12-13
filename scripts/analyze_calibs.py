#!/usr/bin/env python3
from math import sqrt
from math import atan2
from math import asin

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

print("r_C1C0 - x [m]:")
print("  mean:  ", np.mean(r_C1C0[:, 0]))
print("  median:", np.median(r_C1C0[:, 0]))
print("  std:   ", np.std(r_C1C0[:, 0]))
print("")

print("r_C1C0 - y [m]:")
print("  mean:  ", np.mean(r_C1C0[:, 1]))
print("  median:", np.median(r_C1C0[:, 1]))
print("  std:   ", np.std(r_C1C0[:, 1]))
print("")

print("r_C1C0 - z [m]:")
print("  mean:  ", np.mean(r_C1C0[:, 2]))
print("  median:", np.median(r_C1C0[:, 2]))
print("  std:   ", np.std(r_C1C0[:, 2]))
print("")

print("T_C1C0 - roll [deg]:")
print("  mean:  ", np.mean(rpy[:, 0]))
print("  median:", np.median(rpy[:, 0]))
print("  std:   ", np.std(rpy[:, 0]))
print("")

print("T_C1C0 - pitch [deg]:")
print("  mean:  ", np.mean(rpy[:, 1]))
print("  median:", np.median(rpy[:, 1]))
print("  std:   ", np.std(rpy[:, 1]))
print("")

print("T_C1C0 - yaw [deg]:")
print("  mean:  ", np.mean(rpy[:, 2]))
print("  median:", np.median(rpy[:, 2]))
print("  std:   ", np.std(rpy[:, 2]))

