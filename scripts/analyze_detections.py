#!/usr/bin/env python3
import os
from os import listdir
from os.path import join
from os.path import basename

import numpy as np
import matplotlib.pylab as plt


grids_dir = "/data/grid_detection"

# Load data
yac_cam0_grids_dir = join(grids_dir, "yac_cam0_grids")
yac_cam1_grids_dir = join(grids_dir, "yac_cam1_grids")
kalibr_cam0_grids_dir = join(grids_dir, "kalibr_cam0_grids")
kalibr_cam1_grids_dir = join(grids_dir, "kalibr_cam1_grids")

yac_cam0_grid_paths = [join(yac_cam0_grids_dir, x) for x in listdir(yac_cam0_grids_dir)]
yac_cam1_grid_paths = [join(yac_cam1_grids_dir, x) for x in listdir(yac_cam1_grids_dir)]
yac_cam0_grid_paths.sort()
yac_cam1_grid_paths.sort()

kalibr_cam0_grid_paths = [join(kalibr_cam0_grids_dir, x) for x in listdir(kalibr_cam0_grids_dir)]
kalibr_cam1_grid_paths = [join(kalibr_cam1_grids_dir, x) for x in listdir(kalibr_cam1_grids_dir)]
kalibr_cam0_grid_paths.sort()
kalibr_cam1_grid_paths.sort()

yac_cam0_ts = [int(basename(x.replace(".csv", ""))) for x in yac_cam0_grid_paths]
yac_cam1_ts = [int(basename(x.replace(".csv", ""))) for x in yac_cam1_grid_paths]
kalibr_cam0_ts = [int(basename(x.replace(".csv", ""))) for x in kalibr_cam0_grid_paths]
kalibr_cam1_ts = [int(basename(x.replace(".csv", ""))) for x in kalibr_cam1_grid_paths]

# find common timestamps
common_ts = list(set(yac_cam0_ts).intersection(set(kalibr_cam0_ts)))
common_ts.sort()

yac_cam0_grids = {}
for i in range(len(yac_cam0_ts)):
    ts = yac_cam0_ts[i]
    grid_path = yac_cam0_grid_paths[i]

    if ts in common_ts:
        grid_data = np.loadtxt(open(grid_path, "rb"), delimiter=",", skiprows=1)
        keypoints = grid_data[:, 0:2]
        object_points = np.block([grid_data[:, 2:4], np.zeros((grid_data.shape[0], 1))])
        yac_cam0_grids[ts] = { "keypoints": keypoints, "object_points": object_points }

# yac_cam1_grids = {}
# for i in range(len(yac_cam1_ts)):
#     ts = yac_cam1_ts[i]
#     grid_path = yac_cam1_grid_paths[i]
#
#     if ts in common_ts:
#         grid_data = np.loadtxt(open(grid_path, "rb"), delimiter=",", skiprows=1)
#         keypoints = grid_data[:, 0:2]
#         object_points = np.block([grid_data[:, 2:4], np.zeros((grid_data.shape[0], 1))])
#         yac_cam1_grids[ts] = { "keypoints": keypoints, "object_points": object_points }

kalibr_cam0_grids = {}
for i in range(len(kalibr_cam0_ts)):
    ts = kalibr_cam0_ts[i]
    grid_path = kalibr_cam0_grid_paths[i]

    if ts in common_ts:
        grid_data = np.loadtxt(open(grid_path, "rb"), delimiter=",", skiprows=1)
        keypoints = grid_data[:, 0:2]
        object_points = np.block([grid_data[:, 2:4], np.zeros((grid_data.shape[0], 1))])
        kalibr_cam0_grids[ts] = { "keypoints": keypoints, "object_points": object_points }

# kalibr_cam1_grids = {}
# for i in range(len(kalibr_cam1_ts)):
#     ts = kalibr_cam1_ts[i]
#     grid_path = kalibr_cam1_grid_paths[i]
#
#     if ts in common_ts:
#         grid_data = np.loadtxt(open(grid_path, "rb"), delimiter=",", skiprows=1)
#         keypoints = grid_data[:, 0:2]
#         object_points = np.block([grid_data[:, 2:4], np.zeros((grid_data.shape[0], 1))])
#         kalibr_cam1_grids[ts] = { "keypoints": keypoints, "object_points": object_points }


nb_frames = 0
nb_frames_equal_keypoints = 0
nb_kalibr_keypoints = 0
nb_yac_keypoints = 0

kalibr_cam0_keypoints = None
yac_cam0_keypoints = None

kalibr_cam0_keypoints_frames = []
yac_cam0_keypoints_frames = []

for ts in common_ts:
    nb_frames += 1
    kalibr_grid = kalibr_cam0_grids[ts]
    yac_grid = yac_cam0_grids[ts]

    kalibr_keypoints = kalibr_grid["keypoints"]
    yac_keypoints = yac_grid["keypoints"]
    kalibr_keypoints = kalibr_keypoints[np.argsort(kalibr_keypoints[:, 0])]
    yac_keypoints = yac_keypoints[np.argsort(yac_keypoints[:, 0])]

    kalibr_cam0_keypoints_frames.append(len(kalibr_keypoints))
    yac_cam0_keypoints_frames.append(len(yac_keypoints))

    if kalibr_cam0_keypoints is None:
        kalibr_cam0_keypoints = kalibr_keypoints
        yac_cam0_keypoints = yac_keypoints
    else:
        kalibr_cam0_keypoints = np.append(kalibr_cam0_keypoints, kalibr_keypoints, axis=0)
        yac_cam0_keypoints = np.append(yac_cam0_keypoints, yac_keypoints, axis=0)

    # print("kalibr nb_keypoints: %d" % len(kalibr_grid["keypoints"]))
    # print("yac nb_keypoints: %d" % len(yac_grid["keypoints"]))
    nb_kalibr_keypoints += len(kalibr_keypoints)
    nb_yac_keypoints += len(yac_keypoints)

    # print("kalibr keypoints: %d" % len(kalibr_keypoints))
    # print("yac keypoints: %d" % len(yac_keypoints))
    # exit(0)

    if len(kalibr_keypoints) == len(yac_keypoints):
        nb_frames_equal_keypoints += 1
        diff = kalibr_keypoints - yac_keypoints
        if np.count_nonzero(diff) > 0:
            print(diff)
            fig, (ax0, ax1) = plt.subplots(2, 1)

            ax0.plot(yac_keypoints[:, 0], yac_keypoints[:, 1], 'ro')
            ax0.set_xlim([0, 752])
            ax0.set_ylim([0, 480])
            ax0.invert_yaxis()
            ax0.xaxis.set_label_position('top')
            ax0.xaxis.tick_top()
            ax0.set_xticks(np.arange(0, 752, step=100))
            ax0.set_xlabel("")
            ax0.set_title("YAC\n", fontweight="bold")

            ax1.plot(kalibr_keypoints[:, 0], kalibr_grid["keypoints"][:, 1], 'ro')
            ax1.set_xlim([0, 752])
            ax1.set_ylim([0, 480])
            ax1.invert_yaxis()
            ax1.xaxis.set_label_position('top')
            ax1.xaxis.tick_top()
            ax1.set_xticks(np.arange(0, 752, step=100))
            ax1.set_xlabel("")
            ax1.set_title("Kalibr\n", fontweight="bold")

            plt.show()

    # if (len(kalibr_grid["keypoints"]) != len(yac_grid["keypoints"])):
    #     fig, (ax0, ax1) = plt.subplots(2, 1)
    #
    #     ax0.plot(yac_grid["keypoints"][:, 0], yac_grid["keypoints"][:, 1], 'ro')
    #     ax0.set_xlim([0, 752])
    #     ax0.set_ylim([0, 480])
    #     ax0.invert_yaxis()
    #     ax0.xaxis.set_label_position('top')
    #     ax0.xaxis.tick_top()
    #     ax0.set_xticks(np.arange(0, 752, step=100))
    #     ax0.set_xlabel("")
    #     ax0.set_title("YAC\n", fontweight="bold")
    #
    #     ax1.plot(kalibr_grid["keypoints"][:, 0], kalibr_grid["keypoints"][:, 1], 'ro')
    #     ax1.set_xlim([0, 752])
    #     ax1.set_ylim([0, 480])
    #     ax1.invert_yaxis()
    #     ax1.xaxis.set_label_position('top')
    #     ax1.xaxis.tick_top()
    #     ax1.set_xticks(np.arange(0, 752, step=100))
    #     ax1.set_xlabel("")
    #     ax1.set_title("Kalibr\n", fontweight="bold")
    #
    #     plt.show()

    # kalibr_keypoints = kalibr_grid["keypoints"]
    # yac_keypoints = yac_grid["keypoints"]
    # nb_keypoints = kalibr_keypoints.shape[0]
    #
    # kalibr_keypoints = kalibr_keypoints[np.argsort(kalibr_keypoints[:, 0])]
    # yac_keypoints = yac_keypoints[np.argsort(yac_keypoints[:, 0])]

first_ts = common_ts[0]
time = (np.array(common_ts) - first_ts) * 1e-9
plt.plot(time, kalibr_cam0_keypoints_frames, 'r-', label="kalibr")
plt.plot(time, yac_cam0_keypoints_frames, 'b-', label="custom")
plt.legend(loc=0)
plt.show()

print("nb_kalibr_keypoints: %d" % nb_kalibr_keypoints)
print("nb_yac_keypoints: %d" % nb_yac_keypoints)
print("diff: %d" % (nb_kalibr_keypoints - nb_yac_keypoints))

print("nb_frames: %d" % nb_frames)
print("nb_frames_equal_keypoints: %d" % nb_frames_equal_keypoints)

# plt.figure(1)
# plt.subplot(211)
# plt.hist(kalibr_cam0_keypoints[:, 0])
# plt.subplot(212)
# plt.hist(yac_cam0_keypoints[:, 0])
# plt.figure(2)
# plt.subplot(211)
# plt.hist(kalibr_cam0_keypoints[:, 1])
# plt.subplot(212)
# plt.hist(yac_cam0_keypoints[:, 1])
# plt.show()


# plt.hist(

    # for i in range(min(len(kalibr_keypoints), len(yac_keypoints))):
    #     dx = kalibr_keypoints[i][0] - yac_keypoints[i][0]
    #     dy = kalibr_keypoints[i][1] - yac_keypoints[i][1]
    #     if dx or dy:
    #         print("kalibr: ", kalibr_keypoints[i])
    #         print("yac: ", yac_keypoints[i])

    # exit(0)

    # fig, (ax0, ax1) = plt.subplots(2, 1)

    # ax0.plot(yac_grid["keypoints"][:, 0], yac_grid["keypoints"][:, 1], 'ro')
    # ax0.set_xlim([0, 752])
    # ax0.set_ylim([0, 480])
    # ax0.set_xticks(np.arange(0, 752, step=100))
    # ax0.invert_yaxis()
    # ax0.xaxis.tick_top()
    #
    # ax1.plot(kalibr_grid["keypoints"][:, 0], kalibr_grid["keypoints"][:, 1], 'ro')
    # ax1.set_xlim([0, 752])
    # ax1.set_ylim([0, 480])
    # ax1.set_xticks(np.arange(0, 752, step=100))
    # ax1.invert_yaxis()
    # ax1.xaxis.tick_top()
    #
    # plt.show()


# print(kalibr_grid)



# print(yac_grids0[0])
# print(kalibr_grids0[0])

