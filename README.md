YAC - Yet Another Calibrator
============================

<a href="https://github.com/chutsu/yac/actions?query=ci">
  <img src="https://github.com/chutsu/yac/workflows/ci/badge.svg">
</a>

Yet Another Calibrator (YAC) is a calibration tool for calibrating RGB
cameras. Specifically:

- Camera intrinsics
- Stereo camera intrinsics and extrinsics

Supported projection-distortion models:

- pinhole-radtan4
- pinhole-equi4


Build
-----

    cd <your catkin workspace>
    git clone https://github.com/chutsu/yac
    catkin build -DCMAKE_BUILD_TYPE=Release yac yac_ros
    # Note: If you don't build in release mode the optimization will be very very very slow.


Calibration Target
------------------

`yac` uses a grid of AprilTags, a.k.a an AprilGrid, as a calibration target.

<p align="center">
<img src="docs/aprilgrid.png" alt="AprilGrid" width="40%"/>
</p>

Click [here](docs/aprilgrid_A0.pdf) to download and print the AprilGrid.
During calibration make sure the calibration target is as flat as possible.


Calibrate Camera Intrinsics or Stereo Intrinsics and Extrinsics
---------------------------------------------------------------

First inorder to calibrate a monocular camera or stereo camera pair we need to
create the calibration configuration file. For example `calib_intel_d435i.yaml`
configuration file:

```
ros:
  bag: "/data/intel_d435i/imucam-1.bag"
  cam0_topic: "/stereo/camera0/image"

calib_target:
  target_type: 'aprilgrid'  # Target type
  tag_rows: 6               # Number of rows
  tag_cols: 6               # Number of cols
  tag_size: 0.088           # Size of apriltag, edge to edge [m]
  tag_spacing: 0.3          # Ratio of space between tags to tagSize
                            # Example: tag_size=2m, spacing=0.5m
                            #          --> tag_spacing=0.25[-]

cam0:
  proj_model: "pinhole"
  dist_model: "radtan4"
  lens_hfov: 69.4
  lens_vfov: 42.5
  resolution: [640.0, 480.0]
  rate: 30.0

cam1:
  proj_model: "pinhole"
  dist_model: "radtan4"
  lens_hfov: 69.4
  lens_vfov: 42.5
  resolution: [640.0, 480.0]
  rate: 30.0
```

The above tells `yac` where to find the rosbag containing the calibration data,
the calibration target and camera details. Now, we need to pass that to a yac
calibrator via the roslaunch file `calib_mono.launch` or `calib_stereo.launch`:

```
# To calibrate cam0 intrinsics only
roslaunch yac calib_mono.launch config_file:=<path to calib_intel_d435i.yaml>

# To calibrate stereo camera pair intrinsics and extrinsics
roslaunch yac calib_stereo.launch config_file:=<path to calib_intel_d435i.yaml>
```


LICENCE
-------

MIT
