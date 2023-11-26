YAC - Yet Another Calibrator
============================

<a href="https://github.com/chutsu/yac/actions?query=ci">
  <img src="https://github.com/chutsu/yac/actions/workflows/ccpp.yml/badge.svg">
</a>

Yet Another Calibrator (YAC) is a calibration tool for calibrating RGB
cameras. Specifically:

- Camera intrinsics and extrinsics
- Camera-IMU extrinsic
- Mocap-marker to camera extrinsics

Supported projection-distortion models:

- pinhole-radtan4
- pinhole-equi4


Dependencies
-----

    # Ubuntu packages
    libyaml-cpp-dev
    libeigen3-dev
    libceres-dev
    libopencv-dev
    libomp-dev (optional)

    # Custom (see deps dir)
    Modified apriltags (https://github.com/chutsu/apriltags)


Build
-----

For the lazy :

    git clone https://github.com/chutsu/yac
    cd yac
    make deps
    make lib


Or standard approach:

    git clone https://github.com/chutsu/yac
    cd yac
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make


Calibration Target
------------------

`yac` uses a grid of AprilTags, a.k.a an AprilGrid, as a calibration target.

<p align="center">
  <img src="docs/aprilgrid.png" alt="AprilGrid" width="40%"/>
</p>

Click [here](docs/aprilgrid_A0.pdf) to download and print the AprilGrid.
During data collection make sure the calibration target is as flat as possible.


Calibrate Camera Intrinsics and Extrinsics
------------------------------------------

First inorder to calibrate a monocular camera or stereo camera pair we need to
create the calibration configuration file. For example `calib_intel_d435i.yaml`
configuration file:

```
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
  resolution: [640.0, 480.0]

cam1:
  proj_model: "pinhole"
  dist_model: "radtan4"
  resolution: [640.0, 480.0]
```


Calibrate Camera(s)
~~~~~~~~~~~~~~~~~~~

```
calib_camera <path to config file> <path to data>
```


Calibrate Camera-IMU
~~~~~~~~~~~~~~~~~~~~

```
calib_vi <path to config file> <path to data>
```


LICENCE
-------

MIT
