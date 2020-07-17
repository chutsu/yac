YAC - Yet Another Calibrator
============================

<a href="https://github.com/chutsu/yac/actions?query=ci">
  <img src="https://github.com/chutsu/yac/workflows/ci/badge.svg">
</a>

Yet Another Calibrator (YAC) is a calibration tool for calibrating RGB
cameras. Specifically:

- [Calibrate monocular camera intrinsics](#calibrate-monocular-camera-intrinsics)
- [Calibrate stereo camera intrinsics and extrinsics](#calibrate-stereo-camera-intrinsics-and-extrinsics)
- [Calibrate mocap marker to camera extrinsics](#calibrate-mocap-marker-to-camera-extrinsics)


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



Calibrate Monocular Camera Intrinsics
-------------------------------------

First inorder to calibrate a monocular camera we need to create the calibration
configuration file.  Example `calib_intel_d435i.yaml` configuration file:

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
```

The above tells `yac` where to find the rosbag containing the calibration data,
the calibration target and camera details. Now, we need to create a launch
file `calib_mono.launch`:

```
<launch>
  <node pkg="yac_ros" type="calib_mono_node" name="calib_mono_node"
        required="true" output="screen">
    <param name="config_file"
           value="$(find yac_ros)/config/calib_intel_d435i.yaml" />
  </node>
</launch>
```

where we call the `calib_mono_node` and supply the calibration configuration
file `calib_intel_d435i.yaml`. To run the calibration issue the following
commands in the terminal:

```
source <your catkin workspace>/devel/setup.bash
roslaunch yac_ros calib_mono.launch
```



Calibrate Stereo Camera Intrinsics and Extrinsics
-------------------------------------------------

First inorder to calibrate a stereo camera we need to create the calibration
configuration file.  Example `calib_intel_d435i.yaml` configuration file:

    ros:
      bag: "/data/intel_d435i/imucam-1.bag"
      cam0_topic: "/stereo/camera0/image"
      cam1_topic: "/stereo/camera1/image"

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

The above tells `yac` where to find the rosbag containing the calibration data,
the calibration target and camera details. Now, we need to create a launch
file `calib_stereo.launch`:

```
<launch>
  <node pkg="yac_ros" type="calib_stereo_node" name="calib_stereo_node"
        required="true" output="screen">
    <param name="config_file"
           value="$(find yac_ros)/config/calib_intel_d435i.yaml" />
  </node>
</launch>
```

where we call the `calib_stereo_node` and supply the calibration configuration
file `calib_intel_d435i.yaml`. To run the calibration issue the following
commands in the terminal:

```
source <your catkin workspace>/devel/setup.bash
roslaunch yac_ros calib_stereo.launch
```



Calibrate Mocap Marker to Camera Extrinsics
-------------------------------------------

Example `calib_mocap_camera.yaml` configuration file:

    ros:
      train_bag: "/data/aabm/200115-vicon-train.bag"
      test_bag: "/data/aabm/200115-vicon-test.bag"
      cam0_topic: "/cam0/image"
      body0_topic: "/body0/pose"
      target0_topic: "/target0/pose"

    calib_target:
      target_type: 'aprilgrid'  # Target type
      tag_rows: 6               # Number of rows
      tag_cols: 6               # Number of cols
      tag_size: 0.088           # Size of apriltag, edge to edge [m]
      tag_spacing: 0.3          # Ratio of space between tags to tagSize
                                # Example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]

    cam0:
      resolution: [640.0, 480.0]
      proj_model: "pinhole"
      dist_model: "radtan4"
      lens_hfov: 69.4
      lens_vfov: 42.5
      rate: 30.0


LICENCE
-------

MIT
