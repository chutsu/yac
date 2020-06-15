# YAC - Yet Another Calibrator

Yet Another Calibrator (YAC) is a calibration tool for calibrating RGB
cameras. Specifically:

- Monocular camera intrinsics
- Stereo camera intrinsics and extrinsics
- Mocap marker to camera extrinsics


## Build

    cd <your catkin workspace>
    git clone https://github.com/chutsu/yac
    catkin build -DCMAKE_BUILD_TYPE=Release yac yac_ros
    # Note: If you don't build in release mode the optimization will be very very very slow.


## LICENCE

MIT
