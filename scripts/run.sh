#/bin/bash
set -e

# Install Docker
# sudo apt install docker.io -yyy -q
# sudo systemctl start docker
# sudo systemctl enable docker  # Now reboot computer

# Add user to docker group for permissions
# sudo groupadd docker
# sudo usermod -aG docker ${USER}
# su -s ${USER}

# Build docker image
# docker image build -t yac_docker .

# Run docker image
# Note: -v /data:/data means allow docker instance to have access to /data dir
# docker run \
#   -e DISPLAY=$DISPLAY \
#   -v /tmp/.X11-unix:/tmp/.X11-unix \
#   -v /data:/data \ 
#   --network="host" \
#   -it --rm yac_docker


# Build YAC
# cd yac
# mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4

# cd ~/catkin_ws
# catkin build -DCMAKE_BUILD_TYPE=Debug yac -j2
# catkin build -DCMAKE_BUILD_TYPE=Release yac -j2
# catkin build -DCMAKE_BUILD_TYPE=Release yac yac_ros -j2

make
# make debug
# make tests
source ~/catkin_ws/devel/setup.bash
# rm -rf /tmp/aprilgrid_test

# python3 scripts/analyze_detections.py

# --prefix 'gdb -ex run'
# rosrun yac test_aprilgrid --target test_aprilgrid_constructor
# rosrun yac test_aprilgrid --target test_aprilgrid_grid_index
# rosrun yac test_aprilgrid --target test_aprilgrid_object_point
# rosrun yac test_aprilgrid --target test_aprilgrid_keypoint
# rosrun yac test_aprilgrid --target test_aprilgrid_keypoints
# rosrun yac test_aprilgrid --target test_aprilgrid_add
# rosrun yac test_aprilgrid --target test_aprilgrid_remove
# rosrun yac test_aprilgrid --target test_aprilgrid_estimate
# rosrun yac test_aprilgrid --target test_aprilgrid_save_and_load
# rosrun yac test_aprilgrid --target test_aprilgrid_print
# rosrun yac test_aprilgrid --target test_aprilgrid_detect
# rosrun yac test_aprilgrid --target test_aprilgrid_detect2
# rosrun yac test_calib_data
# rosrun yac test_calib_mocap
# rosrun yac test_calib_mono
# rosrun yac test_calib_stereo
# rosrun yac test_calib_vi
# rosrun yac test_calib_vi

# roslaunch yac_ros calib_capture.launch
# roslaunch yac_ros calib_mono.launch
# roslaunch yac_ros calib_stereo.launch
# roslaunch yac_ros calib_mocap.launch
# rosrun yac_ros calib_mono

# Build tests
# cd tests
# ./test_aprilgrid --target
# ./test_calib_data
# ./test_calib_mono
# rosrun yac test_calib_stereo
