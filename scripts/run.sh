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

tmux send-keys -t dev -R "\
cd ~/sync/projects/yac \
&& make release \
&& source ~/catkin_ws/devel/setup.bash \
&& roslaunch yac_ros calib_intel_d435i.launch
" C-m
exit
# && rosrun --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args' yac test_calib_vi --target test_calib_vi
# && rosrun yac test_calib_stereo --target test_calib_stereo_solve \
# && rosrun --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args' yac test_marg_error --target test_marg_error_check_gradients
# && rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve \
# && rosrun yac test_calib_stereo --target test_calib_stereo_solve
# && roslaunch yac_ros calib_stereo_nbv.launch
# && roslaunch yac_ros calib_mono_nbv.launch
# && rosrun yac test_calib_nbv --target test_nbv_find
# && rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve


# make
make lib
# make lib_debug
# make release
# make debug
# make tests
source ~/catkin_ws/devel/setup.bash
# rm -rf /tmp/aprilgrid_test
# rosrun yac test_calib_data --target test_blur_measure

# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'

# -- APRILGRID
# rosrun yac test_aprilgrid --target test_aprilgrid_constructor
# rosrun yac test_aprilgrid --target test_aprilgrid_grid_index
# rosrun yac test_aprilgrid --target test_aprilgrid_object_point
# rosrun yac test_aprilgrid --target test_aprilgrid_keypoint
# rosrun yac test_aprilgrid --target test_aprilgrid_keypoints
# rosrun yac test_aprilgrid --target test_aprilgrid_add
# rosrun yac test_aprilgrid --target test_aprilgrid_remove
# rosrun yac test_aprilgrid --target test_aprilgrid_estimate
# rosrun yac test_aprilgrid --target test_aprilgrid_intersect
# rosrun yac test_aprilgrid --target test_aprilgrid_intersect2
# rosrun yac test_aprilgrid --target test_aprilgrid_sample
# rosrun yac test_aprilgrid --target test_aprilgrid_save_and_load
# rosrun yac test_aprilgrid --target test_aprilgrid_print
# rosrun yac test_aprilgrid --target test_aprilgrid_detect
# rosrun yac test_aprilgrid --target test_aprilgrid_detect2
# rosrun yac test_aprilgrid --target test_aprilgrid_detect3

# -- CALIBRATION DATA
# rosrun yac test_calib_data

# -- MOCAP CALIBRATION
# rosrun yac test_calib_mocap

# -- MONOCULAR-CAMERA CALIBRATION
# rosrun yac test_calib_mono
# rosrun yac test_calib_mono --target test_calib_mono_solve
# rosrun yac test_calib_mono --target test_calib_mono_inc_solve

# -- STEREO-CAMERA CALIBRATION
# rosrun yac test_calib_stereo
# rosrun yac test_calib_stereo --target test_reproj_error
# rosrun yac test_calib_stereo --target test_calib_stereo_solve
# rm -rf /data/euroc/calib/cam_april/grid0

# -- STEREO-CAMERA INCREMENTAL CALIBRATION
# rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve

# -- VISUAL-INERTIAL CALIBRATION
# rosrun yac test_calib_vi --target test_reproj_error
# rosrun yac test_calib_vi --target test_calib_vi_sim
# rosrun yac test_calib_vi --target test_calib_vi

# -- NBV
# rosrun yac test_calib_nbv --target test_calib_target_origin
# rosrun yac test_calib_nbv --target test_calib_init_poses
# rosrun yac test_calib_nbv --target test_calib_nbv_poses
# rosrun yac test_calib_nbv --target test_nbv_draw
# rosrun yac test_calib_nbv --target test_nbv_test_grid
# rosrun yac test_calib_nbv --target test_nbv_find
# rosrun yac test_calib_nbv --target test_nbv_find2

# -- MARG ERROR
# rosrun yac test_marg_error --target test_marg_block

# -- ROS NODES
# roslaunch yac_ros calib_capture.launch
# roslaunch yac_ros calib_mono.launch
# roslaunch yac_ros calib_stereo.launch
# roslaunch yac_ros calib_mocap.launch
# roslaunch yac_ros calib_mono_nbv.launch

# gnuplot -p scripts/plots/residuals.gpi
# gnuplot -p scripts/plots/reprojection_errors.gpi

# make
# RUNS=(1 2 3 4 5 6 7 8 9 10)
# for RUN in ${RUNS[@]}; do
#   rosrun yac test_calib_stereo --target test_calib_stereo_inc_solve >> output.txt
#   cp /tmp/calib_stereo.yaml ./calib_stereo-${RUN}.yaml
# done
# python3 scripts/analyze_calibs.py

# octave-cli scripts/octave/plot_calib_poses.m
# octave-cli scripts/octave/plot_biases.m
# octave-cli scripts/octave/plot_calib_vi.m
# octave-cli scripts/octave/plot_calib_vi_sim.m
# python3 scripts/analyze_detections.py
