#/bin/bash
set -e

tmux send-keys -t dev -R "\
cd ~/projects/yac \
&& make lib \
&& source ~/catkin_ws/devel/setup.bash \
&& rosrun yac test_calib_camera
" C-m C-m
exit

# && rosrun yac test_calib_data
# && rosrun yac test_util_cv
# && rosrun yac test_util_aprilgrid
# && rosrun yac test_calib_nbv --target test_calib_orbit_trajs
# && cd scripts/octave; octave-cli plot_nbt_trajs.m

# && rosrun yac test_calib_vi --target test_calib_vi_init
# && rosrun yac test_calib_vi --target test_calib_vi
# && rosrun yac test_marg_error --target test_marg_error_swe
# && rosrun yac test_marg_error --target test_marg_error_swe
# && rosrun yac test_calib_vi --target test_reproj_error_td
# && cd scripts/octave && octave-cli plot_nbt_sim_imu.m
# && rosrun yac test_calib_nbv --target test_calib_orbit_trajs
# && roslaunch yac_ros calib_stereo_nbv.launch
# && roslaunch yac_ros calib_stereo_nbv.launch
# && rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve
# && rosrun yac test_calib_stereo --target test_calib_stereo_solve
# && rosrun yac test_calib_stereo --target test_calib_stereo_solve \
# && rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve \
# && rosrun yac test_calib_stereo --target test_calib_stereo_solve
# && roslaunch yac_ros calib_stereo_nbv.launch
# && roslaunch yac_ros calib_mono_nbv.launch
# && rosrun yac test_calib_stereo_inc --target test_calib_stereo_inc_solve


# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
source ~/catkin_ws/devel/setup.bash

# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'

# UTIL
# rosrun yac test_util_aprilgrid --target test_aprilgrid_constructor
# rosrun yac test_util_aprilgrid --target test_aprilgrid_grid_index
# rosrun yac test_util_aprilgrid --target test_aprilgrid_object_point
# rosrun yac test_util_aprilgrid --target test_aprilgrid_keypoint
# rosrun yac test_util_aprilgrid --target test_aprilgrid_keypoints
# rosrun yac test_util_aprilgrid --target test_aprilgrid_add
# rosrun yac test_util_aprilgrid --target test_aprilgrid_remove
# rosrun yac test_util_aprilgrid --target test_aprilgrid_estimate
# rosrun yac test_util_aprilgrid --target test_aprilgrid_intersect
# rosrun yac test_util_aprilgrid --target test_aprilgrid_intersect2
# rosrun yac test_util_aprilgrid --target test_aprilgrid_sample
# rosrun yac test_util_aprilgrid --target test_aprilgrid_save_and_load
# rosrun yac test_util_aprilgrid --target test_aprilgrid_print
# rosrun yac test_util_aprilgrid --target test_aprilgrid_detect
# rosrun yac test_util_aprilgrid --target test_aprilgrid_detect2
# rosrun yac test_util_aprilgrid --target test_aprilgrid_detect3
# rosrun yac test_util_config
# rosrun yac test_util_cv
# rosrun yac test_util_data
# rosrun yac test_util_fs
# rosrun yac test_util_net
# rosrun yac test_util_sim
# rosrun yac test_util_timeline

# -- CALIBRATION DATA
# rosrun yac test_calib_data

# -- MOCAP CALIBRATION
# rosrun yac test_calib_mocap

# -- VISUAL-INERTIAL CALIBRATION
# rosrun yac test_calib_vi --target test_calib_vi_init_poses
# rosrun yac test_calib_vi --target test_reproj_error
# rosrun yac test_calib_vi --target test_calib_vi_sim
# rosrun yac test_calib_vi --target test_calib_vi

# -- NBV
# rosrun yac test_calib_nbv --target test_calib_target_origin
# rosrun yac test_calib_nbv --target test_calib_init_poses
# rosrun yac test_calib_nbv --target test_calib_nbv_poses
# rosrun yac test_calib_nbv --target test_calib_orbit_trajs
# rosrun yac test_calib_nbv --target test_calib_pan_trajs
# rosrun yac test_calib_nbv --target test_nbv_draw
# rosrun yac test_calib_nbv --target test_nbv_test_grid
# rosrun yac test_calib_nbv --target test_nbv_find
# rosrun yac test_calib_nbv --target test_nbv_find2
# rosrun yac test_calib_nbv --target test_simulate_cameras
# rosrun yac test_calib_nbv --target test_simulate_imu
# rosrun yac test_calib_nbv --target test_nbt_eval_traj

# -- MARG ERROR
# rosrun yac test_marg_error --target test_marg_block

# -- ROS NODES
# roslaunch yac_ros calib_capture.launch \
#   cam0_topic:=/rs/ir0/image

# roslaunch yac_ros calib_camera.launch
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

# cd scripts/octave
# octave-cli nbt.m
# # octave-cli plot_calib_poses.m
# octave-cli plot_biases.m
# octave-cli plot_calib_vi.m
# octave-cli plot_calib_vi_sim.m
# octave-cli plot_calib_vi_init_poses.m
# octave-cli plot_nbt_sim_imu.m
# octave-cli plot_nbt_trajs.m
# python3 scripts/analyze_detections.py
