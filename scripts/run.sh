#/bin/bash
set -e

# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'

# YAC-UTIL
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_constructor"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_grid_index"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_object_point"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_keypoint"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_keypoints"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_add"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_remove"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_estimate"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_intersect"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_intersect2"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_sample"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_save_and_load"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_print"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_detect"
# RUN_CMD="rosrun yac test_util_config"
# RUN_CMD="rosrun yac test_util_cv"
# RUN_CMD="rosrun yac test_util_data"
# RUN_CMD="rosrun yac test_util_fs"
# RUN_CMD="rosrun yac test_util_net"
# RUN_CMD="rosrun yac test_util_sim"
# RUN_CMD="rosrun yac test_util_timeline"

# YAC - CALIBRATION DATA
# RUN_CMD="rosrun yac test_calib_data"

# YAC - CAMERA CALIBRATION
# RUN_CMD="rosrun yac test_calib_camera"

# YAC - MOCAP CALIBRATION
# RUN_CMD="rosrun yac test_calib_mocap"

# YAC - VISUAL-INERTIAL CALIBRATION
# RUN_CMD="rosrun yac test_calib_vi"
# RUN_CMD="rosrun yac test_calib_vi --target test_fiducial_error"
# RUN_CMD="rosrun yac test_calib_vi --target test_calib_vi"

# YAC - NBV
# RUN_CMD="rosrun yac test_calib_nbv"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_target_origin"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_init_poses"
RUN_CMD="rosrun yac test_calib_nbv --target test_calib_nbv_poses"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_orbit_trajs"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_pan_trajs"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_draw"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_test_grid"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_find"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_find2"
# RUN_CMD="rosrun yac test_calib_nbv --target test_simulate_cameras"
# RUN_CMD="rosrun yac test_calib_nbv --target test_simulate_imu"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbt_eval_traj"

# YAC - MARG ERROR
# RUN_CMD="rosrun yac test_marg_error --target test_marg_block"

# YAC - ROS NODES
# RUN_CMD="roslaunch yac_ros calib_capture.launch cam0_topic:=/rs/ir0/image"
# RUN_CMD="roslaunch yac_ros calib_camera.launch"
# RUN_CMD="roslaunch yac_ros calib_mocap.launch"
# RUN_CMD="roslaunch yac_ros calib_mono_nbv.launch"

tmux send-keys -t dev -R "\
cd ~/projects/yac \
&& make relwithdeb \
&& source ~/catkin_ws/devel/setup.bash \
&& ${RUN_CMD}
" C-m C-m
exit

# python3 scripts/plot_frames.py
# python3 scripts/plot_poses.py
# python3 scripts/sandbox.py

# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
