#/bin/bash
set -e

# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'

# YAC-UTIL
# RUN_CMD="rosrun yac test_util_aprilgrid"
# RUN_CMD="rosrun yac test_util_config"
# RUN_CMD="rosrun yac test_util_cv"
# RUN_CMD="rosrun yac test_util_data"
# RUN_CMD="rosrun yac test_util_fs"
# RUN_CMD="rosrun yac test_util_net"
# RUN_CMD="rosrun yac test_util_sim"
# RUN_CMD="rosrun yac test_util_timeline"

# YAC - CALIBRATION DATA
# RUN_CMD="rosrun yac test_calib_data"

# YAC - ERRORS
# RUN_CMD="rosrun yac test_calib_errors"
# RUN_CMD="rosrun yac test_calib_errors --target test_pose_error"
# RUN_CMD="rosrun yac test_calib_errors --target test_reproj_error"
# RUN_CMD="rosrun yac test_calib_errors --target test_fiducial_error"
# RUN_CMD="rosrun yac test_calib_errors --target test_imu_error"
RUN_CMD="rosrun yac test_calib_errors --target test_marg_error"

# YAC - CAMERA CALIBRATION
# RUN_CMD="rosrun yac test_calib_camera"
# RUN_CMD="rosrun yac test_calib_camera --target test_initialize_camera"
# RUN_CMD="rosrun yac test_calib_camera --target test_calib_camera_find_nbv"
# RUN_CMD="rosrun yac test_calib_camera --target test_calib_camera_mono"
# RUN_CMD="rosrun yac test_calib_camera --target test_calib_camera_stereo"
# RUN_CMD="rosrun yac test_calib_camera --target test_calib_camera_validate"


# YAC - MOCAP CALIBRATION
# RUN_CMD="rosrun yac test_calib_mocap"

# YAC - VISUAL-INERTIAL CALIBRATION
# RUN_CMD="rosrun yac test_calib_vi"
# RUN_CMD="rosrun yac test_calib_vi --target test_calib_vi"

# YAC - NBV
# RUN_CMD="rosrun yac test_calib_nbv"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_target_origin"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_init_poses"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_nbv_poses"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_draw"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_test_grid"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbv_find"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_orbit_trajs"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_pan_trajs"
# RUN_CMD="rosrun yac test_calib_nbv --target test_simulate_cameras"
# RUN_CMD="rosrun yac test_calib_nbv --target test_simulate_imu"
# RUN_CMD="rosrun yac test_calib_nbv --target test_nbt_eval_traj"

# YAC - MARG ERROR
# RUN_CMD="rosrun yac test_marg_error"
# RUN_CMD="rosrun yac test_marg_error --target test_marg_block"

# YAC - ROS NODES
# RUN_CMD="roslaunch yac_ros record_camera.launch"
# RUN_CMD="roslaunch yac_ros record_mocap.launch"
# RUN_CMD="roslaunch yac_ros calib_camera.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/calib_euroc.yaml"
# RUN_CMD="roslaunch yac_ros calib_intel_d435i.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/calib_intel_d435i.yaml"
# RUN_CMD="roslaunch yac_ros calib_mocap.launch"

tmux send-keys -t dev -R "\
cd ~/projects/yac \
&& make relwithdeb \
&& source ~/catkin_ws/devel/setup.bash \
&& ${RUN_CMD}
" C-m C-m
exit

# python3 scripts/plot_frames.py
# python3 scripts/plot_poses.py
# python3 scripts/plot_stats.py
# python3 scripts/sandbox.py

# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
