#!/bin/bash
set -e

# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'

# YAC-UTIL
# RUN_CMD="./test_util_aprilgrid"
# RUN_CMD="./test_util_aprilgrid --target test_aprilgrid_profile"
# RUN_CMD="./test_util_config"
# RUN_CMD="./test_util_cv"
# RUN_CMD="./test_util_cv --target test_pinhole_radtan4_project"
# RUN_CMD="./test_util_cv --target test_pinhole_radtan4_project_jacobian"
# RUN_CMD="./test_util_cv --target test_pinhole_radtan4_undistort"
# RUN_CMD="./test_util_core"
# RUN_CMD="./test_util_data"
# RUN_CMD="./test_util_fs"
# RUN_CMD="./test_util_net"
# RUN_CMD="./test_util_sim"
# RUN_CMD="./test_util_timeline"

# YAC - CALIBRATION DATA
# RUN_CMD="./test_calib_data"

# YAC - RESIDUALS
# RUN_CMD="./test_calib_residuals"
# RUN_CMD="./test_calib_residuals --target test_prior"
# RUN_CMD="./test_calib_residuals --target test_pose_prior"
# RUN_CMD="./test_calib_residuals --target test_reproj_residual"
# RUN_CMD="./test_calib_residuals --target test_fiducial_residual"
# RUN_CMD="./test_calib_residuals --target test_imu_residual"

# YAC - CAMERA CALIBRATION
# RUN_CMD="./test_calib_camera"
# RUN_CMD="./test_calib_camera --target test_calib_view"
# RUN_CMD="./test_calib_camera --target test_calib_camera_add_camera_data"
# RUN_CMD="./test_calib_camera --target test_calib_camera_add_camera"
# RUN_CMD="./test_calib_camera --target test_calib_camera_add_pose"
# RUN_CMD="./test_calib_camera --target test_calib_camera_add_and_remove_view"
# RUN_CMD="./test_calib_camera --target test_calib_camera_add_nbv_view"
# RUN_CMD="./test_calib_camera --target test_calib_camera_find_nbv"
# RUN_CMD="./test_calib_camera --target test_calib_camera_filter_all_views"
# RUN_CMD="./test_calib_camera --target test_calib_camera_remove_all_views"
# RUN_CMD="./test_calib_camera --target test_calib_camera_remove_outliers"
# RUN_CMD="./test_calib_camera --target test_calib_camera_solve_batch"
# RUN_CMD="./test_calib_camera --target test_calib_camera_solve_inc"
# RUN_CMD="./test_calib_camera --target test_calib_camera_mono"
# RUN_CMD="./test_calib_camera --target test_calib_camera_stereo"
# RUN_CMD="./test_calib_camera --target test_marg_residual"
RUN_CMD="./calib_euroc"
# RUN_CMD="./calib_inspect '/tmp/calib-results.yaml' /data/euroc/cam_april"
# RUN_CMD="./calib_inspect '/home/chutsu/projects/yac/yac/configs/calib-kalibr.yaml' /data/euroc/imu_april"
# RUN_CMD="./calib_inspect '/home/chutsu/projects/yac/yac/configs/calib-yac.yaml' /data/euroc/cam_april"
# RUN_CMD="./calib_inspect '/data/euroc_results/configs/euroc/euroc.yaml' /data/euroc/cam_april"
# RUN_CMD="./calib_sandbox '/tmp/yac_data/calib.yaml'"


# YAC - MOCAP CALIBRATION
# RUN_CMD="test_calib_mocap"

# YAC - VISUAL-INERTIAL CALIBRATION
# RUN_CMD="./test_calib_vi"
# RUN_CMD="./test_calib_vi --target test_calib_vi"
# RUN_CMD="./test_calib_vi --target test_calib_vi_batch"
# RUN_CMD="./test_calib_vi --target test_calib_vi_online"
# RUN_CMD="./test_calib_vi --target test_calib_vi_copy_constructor"

# YAC - NBT
# RUN_CMD="./test_calib_nbt"
# RUN_CMD="./test_calib_nbt --target test_nbt_orbit_trajs"
# RUN_CMD="./test_calib_nbt --target test_nbt_pan_trajs"
# RUN_CMD="./test_calib_nbt --target test_nbt_figure8_trajs"
# RUN_CMD="./test_calib_nbt --target test_simulate_cameras"
# RUN_CMD="./test_calib_nbt --target test_simulate_imu"
# RUN_CMD="./test_calib_nbt --target test_nbt_eval"
# RUN_CMD="./test_calib_nbt --target test_nbt_find"

# YAC - NBV
# RUN_CMD="./test_calib_nbv"
# RUN_CMD="./test_calib_nbv --target test_calib_target_origin"
# RUN_CMD="./test_calib_nbv --target test_calib_init_poses"
# RUN_CMD="./test_calib_nbv --target test_calib_nbv_poses"
# RUN_CMD="./test_calib_nbv --target test_nbv_draw"
# RUN_CMD="./test_calib_nbv --target test_nbv_test_grid"
# RUN_CMD="./test_calib_nbv --target test_nbv_find"
# RUN_CMD="./test_calib_nbv --target test_calib_orbit_trajs"
# RUN_CMD="./test_calib_nbv --target test_calib_pan_trajs"
# RUN_CMD="./test_calib_nbv --target test_simulate_cameras"
# RUN_CMD="./test_calib_nbv --target test_simulate_imu"
# RUN_CMD="./test_calib_nbv --target test_nbt_eval_traj"

# YAC - MARG RESIDUAL
# RUN_CMD="./test_marg_residual"
# RUN_CMD="./test_marg_residual --target test_marg_block"

# YAC - ROS NODES
# RUN_CMD="roslaunch yac_ros record_camera.launch"
# RUN_CMD="roslaunch yac_ros record_mocap.launch"
# RUN_CMD="roslaunch yac_ros calib_mocap.launch"

# RUN_CMD="roslaunch yac_ros calib_camera.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/euroc-calib_cameras.yaml"

# RUN_CMD="roslaunch yac_ros calib_imucam.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/euroc-calib_imucam.yaml"

# RUN_CMD="roslaunch yac_ros intel_d435i-calib_camera.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/intel_d435i-calib_camera.yaml"

# RUN_CMD="roslaunch yac_ros intel_d435i-calib_imucam.launch \
#   config_file:=/home/chutsu/projects/yac/yac_ros/config/intel_d435i-calib_imucam.yaml"

# RUN_CMD="roslaunch yac_ros intel_d435i-calib_imucam.launch"

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
cd ~/projects/yac \
&& sudo make lib_relwithdeb \
&& cd build && ${RUN_CMD}
" C-m C-m
exit

# python3 scripts/aprilgrid_generate.py --nx 6 --ny 6 --tsize 0.088
# python3 scripts/marg_sandbox.py
python3 scripts/blake_zisserman.py
# python3 scripts/plot_frames.py
# python3 scripts/plot_imu.py
# python3 scripts/plot_info.py
# python3 scripts/plot_nbt.py
# python3 scripts/plot_poses.py /tmp/calib-estimates.yaml
# python3 scripts/plot_stats.py
# python3 scripts/plot_xyz.py /tmp/calib-estimates.csv "#ts" rx ry rz
# python3 scripts/sandbox.py

# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
