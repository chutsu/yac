#/bin/bash
set -e

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
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_detect2"
# RUN_CMD="rosrun yac test_util_aprilgrid --target test_aprilgrid_detect3"
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
RUN_CMD="rosrun yac test_calib_vi"
# RUN_CMD="rosrun yac test_calib_vi --target test_reproj_error"
# RUN_CMD="rosrun yac test_calib_vi --target test_calib_vi_sim"
# RUN_CMD="rosrun yac test_calib_vi --target test_calib_vi"

# YAC - NBV
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_target_origin"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_init_poses"
# RUN_CMD="rosrun yac test_calib_nbv --target test_calib_nbv_poses"
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

tmux send-keys -t dev -R "\
cd ~/projects/yac \
&& make lib \
&& source ~/catkin_ws/devel/setup.bash \
&& ${RUN_CMD}
" C-m C-m
exit

# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
# source ~/catkin_ws/devel/setup.bash
# --prefix 'gdb -ex=run -ex=\"set confirm off\" -ex=bt -ex=quit -args'
# --prefix 'gdb -ex run'
# --prefix 'gdb -ex run -args'
