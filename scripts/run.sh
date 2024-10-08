#!/bin/bash
set -e

# python3 scripts/aprilgrid_generate.py --nx 6 --ny 6 --tsize 0.088 && exit
# python3 scripts/marg_sandbox.py && exit
# python3 scripts/blake_zisserman.py && exit
# python3 scripts/plot_frames.py && exit
# python3 scripts/plot_imu.py && exit
# python3 scripts/plot_camera.py && exit
# python3 scripts/plot_info.py && exit
# python3 scripts/plot_nbt.py && exit
# python3 scripts/plot_poses.py /tmp/calib-estimates.yaml && exit
# python3 scripts/plot_poses.py /tmp/poses_est.csv && exit
# python3 scripts/plot_poses.py /tmp/poses_gnd.csv && exit
# python3 scripts/plot_poses.py /tmp/calib_poses.csv && exit
# python3 scripts/plot_stats.py && exit
# python3 scripts/plot_xyz.py /tmp/vel.csv "#ts" vx vy vz && exit
# python3 scripts/plot_xyz.py /tmp/acc.csv "#ts" ax ay az && exit
# python3 scripts/plot_xyz.py /tmp/gyr.csv "#ts" wx wy wz && exit
# python3 scripts/calib_gimbal.py
# python3 scripts/sandbox.py && exit
# python3 scripts/lissajous.py && exit
# python3 scripts/lissajous.py > /tmp/out.txt && exit
# dot -Txdot scripts/pipeline.dot | dot2tex -s -c --autosize > /tmp/test.tex \
#   && pdflatex /tmp/test.tex \
#   && exit

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
# RUN_CMD="./test_calib_residuals --target test_eval"
# RUN_CMD="./test_calib_residuals --target test_prior"
# RUN_CMD="./test_calib_residuals --target test_pose_prior"
# RUN_CMD="./test_calib_residuals --target test_reproj_residual"
# RUN_CMD="./test_calib_residuals --target test_fiducial_residual"
# RUN_CMD="./test_calib_residuals --target test_mocap_residual"
# RUN_CMD="./test_calib_residuals --target test_imu_residual"
# RUN_CMD="./test_calib_residuals --target test_marg_residual"

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
# RUN_CMD="./calib_euroc"
# RUN_CMD="./calib_mocap /data/calibration_s550_jetson_20220718/"
# RUN_CMD="./calib_info 'camera' /tmp/calib_camera-results.yaml /data/euroc/cam_april/mav0 /tmp/calib_info-camera.csv"
# RUN_CMD="./calib_info 'camera-imu' /tmp/calib_imu-results.yaml /data/euroc/imu_april/mav0 /tmp/calib_info-camera_imu.csv"
# RUN_CMD="./calib_preprocess 6 6 0.088 0.3 /data/euroc/cam_april/mav0/cam0/data /tmp/grid0/cam0"

# RUN_CMD="./calib_inspect 'camera' '/tmp/calib_camera-results.yaml' /data/euroc/cam_april"
# RUN_CMD="./calib_inspect 'camera' '/data/euroc_results/configs/yac/calib_camera-results.yaml' /data/euroc/cam_april"
# RUN_CMD="./calib_inspect 'camera' '/data/yac_experiments/euroc_results/configs/kalibr/imu_april-camchain-yac.yaml' /data/euroc/cam_april"

# RUN_CMD="./calib_inspect 'camera-imu' '/data/euroc_results/configs/kalibr/kalibr.yaml' /data/euroc/imu_april/mav0"
# RUN_CMD="./calib_inspect 'camera-imu' '/data/euroc_results/configs/yac/calib_imu-results.yaml' /data/euroc/imu_april/mav0"

# YAC - MOCAP CALIBRATION
# RUN_CMD="./test_calib_mocap"

# YAC - VISUAL-INERTIAL CALIBRATION
# RUN_CMD="./test_calib_vi"
# RUN_CMD="time ./test_calib_vi --target test_calib_vi_batch"
# RUN_CMD="./test_calib_vi --target test_calib_vi_online"
# RUN_CMD="./test_calib_vi --target test_calib_vi_time_delay"

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

# YAC - NBT
# RUN_CMD="./test_calib_nbt"
# RUN_CMD="./test_calib_nbt --target test_lissajous_traj"
# RUN_CMD="./test_calib_nbt --target test_nbt_lissajous_trajs"
# RUN_CMD="./test_calib_nbt --target test_simulate_cameras"
# RUN_CMD="./test_calib_nbt --target test_simulate_imu"
# RUN_CMD="./test_calib_nbt --target test_nbt_eval"
# RUN_CMD="./test_calib_nbt --target test_nbt_find"

# YAC - SOLVER
# RUN_CMD="./test_solver"
# gdb -ex run -ex bt

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/projects/yac \
# && sudo make lib \
# && cd build \
# && $RUN_CMD
# " C-m
# exit

# && ./calib_inspect '/tmp/calib_imu-results.yaml' /data/euroc/imu_april/mav0

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/projects/yac \
# && sudo make lib \
# && cd build \
# && ./calib_camera \
#   /tmp/calib_data/calib_camera/calib-results.yaml \
#   /tmp/calib_data/calib_camera
# " C-m
# exit

# rm -rf /tmp/calib_data
# rm -rf /tmp/calib_data/calib_imu
# rm -rf /tmp/calib_data/calib_camera

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   cd ~/projects/yac/build \
#   && cmake --build . --target calib_gimbal2 .. \
#   && ./calib_gimbal2 /data/gimbal_experiments/calib/calib_gimbal
# " c-m
# exit

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
  cd ~/projects/yac/build \
  && cmake --build . --target calib_gimbal2 .. \
  && ./calib_gimbal2 /data/gimbal_experiments/exp-240703/calib/calib_gimbal
" c-m
exit
# && ./calib_gimbal2 /data/gimbal_experiments/exp-240303/calib/calib_gimbal/
# && ./calib_gimbal2 /data/gimbal_experiments/exp-240220/calib-240212/calib_gimbal/
# && ./calib_gimbal2 /data/gimbal_experiments/exp-240220/calib-240225/calib_gimbal/

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   cd ~/projects/yac/build \
#   && cmake --build . --target calib_mocap .. \
#   && cd .. \
#   && ./build/calib_mocap config/intel_d435i.yaml /data/mocap_experiments/calib0
# " c-m
# exit

# python3 scripts/plot_gimbal_frames.py
# python3 scripts/plot_gimbal_frames2.py

# make
# make lib_debug
# make debug
# make lib
# make release
# make tests
