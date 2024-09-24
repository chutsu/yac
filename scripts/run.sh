#!/bin/bash
set -e

TARGET="dev"
# CMD="make debug && ./build/TestCalibCamera"
# CMD="make debug && ./build/TestCameraChain"
# CMD="make debug && ./build/TestCameraModel"
CMD="make debug && ./build/TestInertialError"
# CMD="make debug && ./build/TestReprojectionError"
# CMD="make debug && ./build/TestSim"
tmux send-keys -t $TARGET -R C-l C-m
tmux send-keys -t $TARGET -R "cd ~/projects/yac && clear && $CMD" C-m C-m
exit

# python3 scripts/plot_sim.py
# python3 scripts/plot_test_inertial_error_solve.py
