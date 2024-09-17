#!/bin/bash
set -e

TARGET="dev"
# CMD="make release && gdb -ex=run -ex=bt -ex=quit ./build/TestCalibCamera"
CMD="make debug && ./build/TestCameraResidual"
# CMD="make debug && ./build/TestCameraModel"
tmux send-keys -t $TARGET -R C-l C-m
tmux send-keys -t $TARGET -R "cd ~/projects/yac && clear && $CMD" C-m C-m
exit
