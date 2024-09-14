#!/bin/bash
set -e

TARGET="dev"
CMD="make debug && ./lib/build/TestCameraResidual"
# CMD="make debug && ./lib/build/TestCameraModel"
tmux send-keys -t $TARGET -R C-l C-m
tmux send-keys -t $TARGET -R "cd ~/projects/yac && clear && $CMD" C-m C-m
exit
