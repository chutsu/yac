#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"
REPO_URL=https://github.com/chutsu/apriltags

apt_install cmake libeigen3-dev libv4l-dev libopencv-*
install_git_repo $REPO_URL apriltags
