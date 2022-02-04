#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

# VERY IMPORTANT! We need to specifically tell cmake we have our own custom
# installation of eigen3 or else you'll end up with seg-faults during
# optimization
# export CMAKE_EXTRA_ARGS="-DEigen3_DIR=${PREFIX}/share/eigen3/cmake/"

# install_dependencies() {
#     apt-get update -qq
#     apt-get install -qq -y cmake \
#                            libgoogle-glog-dev \
#                            libatlas-base-dev \
#                            libeigen3-dev \
#                            libsuitesparse-dev
# }
#
# # MAIN
# install_dependencies
# install_git_repo \
#   https://github.com/ceres-solver/ceres-solver.git \
#   ceres-solver

apt_install libceres-dev
