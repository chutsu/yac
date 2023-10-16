#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

# Install AprilTags3
apt_install libapriltag3

# # Build
# cd src/apriltag3

# mkdir -p build
# cd build
# cmake .. \
#   -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
#   -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
#   -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
# make
# make install
