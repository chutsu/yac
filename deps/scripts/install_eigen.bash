#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

# Install Eigen3
apt_install libeigen3-dev
apt_install libeigen3-doc


# VERSION=3.4
# echo "building eigen [$VERSION] ..."
#
# # Clone
# if [ ! -d $INSTALL_PREFIX/src/eigen ]; then
#   cd $INSTALL_PREFIX/src
#   git clone --quiet https://gitlab.com/libeigen/eigen.git
#   cd ..
# fi
#
# # Build
# cd $INSTALL_PREFIX/src/eigen
# git checkout ${VERSION} --quiet
#
# mkdir -p build
# cd build
# cmake .. \
#   -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
#   -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
#   -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
# make install
