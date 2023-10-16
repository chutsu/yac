#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"
echo "building apriltag3 ..."

# Clone
if [ ! -d $INSTALL_PREFIX/src/apriltag3 ]; then
  cd $INSTALL_PREFIX/src
  git clone --quiet https://github.com/AprilRobotics/apriltag apriltag3
  cd ..
fi

# Build
cd $INSTALL_PREFIX/src/apriltag3

mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make
make install
