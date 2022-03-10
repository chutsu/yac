#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}
VERSION="v3.2.0"
echo "building apriltag3 [$VERSION] ..."

# Clone
if [ ! -d src/apriltag3 ]; then
  cd src
  git clone https://github.com/AprilRobotics/apriltag apriltag3
  cd ..
fi

# Build
cd src/apriltag3
git checkout ${VERSION} --quiet

mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make
make install
cd ../../../

# Rename AprilTags include dir
mkdir -p include/apriltags3
mv -f include/AprilTags/* include/apriltags3/
rm -rf include/AprilTags
