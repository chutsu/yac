#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Clone
if [ ! -d src/apriltag ]; then
  cd src
  git clone https://github.com/chutsu/apriltag
  cd ..
fi

# Build
cd src/apriltag

make deps
git submodule init
git submodule update

rm -rf build
mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make
make install
