#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}
VERSION=3.4

# Clone
if [ ! -d src/eigen ]; then
  cd src
  git clone https://gitlab.com/libeigen/eigen.git
  cd ..
fi

# Build
cd src/eigen
git checkout ${VERSION}

rm -rf build
mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make install
