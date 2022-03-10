#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}
VERSION=2.0.0
echo "building ceres-solver [$VERSION] ..."

# Clone
if [ ! -d src/ceres-solver ]; then
  cd src
  git clone https://github.com/ceres-solver/ceres-solver
  cd ..
fi

# Build
cd src/ceres-solver
git checkout ${VERSION} --quiet

mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make
make install
