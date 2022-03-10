#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

VERSION=2.0.0
echo "building ceres-solver [$VERSION] ..."

# Clone
if [ ! -d $INSTALL_PREFIX/src/ceres-solver ]; then
  cd $INSTALL_PREFIX/src
  git clone --quiet https://github.com/ceres-solver/ceres-solver
  cd ..
fi

# Build
cd $INSTALL_PREFIX/src/ceres-solver
git checkout ${VERSION} --quiet

mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make -j4
make install
