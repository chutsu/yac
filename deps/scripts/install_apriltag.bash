#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"
echo "building apriltag ..."

# Build
cp -r $PWD/src/apriltag $INSTALL_PREFIX/src/apriltag
cd $INSTALL_PREFIX/src/apriltag

mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_PREFIX_PATH=$INSTALL_PREFIX
make
make install
mv $INSTALL_PREFIX/lib/libapriltag.a $INSTALL_PREFIX/lib/libapriltags.a
