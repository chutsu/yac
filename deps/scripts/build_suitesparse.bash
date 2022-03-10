#!/bin/bash
set -e
INSTALL_PREFIX=${PWD}
VERSION="v5.10.1"
echo "building suitesparse [$VERSION] ..."

# Clone
if [ ! -d src/SuiteSparse ]; then
  cd src
  git clone --quiet https://github.com/DrTimothyAldenDavis/SuiteSparse
  cd ..
fi

# Install Deps
# sudo apt-get install libgmp3-dev -y -qq
# sudo apt-get install libmpfr-dev libmpfr-doc -y -qq

# Build
cd src/SuiteSparse
git checkout ${VERSION} --quiet
make library -j4
make install INSTALL="$INSTALL_PREFIX"
