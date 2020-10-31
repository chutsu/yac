#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Clone
if [ ! -d src/SuiteSparse ]; then
  cd src
  git clone https://github.com/DrTimothyAldenDavis/SuiteSparse
  cd ..
fi

# Install Deps
sudo apt-get install libgmp3-dev -y -qq
sudo apt-get install libmpfr-dev libmpfr-doc -y -qq

# Build
cd src/SuiteSparse
make clean
make library
make install INSTALL=$INSTALL_PREFIX
