#!/bin/bash
set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

# Install Deps
apt_install libgmp3-dev
apt_install libmpfr-dev
apt_install libmpfr-doc
apt_install libsuitesparse-dev
apt_install libsuitesparse-doc

# VERSION="v5.10.1"
# echo "building suitesparse [$VERSION] ..."
#
# # Clone
# if [ ! -d $INSTALL_PREFIX/src/SuiteSparse ]; then
#   cd $INSTALL_PREFIX/src
#   git clone --quiet https://github.com/DrTimothyAldenDavis/SuiteSparse
#   cd ..
# fi
#
# # Install Deps
# # sudo apt-get install libgmp3-dev -y -qq
# # sudo apt-get install libmpfr-dev libmpfr-doc -y -qq
#
# # Build
# cd $INSTALL_PREFIX/src/SuiteSparse
# git checkout ${VERSION} --quiet
# make library -j4
# make install INSTALL="$INSTALL_PREFIX"

sudo apt-get install -y -qq libgmp3-dev libmpfr-dev libmpfr-doc
sudo apt-get install -y -qq libsuitesparse-dev
sudo apt-get install -y -qq libsuitesparse-doc
