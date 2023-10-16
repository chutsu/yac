#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DEBIAN_FRONTEND="noninteractive"
export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$INSTALL_PREFIX"
mkdir -p "$DOWNLOAD_PATH"

install() {
  echo -n "Installing $1 ... ";
  if "$BASEDIR"/install_"$1".bash > "install_${1}.log" 2>&1
  then
    echo "DONE!"
  else
    echo "FAILED!"
    cat "install_${1}.log"
  fi
}

install_base() {
  apt_install \
    dialog \
    apt-utils \
    git \
    mercurial \
    cmake \
    g++ \
    clang \
    wget
    # libasan4 \
    # libasan5
}

apt_update
install_base
# install ros
install eigen
install suitesparse
install ceres
install opencv
install yamlcpp
install apriltag
install apriltag3
