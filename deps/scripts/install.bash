#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DEBIAN_FRONTEND="noninteractive"
export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$PREFIX"
mkdir -p "$DOWNLOAD_PATH"

install() {
  echo "Installing $1 ..." && "$BASEDIR"/install_"$1".bash > /dev/null
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
    wget \
    libasan4 \
    libasan5

  # For suitesparse
  apt_install libgmp3-dev libmpfr-dev  libmpfr-doc
}

apt_update
install_base
install ros
# install apriltags
# install ceres
# install eigen
install opencv
install yamlcpp
