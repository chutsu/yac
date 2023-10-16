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
  if [ "$BASEDIR"/install_"$1".bash > "install_${1}.log" 2>&1 ]; then
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

check_ubuntu_version() {
  if [ -x "$(command -v lsb_release)" ]; then
    VERSION=$(lsb_release -r | grep -oP '(?<=Release:\s)[0-9.]+')
    if [ $? -eq 0 ] && [ $VERSION != $UBUNTU_VERSION ]; then
      echo "This script is only for Ubuntu $UBUNTU_VERSION"
      exit -1;
    fi
  else
    # lsb_release command is not installed - Not ubuntu
    exit -1;
  fi
}

check_ubuntu_version
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
