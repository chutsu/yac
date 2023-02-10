#!/bin/bash
set -e  # exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

apt_install libopencv-*
