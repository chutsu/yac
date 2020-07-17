#!/bin/bash
set -e  # Exit on first error

mkdir -p /data/euroc_mav/

BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
CALIB_URL=${BASE_URL}"/calibration_datasets"

cd /data/euroc_mav

if [ ! -f cam_april.zip ]; then
  wget "${CALIB_URL}/cam_april/cam_april.zip"
fi
if [ ! -f imu_april.zip ]; then
  wget "${CALIB_URL}/imu_april/imu_april.zip"
fi

echo "unzipping cam_april.zip"
mkdir -p cam_april
unzip -oq cam_april.zip -d cam_april

echo "unzipping imu_april.zip"
mkdir -p imu_april
unzip -oq imu_april.zip -d imu_april
