#/bin/bash
set -e

# Build YAC
cd yac
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4

# Build tests
cd tests
./test_aprilgrid
./test_calib_camera
./test_calib_data
./test_calib_stereo
