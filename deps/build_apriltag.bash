#!/bin/bash
set -e

# Clone
if [ ! -d src/apriltag ]; then
  cd src
  git clone https://github.com/chutsu/apriltag
  cd ..
fi

# Build
cd src/apriltag
make deps
make build
make install
