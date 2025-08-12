#!/bin/bash

# Build and install SLAM-base
set -o errexit
set -o verbose

cd ../b1_cartographer
mkdir build
cd build
cmake .. -G Ninja
ninja
#CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install


