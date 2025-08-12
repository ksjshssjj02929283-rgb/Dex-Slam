#!/bin/bash

# Install the required libraries that are available as debs.
sudo apt-get update
sudo apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libceres-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    python3-sphinx \
    libatlas-base-dev \
    python3-colcon-common-extensions \
    stow

# Install Protocol Buffers and Abseil if available.
# No need to build it ourselves.
case "$(lsb_release -sc)" in
    jammy|bullseye)
        sudo apt-get install -y libgmock-dev protobuf-compiler libabsl-dev ;;
    focal|buster)
        sudo apt-get install -y libgmock-dev protobuf-compiler ;;
    bionic)
        ;;
esac


# Build and install abseil-cpp
set -o errexit
set -o verbose

cd ~
file="Third-Party"
if [ -e "$file" ]; then
  echo "$file already exists."
else
  mkdir Third-Party
fi
cd Third-Party

file="abseil-cpp"
if [ -e "$file" ]; then
  echo "$file already exists."
else
  git clone https://github.com/abseil/abseil-cpp.git
fi
cd abseil-cpp
git checkout d902eb869bcfacc1bad14933ed9af4bed006d481
file="build"
if [ -e "$file" ]; then
  echo "$file already exists."
else
  mkdir build
fi
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl

# Build and install Ceres.
cd -
cd ../../
file="ceres-solver"
if [ -e "$file" ]; then
  echo "$file already exists."
else
  git clone https://github.com/ceres-solver/ceres-solver.git
fi
cd ceres-solver
git checkout 1.14.0
file="build"
if [ -e "$file" ]; then
  echo "$file already exists."
else
  mkdir build
fi
cd build
cmake .. -G Ninja -DCXX11=ON
ninja
#CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install


## Build and install proto3.
#cd ../../
#git clone https://github.com/google/protobuf.git
#cd protobuf
#git checkout tags/v3.4.1
#mkdir build
#cd build
# cmake -G Ninja \
#  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
#  -DCMAKE_BUILD_TYPE=Release \
#  -Dprotobuf_BUILD_TESTS=OFF \
#  ../cmake
#ninja
#sudo ninja install


## Build and install Cartographer.
#cd ../../cartographer
#mkdir build
#cd build
#cmake .. -G Ninja
#ninja
##CTEST_OUTPUT_ON_FAILURE=1 ninja test
#sudo ninja install

## Another way to install lib
# Install rosdep dependencies.
#rosdep update
#rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y




