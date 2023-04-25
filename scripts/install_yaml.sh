#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
git clone https://github.com/jbeder/yaml-cpp.git

pushd yaml-cpp
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make
make install
popd


#Clean
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr yaml-cpp
