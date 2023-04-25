#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
git clone https://gitee.com/xiacanming/osqp-0.5.0.git

pushd osqp-0.5.0
mkdir build && cd build
cmake ../
make
make install
popd

git clone https://gitee.com/xiacanming/osqp-eigen-0.4.1.git
pushd osqp-eigen-0.4.1
mkdir build && cd build
cmake ../
make
make install
popd

#Clean
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr osqp
rm -fr osqp-eigen
