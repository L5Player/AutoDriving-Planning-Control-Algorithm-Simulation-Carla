#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

sudo apt-get install cppad gfortran 

# wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip -O Ipopt-3.12.8.zip
# unzip Ipopt-3.12.8.zip

# Step by step   
pushd Ipopt-3.12.8/ThirdParty/Blas
./get.Blas    
cd ../Lapack
./get.Lapack  
cd ../Mumps  
./get.Mumps  
cd ../Metis  
./get.Metis
cd ../ASL
./get.ASL

cd ..
cd ..

mkdir build  
cd build  
../configure  
make -j4  
make install  

cp -a include/* /usr/include/.  
cp -a lib/* /usr/lib/. 

popd

# Clean up.
cd ..
cd ..
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -rf Ipopt-3.12.8.zip Ipopt-3.12.8
