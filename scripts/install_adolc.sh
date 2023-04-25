#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

sudo apt-get install autoconf automake libtool

echo "Build and install ColPack"

git clone https://github.com/CSCsw/ColPack.git

# sudo works
pushd ColPack
pushd build/automake      # automake folder
sudo autoreconf -vif        # generate configure files based on the machince
sudo mkdir mywork           
pushd mywork
fullpath=$(pwd)        # modify fullpath to your destination folder if need
sudo ../configure --prefix=${fullpath}
sudo make -j 4              # Where "4" is the number of cores on your machine
sudo make install           # install lib and include/ColPack to destination

sudo cp -rf include/ColPack /usr/local/include/
sudo cp -r lib/lib* /usr/local/lib/
popd
popd
popd

echo "colpack done."
echo "Build and install ADOL-C"

wget https://www.coin-or.org/download/source/ADOL-C/ADOL-C-2.6.3.zip -O ADOL-C-2.6.3.zip
unzip ADOL-C-2.6.3.zip

pushd ADOL-C-2.6.3
sudo autoreconf --install
sudo automake
sudo ./configure --prefix="/home/hope/sg/51simone/Dust/src/scripts/ADOL-C-2.6.3" --enable-sparse --enable-addexa --with-openmp-flag="-fopenmp" --with-colpack="/usr/local/colpack" ADD_CXXFLAGS="-fPIC" ADD_CFLAGS="-fPIC" ADD_FFLAGS="-fPIC"

sudo make -j8 all
sudo make install
echo "Build ADOL-C done."

cd ADOL-C/
sudo cp -rf include/adolc /usr/local/include/
cd ..
sudo cp -r lib64/lib* /usr/local/lib/
popd

#找动态连接库
echo "/usr/local/lib" >>sudo  /etc/ld.so.conf
sudo ldconfig

# Clean up.
sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
sudo rm -fr ADOL-C-2.6.3.zip ADOL-C-2.6.3 ColPack
