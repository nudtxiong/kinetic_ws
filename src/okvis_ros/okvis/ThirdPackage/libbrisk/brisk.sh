#!/bin/bash
cmake  -D CMAKE_INSTALL_PREFIX=../brisk -H. -Bbuild 
cd build
make -j8 
make install
rm -rf build
#sudo
