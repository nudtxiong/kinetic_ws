#!/bin/bash
cmake  -D CMAKE_INSTALL_PREFIX=../opengv -H. -Bbuild 
cd build
make -j8 
make install
rm -rf build
#sudo
