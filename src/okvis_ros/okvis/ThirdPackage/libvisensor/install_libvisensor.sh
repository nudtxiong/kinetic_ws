#!/bin/bash
cmake  -D CMAKE_INSTALL_PREFIX=../VISensor -H. -Bbuild -DDONT_USE_CATKIN=1 
cd build
make -j8 
make install
rm -rf build
#sudo
