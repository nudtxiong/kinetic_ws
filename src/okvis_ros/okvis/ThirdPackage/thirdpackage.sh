#!/bin/bash
cd libbrisk
mkdir build 
cd build
cmake -D CMAKE_INSTALL_PREFIX=../../brisk ..
make -j8 
make install
cd ../
rm -rf build/
cd ../
#sudo

 
cd libopengv
mkdir build 
cd build
cmake  -D CMAKE_INSTALL_PREFIX=../../opengv ..
make -j8 
make install
cd ../
rm -rf build/
cd ../
 

cd libvisensor
mkdir build 
cd build
cmake  -D CMAKE_INSTALL_PREFIX=../../VISensor -DDONT_USE_CATKIN=1 ..
make -j8 
make install
cd ../
rm -rf build/
cd ../
#sudo
