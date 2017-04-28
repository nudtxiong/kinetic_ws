cmake_minimum_required(VERSION 2.8.3)
project(learning_opencv_boost_eigen)

#  std=gnu++0x" enables c++ 11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS})
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  std_srvs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
)

set(learning_opencv_boost_eigen_include
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
               ${Boost_INCLUDE_DIRS}
               ${EIGEN3_INCLUDE_DIRS}
)
include_directories(${learning_opencv_boost_eigen_include})
add_executable(boost_bitset_node
   src/boost_bitset_node.cpp
)
add_executable(eigen_learning_node
   src/eigen_learning.cpp
)
add_executable(cpp_learning_node
   src/cppreference.cpp
)
target_link_libraries(cpp_learning_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
target_link_libraries(boost_bitset_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
target_link_libraries(eigen_learning_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
