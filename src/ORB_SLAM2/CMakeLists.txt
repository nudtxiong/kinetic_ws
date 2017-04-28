cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs 
  std_srvs
  image_transport
  roscpp
  rospy
  sensor_msgs
  tf
  cmake_modules
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o/)
#add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
#${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/include
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES loop_detector
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_filters message_generation opencv2/ roscpp rospy sensor_msgs std_msgs std_srvs
#  DEPENDS system_lib
)

#find_package(Pangolin REQUIRED)
include(/opt/ml_libraries/Pangolin/lib/cmake/Pangolin/PangolinConfig.cmake)
include_directories(${Pangolin_INCLUDE_DIRS})
add_library(ORB_SLAM2 SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)
target_link_libraries(ORB_SLAM2
${OpenCV_LIBS}
${catkin_LIBRARIES}
 pangolin
 DBoW2
 g2o
)

set(LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBS}
 pangolin
 DBoW2
 g2o
 ORB_SLAM2
)

# Node for monocular camera
add_executable(Mono ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_mono.cc)
target_link_libraries(Mono ${LIBS})
# Node for stereo camera
add_executable(Stereo ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_stereo.cc)
target_link_libraries(Stereo ${LIBS})
# Node for RGB-D camera
add_executable(RGBD ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc)
target_link_libraries(RGBD ${LIBS})
