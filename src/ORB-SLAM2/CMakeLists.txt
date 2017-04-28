cmake_minimum_required(VERSION 2.8.3)
project(pporb_slam2)

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
add_library(ppORB_SLAM2 SHARED
    src/System.cpp
    src/Tracking.cpp
    src/LocalMapping.cpp
    src/LoopClosing.cpp
    src/ORBextractor.cpp
    src/ORBmatcher.cpp
    src/FrameDrawer.cpp
    src/Converter.cpp
    src/MapPoint.cpp
    src/KeyFrame.cpp
    src/Map.cpp
    src/MapDrawer.cpp
    src/Optimizer.cpp
    src/PnPsolver.cpp
    src/Frame.cpp
    src/KeyFrameDatabase.cpp
    src/Sim3Solver.cpp
    src/Initializer.cpp
    src/Viewer.cpp
)
target_link_libraries(ppORB_SLAM2
${OpenCV_LIBS}
${catkin_LIBRARIES}
 pangolin
 ppDBoW2
 ppg2o
)

set(LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBS}
 pangolin
 ppDBoW2
 ppg2o
 ppORB_SLAM2
)

# Node for monocular camera
add_executable(ppMono ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_mono.cc)
target_link_libraries(ppMono ${LIBS})
# Node for stereo camera
add_executable(ppStereo ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_stereo.cc)
target_link_libraries(ppStereo ${LIBS})
# Node for RGB-D camera
add_executable(ppRGBD ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc)
target_link_libraries(ppRGBD ${LIBS})
