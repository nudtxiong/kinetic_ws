cmake_minimum_required(VERSION 2.8.3)
project(dense_new)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -DENABLE_SSE -g -fopenmp -Wall -std=c++11 -Ofast -pipe -march=native -mtune=native")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS} -DENABLE_SSE -fopenmp -Ofast -march=native -mtune=native -pipe -Wall")
SET(OpenCV_INCLUDE_DIRS "/usr/include/opencv;/usr/include/opencv2")
SET(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs 
  cv_bridge
  image_transport
  message_generation
  camera_info_manager 
  dynamic_reconfigure
  cmake_modules
  tf
)

add_definitions("-DUSE_ROS")
add_definitions("-DENABLE_SSE")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(libvisensor 1.1.0 REQUIRED)

#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#find_package(CUDA)

#include("/opt/OpenCV3.2/share/OpenCV/OpenCVConfig.cmake")
#include("/opt/OpenCV2.4/share/OpenCV/OpenCVConfig.cmake")
set(OpenCV_DIR /opt/OpenCV3.2/share/OpenCV)
find_package(OpenCV 3.2 REQUIRED)

#set(OpenCV_DIR /opt/OpenCV2.4/share/OpenCV)
#find_package(OpenCV 2.4 REQUIRED)

find_package(X11 REQUIRED)
include_directories("/usr/local/cuda-8.0/include")
include_directories(/usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/glib-2.0/include/)
add_message_files(
  DIRECTORY msg
  FILES visensor_imu.msg 
  visensor_time_host.msg
  visensor_calibration.msg
)

add_service_files(
	FILES
	visensor_calibration_service.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

if(NOT DEFINED CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

generate_dynamic_reconfigure_options(cfg/LSDParams.cfg)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
  camera_info_manager 
)

set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/
#  ${OPENNI2_INCLUDE}
  ${libvisensor_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)



## Declare a cpp library
# add_library(IROS2015
#   src/${PROJECT_NAME}/IROS2015.cpp
# )

## Declare a cpp executable
add_library(dense_new SHARED ${SOURCE_FILES})
add_dependencies(dense_new ${${PROJECT_NAME}_EXPORTED_TARGETS}})

## Specify libraries to link a library or executable target against
target_link_libraries(dense_new 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${libvisensor_LIBRARIES}
   X11
#  OpenNI2
 # ${Boost_FILESYSTEM_LIBRARY} 
 # ${Boost_SYSTEM_LIBRARY}
)

add_executable(my_slam src/main_live_odometry.cpp)
target_link_libraries(my_slam dense_new)

