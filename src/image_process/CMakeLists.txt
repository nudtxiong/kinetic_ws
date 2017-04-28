cmake_minimum_required(VERSION 2.8.3)
project(image_process)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport
                 opencv2/ roscpp rospy sensor_msgs std_msgs std_srvs
)

set(image_process_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)
include_directories(${image_process_includes})

add_executable(image_publish_node
      src/image_publish.cpp
      src/FileFunctions.cpp
)
add_executable(image_write_node
      src/image_write.cpp
)

target_link_libraries(image_publish_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
target_link_libraries(image_write_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
