cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_project)

set(OpenCV_DIR /home/liam/opencv/build/)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV REQUIRED
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /include
)
set(${PROJECT_NAME}_SRC_FILES
  src/object_detection.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories( ${OpenCV_INCLUDE_DIRS} )
