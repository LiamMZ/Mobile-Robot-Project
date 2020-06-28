cmake_minimum_required(VERSION 3.0.2)
project(Mobile-Robot-Project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV REQUIRED
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )