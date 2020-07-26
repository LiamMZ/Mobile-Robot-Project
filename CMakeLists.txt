cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_project)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_DIR /home/liam/opencv/build/)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  OpenCV REQUIRED
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
set(${PROJECT_NAME}_SRC_FILES
  src/object_detection.cpp
)





add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC_FILES})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(obj_det_test src/test.cpp)
target_link_libraries(obj_det_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV})

add_executable(calibration config/calibration/calibration.cpp)
target_link_libraries(calibration ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV})