cmake_minimum_required(VERSION 2.8.3)
project(ir_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  message_generation
  sensor_msgs 
  cv_bridge 
  image_transport 
  ir_converter
)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(GTK2)

include_directories(${catkin_INCLUDE_DIRS}
                      ${GTK2_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
  ir_converter
)

add_executable(ir_viewer src/ir_viewer.cpp)

target_link_libraries(ir_viewer ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES} 
                                ${GTK2_LIBRARIES} false_color)
