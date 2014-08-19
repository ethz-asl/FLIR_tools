cmake_minimum_required(VERSION 2.8.3)
project(ir_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  message_generation
  sensor_msgs 
  cv_bridge 
  image_transport 
)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(GTK2)

include_directories(include 
					${catkin_INCLUDE_DIRS}
                    ${GTK2_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
)
                                
add_library(false_color src/false_color.cpp)

target_link_libraries(false_color ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES} )
                                
add_executable(ir_viewer src/ir_viewer.cpp)

target_link_libraries(ir_viewer ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES} 
                                ${GTK2_LIBRARIES} false_color)
                                
add_executable(ir_converter src/ir_converter.cpp )

target_link_libraries(ir_converter false_color ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES} )
                                
