cmake_minimum_required(VERSION 2.8.3)
project(data_collector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(data_collector src/data_collector.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(data_collector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(data_collector_node
  ${catkin_LIBRARIES}
)
