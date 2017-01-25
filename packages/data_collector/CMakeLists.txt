cmake_minimum_required(VERSION 2.8.3)
project(data_collector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(data_collector src/data_collector.cpp)
add_dependencies(data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
add_executable(old_data_collector src/old_data_collector.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(old_data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(old_data_collector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(collect_data src/collect_data.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(collect_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(collect_data
  data_collector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
