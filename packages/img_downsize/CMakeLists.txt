cmake_minimum_required(VERSION 2.8.3)
project(img_downsize)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_downsizer
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(img_downsize src/img_downsize.cpp)

## Add cmake target dependencies of the executable
# add_dependencies(camera_downsizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(img_downsize
  ${catkin_LIBRARIES}
)

