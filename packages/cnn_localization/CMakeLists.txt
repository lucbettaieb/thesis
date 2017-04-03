cmake_minimum_required(VERSION 2.8.3)
project(cnn_localization)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

set(Caffe_INCLUDE_DIRS caffe/include caffe/distribute/include)
set(CAFFE_LINK_LIBRARY caffe/build/lib)

find_package(TensorFlow REQUIRED)
find_package(Eigen REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cnn_localization
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs image_transport
#  DEPENDS system_libls
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${TensorFlow_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Caffe_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(tensorflow_ros
#   		    	src/tensorflow_ros.cpp
# )
# add_dependencies(tensorflow_ros cv_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tensorflow_ros
#   ${catkin_LIBRARIES}
#   ${Protobuf_LIBRARIES}
#   ${TensorFlow_LIBRARY}
# )

link_directories(${CAFFE_LINK_LIBRARY})
add_library(caffe_ros src/caffe_ros.cpp)
add_dependencies(caffe_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(caffe_ros
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${TensorFlow_LIBRARY}
  caffe
)

## Declare a C++ executable
# add_executable(demo_mode src/demo_mode.cpp)
# add_dependencies(demo_mode cnn_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(demo_mode
#   ${catkin_LIBRARIES}
#   ${Protobuf_LIBRARIES}
#   ${TensorFlow_LIBRARY}
#   cnn_localizer
# )

# add_executable(caffe_deployment src/caffe_deployment.cpp)

# add_dependencies(caffe_deployment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(caffe_deployment
#   ${catkin_LIBRARIES}
# )
