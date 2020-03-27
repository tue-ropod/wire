cmake_minimum_required(VERSION 2.8.3)
project(wire_features_lib)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geolib2
  armadillo_matrix
  problib
)

# Declare a catkin package                                                                                                                                
catkin_package(
  INCLUDE_DIRS
    include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    problib
    message_runtime 
    armadillo_matrix
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build
add_library(wire_features_lib 
src/circle.cpp
src/featureProbabilities.cpp
src/featureProperties.cpp
src/rectangle.cpp
include/circle.h
include/featureProbabilities.h
include/featureProperties.h
include/rectangle.h)
target_link_libraries(wire_features_lib ${catkin_LIBRARIES})