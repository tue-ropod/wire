cmake_minimum_required(VERSION 2.8.3)
project(wire_state_estimators)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  wire_core
  wire_features_lib
  pluginlib
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
    wire_core
    wire_features_lib
    pluginlib
    problib
    message_runtime 
    armadillo_matrix
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build
add_library(position_estimator src/PositionFilter.cpp src/KalmanFilter.cpp)
add_library(multimodel_estimator src/MultiModelFilter.cpp src/PositionFilter.cpp)
add_library(discrete_estimator src/DiscreteFilter.cpp)
add_library(fixed_estimator src/FixedState.cpp)
add_library(positionanddimension_estimator src/PositionAndDimensionFilter.cpp )
 
target_link_libraries(position_estimator ${catkin_LIBRARIES})
target_link_libraries(multimodel_estimator ${catkin_LIBRARIES})
target_link_libraries(discrete_estimator ${catkin_LIBRARIES})
target_link_libraries(fixed_estimator ${catkin_LIBRARIES})
target_link_libraries(positionanddimension_estimator wire_features_lib problib  ${catkin_LIBRARIES} )

add_dependencies(positionanddimension_estimator ${catkin_EXPORTED_TARGETS} )