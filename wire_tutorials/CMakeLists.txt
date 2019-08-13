cmake_minimum_required(VERSION 2.8.3)
project(wire_tutorials)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  wire_msgs
  problib
)

# Declare a catkin package                                                                                                                                
catkin_package(
  CATKIN_DEPENDS
    roscpp
    wire_msgs
    problib
)

include_directories(${catkin_INCLUDE_DIRS})

## Build

# ROS nodes
add_executable(get_world_state src/get_world_state.cpp) 
target_link_libraries(get_world_state ${catkin_LIBRARIES})

add_executable(generate_evidence src/generate_evidence.cpp) 
target_link_libraries(generate_evidence ${catkin_LIBRARIES})
