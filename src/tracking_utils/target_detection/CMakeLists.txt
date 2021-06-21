cmake_minimum_required(VERSION 2.8.3)
project(target_detection)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS Eigen Boost
)
include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)
add_executable(
  sim_detect_node
  src/sim_detect_node.cpp
  src/sim_detect.cpp
)
target_link_libraries(   sim_detect_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


# add_library()