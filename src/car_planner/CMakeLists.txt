cmake_minimum_required(VERSION 2.8.3)
project(car_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS OpenCV Eigen Boost
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


add_library( car_search 
    src/car_search.cpp
    )
target_link_libraries( car_search
    ${catkin_LIBRARIES}
    ompl
)
add_executable(car_node
    src/car_search_node.cpp
)
target_link_libraries(car_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  car_search
)


# add_library()