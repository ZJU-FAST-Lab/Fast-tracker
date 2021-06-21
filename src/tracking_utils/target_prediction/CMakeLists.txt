cmake_minimum_required(VERSION 2.8.3)
project(target_prediction)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs
    geometry_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror



add_library( bezier_predict 
    STATIC
    src/bezier_predict.cpp 
)
target_link_libraries( bezier_predict
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ooqpgensparse 
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran 
)  

# add_executable(
#     hybridAstar
#     src/hybridAstar_node.cpp
#     src/Astar_searcher.cpp
#     src/HybridAstar/hybridAstar_searcher.cpp
#     # src/HybridAstar/front_traj_predict.cpp
#     src/HybridAstar/bezier_predict.cpp
#     ../waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp
# )

# target_link_libraries( hybridAstar
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES} 
#     ooqpgensparse 
#     ooqpsparse 
#     ooqpgondzio 
#     ooqpbase blas ma27 gfortran f2c )

# add_executable(
#     real_tracker_node
#     src/real_tracker_node.cpp
#     src/Astar_searcher.cpp
#     src/HybridAstar/hybridAstar_searcher.cpp
#     # src/HybridAstar/front_traj_predict.cpp
#     src/HybridAstar/bezier_predict.cpp
#     # src/HybridAstar/poly_predict.cpp
#     ../waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp
# )

# target_link_libraries( real_tracker_node
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES} 
#     ooqpgensparse 
#     ooqpsparse 
#     ooqpgondzio 
#     ooqpbase blas ma27 gfortran f2c )

