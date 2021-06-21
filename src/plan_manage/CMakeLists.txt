cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

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
    plan_env
    grid_path_searcher
    quadrotor_msgs
    target_prediction
    sfc_generation
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
add_library(plan_manage
STATIC
src/plan_manage.cpp
)
target_link_libraries(plan_manage
${catkin_LIBRARIES}
sfc_generation
bezier_predict
grid_path_searcher
)
# add_library( Optraj
#     src/Optraj.cpp
# )
# target_link_libraries(Optraj
#     ${catkin_LIBRARIES}
# )


add_executable( tracking_fsm_node 
    src/tracking_fsm_node.cpp
)
target_link_libraries(tracking_fsm_node
${catkin_LIBRARIES}
plan_manage
${PROJECT_SOURCE_DIR}/lib/libOptraj.so
)


# target_link_libraries(demo_node 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ooqpgensparse 
#     ooqpsparse 
#     ooqpgondzio 
#     ooqpbase blas ma27 gfortran f2c  
# )

# add_executable ( random_complex 
#     src/random_complex_generator.cpp )

# target_link_libraries( random_complex
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES} )  

# add_library( grid_path_searcher 
#     src/astar.cpp 
#     src/hybridAstar_searcher.cpp
# )
# target_link_libraries( grid_path_searcher
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     plan_env
# )  

# # add_executable(
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

