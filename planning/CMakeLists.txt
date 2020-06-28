cmake_minimum_required(VERSION 2.8.3)

project(planning CXX)

set(CMAKE_CXX_STANDARD 11)

aux_source_directory(global_planner/include/global_planner gp_headerfiles)

aux_source_directory(planner_core/include core_headerfiles)

aux_source_directory(global_planner/src gp_sourcefiles)

include_directories(
        global_planner/include/global_planner
        planner_core/include
)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        tf
        )
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_library(global_planner_lib
        ${gp_headerfiles}
        ${gp_sourcefiles}
        ${core_headerfiles}
        )

#add_executable(collision_test src/test.cpp)
#target_link_libraries(collision_test collision_lib)