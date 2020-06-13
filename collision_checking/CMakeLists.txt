cmake_minimum_required(VERSION 2.8.3)

project(collision_checking CXX)

set(CMAKE_CXX_STANDARD 11)

aux_source_directory(include/collision_checking headerfiles)

aux_source_directory(src sourcefiles)

include_directories(
    include/collision_checking
    )

add_library(collision_lib
  ${headerfiles}
  ${sourcefiles}
)

add_executable(collision_test src/test.cpp)
target_link_libraries(collision_test collision_lib)