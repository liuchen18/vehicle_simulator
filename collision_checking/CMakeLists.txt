cmake_minimum_required(VERSION 2.8.3)

project(collosion_checking CXX)

set(CMAKE_CXX_STANDARD 11)

aux_source_directory(include/collosion_checking headerfiles)

aux_source_directory(src sourcefiles)

include_directories(
    include/collosion_checking
    )

add_library(collosion_lib
  ${sourcefiles}
  ${headerfiles}
)

add_executable(test src/test.cpp)
target_link_libraries(test collosion_lib)