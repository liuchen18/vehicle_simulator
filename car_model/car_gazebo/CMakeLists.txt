cmake_minimum_required(VERSION 2.8.3)
project(car_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES car_interface_plugin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

add_library(car_interface_plugin
  src/CarInterfacePlugin.cpp
)
target_link_libraries(car_interface_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    car_interface_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

