# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chen/ws_chen/src/planning_control_for_cars/collision_checking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chen/ws_chen/src/planning_control_for_cars/collision_checking

# Include any dependencies generated for this target.
include CMakeFiles/collision_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/collision_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/collision_lib.dir/flags.make

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o: CMakeFiles/collision_lib.dir/flags.make
CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o: src/AABB2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o -c /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/AABB2d.cpp

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_lib.dir/src/AABB2d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/AABB2d.cpp > CMakeFiles/collision_lib.dir/src/AABB2d.cpp.i

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_lib.dir/src/AABB2d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/AABB2d.cpp -o CMakeFiles/collision_lib.dir/src/AABB2d.cpp.s

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.requires:

.PHONY : CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.requires

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.provides: CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/collision_lib.dir/build.make CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.provides.build
.PHONY : CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.provides

CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.provides.build: CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o


CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o: CMakeFiles/collision_lib.dir/flags.make
CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o: src/linesegment2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o -c /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/linesegment2d.cpp

CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/linesegment2d.cpp > CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.i

CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/linesegment2d.cpp -o CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.s

CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.requires:

.PHONY : CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.requires

CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.provides: CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/collision_lib.dir/build.make CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.provides.build
.PHONY : CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.provides

CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.provides.build: CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o


CMakeFiles/collision_lib.dir/src/point2d.cpp.o: CMakeFiles/collision_lib.dir/flags.make
CMakeFiles/collision_lib.dir/src/point2d.cpp.o: src/point2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/collision_lib.dir/src/point2d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_lib.dir/src/point2d.cpp.o -c /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/point2d.cpp

CMakeFiles/collision_lib.dir/src/point2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_lib.dir/src/point2d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/point2d.cpp > CMakeFiles/collision_lib.dir/src/point2d.cpp.i

CMakeFiles/collision_lib.dir/src/point2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_lib.dir/src/point2d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/point2d.cpp -o CMakeFiles/collision_lib.dir/src/point2d.cpp.s

CMakeFiles/collision_lib.dir/src/point2d.cpp.o.requires:

.PHONY : CMakeFiles/collision_lib.dir/src/point2d.cpp.o.requires

CMakeFiles/collision_lib.dir/src/point2d.cpp.o.provides: CMakeFiles/collision_lib.dir/src/point2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/collision_lib.dir/build.make CMakeFiles/collision_lib.dir/src/point2d.cpp.o.provides.build
.PHONY : CMakeFiles/collision_lib.dir/src/point2d.cpp.o.provides

CMakeFiles/collision_lib.dir/src/point2d.cpp.o.provides.build: CMakeFiles/collision_lib.dir/src/point2d.cpp.o


CMakeFiles/collision_lib.dir/src/test.cpp.o: CMakeFiles/collision_lib.dir/flags.make
CMakeFiles/collision_lib.dir/src/test.cpp.o: src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/collision_lib.dir/src/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_lib.dir/src/test.cpp.o -c /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/test.cpp

CMakeFiles/collision_lib.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_lib.dir/src/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/test.cpp > CMakeFiles/collision_lib.dir/src/test.cpp.i

CMakeFiles/collision_lib.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_lib.dir/src/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/src/test.cpp -o CMakeFiles/collision_lib.dir/src/test.cpp.s

CMakeFiles/collision_lib.dir/src/test.cpp.o.requires:

.PHONY : CMakeFiles/collision_lib.dir/src/test.cpp.o.requires

CMakeFiles/collision_lib.dir/src/test.cpp.o.provides: CMakeFiles/collision_lib.dir/src/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/collision_lib.dir/build.make CMakeFiles/collision_lib.dir/src/test.cpp.o.provides.build
.PHONY : CMakeFiles/collision_lib.dir/src/test.cpp.o.provides

CMakeFiles/collision_lib.dir/src/test.cpp.o.provides.build: CMakeFiles/collision_lib.dir/src/test.cpp.o


# Object files for target collision_lib
collision_lib_OBJECTS = \
"CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o" \
"CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o" \
"CMakeFiles/collision_lib.dir/src/point2d.cpp.o" \
"CMakeFiles/collision_lib.dir/src/test.cpp.o"

# External object files for target collision_lib
collision_lib_EXTERNAL_OBJECTS =

libcollision_lib.a: CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o
libcollision_lib.a: CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o
libcollision_lib.a: CMakeFiles/collision_lib.dir/src/point2d.cpp.o
libcollision_lib.a: CMakeFiles/collision_lib.dir/src/test.cpp.o
libcollision_lib.a: CMakeFiles/collision_lib.dir/build.make
libcollision_lib.a: CMakeFiles/collision_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libcollision_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/collision_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/collision_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/collision_lib.dir/build: libcollision_lib.a

.PHONY : CMakeFiles/collision_lib.dir/build

CMakeFiles/collision_lib.dir/requires: CMakeFiles/collision_lib.dir/src/AABB2d.cpp.o.requires
CMakeFiles/collision_lib.dir/requires: CMakeFiles/collision_lib.dir/src/linesegment2d.cpp.o.requires
CMakeFiles/collision_lib.dir/requires: CMakeFiles/collision_lib.dir/src/point2d.cpp.o.requires
CMakeFiles/collision_lib.dir/requires: CMakeFiles/collision_lib.dir/src/test.cpp.o.requires

.PHONY : CMakeFiles/collision_lib.dir/requires

CMakeFiles/collision_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/collision_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/collision_lib.dir/clean

CMakeFiles/collision_lib.dir/depend:
	cd /home/chen/ws_chen/src/planning_control_for_cars/collision_checking && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chen/ws_chen/src/planning_control_for_cars/collision_checking /home/chen/ws_chen/src/planning_control_for_cars/collision_checking /home/chen/ws_chen/src/planning_control_for_cars/collision_checking /home/chen/ws_chen/src/planning_control_for_cars/collision_checking /home/chen/ws_chen/src/planning_control_for_cars/collision_checking/CMakeFiles/collision_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/collision_lib.dir/depend
