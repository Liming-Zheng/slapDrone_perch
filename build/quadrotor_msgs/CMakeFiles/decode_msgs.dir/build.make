# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/dawn/Drone/SlapDrone/slapDrone_perch/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dawn/Drone/SlapDrone/slapDrone_perch/build

# Include any dependencies generated for this target.
include quadrotor_msgs/CMakeFiles/decode_msgs.dir/depend.make

# Include the progress variables for this target.
include quadrotor_msgs/CMakeFiles/decode_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include quadrotor_msgs/CMakeFiles/decode_msgs.dir/flags.make

quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o: quadrotor_msgs/CMakeFiles/decode_msgs.dir/flags.make
quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o: /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs/src/decode_msgs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dawn/Drone/SlapDrone/slapDrone_perch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o"
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o -c /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs/src/decode_msgs.cpp

quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.i"
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs/src/decode_msgs.cpp > CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.i

quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.s"
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs/src/decode_msgs.cpp -o CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.s

# Object files for target decode_msgs
decode_msgs_OBJECTS = \
"CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o"

# External object files for target decode_msgs
decode_msgs_EXTERNAL_OBJECTS =

/home/dawn/Drone/SlapDrone/slapDrone_perch/devel/lib/libdecode_msgs.so: quadrotor_msgs/CMakeFiles/decode_msgs.dir/src/decode_msgs.cpp.o
/home/dawn/Drone/SlapDrone/slapDrone_perch/devel/lib/libdecode_msgs.so: quadrotor_msgs/CMakeFiles/decode_msgs.dir/build.make
/home/dawn/Drone/SlapDrone/slapDrone_perch/devel/lib/libdecode_msgs.so: quadrotor_msgs/CMakeFiles/decode_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dawn/Drone/SlapDrone/slapDrone_perch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/dawn/Drone/SlapDrone/slapDrone_perch/devel/lib/libdecode_msgs.so"
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/decode_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
quadrotor_msgs/CMakeFiles/decode_msgs.dir/build: /home/dawn/Drone/SlapDrone/slapDrone_perch/devel/lib/libdecode_msgs.so

.PHONY : quadrotor_msgs/CMakeFiles/decode_msgs.dir/build

quadrotor_msgs/CMakeFiles/decode_msgs.dir/clean:
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && $(CMAKE_COMMAND) -P CMakeFiles/decode_msgs.dir/cmake_clean.cmake
.PHONY : quadrotor_msgs/CMakeFiles/decode_msgs.dir/clean

quadrotor_msgs/CMakeFiles/decode_msgs.dir/depend:
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dawn/Drone/SlapDrone/slapDrone_perch/src /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs /home/dawn/Drone/SlapDrone/slapDrone_perch/build /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs/CMakeFiles/decode_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : quadrotor_msgs/CMakeFiles/decode_msgs.dir/depend

