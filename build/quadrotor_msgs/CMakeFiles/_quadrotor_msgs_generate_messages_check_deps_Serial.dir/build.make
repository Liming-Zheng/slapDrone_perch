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

# Utility rule file for _quadrotor_msgs_generate_messages_check_deps_Serial.

# Include the progress variables for this target.
include quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/progress.make

quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial:
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && ../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py quadrotor_msgs /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs/msg/Serial.msg std_msgs/Header

_quadrotor_msgs_generate_messages_check_deps_Serial: quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial
_quadrotor_msgs_generate_messages_check_deps_Serial: quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/build.make

.PHONY : _quadrotor_msgs_generate_messages_check_deps_Serial

# Rule to build all files generated by this target.
quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/build: _quadrotor_msgs_generate_messages_check_deps_Serial

.PHONY : quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/build

quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/clean:
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs && $(CMAKE_COMMAND) -P CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/cmake_clean.cmake
.PHONY : quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/clean

quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/depend:
	cd /home/dawn/Drone/SlapDrone/slapDrone_perch/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dawn/Drone/SlapDrone/slapDrone_perch/src /home/dawn/Drone/SlapDrone/slapDrone_perch/src/quadrotor_msgs /home/dawn/Drone/SlapDrone/slapDrone_perch/build /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs /home/dawn/Drone/SlapDrone/slapDrone_perch/build/quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : quadrotor_msgs/CMakeFiles/_quadrotor_msgs_generate_messages_check_deps_Serial.dir/depend

