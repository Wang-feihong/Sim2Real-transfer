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
CMAKE_SOURCE_DIR = /home/wfh/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wfh/catkin_ws/build

# Include any dependencies generated for this target.
include optoforce/optoforce/CMakeFiles/optoforce.dir/depend.make

# Include the progress variables for this target.
include optoforce/optoforce/CMakeFiles/optoforce.dir/progress.make

# Include the compile flags for this target's objects.
include optoforce/optoforce/CMakeFiles/optoforce.dir/flags.make

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o: optoforce/optoforce/CMakeFiles/optoforce.dir/flags.make
optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o: /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wfh/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o -c /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_driver.cpp

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.i"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_driver.cpp > CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.i

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.s"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_driver.cpp -o CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.s

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o: optoforce/optoforce/CMakeFiles/optoforce.dir/flags.make
optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o: /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_array_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wfh/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o -c /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_array_driver.cpp

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.i"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_array_driver.cpp > CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.i

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.s"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_array_driver.cpp -o CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.s

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o: optoforce/optoforce/CMakeFiles/optoforce.dir/flags.make
optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o: /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_acquisition.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wfh/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o -c /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_acquisition.cpp

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.i"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_acquisition.cpp > CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.i

optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.s"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wfh/catkin_ws/src/optoforce/optoforce/src/optoforce_acquisition.cpp -o CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.s

# Object files for target optoforce
optoforce_OBJECTS = \
"CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o" \
"CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o" \
"CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o"

# External object files for target optoforce
optoforce_EXTERNAL_OBJECTS =

/home/wfh/catkin_ws/devel/lib/liboptoforce.a: optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_driver.cpp.o
/home/wfh/catkin_ws/devel/lib/liboptoforce.a: optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_array_driver.cpp.o
/home/wfh/catkin_ws/devel/lib/liboptoforce.a: optoforce/optoforce/CMakeFiles/optoforce.dir/src/optoforce_acquisition.cpp.o
/home/wfh/catkin_ws/devel/lib/liboptoforce.a: optoforce/optoforce/CMakeFiles/optoforce.dir/build.make
/home/wfh/catkin_ws/devel/lib/liboptoforce.a: optoforce/optoforce/CMakeFiles/optoforce.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wfh/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library /home/wfh/catkin_ws/devel/lib/liboptoforce.a"
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && $(CMAKE_COMMAND) -P CMakeFiles/optoforce.dir/cmake_clean_target.cmake
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optoforce.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optoforce/optoforce/CMakeFiles/optoforce.dir/build: /home/wfh/catkin_ws/devel/lib/liboptoforce.a

.PHONY : optoforce/optoforce/CMakeFiles/optoforce.dir/build

optoforce/optoforce/CMakeFiles/optoforce.dir/clean:
	cd /home/wfh/catkin_ws/build/optoforce/optoforce && $(CMAKE_COMMAND) -P CMakeFiles/optoforce.dir/cmake_clean.cmake
.PHONY : optoforce/optoforce/CMakeFiles/optoforce.dir/clean

optoforce/optoforce/CMakeFiles/optoforce.dir/depend:
	cd /home/wfh/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wfh/catkin_ws/src /home/wfh/catkin_ws/src/optoforce/optoforce /home/wfh/catkin_ws/build /home/wfh/catkin_ws/build/optoforce/optoforce /home/wfh/catkin_ws/build/optoforce/optoforce/CMakeFiles/optoforce.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : optoforce/optoforce/CMakeFiles/optoforce.dir/depend

