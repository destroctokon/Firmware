# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/aldair/src/Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aldair/src/Firmware

# Include any dependencies generated for this target.
include src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/depend.make

# Include the progress variables for this target.
include src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/flags.make

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/flags.make
src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj: src/lib/runway_takeoff/RunwayTakeoff.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj"
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj -c /home/aldair/src/Firmware/src/lib/runway_takeoff/RunwayTakeoff.cpp

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.i"
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/lib/runway_takeoff/RunwayTakeoff.cpp > CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.i

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.s"
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/lib/runway_takeoff/RunwayTakeoff.cpp -o CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.s

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.requires:

.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.requires

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.provides: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.requires
	$(MAKE) -f src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/build.make src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.provides.build
.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.provides

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.provides.build: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj


# Object files for target lib__runway_takeoff
lib__runway_takeoff_OBJECTS = \
"CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj"

# External object files for target lib__runway_takeoff
lib__runway_takeoff_EXTERNAL_OBJECTS =

src/lib/runway_takeoff/liblib__runway_takeoff.a: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj
src/lib/runway_takeoff/liblib__runway_takeoff.a: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/build.make
src/lib/runway_takeoff/liblib__runway_takeoff.a: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblib__runway_takeoff.a"
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && $(CMAKE_COMMAND) -P CMakeFiles/lib__runway_takeoff.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__runway_takeoff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/build: src/lib/runway_takeoff/liblib__runway_takeoff.a

.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/build

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/requires: src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/RunwayTakeoff.cpp.obj.requires

.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/requires

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/clean:
	cd /home/aldair/src/Firmware/src/lib/runway_takeoff && $(CMAKE_COMMAND) -P CMakeFiles/lib__runway_takeoff.dir/cmake_clean.cmake
.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/clean

src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/lib/runway_takeoff /home/aldair/src/Firmware /home/aldair/src/Firmware/src/lib/runway_takeoff /home/aldair/src/Firmware/src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/runway_takeoff/CMakeFiles/lib__runway_takeoff.dir/depend

