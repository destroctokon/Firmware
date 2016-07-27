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
include src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/depend.make

# Include the progress variables for this target.
include src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/flags.make

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/flags.make
src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj: src/drivers/airspeed/airspeed.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/airspeed && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj -c /home/aldair/src/Firmware/src/drivers/airspeed/airspeed.cpp

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__airspeed.dir/airspeed.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/airspeed && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/airspeed/airspeed.cpp > CMakeFiles/drivers__airspeed.dir/airspeed.cpp.i

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__airspeed.dir/airspeed.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/airspeed && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/airspeed/airspeed.cpp -o CMakeFiles/drivers__airspeed.dir/airspeed.cpp.s

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.requires:

.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.requires

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.provides: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.requires
	$(MAKE) -f src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/build.make src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.provides.build
.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.provides

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.provides.build: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj


# Object files for target drivers__airspeed
drivers__airspeed_OBJECTS = \
"CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj"

# External object files for target drivers__airspeed
drivers__airspeed_EXTERNAL_OBJECTS =

src/drivers/airspeed/libdrivers__airspeed.a: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj
src/drivers/airspeed/libdrivers__airspeed.a: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/build.make
src/drivers/airspeed/libdrivers__airspeed.a: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__airspeed.a"
	cd /home/aldair/src/Firmware/src/drivers/airspeed && $(CMAKE_COMMAND) -P CMakeFiles/drivers__airspeed.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/drivers/airspeed && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__airspeed.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/build: src/drivers/airspeed/libdrivers__airspeed.a

.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/build

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/requires: src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/airspeed.cpp.obj.requires

.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/requires

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/clean:
	cd /home/aldair/src/Firmware/src/drivers/airspeed && $(CMAKE_COMMAND) -P CMakeFiles/drivers__airspeed.dir/cmake_clean.cmake
.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/clean

src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/airspeed /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/airspeed /home/aldair/src/Firmware/src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/airspeed/CMakeFiles/drivers__airspeed.dir/depend

