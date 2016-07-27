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
include src/drivers/bst/CMakeFiles/drivers__bst.dir/depend.make

# Include the progress variables for this target.
include src/drivers/bst/CMakeFiles/drivers__bst.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/bst/CMakeFiles/drivers__bst.dir/flags.make

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj: src/drivers/bst/CMakeFiles/drivers__bst.dir/flags.make
src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj: src/drivers/bst/bst.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/bst && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__bst.dir/bst.cpp.obj -c /home/aldair/src/Firmware/src/drivers/bst/bst.cpp

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__bst.dir/bst.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/bst && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/bst/bst.cpp > CMakeFiles/drivers__bst.dir/bst.cpp.i

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__bst.dir/bst.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/bst && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/bst/bst.cpp -o CMakeFiles/drivers__bst.dir/bst.cpp.s

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.requires:

.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.requires

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.provides: src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.requires
	$(MAKE) -f src/drivers/bst/CMakeFiles/drivers__bst.dir/build.make src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.provides.build
.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.provides

src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.provides.build: src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj


# Object files for target drivers__bst
drivers__bst_OBJECTS = \
"CMakeFiles/drivers__bst.dir/bst.cpp.obj"

# External object files for target drivers__bst
drivers__bst_EXTERNAL_OBJECTS =

src/drivers/bst/libdrivers__bst.a: src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj
src/drivers/bst/libdrivers__bst.a: src/drivers/bst/CMakeFiles/drivers__bst.dir/build.make
src/drivers/bst/libdrivers__bst.a: src/drivers/bst/CMakeFiles/drivers__bst.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__bst.a"
	cd /home/aldair/src/Firmware/src/drivers/bst && $(CMAKE_COMMAND) -P CMakeFiles/drivers__bst.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/drivers/bst && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__bst.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/bst/CMakeFiles/drivers__bst.dir/build: src/drivers/bst/libdrivers__bst.a

.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/build

src/drivers/bst/CMakeFiles/drivers__bst.dir/requires: src/drivers/bst/CMakeFiles/drivers__bst.dir/bst.cpp.obj.requires

.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/requires

src/drivers/bst/CMakeFiles/drivers__bst.dir/clean:
	cd /home/aldair/src/Firmware/src/drivers/bst && $(CMAKE_COMMAND) -P CMakeFiles/drivers__bst.dir/cmake_clean.cmake
.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/clean

src/drivers/bst/CMakeFiles/drivers__bst.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/bst /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/bst /home/aldair/src/Firmware/src/drivers/bst/CMakeFiles/drivers__bst.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/bst/CMakeFiles/drivers__bst.dir/depend

