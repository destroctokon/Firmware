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
include src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/depend.make

# Include the progress variables for this target.
include src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/flags.make

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/flags.make
src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj: src/lib/launchdetection/LaunchDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj -c /home/aldair/src/Firmware/src/lib/launchdetection/LaunchDetector.cpp

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.i"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/lib/launchdetection/LaunchDetector.cpp > CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.i

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.s"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/lib/launchdetection/LaunchDetector.cpp -o CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.s

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.requires:

.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.requires

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.provides: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.requires
	$(MAKE) -f src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/build.make src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.provides.build
.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.provides

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.provides.build: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj


src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/flags.make
src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj: src/lib/launchdetection/CatapultLaunchMethod.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj -c /home/aldair/src/Firmware/src/lib/launchdetection/CatapultLaunchMethod.cpp

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.i"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/lib/launchdetection/CatapultLaunchMethod.cpp > CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.i

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.s"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/lib/launchdetection/CatapultLaunchMethod.cpp -o CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.s

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.requires:

.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.requires

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.provides: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.requires
	$(MAKE) -f src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/build.make src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.provides.build
.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.provides

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.provides.build: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj


# Object files for target lib__launchdetection
lib__launchdetection_OBJECTS = \
"CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj" \
"CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj"

# External object files for target lib__launchdetection
lib__launchdetection_EXTERNAL_OBJECTS =

src/lib/launchdetection/liblib__launchdetection.a: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj
src/lib/launchdetection/liblib__launchdetection.a: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj
src/lib/launchdetection/liblib__launchdetection.a: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/build.make
src/lib/launchdetection/liblib__launchdetection.a: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library liblib__launchdetection.a"
	cd /home/aldair/src/Firmware/src/lib/launchdetection && $(CMAKE_COMMAND) -P CMakeFiles/lib__launchdetection.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/lib/launchdetection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__launchdetection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/build: src/lib/launchdetection/liblib__launchdetection.a

.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/build

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/requires: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/LaunchDetector.cpp.obj.requires
src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/requires: src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/CatapultLaunchMethod.cpp.obj.requires

.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/requires

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/clean:
	cd /home/aldair/src/Firmware/src/lib/launchdetection && $(CMAKE_COMMAND) -P CMakeFiles/lib__launchdetection.dir/cmake_clean.cmake
.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/clean

src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/lib/launchdetection /home/aldair/src/Firmware /home/aldair/src/Firmware/src/lib/launchdetection /home/aldair/src/Firmware/src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/launchdetection/CMakeFiles/lib__launchdetection.dir/depend

