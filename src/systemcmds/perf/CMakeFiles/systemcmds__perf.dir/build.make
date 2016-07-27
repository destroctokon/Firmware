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
include src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/flags.make

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/flags.make
src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj: src/systemcmds/perf/perf.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj"
	cd /home/aldair/src/Firmware/src/systemcmds/perf && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/systemcmds__perf.dir/perf.c.obj   -c /home/aldair/src/Firmware/src/systemcmds/perf/perf.c

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/systemcmds__perf.dir/perf.c.i"
	cd /home/aldair/src/Firmware/src/systemcmds/perf && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/systemcmds/perf/perf.c > CMakeFiles/systemcmds__perf.dir/perf.c.i

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/systemcmds__perf.dir/perf.c.s"
	cd /home/aldair/src/Firmware/src/systemcmds/perf && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/systemcmds/perf/perf.c -o CMakeFiles/systemcmds__perf.dir/perf.c.s

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.requires:

.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.requires

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.provides: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.requires
	$(MAKE) -f src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/build.make src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.provides.build
.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.provides

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.provides.build: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj


# Object files for target systemcmds__perf
systemcmds__perf_OBJECTS = \
"CMakeFiles/systemcmds__perf.dir/perf.c.obj"

# External object files for target systemcmds__perf
systemcmds__perf_EXTERNAL_OBJECTS =

src/systemcmds/perf/libsystemcmds__perf.a: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj
src/systemcmds/perf/libsystemcmds__perf.a: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/build.make
src/systemcmds/perf/libsystemcmds__perf.a: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libsystemcmds__perf.a"
	cd /home/aldair/src/Firmware/src/systemcmds/perf && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__perf.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/systemcmds/perf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__perf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/build: src/systemcmds/perf/libsystemcmds__perf.a

.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/build

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/requires: src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/perf.c.obj.requires

.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/requires

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/clean:
	cd /home/aldair/src/Firmware/src/systemcmds/perf && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__perf.dir/cmake_clean.cmake
.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/clean

src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/systemcmds/perf /home/aldair/src/Firmware /home/aldair/src/Firmware/src/systemcmds/perf /home/aldair/src/Firmware/src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/perf/CMakeFiles/systemcmds__perf.dir/depend

