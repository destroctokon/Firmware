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

# Utility rule file for git_jmavsim.

# Include the progress variables for this target.
include CMakeFiles/git_jmavsim.dir/progress.make

CMakeFiles/git_jmavsim: git_init_Tools_jMAVSim.stamp


git_init_Tools_jMAVSim.stamp: .gitmodules
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating git_init_Tools_jMAVSim.stamp"
	touch /home/aldair/src/Firmware/git_init_Tools_jMAVSim.stamp

git_jmavsim: CMakeFiles/git_jmavsim
git_jmavsim: git_init_Tools_jMAVSim.stamp
git_jmavsim: CMakeFiles/git_jmavsim.dir/build.make

.PHONY : git_jmavsim

# Rule to build all files generated by this target.
CMakeFiles/git_jmavsim.dir/build: git_jmavsim

.PHONY : CMakeFiles/git_jmavsim.dir/build

CMakeFiles/git_jmavsim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/git_jmavsim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/git_jmavsim.dir/clean

CMakeFiles/git_jmavsim.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware /home/aldair/src/Firmware /home/aldair/src/Firmware /home/aldair/src/Firmware/CMakeFiles/git_jmavsim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/git_jmavsim.dir/depend

