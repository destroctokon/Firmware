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
include src/modules/commander/CMakeFiles/modules__commander.dir/depend.make

# Include the progress variables for this target.
include src/modules/commander/CMakeFiles/modules__commander.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/commander/CMakeFiles/modules__commander.dir/flags.make

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj: src/modules/commander/commander.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/commander.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/commander.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/commander.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/commander.cpp > CMakeFiles/modules__commander.dir/commander.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/commander.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/commander.cpp -o CMakeFiles/modules__commander.dir/commander.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj: src/modules/commander/state_machine_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/state_machine_helper.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/state_machine_helper.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/state_machine_helper.cpp > CMakeFiles/modules__commander.dir/state_machine_helper.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/state_machine_helper.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/state_machine_helper.cpp -o CMakeFiles/modules__commander.dir/state_machine_helper.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj: src/modules/commander/commander_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/commander_helper.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/commander_helper.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/commander_helper.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/commander_helper.cpp > CMakeFiles/modules__commander.dir/commander_helper.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/commander_helper.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/commander_helper.cpp -o CMakeFiles/modules__commander.dir/commander_helper.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj: src/modules/commander/calibration_routines.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/calibration_routines.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/calibration_routines.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/calibration_routines.cpp > CMakeFiles/modules__commander.dir/calibration_routines.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/calibration_routines.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/calibration_routines.cpp -o CMakeFiles/modules__commander.dir/calibration_routines.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj: src/modules/commander/accelerometer_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/accelerometer_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/accelerometer_calibration.cpp > CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/accelerometer_calibration.cpp -o CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj: src/modules/commander/gyro_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/gyro_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/gyro_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/gyro_calibration.cpp > CMakeFiles/modules__commander.dir/gyro_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/gyro_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/gyro_calibration.cpp -o CMakeFiles/modules__commander.dir/gyro_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj: src/modules/commander/mag_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/mag_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/mag_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/mag_calibration.cpp > CMakeFiles/modules__commander.dir/mag_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/mag_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/mag_calibration.cpp -o CMakeFiles/modules__commander.dir/mag_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj: src/modules/commander/baro_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/baro_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/baro_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/baro_calibration.cpp > CMakeFiles/modules__commander.dir/baro_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/baro_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/baro_calibration.cpp -o CMakeFiles/modules__commander.dir/baro_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj: src/modules/commander/rc_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/rc_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/rc_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/rc_calibration.cpp > CMakeFiles/modules__commander.dir/rc_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/rc_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/rc_calibration.cpp -o CMakeFiles/modules__commander.dir/rc_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj: src/modules/commander/airspeed_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/airspeed_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/airspeed_calibration.cpp > CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/airspeed_calibration.cpp -o CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj: src/modules/commander/esc_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/esc_calibration.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/esc_calibration.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/esc_calibration.cpp > CMakeFiles/modules__commander.dir/esc_calibration.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/esc_calibration.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/esc_calibration.cpp -o CMakeFiles/modules__commander.dir/esc_calibration.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj


src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj: src/modules/commander/CMakeFiles/modules__commander.dir/flags.make
src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj: src/modules/commander/PreflightCheck.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj -c /home/aldair/src/Firmware/src/modules/commander/PreflightCheck.cpp

src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander.dir/PreflightCheck.cpp.i"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/modules/commander/PreflightCheck.cpp > CMakeFiles/modules__commander.dir/PreflightCheck.cpp.i

src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander.dir/PreflightCheck.cpp.s"
	cd /home/aldair/src/Firmware/src/modules/commander && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/modules/commander/PreflightCheck.cpp -o CMakeFiles/modules__commander.dir/PreflightCheck.cpp.s

src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.requires:

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.requires

src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.provides: src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.requires
	$(MAKE) -f src/modules/commander/CMakeFiles/modules__commander.dir/build.make src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.provides.build
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.provides

src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.provides.build: src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj


# Object files for target modules__commander
modules__commander_OBJECTS = \
"CMakeFiles/modules__commander.dir/commander.cpp.obj" \
"CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj" \
"CMakeFiles/modules__commander.dir/commander_helper.cpp.obj" \
"CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj" \
"CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj" \
"CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj"

# External object files for target modules__commander
modules__commander_EXTERNAL_OBJECTS =

src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/build.make
src/modules/commander/libmodules__commander.a: src/modules/commander/CMakeFiles/modules__commander.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking CXX static library libmodules__commander.a"
	cd /home/aldair/src/Firmware/src/modules/commander && $(CMAKE_COMMAND) -P CMakeFiles/modules__commander.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/modules/commander && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__commander.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/commander/CMakeFiles/modules__commander.dir/build: src/modules/commander/libmodules__commander.a

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/build

src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/commander.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/state_machine_helper.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/commander_helper.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/calibration_routines.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/accelerometer_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/gyro_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/mag_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/baro_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/rc_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/airspeed_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/esc_calibration.cpp.obj.requires
src/modules/commander/CMakeFiles/modules__commander.dir/requires: src/modules/commander/CMakeFiles/modules__commander.dir/PreflightCheck.cpp.obj.requires

.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/requires

src/modules/commander/CMakeFiles/modules__commander.dir/clean:
	cd /home/aldair/src/Firmware/src/modules/commander && $(CMAKE_COMMAND) -P CMakeFiles/modules__commander.dir/cmake_clean.cmake
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/clean

src/modules/commander/CMakeFiles/modules__commander.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/modules/commander /home/aldair/src/Firmware /home/aldair/src/Firmware/src/modules/commander /home/aldair/src/Firmware/src/modules/commander/CMakeFiles/modules__commander.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/commander/CMakeFiles/modules__commander.dir/depend

