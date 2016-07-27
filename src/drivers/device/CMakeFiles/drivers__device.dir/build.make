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
include src/drivers/device/CMakeFiles/drivers__device.dir/depend.make

# Include the progress variables for this target.
include src/drivers/device/CMakeFiles/drivers__device.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/device/CMakeFiles/drivers__device.dir/flags.make

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj: src/drivers/device/ringbuffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/ringbuffer.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/ringbuffer.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/ringbuffer.cpp > CMakeFiles/drivers__device.dir/ringbuffer.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/ringbuffer.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/ringbuffer.cpp -o CMakeFiles/drivers__device.dir/ringbuffer.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj: src/drivers/device/integrator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/integrator.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/integrator.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/integrator.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/integrator.cpp > CMakeFiles/drivers__device.dir/integrator.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/integrator.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/integrator.cpp -o CMakeFiles/drivers__device.dir/integrator.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj: src/drivers/device/device_nuttx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/device_nuttx.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/device_nuttx.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/device_nuttx.cpp > CMakeFiles/drivers__device.dir/device_nuttx.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/device_nuttx.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/device_nuttx.cpp -o CMakeFiles/drivers__device.dir/device_nuttx.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj: src/drivers/device/cdev.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/cdev.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/cdev.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/cdev.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/cdev.cpp > CMakeFiles/drivers__device.dir/cdev.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/cdev.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/cdev.cpp -o CMakeFiles/drivers__device.dir/cdev.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj: src/drivers/device/i2c_nuttx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/i2c_nuttx.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/i2c_nuttx.cpp > CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/i2c_nuttx.cpp -o CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj: src/drivers/device/pio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/pio.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/pio.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/pio.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/pio.cpp > CMakeFiles/drivers__device.dir/pio.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/pio.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/pio.cpp -o CMakeFiles/drivers__device.dir/pio.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj


src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj: src/drivers/device/CMakeFiles/drivers__device.dir/flags.make
src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj: src/drivers/device/spi.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__device.dir/spi.cpp.obj -c /home/aldair/src/Firmware/src/drivers/device/spi.cpp

src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__device.dir/spi.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/device/spi.cpp > CMakeFiles/drivers__device.dir/spi.cpp.i

src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__device.dir/spi.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/device && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/device/spi.cpp -o CMakeFiles/drivers__device.dir/spi.cpp.s

src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.requires:

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.requires

src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.provides: src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.requires
	$(MAKE) -f src/drivers/device/CMakeFiles/drivers__device.dir/build.make src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.provides.build
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.provides

src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.provides.build: src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj


# Object files for target drivers__device
drivers__device_OBJECTS = \
"CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj" \
"CMakeFiles/drivers__device.dir/integrator.cpp.obj" \
"CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj" \
"CMakeFiles/drivers__device.dir/cdev.cpp.obj" \
"CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj" \
"CMakeFiles/drivers__device.dir/pio.cpp.obj" \
"CMakeFiles/drivers__device.dir/spi.cpp.obj"

# External object files for target drivers__device
drivers__device_EXTERNAL_OBJECTS =

src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/build.make
src/drivers/device/libdrivers__device.a: src/drivers/device/CMakeFiles/drivers__device.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libdrivers__device.a"
	cd /home/aldair/src/Firmware/src/drivers/device && $(CMAKE_COMMAND) -P CMakeFiles/drivers__device.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/drivers/device && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__device.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/device/CMakeFiles/drivers__device.dir/build: src/drivers/device/libdrivers__device.a

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/build

src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/ringbuffer.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/integrator.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/device_nuttx.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/cdev.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/i2c_nuttx.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/pio.cpp.obj.requires
src/drivers/device/CMakeFiles/drivers__device.dir/requires: src/drivers/device/CMakeFiles/drivers__device.dir/spi.cpp.obj.requires

.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/requires

src/drivers/device/CMakeFiles/drivers__device.dir/clean:
	cd /home/aldair/src/Firmware/src/drivers/device && $(CMAKE_COMMAND) -P CMakeFiles/drivers__device.dir/cmake_clean.cmake
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/clean

src/drivers/device/CMakeFiles/drivers__device.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/device /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/device /home/aldair/src/Firmware/src/drivers/device/CMakeFiles/drivers__device.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/device/CMakeFiles/drivers__device.dir/depend
