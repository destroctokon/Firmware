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
include src/drivers/px4io/CMakeFiles/drivers__px4io.dir/depend.make

# Include the progress variables for this target.
include src/drivers/px4io/CMakeFiles/drivers__px4io.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj: src/drivers/px4io/px4io.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4io.dir/px4io.cpp.obj -c /home/aldair/src/Firmware/src/drivers/px4io/px4io.cpp

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4io.dir/px4io.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/px4io/px4io.cpp > CMakeFiles/drivers__px4io.dir/px4io.cpp.i

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4io.dir/px4io.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/px4io/px4io.cpp -o CMakeFiles/drivers__px4io.dir/px4io.cpp.s

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.requires:

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.provides: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.requires
	$(MAKE) -f src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.provides.build
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.provides

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.provides.build: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj


src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj: src/drivers/px4io/px4io_uploader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj -c /home/aldair/src/Firmware/src/drivers/px4io/px4io_uploader.cpp

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/px4io/px4io_uploader.cpp > CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.i

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/px4io/px4io_uploader.cpp -o CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.s

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.requires:

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.provides: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.requires
	$(MAKE) -f src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.provides.build
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.provides

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.provides.build: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj


src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj: src/drivers/px4io/px4io_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj -c /home/aldair/src/Firmware/src/drivers/px4io/px4io_serial.cpp

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/px4io/px4io_serial.cpp > CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.i

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/px4io/px4io_serial.cpp -o CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.s

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.requires:

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.provides: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.requires
	$(MAKE) -f src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.provides.build
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.provides

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.provides.build: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj


src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj: src/drivers/px4io/px4io_i2c.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj -c /home/aldair/src/Firmware/src/drivers/px4io/px4io_i2c.cpp

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.i"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldair/src/Firmware/src/drivers/px4io/px4io_i2c.cpp > CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.i

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.s"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldair/src/Firmware/src/drivers/px4io/px4io_i2c.cpp -o CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.s

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.requires:

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.provides: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.requires
	$(MAKE) -f src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.provides.build
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.provides

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.provides.build: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj


src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/flags.make
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj: src/drivers/px4io/px4io_params.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__px4io.dir/px4io_params.c.obj   -c /home/aldair/src/Firmware/src/drivers/px4io/px4io_params.c

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__px4io.dir/px4io_params.c.i"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/px4io/px4io_params.c > CMakeFiles/drivers__px4io.dir/px4io_params.c.i

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__px4io.dir/px4io_params.c.s"
	cd /home/aldair/src/Firmware/src/drivers/px4io && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/px4io/px4io_params.c -o CMakeFiles/drivers__px4io.dir/px4io_params.c.s

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.requires:

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.provides: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.requires
	$(MAKE) -f src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.provides.build
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.provides

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.provides.build: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj


# Object files for target drivers__px4io
drivers__px4io_OBJECTS = \
"CMakeFiles/drivers__px4io.dir/px4io.cpp.obj" \
"CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj" \
"CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj" \
"CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj" \
"CMakeFiles/drivers__px4io.dir/px4io_params.c.obj"

# External object files for target drivers__px4io
drivers__px4io_EXTERNAL_OBJECTS =

src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build.make
src/drivers/px4io/libdrivers__px4io.a: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libdrivers__px4io.a"
	cd /home/aldair/src/Firmware/src/drivers/px4io && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4io.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/drivers/px4io && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__px4io.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build: src/drivers/px4io/libdrivers__px4io.a

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/build

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io.cpp.obj.requires
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_uploader.cpp.obj.requires
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_serial.cpp.obj.requires
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_i2c.cpp.obj.requires
src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires: src/drivers/px4io/CMakeFiles/drivers__px4io.dir/px4io_params.c.obj.requires

.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/requires

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/clean:
	cd /home/aldair/src/Firmware/src/drivers/px4io && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4io.dir/cmake_clean.cmake
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/clean

src/drivers/px4io/CMakeFiles/drivers__px4io.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/px4io /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/px4io /home/aldair/src/Firmware/src/drivers/px4io/CMakeFiles/drivers__px4io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/px4io/CMakeFiles/drivers__px4io.dir/depend

