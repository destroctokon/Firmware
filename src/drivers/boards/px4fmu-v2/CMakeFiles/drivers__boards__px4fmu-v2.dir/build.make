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
include src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/depend.make

# Include the progress variables for this target.
include src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj: src/drivers/boards/common/board_name.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/common/board_name.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/common/board_name.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/common/board_name.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj: src/drivers/boards/common/board_dma_alloc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/common/board_dma_alloc.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/common/board_dma_alloc.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/common/board_dma_alloc.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj: src/drivers/boards/px4fmu-v2/px4fmu_can.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_can.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_can.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_can.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj: src/drivers/boards/px4fmu-v2/px4fmu2_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_init.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_init.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_init.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj: src/drivers/boards/px4fmu-v2/px4fmu_timer_config.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_timer_config.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_timer_config.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_timer_config.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj: src/drivers/boards/px4fmu-v2/px4fmu_spi.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_spi.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_spi.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_spi.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj: src/drivers/boards/px4fmu-v2/px4fmu_usb.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_usb.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_usb.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu_usb.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj


src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/flags.make
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj: src/drivers/boards/px4fmu-v2/px4fmu2_led.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj   -c /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_led.c

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.i"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_led.c > CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.i

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.s"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/px4fmu2_led.c -o CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.s

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.requires:

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.provides: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.requires
	$(MAKE) -f src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.provides.build
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.provides

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.provides.build: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj


# Object files for target drivers__boards__px4fmu-v2
drivers__boards__px4fmu__v2_OBJECTS = \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj" \
"CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj"

# External object files for target drivers__boards__px4fmu-v2
drivers__boards__px4fmu__v2_EXTERNAL_OBJECTS =

src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build.make
src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldair/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking C static library libdrivers__boards__px4fmu-v2.a"
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__boards__px4fmu-v2.dir/cmake_clean_target.cmake
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__boards__px4fmu-v2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build: src/drivers/boards/px4fmu-v2/libdrivers__boards__px4fmu-v2.a

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/build

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_name.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/__/common/board_dma_alloc.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_can.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_init.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_timer_config.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_spi.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu_usb.c.obj.requires
src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires: src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/px4fmu2_led.c.obj.requires

.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/requires

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/clean:
	cd /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__boards__px4fmu-v2.dir/cmake_clean.cmake
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/clean

src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/depend:
	cd /home/aldair/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 /home/aldair/src/Firmware /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2 /home/aldair/src/Firmware/src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/boards/px4fmu-v2/CMakeFiles/drivers__boards__px4fmu-v2.dir/depend

