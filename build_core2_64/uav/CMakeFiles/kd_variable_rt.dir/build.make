# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/cmake

# The command to remove a file.
RM = /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ateverz/Documents/projects/kd_variable_x4_flair

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ateverz/Documents/projects/kd_variable_x4_flair/build

# Include any dependencies generated for this target.
include uav/CMakeFiles/kd_variable_rt.dir/depend.make

# Include the progress variables for this target.
include uav/CMakeFiles/kd_variable_rt.dir/progress.make

# Include the compile flags for this target's objects.
include uav/CMakeFiles/kd_variable_rt.dir/flags.make

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o: ../uav/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/main.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/main.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/main.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/main.cpp > CMakeFiles/kd_variable_rt.dir/src/main.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/main.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/main.cpp -o CMakeFiles/kd_variable_rt.dir/src/main.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o: ../uav/src/kd_variable.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/kd_variable.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/kd_variable.cpp > CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/kd_variable.cpp -o CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o: ../uav/src/Sliding.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding.cpp > CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding.cpp -o CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o: ../uav/src/Sliding_pos.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_pos.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_pos.cpp > CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_pos.cpp -o CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o: ../uav/src/Sliding_force.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_force.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_force.cpp > CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/Sliding_force.cpp -o CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o: ../uav/src/NMethods.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/NMethods.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/NMethods.cpp > CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/NMethods.cpp -o CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o: ../uav/src/TargetJR3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetJR3.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetJR3.cpp > CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetJR3.cpp -o CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o: ../uav/src/TargetEthJR3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetEthJR3.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetEthJR3.cpp > CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/TargetEthJR3.cpp -o CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o: ../uav/src/DILWAC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/DILWAC.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/DILWAC.cpp > CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/DILWAC.cpp -o CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o: ../uav/src/ANN.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/ANN.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/ANN.cpp > CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/ANN.cpp -o CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o


uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o: uav/CMakeFiles/kd_variable_rt.dir/flags.make
uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o: ../uav/src/CNN.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o -c /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/CNN.cpp

uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.i"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/CNN.cpp > CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.i

uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.s"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && /opt/robomap3/2.1.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-g++  --sysroot=/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ateverz/Documents/projects/kd_variable_x4_flair/uav/src/CNN.cpp -o CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.s

uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.requires:

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.requires

uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.provides: uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.requires
	$(MAKE) -f uav/CMakeFiles/kd_variable_rt.dir/build.make uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.provides.build
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.provides

uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.provides.build: uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o


# Object files for target kd_variable_rt
kd_variable_rt_OBJECTS = \
"CMakeFiles/kd_variable_rt.dir/src/main.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o" \
"CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o"

# External object files for target kd_variable_rt
kd_variable_rt_EXTERNAL_OBJECTS =

uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/build.make
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairIntelAero.a
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairMeta.a
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairFilter.a
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libHdsSensorActuator.a
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairSensorActuator.a
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairCore_rt.a
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libxml2.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/librtdm.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libnative.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libxenomai.so
uav/kd_variable_rt: /home/ateverz/flair/flair-install/lib/core2-64/libFlairCore_rt.a
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libxml2.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/librtdm.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libnative.so
uav/kd_variable_rt: /opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libxenomai.so
uav/kd_variable_rt: uav/CMakeFiles/kd_variable_rt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ateverz/Documents/projects/kd_variable_x4_flair/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX executable kd_variable_rt"
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kd_variable_rt.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "kd_variable_rt built for core2-64 architecture"

# Rule to build all files generated by this target.
uav/CMakeFiles/kd_variable_rt.dir/build: uav/kd_variable_rt

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/build

uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/main.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/kd_variable.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_pos.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/Sliding_force.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/NMethods.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/TargetJR3.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/TargetEthJR3.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/DILWAC.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/ANN.cpp.o.requires
uav/CMakeFiles/kd_variable_rt.dir/requires: uav/CMakeFiles/kd_variable_rt.dir/src/CNN.cpp.o.requires

.PHONY : uav/CMakeFiles/kd_variable_rt.dir/requires

uav/CMakeFiles/kd_variable_rt.dir/clean:
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav && $(CMAKE_COMMAND) -P CMakeFiles/kd_variable_rt.dir/cmake_clean.cmake
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/clean

uav/CMakeFiles/kd_variable_rt.dir/depend:
	cd /home/ateverz/Documents/projects/kd_variable_x4_flair/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ateverz/Documents/projects/kd_variable_x4_flair /home/ateverz/Documents/projects/kd_variable_x4_flair/uav /home/ateverz/Documents/projects/kd_variable_x4_flair/build /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav /home/ateverz/Documents/projects/kd_variable_x4_flair/build/uav/CMakeFiles/kd_variable_rt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : uav/CMakeFiles/kd_variable_rt.dir/depend

