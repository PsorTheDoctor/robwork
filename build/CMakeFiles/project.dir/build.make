# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/adam/robwork

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adam/robwork/build

# Include any dependencies generated for this target.
include CMakeFiles/project.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/project.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/project.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/project.dir/flags.make

CMakeFiles/project.dir/src/project/project.cpp.o: CMakeFiles/project.dir/flags.make
CMakeFiles/project.dir/src/project/project.cpp.o: /home/adam/robwork/src/project/project.cpp
CMakeFiles/project.dir/src/project/project.cpp.o: CMakeFiles/project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adam/robwork/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/project.dir/src/project/project.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/project.dir/src/project/project.cpp.o -MF CMakeFiles/project.dir/src/project/project.cpp.o.d -o CMakeFiles/project.dir/src/project/project.cpp.o -c /home/adam/robwork/src/project/project.cpp

CMakeFiles/project.dir/src/project/project.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/project.dir/src/project/project.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adam/robwork/src/project/project.cpp > CMakeFiles/project.dir/src/project/project.cpp.i

CMakeFiles/project.dir/src/project/project.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/project.dir/src/project/project.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adam/robwork/src/project/project.cpp -o CMakeFiles/project.dir/src/project/project.cpp.s

# Object files for target project
project_OBJECTS = \
"CMakeFiles/project.dir/src/project/project.cpp.o"

# External object files for target project
project_EXTERNAL_OBJECTS =

project: CMakeFiles/project.dir/src/project/project.cpp.o
project: CMakeFiles/project.dir/build.make
project: /usr/lib/x86_64-linux-gnu/libxerces-c.so
project: /usr/lib/x86_64-linux-gnu/libOpenGL.so
project: /usr/lib/x86_64-linux-gnu/libGLX.so
project: /usr/lib/x86_64-linux-gnu/libGLU.so
project: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_csgjs.a
project: /usr/lib/x86_64-linux-gnu/libsdurw_pathplanners.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_pathoptimization.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_simulation.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_opengl.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_assembly.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_task.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_calibration.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_csg.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_control.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_proximitystrategies.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_plugin.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_graspplanning.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_loaders.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_pathplanning.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libfcl.so
project: /usr/lib/x86_64-linux-gnu/libassimp.so
project: /usr/lib/x86_64-linux-gnu/libdl.so
project: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libsdurw_algorithms.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libyaobi.so.1
project: /usr/lib/x86_64-linux-gnu/libpqp.so.1
project: /usr/lib/x86_64-linux-gnu/libOpenGL.so
project: /usr/lib/x86_64-linux-gnu/libGLX.so
project: /usr/lib/x86_64-linux-gnu/libGLU.so
project: /usr/lib/x86_64-linux-gnu/libxerces-c.so
project: /usr/lib/x86_64-linux-gnu/libsdurw_graphics.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_invkin.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_trajectory.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_proximity.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_models.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_sensor.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_geometry.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_kinematics.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_math.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_common.so.2.5.6
project: /usr/lib/x86_64-linux-gnu/libsdurw_core.so.2.5.6
project: /usr/lib/gcc/x86_64-linux-gnu/9/libgomp.so
project: /usr/lib/x86_64-linux-gnu/libpthread.so
project: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
project: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
project: CMakeFiles/project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/adam/robwork/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable project"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/project.dir/build: project
.PHONY : CMakeFiles/project.dir/build

CMakeFiles/project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/project.dir/clean

CMakeFiles/project.dir/depend:
	cd /home/adam/robwork/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adam/robwork /home/adam/robwork /home/adam/robwork/build /home/adam/robwork/build /home/adam/robwork/build/CMakeFiles/project.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/project.dir/depend
