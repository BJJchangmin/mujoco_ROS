# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/ycm/mujoco/mujoco_ROS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ycm/mujoco/mujoco_ROS/build

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ycm/mujoco/mujoco_ROS/build/CMakeFiles /home/ycm/mujoco/mujoco_ROS/build/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ycm/mujoco/mujoco_ROS/build/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named simulation

# Build rule for target.
simulation: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 simulation
.PHONY : simulation

# fast build rule for target.
simulation/fast:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/build
.PHONY : simulation/fast

src/controller/TrackingController.o: src/controller/TrackingController.cpp.o

.PHONY : src/controller/TrackingController.o

# target to build an object file
src/controller/TrackingController.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/controller/TrackingController.cpp.o
.PHONY : src/controller/TrackingController.cpp.o

src/controller/TrackingController.i: src/controller/TrackingController.cpp.i

.PHONY : src/controller/TrackingController.i

# target to preprocess a source file
src/controller/TrackingController.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/controller/TrackingController.cpp.i
.PHONY : src/controller/TrackingController.cpp.i

src/controller/TrackingController.s: src/controller/TrackingController.cpp.s

.PHONY : src/controller/TrackingController.s

# target to generate assembly for a file
src/controller/TrackingController.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/controller/TrackingController.cpp.s
.PHONY : src/controller/TrackingController.cpp.s

src/glfw_adapter.o: src/glfw_adapter.cc.o

.PHONY : src/glfw_adapter.o

# target to build an object file
src/glfw_adapter.cc.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_adapter.cc.o
.PHONY : src/glfw_adapter.cc.o

src/glfw_adapter.i: src/glfw_adapter.cc.i

.PHONY : src/glfw_adapter.i

# target to preprocess a source file
src/glfw_adapter.cc.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_adapter.cc.i
.PHONY : src/glfw_adapter.cc.i

src/glfw_adapter.s: src/glfw_adapter.cc.s

.PHONY : src/glfw_adapter.s

# target to generate assembly for a file
src/glfw_adapter.cc.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_adapter.cc.s
.PHONY : src/glfw_adapter.cc.s

src/glfw_dispatch.o: src/glfw_dispatch.cc.o

.PHONY : src/glfw_dispatch.o

# target to build an object file
src/glfw_dispatch.cc.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_dispatch.cc.o
.PHONY : src/glfw_dispatch.cc.o

src/glfw_dispatch.i: src/glfw_dispatch.cc.i

.PHONY : src/glfw_dispatch.i

# target to preprocess a source file
src/glfw_dispatch.cc.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_dispatch.cc.i
.PHONY : src/glfw_dispatch.cc.i

src/glfw_dispatch.s: src/glfw_dispatch.cc.s

.PHONY : src/glfw_dispatch.s

# target to generate assembly for a file
src/glfw_dispatch.cc.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/glfw_dispatch.cc.s
.PHONY : src/glfw_dispatch.cc.s

src/lodepng.o: src/lodepng.cpp.o

.PHONY : src/lodepng.o

# target to build an object file
src/lodepng.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/lodepng.cpp.o
.PHONY : src/lodepng.cpp.o

src/lodepng.i: src/lodepng.cpp.i

.PHONY : src/lodepng.i

# target to preprocess a source file
src/lodepng.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/lodepng.cpp.i
.PHONY : src/lodepng.cpp.i

src/lodepng.s: src/lodepng.cpp.s

.PHONY : src/lodepng.s

# target to generate assembly for a file
src/lodepng.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/lodepng.cpp.s
.PHONY : src/lodepng.cpp.s

src/main.o: src/main.cc.o

.PHONY : src/main.o

# target to build an object file
src/main.cc.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/main.cc.o
.PHONY : src/main.cc.o

src/main.i: src/main.cc.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cc.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/main.cc.i
.PHONY : src/main.cc.i

src/main.s: src/main.cc.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cc.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/main.cc.s
.PHONY : src/main.cc.s

src/platform_ui_adapter.o: src/platform_ui_adapter.cc.o

.PHONY : src/platform_ui_adapter.o

# target to build an object file
src/platform_ui_adapter.cc.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/platform_ui_adapter.cc.o
.PHONY : src/platform_ui_adapter.cc.o

src/platform_ui_adapter.i: src/platform_ui_adapter.cc.i

.PHONY : src/platform_ui_adapter.i

# target to preprocess a source file
src/platform_ui_adapter.cc.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/platform_ui_adapter.cc.i
.PHONY : src/platform_ui_adapter.cc.i

src/platform_ui_adapter.s: src/platform_ui_adapter.cc.s

.PHONY : src/platform_ui_adapter.s

# target to generate assembly for a file
src/platform_ui_adapter.cc.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/platform_ui_adapter.cc.s
.PHONY : src/platform_ui_adapter.cc.s

src/robot/MuJoCoInterface.o: src/robot/MuJoCoInterface.cpp.o

.PHONY : src/robot/MuJoCoInterface.o

# target to build an object file
src/robot/MuJoCoInterface.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/MuJoCoInterface.cpp.o
.PHONY : src/robot/MuJoCoInterface.cpp.o

src/robot/MuJoCoInterface.i: src/robot/MuJoCoInterface.cpp.i

.PHONY : src/robot/MuJoCoInterface.i

# target to preprocess a source file
src/robot/MuJoCoInterface.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/MuJoCoInterface.cpp.i
.PHONY : src/robot/MuJoCoInterface.cpp.i

src/robot/MuJoCoInterface.s: src/robot/MuJoCoInterface.cpp.s

.PHONY : src/robot/MuJoCoInterface.s

# target to generate assembly for a file
src/robot/MuJoCoInterface.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/MuJoCoInterface.cpp.s
.PHONY : src/robot/MuJoCoInterface.cpp.s

src/robot/RobotLeg.o: src/robot/RobotLeg.cpp.o

.PHONY : src/robot/RobotLeg.o

# target to build an object file
src/robot/RobotLeg.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/RobotLeg.cpp.o
.PHONY : src/robot/RobotLeg.cpp.o

src/robot/RobotLeg.i: src/robot/RobotLeg.cpp.i

.PHONY : src/robot/RobotLeg.i

# target to preprocess a source file
src/robot/RobotLeg.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/RobotLeg.cpp.i
.PHONY : src/robot/RobotLeg.cpp.i

src/robot/RobotLeg.s: src/robot/RobotLeg.cpp.s

.PHONY : src/robot/RobotLeg.s

# target to generate assembly for a file
src/robot/RobotLeg.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/robot/RobotLeg.cpp.s
.PHONY : src/robot/RobotLeg.cpp.s

src/simulate.o: src/simulate.cc.o

.PHONY : src/simulate.o

# target to build an object file
src/simulate.cc.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/simulate.cc.o
.PHONY : src/simulate.cc.o

src/simulate.i: src/simulate.cc.i

.PHONY : src/simulate.i

# target to preprocess a source file
src/simulate.cc.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/simulate.cc.i
.PHONY : src/simulate.cc.i

src/simulate.s: src/simulate.cc.s

.PHONY : src/simulate.s

# target to generate assembly for a file
src/simulate.cc.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/simulate.cc.s
.PHONY : src/simulate.cc.s

src/trajectory/MotionTrajectory.o: src/trajectory/MotionTrajectory.cpp.o

.PHONY : src/trajectory/MotionTrajectory.o

# target to build an object file
src/trajectory/MotionTrajectory.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/trajectory/MotionTrajectory.cpp.o
.PHONY : src/trajectory/MotionTrajectory.cpp.o

src/trajectory/MotionTrajectory.i: src/trajectory/MotionTrajectory.cpp.i

.PHONY : src/trajectory/MotionTrajectory.i

# target to preprocess a source file
src/trajectory/MotionTrajectory.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/trajectory/MotionTrajectory.cpp.i
.PHONY : src/trajectory/MotionTrajectory.cpp.i

src/trajectory/MotionTrajectory.s: src/trajectory/MotionTrajectory.cpp.s

.PHONY : src/trajectory/MotionTrajectory.s

# target to generate assembly for a file
src/trajectory/MotionTrajectory.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/trajectory/MotionTrajectory.cpp.s
.PHONY : src/trajectory/MotionTrajectory.cpp.s

src/utilities/data_logging.o: src/utilities/data_logging.cpp.o

.PHONY : src/utilities/data_logging.o

# target to build an object file
src/utilities/data_logging.cpp.o:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/utilities/data_logging.cpp.o
.PHONY : src/utilities/data_logging.cpp.o

src/utilities/data_logging.i: src/utilities/data_logging.cpp.i

.PHONY : src/utilities/data_logging.i

# target to preprocess a source file
src/utilities/data_logging.cpp.i:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/utilities/data_logging.cpp.i
.PHONY : src/utilities/data_logging.cpp.i

src/utilities/data_logging.s: src/utilities/data_logging.cpp.s

.PHONY : src/utilities/data_logging.s

# target to generate assembly for a file
src/utilities/data_logging.cpp.s:
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/utilities/data_logging.cpp.s
.PHONY : src/utilities/data_logging.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... simulation"
	@echo "... src/controller/TrackingController.o"
	@echo "... src/controller/TrackingController.i"
	@echo "... src/controller/TrackingController.s"
	@echo "... src/glfw_adapter.o"
	@echo "... src/glfw_adapter.i"
	@echo "... src/glfw_adapter.s"
	@echo "... src/glfw_dispatch.o"
	@echo "... src/glfw_dispatch.i"
	@echo "... src/glfw_dispatch.s"
	@echo "... src/lodepng.o"
	@echo "... src/lodepng.i"
	@echo "... src/lodepng.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/platform_ui_adapter.o"
	@echo "... src/platform_ui_adapter.i"
	@echo "... src/platform_ui_adapter.s"
	@echo "... src/robot/MuJoCoInterface.o"
	@echo "... src/robot/MuJoCoInterface.i"
	@echo "... src/robot/MuJoCoInterface.s"
	@echo "... src/robot/RobotLeg.o"
	@echo "... src/robot/RobotLeg.i"
	@echo "... src/robot/RobotLeg.s"
	@echo "... src/simulate.o"
	@echo "... src/simulate.i"
	@echo "... src/simulate.s"
	@echo "... src/trajectory/MotionTrajectory.o"
	@echo "... src/trajectory/MotionTrajectory.i"
	@echo "... src/trajectory/MotionTrajectory.s"
	@echo "... src/utilities/data_logging.o"
	@echo "... src/utilities/data_logging.i"
	@echo "... src/utilities/data_logging.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

