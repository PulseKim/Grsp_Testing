# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mrl/Test/Grasping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mrl/Test/Grasping/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/Contact.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/Contact.cpp.o: ../Contact.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/Contact.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Contact.cpp.o -c /home/mrl/Test/Grasping/Contact.cpp

CMakeFiles/test.dir/Contact.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Contact.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/Contact.cpp > CMakeFiles/test.dir/Contact.cpp.i

CMakeFiles/test.dir/Contact.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Contact.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/Contact.cpp -o CMakeFiles/test.dir/Contact.cpp.s

CMakeFiles/test.dir/Controller.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/Controller.cpp.o: ../Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test.dir/Controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Controller.cpp.o -c /home/mrl/Test/Grasping/Controller.cpp

CMakeFiles/test.dir/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/Controller.cpp > CMakeFiles/test.dir/Controller.cpp.i

CMakeFiles/test.dir/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/Controller.cpp -o CMakeFiles/test.dir/Controller.cpp.s

CMakeFiles/test.dir/Grasp.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/Grasp.cpp.o: ../Grasp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test.dir/Grasp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Grasp.cpp.o -c /home/mrl/Test/Grasping/Grasp.cpp

CMakeFiles/test.dir/Grasp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Grasp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/Grasp.cpp > CMakeFiles/test.dir/Grasp.cpp.i

CMakeFiles/test.dir/Grasp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Grasp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/Grasp.cpp -o CMakeFiles/test.dir/Grasp.cpp.s

CMakeFiles/test.dir/HandMaker.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/HandMaker.cpp.o: ../HandMaker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test.dir/HandMaker.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/HandMaker.cpp.o -c /home/mrl/Test/Grasping/HandMaker.cpp

CMakeFiles/test.dir/HandMaker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/HandMaker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/HandMaker.cpp > CMakeFiles/test.dir/HandMaker.cpp.i

CMakeFiles/test.dir/HandMaker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/HandMaker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/HandMaker.cpp -o CMakeFiles/test.dir/HandMaker.cpp.s

CMakeFiles/test.dir/IkSolver.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/IkSolver.cpp.o: ../IkSolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/test.dir/IkSolver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/IkSolver.cpp.o -c /home/mrl/Test/Grasping/IkSolver.cpp

CMakeFiles/test.dir/IkSolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/IkSolver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/IkSolver.cpp > CMakeFiles/test.dir/IkSolver.cpp.i

CMakeFiles/test.dir/IkSolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/IkSolver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/IkSolver.cpp -o CMakeFiles/test.dir/IkSolver.cpp.s

CMakeFiles/test.dir/MyWindow.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/MyWindow.cpp.o: ../MyWindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/test.dir/MyWindow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/MyWindow.cpp.o -c /home/mrl/Test/Grasping/MyWindow.cpp

CMakeFiles/test.dir/MyWindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/MyWindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/MyWindow.cpp > CMakeFiles/test.dir/MyWindow.cpp.i

CMakeFiles/test.dir/MyWindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/MyWindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/MyWindow.cpp -o CMakeFiles/test.dir/MyWindow.cpp.s

CMakeFiles/test.dir/SkelParser.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/SkelParser.cpp.o: ../SkelParser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/test.dir/SkelParser.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/SkelParser.cpp.o -c /home/mrl/Test/Grasping/SkelParser.cpp

CMakeFiles/test.dir/SkelParser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/SkelParser.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/SkelParser.cpp > CMakeFiles/test.dir/SkelParser.cpp.i

CMakeFiles/test.dir/SkelParser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/SkelParser.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/SkelParser.cpp -o CMakeFiles/test.dir/SkelParser.cpp.s

CMakeFiles/test.dir/main.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/test.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/main.cpp.o -c /home/mrl/Test/Grasping/main.cpp

CMakeFiles/test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mrl/Test/Grasping/main.cpp > CMakeFiles/test.dir/main.cpp.i

CMakeFiles/test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mrl/Test/Grasping/main.cpp -o CMakeFiles/test.dir/main.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/Contact.cpp.o" \
"CMakeFiles/test.dir/Controller.cpp.o" \
"CMakeFiles/test.dir/Grasp.cpp.o" \
"CMakeFiles/test.dir/HandMaker.cpp.o" \
"CMakeFiles/test.dir/IkSolver.cpp.o" \
"CMakeFiles/test.dir/MyWindow.cpp.o" \
"CMakeFiles/test.dir/SkelParser.cpp.o" \
"CMakeFiles/test.dir/main.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/Contact.cpp.o
test: CMakeFiles/test.dir/Controller.cpp.o
test: CMakeFiles/test.dir/Grasp.cpp.o
test: CMakeFiles/test.dir/HandMaker.cpp.o
test: CMakeFiles/test.dir/IkSolver.cpp.o
test: CMakeFiles/test.dir/MyWindow.cpp.o
test: CMakeFiles/test.dir/SkelParser.cpp.o
test: CMakeFiles/test.dir/main.cpp.o
test: CMakeFiles/test.dir/build.make
test: /usr/local/lib/libdart-gui.so.6.10.0
test: /usr/local/lib/libdart-collision-bullet.so.6.10.0
test: /usr/local/lib/libdart-optimizer-ipopt.so.6.10.0
test: /usr/local/lib/libdart-utils.so.6.10.0
test: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
test: /usr/lib/x86_64-linux-gnu/libglut.so
test: /usr/lib/x86_64-linux-gnu/libXmu.so
test: /usr/lib/x86_64-linux-gnu/libXi.so
test: /usr/lib/x86_64-linux-gnu/libGLU.so
test: /usr/local/lib/libdart-external-lodepng.so.6.10.0
test: /usr/local/lib/libdart-external-imgui.so.6.10.0
test: /usr/lib/x86_64-linux-gnu/libGL.so
test: /usr/lib/x86_64-linux-gnu/libBulletDynamics.so
test: /usr/lib/x86_64-linux-gnu/libBulletCollision.so
test: /usr/lib/x86_64-linux-gnu/libLinearMath.so
test: /usr/lib/x86_64-linux-gnu/libBulletSoftBody.so
test: /usr/local/lib/libdart.so.6.10.0
test: /usr/local/lib/libdart-external-odelcpsolver.so.6.10.0
test: /usr/lib/x86_64-linux-gnu/libccd.so
test: /usr/local/lib/libfcl.so
test: /usr/lib/x86_64-linux-gnu/libassimp.so
test: /usr/local/lib/libboost_filesystem.so
test: /usr/local/lib/libboost_system.so
test: /usr/lib/liboctomap.so
test: /usr/lib/liboctomath.so
test: /usr/local/lib/libboost_regex.so
test: /usr/lib/libipopt.so
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mrl/Test/Grasping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/mrl/Test/Grasping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mrl/Test/Grasping /home/mrl/Test/Grasping /home/mrl/Test/Grasping/build /home/mrl/Test/Grasping/build /home/mrl/Test/Grasping/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

